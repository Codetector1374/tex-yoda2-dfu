use super::*;
use usb_device::endpoint::EndpointType;
use crate::dfu::{State, Status, APP_BASE, FLASH_END, PAGE_SIZE};
use std::collections::VecDeque;
use std::sync::{Arc, Mutex};

#[derive(Clone, Copy)]
enum Event {
    Reset,
    Suspend,
    Resume,
    Data { setup: bool, out: bool, ack: bool },
}

#[derive(Default)]
struct BusState {
    events: VecDeque<Event>,
    setup: Option<Vec<u8>>,
    received: VecDeque<Vec<u8>>,
    transmitted: Option<Vec<u8>>,
    busy: bool,
    stalled: [bool; 2],
    address: u8,
    block_writes: usize,
    suspends: usize,
    resumes: usize,
}

#[derive(Clone, Default)]
struct Host(Arc<Mutex<BusState>>);
struct FakeBus(Host);

impl UsbBus for FakeBus {
    fn alloc_ep(&mut self, _: UsbDirection, _: Option<EndpointAddress>, _: EndpointType, _: u16, _: u8) -> usb_device::Result<EndpointAddress> {
        panic!("fixed EP0 must not use dynamic endpoint allocation")
    }
    fn enable(&mut self) {}
    fn reset(&self) {
        let mut state = self.0.0.lock().unwrap();
        state.setup = None;
        state.received.clear();
        state.transmitted = None;
        state.busy = false;
        state.stalled = [false; 2];
        state.address = 0;
    }
    fn set_device_address(&self, address: u8) { self.0.0.lock().unwrap().address = address; }
    fn write(&self, address: EndpointAddress, bytes: &[u8]) -> usb_device::Result<usize> {
        assert_eq!(u8::from(address), IN);
        assert!(bytes.len() <= PACKET);
        let mut state = self.0.0.lock().unwrap();
        if state.block_writes != 0 {
            state.block_writes -= 1;
            return Err(UsbError::WouldBlock);
        }
        if state.busy { return Err(UsbError::WouldBlock); }
        assert!(!state.stalled[1]);
        state.busy = true;
        state.transmitted = Some(bytes.to_vec());
        Ok(bytes.len())
    }
    fn read(&self, address: EndpointAddress, bytes: &mut [u8]) -> usb_device::Result<usize> {
        assert_eq!(u8::from(address), OUT);
        let mut state = self.0.0.lock().unwrap();
        // Match HT32: SETUP has priority over the independent pending OUT SRAM.
        let packet = match state.setup.take() {
            Some(packet) => packet,
            None => state.received.pop_front().ok_or(UsbError::WouldBlock)?,
        };
        if packet.len() > bytes.len() { return Err(UsbError::BufferOverflow); }
        bytes[..packet.len()].copy_from_slice(&packet);
        Ok(packet.len())
    }
    fn set_stalled(&self, address: EndpointAddress, stalled: bool) {
        self.0.0.lock().unwrap().stalled[usize::from(address.is_in())] = stalled;
    }
    fn is_stalled(&self, address: EndpointAddress) -> bool {
        self.0.0.lock().unwrap().stalled[usize::from(address.is_in())]
    }
    fn suspend(&self) { self.0.0.lock().unwrap().suspends += 1; }
    fn resume(&self) { self.0.0.lock().unwrap().resumes += 1; }
    fn poll(&self) -> PollResult {
        match self.0.0.lock().unwrap().events.pop_front() {
            None => PollResult::None,
            Some(Event::Reset) => PollResult::Reset,
            Some(Event::Suspend) => PollResult::Suspend,
            Some(Event::Resume) => PollResult::Resume,
            Some(Event::Data { setup, out, ack }) => PollResult::Data {
                ep_setup: u16::from(setup), ep_out: u16::from(out), ep_in_complete: u16::from(ack),
            },
        }
    }
}

impl Host {
    fn setup(&self, bm: u8, code: u8, value: u16, index: u16, length: u16) {
        self.setup_with_events(bm, code, value, index, length, false, None);
    }
    fn setup_with_events(&self, bm: u8, code: u8, value: u16, index: u16, length: u16, ack: bool, old_out: Option<&[u8]>) {
        let v = value.to_le_bytes();
        let i = index.to_le_bytes();
        let l = length.to_le_bytes();
        let mut state = self.0.lock().unwrap();
        // A physical SETUP aborts a previous unsent IN packet.
        state.transmitted = None;
        state.busy = false;
        state.setup = Some(vec![bm, code, v[0], v[1], i[0], i[1], l[0], l[1]]);
        if let Some(bytes) = old_out { state.received.push_back(bytes.to_vec()); }
        state.events.push_back(Event::Data { setup: true, out: old_out.is_some(), ack });
    }
    fn out(&self, bytes: &[u8], ack: bool) {
        let mut state = self.0.lock().unwrap();
        if ack { state.busy = false; state.transmitted = None; }
        state.received.push_back(bytes.to_vec());
        state.events.push_back(Event::Data { setup: false, out: true, ack });
    }
    fn ack(&self) {
        let mut state = self.0.lock().unwrap();
        state.busy = false;
        state.transmitted = None;
        state.events.push_back(Event::Data { setup: false, out: false, ack: true });
    }
    fn packet(&self) -> Vec<u8> { self.0.lock().unwrap().transmitted.take().expect("expected an IN packet") }
    fn has_packet(&self) -> bool { self.0.lock().unwrap().transmitted.is_some() }
    fn stalled(&self) -> bool { self.0.lock().unwrap().stalled == [true; 2] }
    fn reset(&self) { self.0.lock().unwrap().events.push_back(Event::Reset); }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Operation { Erase(u32), Program(u32, u32) }
struct FlashState { bytes: Vec<u8>, operations: Vec<Operation> }
#[derive(Clone)]
struct MemoryFlash(Arc<Mutex<FlashState>>);

impl MemoryFlash {
    fn new() -> Self {
        Self(Arc::new(Mutex::new(FlashState {
            bytes: (0..0x10000usize).map(|i| (i.wrapping_mul(37) + i / 3) as u8).collect(),
            operations: Vec::new(),
        })))
    }
    fn bytes(&self, start: u32, count: usize) -> Vec<u8> {
        self.0.lock().unwrap().bytes[start as usize..start as usize + count].to_vec()
    }
    fn operations(&self) -> Vec<Operation> { self.0.lock().unwrap().operations.clone() }
}

impl Flash for MemoryFlash {
    fn read_word(&self, address: u32) -> u32 {
        assert_eq!(address % 4, 0);
        let state = self.0.lock().unwrap();
        u32::from_le_bytes(state.bytes[address as usize..address as usize + 4].try_into().unwrap())
    }
    fn erase_page(&mut self, address: u32) -> Result<(), Status> {
        assert!(address >= APP_BASE && address + PAGE_SIZE <= FLASH_END);
        assert_eq!(address % PAGE_SIZE, 0);
        let mut state = self.0.lock().unwrap();
        state.operations.push(Operation::Erase(address));
        state.bytes[address as usize..(address + PAGE_SIZE) as usize].fill(0xff);
        Ok(())
    }
    fn program_word(&mut self, address: u32, word: u32) -> Result<(), Status> {
        assert!(address >= APP_BASE && address + 4 <= FLASH_END);
        assert_eq!(address % 4, 0);
        let mut state = self.0.lock().unwrap();
        state.operations.push(Operation::Program(address, word));
        for (offset, byte) in word.to_le_bytes().into_iter().enumerate() { state.bytes[address as usize + offset] &= byte; }
        Ok(())
    }
}

type TestDevice = Device<FakeBus>;

fn fixture(configured: bool, f: impl FnOnce(&Host, &mut TestDevice, &mut Dfu<MemoryFlash>, &MemoryFlash)) {
    let host = Host::default();
    let flash = MemoryFlash::new();
    let mut dfu = Dfu::new(flash.clone());
    let mut device = Device::new(FakeBus(host.clone()));
    host.reset();
    device.poll(&mut dfu);
    if configured {
        for (request, value) in [(5, 7), (9, 1)] {
            host.setup(0, request, value, 0, 0);
            device.poll(&mut dfu);
            ack_status(&host, &mut device, &mut dfu);
        }
    }
    assert!(!dfu.has_activity());
    f(&host, &mut device, &mut dfu, &flash);
}

fn ack_status(host: &Host, device: &mut TestDevice, dfu: &mut Dfu<MemoryFlash>) {
    assert!(host.packet().is_empty());
    host.ack();
    device.poll(dfu);
}

fn read_control(host: &Host, device: &mut TestDevice, dfu: &mut Dfu<MemoryFlash>, bm: u8, code: u8, value: u16, index: u16, length: u16) -> (Vec<u8>, Vec<usize>) {
    host.setup(bm, code, value, index, length);
    device.poll(dfu);
    let mut bytes = Vec::new();
    let mut packets = Vec::new();
    loop {
        let packet = host.packet();
        let final_packet = packet.len() < PACKET || bytes.len() + packet.len() == length as usize;
        packets.push(packet.len());
        bytes.extend_from_slice(&packet);
        if final_packet {
            host.out(&[], true);
            device.poll(dfu);
            break;
        }
        host.ack();
        device.poll(dfu);
    }
    assert!(!host.has_packet());
    (bytes, packets)
}

fn image() -> Vec<u8> {
    let mut bytes: Vec<u8> = (0..512).map(|i| (i * 73) as u8).collect();
    bytes[..4].copy_from_slice(&0x2000_4000u32.to_le_bytes());
    bytes[4..8].copy_from_slice(&(APP_BASE + 9).to_le_bytes());
    bytes
}

fn download(host: &Host, device: &mut TestDevice, dfu: &mut Dfu<MemoryFlash>) {
    host.setup(0x21, 1, 0, 0, 512);
    device.poll(dfu);
    for chunk in image().chunks(PACKET) {
        host.out(chunk, false);
        device.poll(dfu);
    }
}

fn upload_without_status(host: &Host, device: &mut TestDevice, dfu: &mut Dfu<MemoryFlash>) -> Vec<u8> {
    host.setup(0xa1, 2, 0, 0, 512);
    device.poll(dfu);
    let mut bytes = Vec::new();
    for index in 0..8 {
        let packet = host.packet();
        assert_eq!(packet.len(), PACKET);
        bytes.extend_from_slice(&packet);
        if index != 7 { host.ack(); device.poll(dfu); }
    }
    bytes
}

#[test]
fn fixed_descriptors_enumerate_without_dfu_activity() {
    fixture(false, |host, device, dfu, _| {
        for (bm, value, index, expected) in [
            (0x80, 0x0100, 0, DEVICE), (0x80, 0x0200, 0, CONFIG),
            (0x81, 0x2100, 0, DFU),
        ] {
            let (bytes, packets) = read_control(host, device, dfu, bm, 6, value, index, 255);
            assert_eq!(bytes, expected);
            assert_eq!(packets, [expected.len()]);
        }
        let (bytes, packets) = read_control(host, device, dfu, 0x80, 6, 0x0100, 0, 8);
        assert_eq!(bytes, DEVICE[..8]);
        assert_eq!(packets, [8]);
        assert_eq!(CONFIG.len(), u16::from_le_bytes([CONFIG[2], CONFIG[3]]) as usize);
        assert_eq!(&CONFIG[18..], DFU);
        assert!(!dfu.has_activity());
    });
}

#[test]
fn address_and_configuration_change_only_on_status_ack_including_coalesced_setup() {
    fixture(false, |host, device, dfu, _| {
        host.setup(0, 5, 7, 0, 0);
        device.poll(dfu);
        assert_eq!(host.0.lock().unwrap().address, 0);
        assert!(host.packet().is_empty());
        host.setup_with_events(0x80, 8, 0, 0, 1, true, None);
        device.poll(dfu);
        assert_eq!(host.0.lock().unwrap().address, 7);
        assert_eq!(host.packet(), [0]);
        host.out(&[], true);
        device.poll(dfu);
        host.setup(0, 9, 1, 0, 0);
        device.poll(dfu);
        assert!(!device.configured);
        assert!(host.packet().is_empty());
        host.setup_with_events(0x81, 10, 0, 0, 1, true, None);
        device.poll(dfu);
        assert!(device.configured);
        assert_eq!(host.packet(), [0]);
        host.out(&[], true);
        device.poll(dfu);
        host.setup(0, 9, 0, 0, 0);
        device.poll(dfu);
        assert!(host.packet().is_empty());
        // An unacknowledged configuration change is canceled, not committed by SETUP.
        assert_eq!(read_control(host, device, dfu, 0x80, 8, 0, 0, 1).0, [1]);
        host.setup(0, 9, 0, 0, 0);
        device.poll(dfu);
        ack_status(host, device, dfu);
        assert!(!device.configured);
        host.setup(0, 5, 0, 0, 0);
        device.poll(dfu);
        ack_status(host, device, dfu);
        assert_eq!(host.0.lock().unwrap().address, 0);
        assert!(!dfu.has_activity());
    });
}

#[test]
fn standard_status_interface_and_endpoint_halt_are_real_handshakes() {
    fixture(true, |host, device, dfu, _| {
        for (bm, index) in [(0x80, 0), (0x81, 0), (0x82, 0), (0x82, 0x80)] {
            assert_eq!(read_control(host, device, dfu, bm, 0, 0, index, 2).0, [0, 0]);
        }
        assert_eq!(read_control(host, device, dfu, 0x81, 10, 0, 0, 1).0, [0]);
        host.setup(1, 11, 0, 0, 0);
        device.poll(dfu);
        ack_status(host, device, dfu);
        host.setup(2, 3, 0, 0x80, 0);
        device.poll(dfu);
        assert!(!host.0.lock().unwrap().stalled[1]);
        ack_status(host, device, dfu);
        assert!(host.0.lock().unwrap().stalled[1]);
        host.setup(2, 1, 0, 0x80, 0);
        device.poll(dfu);
        ack_status(host, device, dfu);
        assert_eq!(host.0.lock().unwrap().stalled, [false; 2]);
        assert!(!dfu.has_activity());
    });
}

#[test]
fn unsupported_standard_requests_stall_and_next_setup_recovers() {
    fixture(true, |host, device, dfu, _| {
        for (bm, code, value, index, length) in [
            (0x80, 6, 0x0600, 0, 10), (0x80, 6, 0x0303, 0x0409, 255),
            (0x80, 6, 0x0301, 0x0411, 255), (0x81, 10, 0, 1, 1),
            (0x82, 0, 0, 0x81, 2), (0x80, 0, 1, 0, 2),
            (0, 5, 128, 0, 0), (0, 9, 2, 0, 0), (1, 11, 1, 0, 0),
            (0, 3, 1, 0, 0), (2, 3, 0, 1, 0), (0x82, 12, 0, 0, 2),
        ] {
            host.setup(bm, code, value, index, length);
            device.poll(dfu);
            assert!(host.stalled());
            assert!(!host.has_packet());
            assert_eq!(read_control(host, device, dfu, 0x80, 6, 0x0100, 0, 18).0, DEVICE);
        }
        assert!(!dfu.has_activity());
        assert_eq!(dfu.state(), State::Idle);
    });
}

#[test]
fn download_512_and_getstatus_release_flash_only_after_both_status_handshakes() {
    fixture(true, |host, device, dfu, flash| {
        download(host, device, dfu);
        assert_eq!(dfu.state(), State::Idle);
        dfu.tick(0);
        assert!(flash.operations().is_empty());
        ack_status(host, device, dfu);
        assert_eq!(dfu.state(), State::DownloadSync);
        host.setup(0xa1, 3, 0, 0, 6);
        device.poll(dfu);
        assert_eq!(host.packet(), [0, 10, 0, 0, 4, 0]);
        dfu.tick(1);
        assert!(flash.operations().is_empty());
        host.ack();
        device.poll(dfu);
        dfu.tick(2);
        assert!(flash.operations().is_empty());
        host.out(&[], false);
        device.poll(dfu);
        dfu.tick(3);
        assert_eq!(flash.operations(), [Operation::Erase(APP_BASE)]);
        assert_eq!(dfu.state(), State::DownloadBusy);
    });
}

#[test]
fn full_512_byte_upload_has_no_extra_zlp_and_commits_only_on_status_out() {
    fixture(true, |host, device, dfu, flash| {
        assert_eq!(upload_without_status(host, device, dfu), flash.bytes(APP_BASE, 512));
        assert_eq!(dfu.state(), State::Idle);
        host.ack();
        device.poll(dfu);
        assert!(!host.has_packet());
        assert_eq!(dfu.state(), State::Idle);
        host.out(&[], false);
        device.poll(dfu);
        assert_eq!(dfu.state(), State::UploadIdle);
        let (bytes, packets) = read_control(host, device, dfu, 0xa1, 2, 1, 0, 512);
        assert_eq!(bytes, flash.bytes(APP_BASE + 512, 512));
        assert_eq!(packets, [64; 8]);
        assert!(flash.operations().is_empty());
    });
}

#[test]
fn short_upload_multiple_of_64_requires_zlp_and_empty_upload_also_terminates() {
    fixture(true, |host, device, dfu, flash| {
        // Leave exactly 64 bytes in application flash using valid sequential requests.
        for block in 0..109 {
            assert_eq!(read_control(host, device, dfu, 0xa1, 2, block, 0, 512).0.len(), 512);
        }
        assert_eq!(read_control(host, device, dfu, 0xa1, 2, 109, 0, 448).1, [64; 7]);
        host.setup(0xa1, 2, 110, 0, 512);
        device.poll(dfu);
        assert_eq!(host.packet(), flash.bytes(FLASH_END - 64, 64));
        host.ack();
        device.poll(dfu);
        assert!(host.packet().is_empty()); // Required DATA ZLP, not a status IN.
        assert_eq!(dfu.state(), State::UploadIdle);
        host.ack();
        device.poll(dfu);
        assert_eq!(dfu.state(), State::UploadIdle);
        host.out(&[], false);
        device.poll(dfu);
        assert_eq!(dfu.state(), State::Idle);
        // A new stream can also end with an empty packet at the exact flash boundary.
        for block in 0..110 { read_control(host, device, dfu, 0xa1, 2, block, 0, 512); }
        let (bytes, packets) = read_control(host, device, dfu, 0xa1, 2, 110, 0, 512);
        assert!(bytes.is_empty());
        assert_eq!(packets, [0]);
        assert_eq!(dfu.state(), State::Idle);
    });
}

#[test]
fn superseding_setup_does_not_commit_download_or_a_data_in_ack() {
    fixture(true, |host, device, dfu, flash| {
        download(host, device, dfu);
        assert!(host.packet().is_empty());
        assert_eq!(read_control(host, device, dfu, 0xa1, 5, 0, 0, 1).0, [State::Idle as u8]);
        assert!(flash.operations().is_empty());
        upload_without_status(host, device, dfu);
        host.setup_with_events(0xa1, 2, 0, 0, 512, true, None);
        device.poll(dfu);
        assert_eq!(host.packet(), flash.bytes(APP_BASE, 64));
        assert_eq!(dfu.state(), State::Idle);
        assert!(!host.stalled());
    });
}

#[test]
fn coalesced_status_in_ack_precedes_the_next_setup() {
    fixture(true, |host, device, dfu, _| {
        download(host, device, dfu);
        assert!(host.packet().is_empty());
        host.setup_with_events(0xa1, 5, 0, 0, 1, true, None);
        device.poll(dfu);
        assert_eq!(host.packet(), [State::DownloadSync as u8]);
        host.out(&[], true);
        device.poll(dfu);
        assert_eq!(dfu.state(), State::DownloadSync);
    });
}

#[test]
fn coalesced_old_status_out_and_last_data_ack_precede_next_setup() {
    fixture(true, |host, device, dfu, flash| {
        upload_without_status(host, device, dfu);
        host.setup_with_events(0xa1, 2, 1, 0, 512, true, Some(&[]));
        device.poll(dfu);
        assert_eq!(dfu.state(), State::UploadIdle);
        assert_eq!(host.packet(), flash.bytes(APP_BASE + 512, 64));
        assert!(!host.stalled());
    });
}

#[test]
fn coalesced_old_data_out_is_discarded_not_a_status_commit() {
    fixture(true, |host, device, dfu, flash| {
        host.setup(0x21, 1, 0, 0, 512);
        device.poll(dfu);
        host.setup_with_events(0xa1, 5, 0, 0, 1, false, Some(&image()[..64]));
        device.poll(dfu);
        assert_eq!(host.packet(), [State::Idle as u8]);
        host.out(&[], true);
        device.poll(dfu);
        assert!(flash.operations().is_empty());
    });
}

#[test]
fn reset_cancels_partial_out_queued_status_and_getstatus_work() {
    fixture(true, |host, device, dfu, flash| {
        host.setup(0x21, 1, 0, 0, 512);
        device.poll(dfu);
        host.out(&image()[..64], false);
        device.poll(dfu);
        host.reset();
        device.poll(dfu);
        assert_eq!(dfu.state(), State::Idle);
        assert!(!device.configured);
        assert_eq!(host.0.lock().unwrap().address, 0);
        assert!(dfu.has_activity());
        assert!(flash.operations().is_empty());
    });
    for after_download_ack in [false, true] {
        fixture(true, |host, device, dfu, flash| {
            download(host, device, dfu);
            if after_download_ack {
                ack_status(host, device, dfu);
                host.setup(0xa1, 3, 0, 0, 6);
                device.poll(dfu);
                assert_eq!(host.packet().len(), 6);
                host.ack();
                device.poll(dfu);
            }
            host.reset();
            device.poll(dfu);
            for now in 0..20 { dfu.tick(now); }
            assert_eq!(dfu.state(), State::Idle);
            assert!(flash.operations().is_empty());
            assert!(dfu.has_activity());
        });
    }
}

#[test]
fn malformed_out_lengths_never_release_flash_and_clearstatus_recovers() {
    for (declared, packets) in [
        (513, Vec::<Vec<u8>>::new()),
        (512, vec![vec![0; 63]]),
        (3, vec![vec![0; 4]]),
        (512, vec![vec![0; 65]]),
    ] {
        fixture(true, |host, device, dfu, flash| {
            host.setup(0x21, 1, 0, 0, declared);
            device.poll(dfu);
            for packet in packets { host.out(&packet, false); device.poll(dfu); }
            assert!(host.stalled());
            assert_eq!(dfu.state(), State::Error);
            dfu.tick(0);
            assert!(flash.operations().is_empty());
            host.setup(0x21, 4, 0, 0, 0);
            device.poll(dfu);
            ack_status(host, device, dfu);
            assert_eq!(dfu.state(), State::Idle);
        });
    }
}

#[test]
fn malformed_setup_discards_staged_action_and_late_ack_cannot_commit() {
    fixture(true, |host, device, dfu, flash| {
        download(host, device, dfu);
        assert!(host.packet().is_empty());
        {
            let mut state = host.0.lock().unwrap();
            state.setup = Some(vec![0; 7]);
            state.transmitted = None;
            state.busy = false;
            state.events.push_back(Event::Data { setup: true, out: false, ack: false });
        }
        device.poll(dfu);
        assert!(host.stalled());
        assert_eq!(dfu.state(), State::Error);
        host.ack();
        device.poll(dfu);
        dfu.tick(0);
        assert!(flash.operations().is_empty());
        assert_eq!(dfu.state(), State::Error);
        assert_eq!(read_control(host, device, dfu, 0x80, 6, 0x0100, 0, 18).0, DEVICE);
    });
}

#[test]
fn premature_or_nonzero_status_out_cannot_commit_upload() {
    for complete_data in [false, true] {
        fixture(true, |host, device, dfu, flash| {
            if complete_data { upload_without_status(host, device, dfu); }
            else {
                host.setup(0xa1, 2, 0, 0, 512);
                device.poll(dfu);
                assert_eq!(host.packet().len(), 64);
            }
            host.out(if complete_data { &[1] } else { &[] }, true);
            device.poll(dfu);
            assert!(host.stalled());
            assert_eq!(dfu.state(), State::Error);
            assert!(flash.operations().is_empty());
        });
    }
}

#[test]
fn would_block_retries_packets_without_replaying_request_callbacks() {
    fixture(true, |host, device, dfu, flash| {
        host.0.lock().unwrap().block_writes = 2;
        host.setup(0xa1, 2, 0, 0, 512);
        device.poll(dfu);
        assert!(!host.has_packet());
        device.poll(dfu);
        assert!(!host.has_packet());
        device.poll(dfu);
        let mut bytes = Vec::new();
        for index in 0..8 {
            bytes.extend_from_slice(&host.packet());
            if index != 7 { host.ack(); device.poll(dfu); }
        }
        assert_eq!(bytes, flash.bytes(APP_BASE, 512));
        host.out(&[], true);
        device.poll(dfu);
        assert_eq!(dfu.state(), State::UploadIdle);
        assert_eq!(read_control(host, device, dfu, 0xa1, 2, 1, 0, 1).0, flash.bytes(APP_BASE + 512, 1));
        host.setup(0x21, 6, 0, 0, 0);
        device.poll(dfu);
        ack_status(host, device, dfu);
        host.setup(0x21, 1, 0, 0, 512);
        device.poll(dfu);
        let image = image();
        for chunk in image[..448].chunks(64) { host.out(chunk, false); device.poll(dfu); }
        host.0.lock().unwrap().block_writes = 1;
        host.out(&image[448..], false);
        device.poll(dfu);
        assert!(!host.has_packet());
        assert_eq!(dfu.state(), State::Idle);
        device.poll(dfu);
        ack_status(host, device, dfu);
        assert_eq!(dfu.state(), State::DownloadSync);
    });
}

#[test]
fn suspend_resume_preserves_a_partial_control_transfer() {
    fixture(true, |host, device, dfu, _| {
        host.setup(0x21, 1, 0, 0, 512);
        device.poll(dfu);
        let image = image();
        host.out(&image[..64], false);
        device.poll(dfu);
        host.0.lock().unwrap().events.push_back(Event::Suspend);
        device.poll(dfu);
        device.poll(dfu);
        assert_eq!(host.0.lock().unwrap().suspends, 1);
        host.0.lock().unwrap().events.push_back(Event::Resume);
        device.poll(dfu);
        assert_eq!(host.0.lock().unwrap().resumes, 1);
        for chunk in image[64..].chunks(64) { host.out(chunk, false); device.poll(dfu); }
        ack_status(host, device, dfu);
        assert_eq!(dfu.state(), State::DownloadSync);
    });
}
