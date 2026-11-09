use super::*;
use std::vec;
use std::vec::Vec;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Operation {
    Erase(u32),
    Program(u32, u32),
}

struct MemoryFlash {
    bytes: Vec<u8>,
    operations: Vec<Operation>,
    fail_program: Option<u32>,
    bad_sp: bool,
}

impl MemoryFlash {
    fn new() -> Self {
        Self {
            bytes: vec![0xa5; 0x10000],
            operations: Vec::new(),
            fail_program: None,
            bad_sp: false,
        }
    }
}

impl Flash for MemoryFlash {
    fn read_word(&self, address: u32) -> u32 {
        assert_eq!(address % 4, 0);
        u32::from_le_bytes(self.bytes[address as usize..address as usize + 4].try_into().unwrap())
    }

    fn erase_page(&mut self, address: u32) -> Result<(), Status> {
        assert!(address >= APP_BASE && address + PAGE_SIZE <= FLASH_END);
        assert_eq!(address % PAGE_SIZE, 0);
        self.operations.push(Operation::Erase(address));
        self.bytes[address as usize..(address + PAGE_SIZE) as usize].fill(0xff);
        Ok(())
    }

    fn program_word(&mut self, address: u32, word: u32) -> Result<(), Status> {
        assert!(address >= APP_BASE && address + 4 <= FLASH_END);
        assert_eq!(address % 4, 0);
        self.operations.push(Operation::Program(address, word));
        if self.fail_program == Some(address) {
            return Err(Status::ErrProg);
        }
        let bytes = if self.bad_sp && address == APP_BASE {
            // A partially programmed final SP can still look like valid SRAM.
            0x2000_2000u32.to_le_bytes()
        } else {
            word.to_le_bytes()
        };
        for (offset, byte) in bytes.into_iter().enumerate() {
            self.bytes[address as usize + offset] &= byte;
        }
        Ok(())
    }
}

fn image(length: usize) -> Vec<u8> {
    assert!(length >= 9);
    let mut bytes: Vec<u8> = (0..length).map(|i| (i.wrapping_mul(73)) as u8).collect();
    bytes[..4].copy_from_slice(&0x2000_4000u32.to_le_bytes());
    bytes[4..8].copy_from_slice(&(APP_BASE + 9).to_le_bytes());
    bytes
}

fn request(direction: UsbDirection, code: u8, value: u16, length: u16) -> Request {
    Request {
        direction,
        request_type: RequestType::Class,
        recipient: Recipient::Interface,
        request: code,
        value,
        index: 0,
        length,
    }
}

fn get_status(engine: &mut Engine<MemoryFlash>) -> [u8; 6] {
    engine.action = Action::None;
    let mut reply = [0; 6];
    assert_eq!(engine.prepare_in(&request(UsbDirection::In, 3, 0, 6), &mut reply), Ok(6));
    reply
}

fn run_ready(engine: &mut Engine<MemoryFlash>, clock: &mut u32) {
    for _ in 0..2000 {
        let before = engine.flash.operations.len();
        engine.tick(*clock);
        assert!(engine.flash.operations.len() - before <= 1, "more than one FMC command in a tick");
        *clock = clock.wrapping_add(1);
        if !engine.ready && !engine.poll_arm && engine.poll_started.is_none() {
            return;
        }
    }
    panic!("bounded worker did not finish");
}

fn download(engine: &mut Engine<MemoryFlash>, block: u16, bytes: &[u8], clock: &mut u32) {
    engine.action = Action::None;
    assert_eq!(engine.prepare_out(&request(UsbDirection::Out, 1, block, bytes.len() as u16), bytes), Ok(()));
    engine.complete();
    assert_eq!(engine.state, State::DownloadSync);
    let reply = get_status(engine);
    assert_eq!(&reply[..5], &[0, 10, 0, 0, State::DownloadBusy as u8]);
    engine.complete();
    run_ready(engine, clock);
    assert_eq!(get_status(engine)[4], State::DownloadIdle as u8);
    engine.complete();
    assert_eq!(engine.state, State::DownloadIdle);
}

fn manifest(engine: &mut Engine<MemoryFlash>, block: u16, clock: &mut u32) {
    engine.action = Action::None;
    assert_eq!(engine.prepare_out(&request(UsbDirection::Out, 1, block, 0), &[]), Ok(()));
    engine.complete();
    assert_eq!(get_status(engine)[4], State::Manifest as u8);
    engine.complete();
    run_ready(engine, clock);
}

#[test]
fn odd_blocks_form_one_stream_and_commit_sp_last() {
    let bytes = image(1031);
    let mut engine = Engine::new(MemoryFlash::new());
    let mut clock = 0;
    let mut offset = 0;
    for (block, length) in [1, 2, 4, 3, 511, 510].into_iter().enumerate() {
        download(&mut engine, block as u16, &bytes[offset..offset + length], &mut clock);
        offset += length;
        assert!(!application_valid(&engine.flash));
        assert_eq!(engine.flash.read_word(APP_BASE), u32::MAX);
        assert_eq!(engine.flash.read_word(APP_BASE + 4), u32::MAX);
    }
    manifest(&mut engine, 6, &mut clock);
    assert_eq!(engine.state, State::ManifestWaitReset);
    assert!(application_valid(&engine.flash));
    assert_eq!(&engine.flash.bytes[APP_BASE as usize..APP_BASE as usize + bytes.len()], bytes.as_slice());
    assert_eq!(engine.flash.bytes[APP_BASE as usize + bytes.len()], 0xff);
    let erases: Vec<u32> = engine.flash.operations.iter().filter_map(|op| match op {
        Operation::Erase(address) => Some(*address), _ => None,
    }).collect();
    assert_eq!(erases, vec![APP_BASE, APP_BASE + PAGE_SIZE]);
    assert_eq!(engine.flash.operations.last(), Some(&Operation::Program(APP_BASE, 0x2000_4000)));
    assert_eq!(engine.flash.operations[engine.flash.operations.len() - 2], Operation::Program(APP_BASE + 4, APP_BASE + 9));
    assert!(engine.flash.bytes[..APP_BASE as usize].iter().all(|b| *b == 0xa5));
    assert!(engine.flash.bytes[FLASH_END as usize..].iter().all(|b| *b == 0xa5));
    assert!(!engine.reset_requested);
    engine.bus_reset();
    assert!(engine.reset_requested);
}

#[test]
fn control_status_is_the_only_commit_point_and_reset_cancels_work() {
    let mut engine = Engine::new(MemoryFlash::new());
    let bytes = image(512);
    engine.prepare_out(&request(UsbDirection::Out, 1, 0, 512), &bytes).unwrap();
    engine.tick(0);
    assert!(engine.flash.operations.is_empty());
    assert_eq!(engine.downloaded, 0);
    // A superseding SETUP discards the action, including its block counter update.
    engine.action = Action::None;
    engine.complete();
    assert_eq!(engine.state, State::Idle);
    engine.prepare_out(&request(UsbDirection::Out, 1, 0, 512), &bytes).unwrap();
    engine.complete();
    assert_eq!(engine.downloaded, 512);
    engine.tick(1);
    assert!(engine.flash.operations.is_empty());
    assert_eq!(get_status(&mut engine)[4], State::DownloadBusy as u8);
    engine.tick(2);
    assert!(engine.flash.operations.is_empty());
    engine.action = Action::None;
    engine.bus_reset();
    engine.tick(3);
    assert!(engine.flash.operations.is_empty());
    assert_eq!(engine.state, State::Idle);
    // Once work starts, reset can stop after erase without ever committing vectors.
    engine.prepare_out(&request(UsbDirection::Out, 1, 0, 512), &bytes).unwrap();
    engine.complete();
    get_status(&mut engine);
    engine.complete();
    engine.tick(4);
    assert_eq!(engine.flash.operations, vec![Operation::Erase(APP_BASE)]);
    engine.bus_reset();
    for now in 5..20 { engine.tick(now); }
    assert_eq!(engine.flash.operations.len(), 1);
    assert!(!application_valid(&engine.flash));
}

#[test]
fn busy_window_is_ten_milliseconds_even_for_one_byte_and_wraps() {
    let mut engine = Engine::new(MemoryFlash::new());
    engine.prepare_out(&request(UsbDirection::Out, 1, 0, 1), &[0]).unwrap();
    engine.complete();
    get_status(&mut engine);
    engine.complete();
    let mut state = [0];
    assert_eq!(engine.prepare_in(&request(UsbDirection::In, 5, 0, 1), &mut state), Ok(1));
    assert_eq!(state[0], State::DownloadBusy as u8);
    engine.tick(u32::MAX - 5);
    engine.tick(u32::MAX - 4);
    assert!(!engine.ready);
    engine.tick(3);
    assert_eq!(engine.state, State::DownloadBusy);
    assert_eq!(engine.prepare_in(&request(UsbDirection::In, 5, 0, 1), &mut state), Ok(1));
    assert_eq!(state[0], State::DownloadBusy as u8);
    engine.tick(4);
    assert_eq!(engine.state, State::DownloadSync);
    assert_eq!(get_status(&mut engine)[4], State::DownloadIdle as u8);
}

#[test]
fn early_status_setup_reports_busy_without_cancelling_the_download() {
    let mut dfu = Dfu::new(MemoryFlash::new());
    dfu.set_configured(true);
    let download = request(UsbDirection::Out, 1, 0, 1);
    dfu.setup(&download);
    dfu.request_out(&download, &[0]).unwrap();
    dfu.status_complete(&download);
    let status = request(UsbDirection::In, 3, 0, 6);
    let mut reply = [0; 6];
    dfu.setup(&status);
    dfu.request_in(&status, &mut reply).unwrap();
    dfu.status_complete(&status);
    dfu.tick(0);
    dfu.setup(&status);
    assert_eq!(dfu.request_in(&status, &mut reply), Ok(6));
    assert_eq!(reply, [Status::Ok as u8, 10, 0, 0, State::DownloadBusy as u8, 0]);
    dfu.status_complete(&status);
    assert_eq!(dfu.state(), State::DownloadBusy);
    assert!(dfu.engine.pending);
    dfu.tick(10);
    dfu.setup(&status);
    dfu.request_in(&status, &mut reply).unwrap();
    assert_eq!(reply[4], State::DownloadIdle as u8);
    assert_eq!(dfu.status(), Status::Ok);
}

#[test]
fn manifestation_checks_vectors_and_entire_body_before_commit() {
    let mut engine = Engine::new(MemoryFlash::new());
    let mut bytes = image(19);
    bytes[4..8].copy_from_slice(&(APP_BASE + 101).to_le_bytes());
    let mut clock = 0;
    download(&mut engine, 0, &bytes, &mut clock);
    manifest(&mut engine, 1, &mut clock);
    assert_eq!(engine.status, Status::ErrFile);
    assert_eq!(engine.state, State::Error);
    assert!(!application_valid(&engine.flash));
    // A corrupted earlier body word must be found by final readback, even though its
    // immediate per-command verification originally succeeded.
    let mut engine = Engine::new(MemoryFlash::new());
    download(&mut engine, 0, &image(19), &mut clock);
    engine.flash.bytes[(APP_BASE + 8) as usize] ^= 1;
    manifest(&mut engine, 1, &mut clock);
    assert_eq!(engine.status, Status::ErrVerify);
    assert_eq!(engine.flash.read_word(APP_BASE + 4), u32::MAX);
    assert_eq!(engine.flash.read_word(APP_BASE), u32::MAX);
}

#[test]
fn failed_body_and_final_commit_never_produce_a_bootable_image() {
    let mut engine = Engine::new(MemoryFlash::new());
    engine.flash.fail_program = Some(APP_BASE + 8);
    let mut clock = 0;
    engine.prepare_out(&request(UsbDirection::Out, 1, 0, 16), &image(16)).unwrap();
    engine.complete();
    get_status(&mut engine);
    engine.complete();
    run_ready(&mut engine, &mut clock);
    assert_eq!(engine.status, Status::ErrProg);
    assert!(!application_valid(&engine.flash));

    let mut engine = Engine::new(MemoryFlash::new());
    engine.flash.bad_sp = true;
    download(&mut engine, 0, &image(13), &mut clock);
    engine.prepare_out(&request(UsbDirection::Out, 1, 1, 0), &[]).unwrap();
    engine.complete();
    get_status(&mut engine);
    engine.complete();
    while engine.ready {
        let count = engine.flash.operations.len();
        engine.tick(clock);
        clock += 1;
        assert!(engine.flash.operations.len() - count <= 1);
    }
    assert_eq!(engine.status, Status::ErrVerify);
    assert!(engine.cleanup_required);
    engine.bus_reset(); // Cancellation must not discard mandatory invalidation.
    engine.tick(clock);
    assert!(!engine.cleanup_required);
    assert!(!application_valid(&engine.flash));
    assert_eq!(engine.flash.operations.last(), Some(&Operation::Erase(APP_BASE)));
}

#[test]
fn capacity_is_byte_based_and_reserved_page_is_never_touched() {
    let mut engine = Engine::new(MemoryFlash::new());
    let bytes = image((FLASH_END - APP_BASE) as usize);
    let mut clock = 0;
    for (block, chunk) in bytes.chunks(512).enumerate() {
        download(&mut engine, block as u16, chunk, &mut clock);
    }
    assert_eq!(engine.prepare_out(&request(UsbDirection::Out, 1, 110, 1), &[0]), Err(Status::ErrAddress));
    manifest(&mut engine, 110, &mut clock);
    assert!(application_valid(&engine.flash));
    assert_eq!(engine.flash.bytes[(FLASH_END - 1) as usize], bytes[bytes.len() - 1]);
    assert!(engine.flash.bytes[FLASH_END as usize..].iter().all(|b| *b == 0xa5));
}

#[test]
fn sequential_upload_abort_clearstatus_and_vector_bounds() {
    let mut engine = Engine::new(MemoryFlash::new());
    let bytes = image(17);
    engine.flash.bytes[APP_BASE as usize..APP_BASE as usize + bytes.len()].copy_from_slice(&bytes);
    assert!(application_valid(&engine.flash));
    let mut reply = [0; 3];
    assert_eq!(engine.prepare_in(&request(UsbDirection::In, 2, 0, 3), &mut reply), Ok(3));
    assert_eq!(reply, bytes[..3]);
    assert_eq!(engine.upload_address, APP_BASE);
    engine.complete();
    assert_eq!(engine.state, State::UploadIdle);
    engine.prepare_in(&request(UsbDirection::In, 2, 1, 3), &mut reply).unwrap();
    assert_eq!(reply, bytes[3..6]);
    engine.complete();
    assert_eq!(engine.prepare_in(&request(UsbDirection::In, 2, 1, 3), &mut reply), Err(Status::ErrStalledPacket));
    engine.prepare_out(&request(UsbDirection::Out, 6, 0, 0), &[]).unwrap();
    assert_eq!(engine.state, State::UploadIdle);
    engine.complete();
    assert_eq!(engine.state, State::Idle);
    engine.reject(Status::ErrAddress);
    engine.prepare_out(&request(UsbDirection::Out, 4, 0, 0), &[]).unwrap();
    assert_eq!(engine.status, Status::ErrAddress);
    engine.complete();
    assert_eq!(engine.status, Status::Ok);
    assert_eq!(engine.state, State::Idle);
    // Upload EOF is a short reply and its state transition also waits for status.
    engine.upload_address = FLASH_END - 1;
    engine.upload_block = 9;
    engine.state = State::UploadIdle;
    assert_eq!(engine.prepare_in(&request(UsbDirection::In, 2, 9, 3), &mut reply), Ok(1));
    engine.complete();
    assert_eq!(engine.state, State::Idle);
    assert!(!vectors_valid(0x2000_0000, APP_BASE + 9, FLASH_END));
    assert!(!vectors_valid(0x2000_4008, APP_BASE + 9, FLASH_END));
    assert!(!vectors_valid(0x2000_3ffc, APP_BASE + 9, FLASH_END));
    assert!(!vectors_valid(0x2000_4000, APP_BASE + 8, FLASH_END));
    assert!(!vectors_valid(0x2000_4000, FLASH_END + 1, FLASH_END));
}

#[test]
fn direct_functional_descriptor_truncates_to_wire_length_not_control_buffer_size() {
    let mut dfu = Dfu {
        configured: false,
        reject_current: false,
        engine: Engine::new(MemoryFlash::new()),
    };
    let mut req = Request {
        direction: UsbDirection::In,
        request_type: RequestType::Standard,
        recipient: Recipient::Interface,
        request: Request::GET_DESCRIPTOR,
        value: 0x2100,
        index: 0,
        length: 255,
    };
    let mut descriptor = [0; 9];
    dfu.setup(&req);
    assert_eq!(dfu.request_in(&req, &mut descriptor), Ok(9));
    assert_eq!(descriptor, FUNCTIONAL_DESCRIPTOR);

    req.length = 4;
    dfu.setup(&req);
    let mut prefix = [0; 4];
    assert_eq!(dfu.request_in(&req, &mut prefix), Ok(4));
    assert_eq!(prefix, FUNCTIONAL_DESCRIPTOR[..4]);

    req.length = 9;
    dfu.setup(&req);
    assert_eq!(dfu.request_in(&req, &mut prefix), Err(Status::ErrStalledPacket));
    assert_eq!(dfu.status(), Status::ErrStalledPacket);
    assert!(dfu.engine.flash.operations.is_empty());
}

#[test]
fn delayed_poll_or_abort_status_cannot_undo_completed_manifestation() {
    for abort in [false, true] {
        let mut engine = Engine::new(MemoryFlash::new());
        let mut clock = 0;
        let bytes = image(2048);
        for (block, chunk) in bytes.chunks(512).enumerate() {
            download(&mut engine, block as u16, chunk, &mut clock);
        }
        engine.prepare_out(&request(UsbDirection::Out, 1, 4, 0), &[]).unwrap();
        engine.complete();
        get_status(&mut engine);
        engine.complete();
        for _ in 0..11 {
            engine.tick(clock);
            clock += 1;
        }
        assert_eq!(engine.state, State::ManifestSync);
        assert!(engine.ready);
        if abort {
            engine.prepare_out(&request(UsbDirection::Out, 6, 0, 0), &[]).unwrap();
        } else {
            assert_eq!(get_status(&mut engine)[4], State::Manifest as u8);
        }
        // The worker may finish while this later control transfer awaits status.
        while engine.ready {
            engine.tick(clock);
            clock += 1;
        }
        assert_eq!(engine.state, State::ManifestWaitReset);
        engine.complete();
        assert_eq!(engine.state, State::ManifestWaitReset);
        assert!(application_valid(&engine.flash));
    }
}

#[test]
fn polling_an_existing_busy_job_never_erases_or_programs_twice() {
    let mut engine = Engine::new(MemoryFlash::new());
    let mut clock = 0;
    engine.prepare_out(&request(UsbDirection::Out, 1, 0, 512), &image(512)).unwrap();
    engine.complete();
    get_status(&mut engine);
    engine.complete();
    for _ in 0..11 {
        engine.tick(clock);
        clock += 1;
    }
    assert_eq!(engine.state, State::DownloadSync);
    assert!(engine.pending);
    assert_eq!(get_status(&mut engine)[4], State::DownloadBusy as u8);
    engine.complete();
    run_ready(&mut engine, &mut clock);
    let erases = engine.flash.operations.iter().filter(|op| matches!(op, Operation::Erase(_))).count();
    let programs = engine.flash.operations.iter().filter(|op| matches!(op, Operation::Program(_, _))).count();
    assert_eq!(erases, 1);
    assert_eq!(programs, (512 - 8) / 4);
    assert_eq!(get_status(&mut engine)[4], State::DownloadIdle as u8);
}
