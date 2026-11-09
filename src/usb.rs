//! Fixed, polling EP0 transport for the single DFU interface. No non-control endpoints.


use crate::dfu::{Dfu, Flash, TRANSFER_SIZE};
use usb_device::bus::{PollResult, UsbBus};
use usb_device::control::{Recipient, Request, RequestType};
use usb_device::endpoint::EndpointAddress;
use usb_device::{UsbDirection, UsbError};

const OUT: u8 = 0;
const IN: u8 = 0x80;
const PACKET: usize = 64;
// USB 2.0, full-speed only: no BOS, qualifier, remote wakeup or alternate settings.
// Optional identity strings are omitted to keep the complete image below 8KiB.
const DEVICE: &[u8] = &[18, 1, 0, 2, 0, 0, 0, 64, 0xed, 0xfe, 0x69, 0x69, 0, 2, 0, 0, 0, 1];
const DFU: &[u8] = &[9, 0x21, 3, 0, 0, 0, 2, 0x10, 1];
const CONFIG: &[u8] = &[
    9, 2, 27, 0, 1, 1, 0, 0x80, 50,
    9, 4, 0, 0, 0, 0xfe, 1, 2, 0,
    9, 0x21, 3, 0, 0, 0, 2, 0x10, 1,
];

#[derive(Clone, Copy, PartialEq, Eq)]
enum Phase {
    Idle,
    DataOut,
    DataIn,
    StatusOut,
    StatusIn,
    Stalled,
}

pub struct Device<B: UsbBus> {
    bus: B,
    buffer: [u8; TRANSFER_SIZE],
    static_in: Option<&'static [u8]>,
    request: Option<Request>,
    phase: Phase,
    length: usize,
    offset: usize,
    zlp: bool,
    in_flight: bool,
    address: u8,
    configured: bool,
    suspended: bool,
}

impl<B: UsbBus> Device<B> {
    /// Own and enable a bus whose fixed EP0 buffers are already initialized.
    pub fn new(mut bus: B) -> Self {
        bus.enable();
        Self {
            bus,
            buffer: [0; TRANSFER_SIZE],
            static_in: None,
            request: None,
            phase: Phase::Idle,
            length: 0,
            offset: 0,
            zlp: false,
            in_flight: false,
            address: 0,
            configured: false,
            suspended: false,
        }
    }

    pub fn poll<F: Flash>(&mut self, dfu: &mut Dfu<F>) {
        let event = self.bus.poll();
        if self.suspended {
            if matches!(event, PollResult::None | PollResult::Suspend) {
                return;
            }
            self.bus.resume();
            self.suspended = false;
        }
        match event {
            PollResult::Reset => {
                self.bus.reset();
                self.request = None;
                self.phase = Phase::Idle;
                self.in_flight = false;
                self.static_in = None;
                self.address = 0;
                self.configured = false;
                dfu.reset_bus();
            }
            PollResult::Suspend => {
                self.bus.suspend();
                self.suspended = true;
                return;
            }
            PollResult::Data { ep_setup, ep_out, ep_in_complete } => {
                if ep_setup & 1 != 0 {
                    // Only a queued STATUS-IN can commit on this ACK. A DATA-IN ACK
                    // next to SETUP must never commit, or advance, the old request.
                    if ep_in_complete & 1 != 0 && self.phase == Phase::StatusIn && self.in_flight {
                        self.complete(dfu);
                    }
                    self.setup(dfu, ep_out & 1 != 0);
                } else {
                    if ep_in_complete & 1 != 0 && self.in_flight {
                        self.in_flight = false;
                        if self.phase == Phase::StatusIn {
                            self.complete(dfu);
                        } else if self.phase == Phase::DataIn && self.offset == self.length && !self.zlp {
                            self.phase = Phase::StatusOut;
                        }
                    }
                    if ep_out & 1 != 0 {
                        self.out(dfu);
                    }
                }
            }
            PollResult::None | PollResult::Resume => {}
        }
        self.transmit(dfu);
    }

    fn setup<F: Flash>(&mut self, dfu: &mut Dfu<F>, old_out: bool) {
        // HT32 has separate SETUP and OUT SRAM, but read() prioritizes SETUP.
        // Save SETUP first, then consume any preceding status OUT before notifying
        // DFU about the new request (which cancels its unacknowledged action).
        let mut bytes = [0; 8];
        match self.bus.read(OUT.into(), &mut bytes) {
            Ok(8) => {}
            Err(UsbError::WouldBlock) => return,
            _ => {
                self.error(dfu);
                return;
            }
        }
        if old_out {
            if matches!(self.phase, Phase::DataIn | Phase::StatusOut) {
                self.out(dfu);
            } else {
                // A superseded DATA-OUT is not a completed status stage.
                let _ = self.bus.read(OUT.into(), &mut self.buffer[..PACKET]);
            }
        }
        self.phase = Phase::Idle;
        self.in_flight = false;
        self.static_in = None;
        let req = parse(bytes);
        self.request = Some(req);
        self.bus.set_stalled(OUT.into(), false);
        self.bus.set_stalled(IN.into(), false);
        dfu.setup(&req);
        if req.direction == UsbDirection::In {
            let length = if req.request_type == RequestType::Standard {
                self.standard_in(&req)
            } else {
                dfu.request_in(&req, &mut self.buffer).ok()
            };
            match length {
                Some(length) if length <= TRANSFER_SIZE => {
                    self.length = core::cmp::min(length, req.length as usize);
                    self.offset = 0;
                    self.zlp = self.length == 0 || (self.length < req.length as usize && self.length % PACKET == 0);
                    self.phase = Phase::DataIn;
                }
                _ => self.stall(),
            }
        } else if req.length == 0 {
            self.length = 0;
            self.accept_out(dfu, &req);
        } else if req.request_type == RequestType::Class && req.length as usize <= TRANSFER_SIZE {
            self.length = req.length as usize;
            self.offset = 0;
            self.phase = Phase::DataOut;
        } else {
            self.error(dfu);
        }
    }

    fn out<F: Flash>(&mut self, dfu: &mut Dfu<F>) {
        if self.phase == Phase::DataOut {
            let remaining = self.length - self.offset;
            match self.bus.read(OUT.into(), &mut self.buffer[self.offset..self.length]) {
                Ok(count) if count <= PACKET && count <= remaining => {
                    self.offset += count;
                    if self.offset == self.length {
                        let req = self.request.unwrap();
                        self.accept_out(dfu, &req);
                    } else if count < PACKET {
                        self.error(dfu);
                    }
                }
                Err(UsbError::WouldBlock) => {}
                _ => self.error(dfu),
            }
        } else {
            // A received zero-length status OUT proves the final DATA-IN was
            // acknowledged, even if its IN-complete event is coalesced or late.
            let final_in = self.phase == Phase::StatusOut
                || (self.phase == Phase::DataIn && self.in_flight && self.offset == self.length && !self.zlp);
            match self.bus.read(OUT.into(), &mut self.buffer[..PACKET]) {
                Ok(0) if final_in => self.complete(dfu),
                Err(UsbError::WouldBlock) => {}
                _ => self.error(dfu),
            }
        }
    }

    fn accept_out<F: Flash>(&mut self, dfu: &mut Dfu<F>, req: &Request) {
        let accepted = if req.request_type == RequestType::Standard {
            self.standard_out(req)
        } else {
            dfu.request_out(req, &self.buffer[..self.length]).is_ok()
        };
        if accepted {
            self.phase = Phase::StatusIn;
        } else {
            self.stall();
        }
    }

    fn transmit<F: Flash>(&mut self, dfu: &mut Dfu<F>) {
        if self.in_flight || !matches!(self.phase, Phase::DataIn | Phase::StatusIn) {
            return;
        }
        if self.phase == Phase::DataIn && self.offset == self.length && !self.zlp {
            self.phase = Phase::StatusOut;
            return;
        }
        let count = if self.phase == Phase::StatusIn { 0 } else { core::cmp::min(PACKET, self.length - self.offset) };
        let data = self.static_in.unwrap_or(&self.buffer);
        let packet = if count == 0 { &[][..] } else { &data[self.offset..self.offset + count] };
        match self.bus.write(IN.into(), packet) {
            Ok(written) if written == count => {
                self.offset += count;
                if count == 0 { self.zlp = false; }
                self.in_flight = true;
            }
            Err(UsbError::WouldBlock) => {} // Retry without replaying class callbacks.
            _ => self.error(dfu),
        }
    }

    fn complete<F: Flash>(&mut self, dfu: &mut Dfu<F>) {
        self.phase = Phase::Idle;
        self.in_flight = false;
        self.static_in = None;
        if let Some(req) = self.request.take() {
            if req.request_type == RequestType::Standard {
                match req.request {
                    Request::SET_ADDRESS => {
                        self.address = req.value as u8;
                        self.bus.set_device_address(self.address);
                    }
                    Request::SET_CONFIGURATION => {
                        self.configured = req.value != 0;
                        dfu.set_configured(self.configured);
                        return;
                    }
                    Request::SET_FEATURE | Request::CLEAR_FEATURE => {
                        self.bus.set_stalled(EndpointAddress::from(req.index as u8), req.request == Request::SET_FEATURE);
                    }
                    _ => {}
                }
            }
            dfu.status_complete(&req);
        }
    }

    fn standard_in(&mut self, req: &Request) -> Option<usize> {
        if req.request == Request::GET_DESCRIPTOR {
            self.static_in = match (req.recipient, req.value, req.index) {
                (Recipient::Device, 0x0100, 0) => Some(DEVICE),
                (Recipient::Device, 0x0200, 0) => Some(CONFIG),
                (Recipient::Interface, 0x2100, 0) => Some(DFU),
                _ => None,
            };
            return self.static_in.map(|data| data.len());
        }
        if req.value != 0 { return None; }
        match (req.recipient, req.request, req.index, req.length) {
            (recipient, Request::GET_STATUS, index, 2) => {
                self.buffer[0] = match recipient {
                    Recipient::Device if index == 0 => 0,
                    Recipient::Interface if index == 0 && self.configured => 0,
                    Recipient::Endpoint if endpoint_zero(index) => u8::from(self.bus.is_stalled(EndpointAddress::from(index as u8))),
                    _ => return None,
                };
                self.buffer[1] = 0;
                Some(2)
            }
            (Recipient::Device, Request::GET_CONFIGURATION, 0, 1) => {
                self.buffer[0] = u8::from(self.configured);
                Some(1)
            }
            (Recipient::Interface, Request::GET_INTERFACE, 0, 1) if self.configured => {
                self.buffer[0] = 0;
                Some(1)
            }
            _ => None,
        }
    }

    fn standard_out(&self, req: &Request) -> bool {
        if req.length != 0 { return false; }
        match (req.recipient, req.request) {
            (Recipient::Device, Request::SET_ADDRESS) => req.index == 0 && req.value < 128 && !self.configured,
            (Recipient::Device, Request::SET_CONFIGURATION) => req.index == 0 && req.value <= 1 && self.address != 0,
            (Recipient::Interface, Request::SET_INTERFACE) => req.index == 0 && req.value == 0 && self.configured,
            // Only the default control pipe exists. Halt is applied after the
            // status handshake; the next SETUP clears its protocol stall.
            (Recipient::Endpoint, Request::SET_FEATURE | Request::CLEAR_FEATURE) => req.value == Request::FEATURE_ENDPOINT_HALT && endpoint_zero(req.index),
            _ => false,
        }
    }

    fn error<F: Flash>(&mut self, dfu: &mut Dfu<F>) {
        dfu.transfer_error();
        self.stall();
    }

    fn stall(&mut self) {
        self.request = None;
        self.phase = Phase::Stalled;
        self.in_flight = false;
        self.static_in = None;
        self.bus.set_stalled(OUT.into(), true);
        self.bus.set_stalled(IN.into(), true);
    }
}

fn endpoint_zero(index: u16) -> bool { index == 0 || index == 0x80 }

fn parse(bytes: [u8; 8]) -> Request {
    Request {
        direction: if bytes[0] & 0x80 == 0 { UsbDirection::Out } else { UsbDirection::In },
        request_type: match (bytes[0] >> 5) & 3 {
            0 => RequestType::Standard,
            1 => RequestType::Class,
            2 => RequestType::Vendor,
            _ => RequestType::Reserved,
        },
        recipient: match bytes[0] & 0x1f {
            0 => Recipient::Device,
            1 => Recipient::Interface,
            2 => Recipient::Endpoint,
            3 => Recipient::Other,
            _ => Recipient::Reserved,
        },
        request: bytes[1],
        value: u16::from_le_bytes([bytes[2], bytes[3]]),
        index: u16::from_le_bytes([bytes[4], bytes[5]]),
        length: u16::from_le_bytes([bytes[6], bytes[7]]),
    }
}

#[cfg(test)]
#[path = "usb_tests.rs"]
mod tests;
