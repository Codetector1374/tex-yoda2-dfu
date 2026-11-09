//! USB DFU 1.1 for the HT32F1654, following the wire protocol and conservative flash
//! layout in `ht32-usb-dfu/source/usbdfu.c`. This is a Rust implementation, not a HAL port.
//!
//! Construct `Dfu::new(flash)` and poll USB **before** each
//! `dfu.tick(monotonic_milliseconds)`. A direct EP0 driver calls `setup`, `request_in`
//! or `request_out`, then `status_complete` only after the status stage is acknowledged.
//! Reset the MCU when `dfu.should_reset()` is true.
//! Flash methods must return after a bounded controller operation and must not allow a
//! new command to overlap a timed-out one. Only acknowledged status stages can release
//! flash work to `tick`; SETUP or bus reset cancels unacknowledged actions.

mod stream;

use stream::Stream;
use usb_device::control::{Recipient, Request, RequestType};
use usb_device::UsbDirection;

pub const APP_BASE: u32 = 0x2000;
pub const FLASH_END: u32 = 0xfc00;
pub const PAGE_SIZE: u32 = 1024;
pub const TRANSFER_SIZE: usize = 512;
const POLL_MS: u32 = 10;
const FUNCTIONAL_DESCRIPTOR: [u8; 9] = [9, 0x21, 0x03, 0, 0, 0, 2, 0x10, 1];

/// DFU 1.1 bStatus values, including hardware-visible errors.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum Status {
    Ok = 0,
    ErrTarget = 1,
    ErrFile = 2,
    ErrWrite = 3,
    ErrErase = 4,
    ErrCheckErased = 5,
    ErrProg = 6,
    ErrVerify = 7,
    ErrAddress = 8,
    ErrNotDone = 9,
    ErrFirmware = 10,
    ErrVendor = 11,
    ErrUsbReset = 12,
    ErrPowerOnReset = 13,
    ErrUnknown = 14,
    ErrStalledPacket = 15,
}

/// Word-aligned access to real flash. Implementations must protect bootloader/reserved pages.
/// `program_word` must not implicitly erase: erase and program are distinct bounded commands.
pub trait Flash {
    fn read_word(&self, address: u32) -> u32;
    fn erase_page(&mut self, address: u32) -> Result<(), Status>;
    fn program_word(&mut self, address: u32, word: u32) -> Result<(), Status>;
}

fn vectors_valid(sp: u32, reset: u32, image_end: u32) -> bool {
    sp > 0x2000_0000
        && sp <= 0x2000_4000
        && sp & 7 == 0
        && reset & 1 != 0
        && reset & !1 >= APP_BASE + 8
        && reset & !1 < image_end
}

/// Check stack alignment/SRAM bounds and a Thumb reset entry within application flash.
/// An interrupted download is invalid because its initial SP remains erased until commit.
pub fn application_valid(flash: &impl Flash) -> bool {
    vectors_valid(flash.read_word(APP_BASE), flash.read_word(APP_BASE + 4), FLASH_END)
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum State {
    Idle = 2,
    DownloadSync = 3,
    DownloadBusy = 4,
    DownloadIdle = 5,
    ManifestSync = 6,
    Manifest = 7,
    ManifestWaitReset = 8,
    UploadIdle = 9,
    Error = 10,
}

#[derive(Clone, Copy)]
enum Action {
    None,
    Download { length: usize, first: bool },
    Upload { length: u32, requested: u32, first: bool },
    Manifest,
    PollWrite,
    FinishWrite,
    PollManifest,
    Idle,
}

/// Single-interface DFU-mode class (FE/01/02), with no allocator, HAL, or interrupt ownership.
pub struct Dfu<F: Flash> {
    configured: bool,
    reject_current: bool,
    engine: Engine<F>,
}

impl<F: Flash> Dfu<F> {
    pub fn new(flash: F) -> Self {
        Self {
            configured: false,
            reject_current: false,
            engine: Engine::new(flash),
        }
    }

    /// Advance timing and at most one erase/program command. Always poll USB first.
    pub fn tick(&mut self, now_ms: u32) {
        self.engine.tick(now_ms);
    }

    pub fn should_reset(&self) -> bool {
        self.engine.reset_requested && !self.engine.cleanup_required
    }

    /// Sticky latch for a boot timeout; enumeration alone does not count as DFU activity.
    /// Once a DFU class request is recognized, a bus reset cannot re-enable auto boot.
    pub fn has_activity(&self) -> bool {
        self.engine.activity
    }

    pub fn state(&self) -> State {
        self.engine.state
    }

    pub fn status(&self) -> Status {
        self.engine.status
    }

    /// Cancel the previous unacknowledged action before dispatching every new SETUP.
    pub fn setup(&mut self, req: &Request) {
        self.engine.action = Action::None;
        self.reject_current = false;
        if self.relevant(req) {
            self.engine.activity = true;
            if self.engine.check_window().is_err() {
                // Judge OUT at SETUP, not after its potentially long data stage.
                self.reject_current = true;
                self.engine.reject(Status::ErrStalledPacket);
            }
        }
    }

    /// Drop the configuration and all uncommitted work after a USB bus reset.
    pub fn reset_bus(&mut self) {
        self.configured = false;
        self.reject_current = false;
        self.engine.bus_reset();
    }

    /// Apply SET_CONFIGURATION only after its status stage has been acknowledged.
    pub fn set_configured(&mut self, configured: bool) {
        self.configured = configured;
        if self.engine.state != State::ManifestWaitReset {
            self.engine.bus_reset();
        }
    }

    /// Prepare a complete OUT data stage; no flash work is committed until status ACK.
    pub fn request_out(&mut self, req: &Request, data: &[u8]) -> Result<(), Status> {
        if !self.relevant(req) {
            return Err(Status::ErrStalledPacket);
        }
        let result = if self.reject_current || !self.valid_interface(req) {
            Err(Status::ErrStalledPacket)
        } else {
            self.engine.prepare_out(req, data)
        };
        if let Err(status) = result {
            self.engine.reject(status);
        }
        result
    }

    /// Prepare IN data, including the interface's standard DFU functional descriptor.
    /// Class requests require room for wLength (at most 512 bytes); descriptors truncate.
    pub fn request_in(&mut self, req: &Request, buffer: &mut [u8]) -> Result<usize, Status> {
        if self.functional_descriptor(req) {
            let length = core::cmp::min(req.length as usize, FUNCTIONAL_DESCRIPTOR.len());
            if buffer.len() < length {
                self.transfer_error();
                return Err(Status::ErrStalledPacket);
            }
            buffer[..length].copy_from_slice(&FUNCTIONAL_DESCRIPTOR[..length]);
            return Ok(length);
        }
        if !self.relevant(req) {
            return Err(Status::ErrStalledPacket);
        }
        let result = if self.reject_current || !self.valid_interface(req) {
            Err(Status::ErrStalledPacket)
        } else {
            self.engine.prepare_in(req, buffer)
        };
        if let Err(status) = result {
            self.engine.reject(status);
        }
        result
    }

    /// Commit the staged action only after the actual USB status-stage ACK.
    /// SET_CONFIGURATION is applied here; callers may instead use `set_configured`.
    pub fn status_complete(&mut self, req: &Request) {
        if req.request_type == RequestType::Standard
            && req.recipient == Recipient::Device
            && req.request == Request::SET_CONFIGURATION
        {
            self.set_configured(req.value != 0);
        } else {
            self.engine.complete();
        }
    }

    /// Cancel an incomplete or malformed USB control transfer and latch its DFU error.
    pub fn transfer_error(&mut self) {
        self.engine.reject(Status::ErrStalledPacket);
    }

    fn functional_descriptor(&self, req: &Request) -> bool {
        req.direction == UsbDirection::In
            && req.request_type == RequestType::Standard
            && req.recipient == Recipient::Interface
            && req.request == Request::GET_DESCRIPTOR
            && req.value == 0x2100
            && req.index == 0
    }

    fn relevant(&self, req: &Request) -> bool {
        req.request_type == RequestType::Class
    }

    fn valid_interface(&self, req: &Request) -> bool {
        self.configured
            && req.recipient == Recipient::Interface
            && req.index == 0
    }
}

struct Engine<F: Flash> {
    flash: F,
    stream: Stream,
    state: State,
    status: Status,
    action: Action,
    downloaded: u32,
    download_block: u16,
    upload_address: u32,
    upload_block: u16,
    pending: bool,
    manifest: bool,
    ready: bool,
    poll_arm: bool,
    poll_started: Option<u32>,
    activity: bool,
    reset_requested: bool,
    cleanup_required: bool,
    cleanup_attempted: bool,
}

impl<F: Flash> Engine<F> {
    fn new(flash: F) -> Self {
        Self {
            flash,
            stream: Stream::new(),
            state: State::Idle,
            status: Status::Ok,
            action: Action::None,
            downloaded: 0,
            download_block: 0,
            upload_address: APP_BASE,
            upload_block: 0,
            pending: false,
            manifest: false,
            ready: false,
            poll_arm: false,
            poll_started: None,
            activity: false,
            reset_requested: false,
            cleanup_required: false,
            cleanup_attempted: false,
        }
    }

    fn cancel(&mut self) {
        self.action = Action::None;
        self.pending = false;
        self.manifest = false;
        self.ready = false;
        self.poll_arm = false;
        self.poll_started = None;
        self.downloaded = 0;
        self.download_block = 0;
        self.upload_address = APP_BASE;
        self.upload_block = 0;
        self.stream.reset();
    }

    fn bus_reset(&mut self) {
        self.action = Action::None;
        if self.state == State::ManifestWaitReset {
            self.reset_requested = true;
        } else {
            self.cancel();
            if self.cleanup_required {
                self.state = State::Error;
            } else {
                self.state = State::Idle;
                self.status = Status::Ok;
            }
        }
    }

    fn reject(&mut self, status: Status) {
        self.action = Action::None;
        if self.state == State::ManifestWaitReset || self.state == State::Error {
            return;
        }
        self.cancel();
        self.status = status;
        self.state = State::Error;
    }

    fn check_window(&self) -> Result<(), Status> {
        if self.poll_arm || self.poll_started.is_some() {
            Err(Status::ErrStalledPacket)
        } else {
            Ok(())
        }
    }

    fn prepare_out(&mut self, req: &Request, data: &[u8]) -> Result<(), Status> {
        self.check_window()?;
        if req.direction != UsbDirection::Out || data.len() != req.length as usize {
            return Err(Status::ErrStalledPacket);
        }
        match req.request {
            1 if !self.pending && !self.cleanup_required
                && matches!(self.state, State::Idle | State::DownloadIdle)
                && data.len() <= TRANSFER_SIZE =>
            {
                let first = self.state == State::Idle;
                if req.value != if first { 0 } else { self.download_block } {
                    return Err(Status::ErrStalledPacket);
                }
                if data.is_empty() {
                    if first {
                        return Err(Status::ErrNotDone);
                    }
                    self.action = Action::Manifest;
                } else {
                    let previous = if first { 0 } else { self.downloaded };
                    if data.len() as u32 > FLASH_END - APP_BASE - previous {
                        return Err(Status::ErrAddress);
                    }
                    self.stream.buffer[..data.len()].copy_from_slice(data);
                    self.action = Action::Download { length: data.len(), first };
                }
            }
            4 if req.value == 0 && req.length == 0 && self.state == State::Error
                && !self.cleanup_required => self.action = Action::Idle,
            6 if req.value == 0 && req.length == 0
                && matches!(self.state, State::Idle | State::DownloadSync | State::DownloadIdle
                    | State::ManifestSync | State::UploadIdle) => self.action = Action::Idle,
            _ => return Err(Status::ErrStalledPacket),
        }
        Ok(())
    }

    fn prepare_in(&mut self, req: &Request, buffer: &mut [u8]) -> Result<usize, Status> {
        self.check_window()?;
        if req.direction != UsbDirection::In || buffer.len() < req.length as usize {
            return Err(Status::ErrStalledPacket);
        }
        match req.request {
            2 if req.length != 0 && req.length as usize <= TRANSFER_SIZE && !self.pending
                && matches!(self.state, State::Idle | State::UploadIdle) =>
            {
                let first = self.state == State::Idle;
                if req.value != if first { 0 } else { self.upload_block } {
                    return Err(Status::ErrStalledPacket);
                }
                let address = if first { APP_BASE } else { self.upload_address };
                let length = core::cmp::min(req.length as u32, FLASH_END - address);
                // Uploads are also a sequential byte stream; unaligned short requests are valid.
                let mut offset = 0;
                while offset < length {
                    let current = address + offset;
                    let word = self.flash.read_word(current & !3).to_le_bytes();
                    let start = (current & 3) as usize;
                    let count = core::cmp::min(4 - start, (length - offset) as usize);
                    buffer[offset as usize..offset as usize + count]
                        .copy_from_slice(&word[start..start + count]);
                    offset += count as u32;
                }
                self.action = Action::Upload { length, requested: req.length as u32, first };
                Ok(length as usize)
            }
            3 if req.value == 0 && req.length == 6 => {
                let mut reply_state = self.state;
                if self.state == State::DownloadSync {
                    if self.pending {
                        reply_state = State::DownloadBusy;
                        self.action = Action::PollWrite;
                    } else {
                        reply_state = State::DownloadIdle;
                        self.action = Action::FinishWrite;
                    }
                } else if self.state == State::ManifestSync {
                    reply_state = State::Manifest;
                    self.action = Action::PollManifest;
                }
                buffer[..6].copy_from_slice(&[
                    self.status as u8,
                    if matches!(reply_state, State::DownloadBusy | State::Manifest) { 10 } else { 0 },
                    0, 0, reply_state as u8, 0,
                ]);
                Ok(6)
            }
            5 if req.value == 0 && req.length == 1 => {
                buffer[0] = self.state as u8;
                Ok(1)
            }
            _ => Err(Status::ErrStalledPacket),
        }
    }

    fn complete(&mut self) {
        let action = core::mem::replace(&mut self.action, Action::None);
        match action {
            Action::None => {}
            Action::Download { length, first } => {
                if first {
                    self.cancel();
                }
                self.stream.block(length);
                self.downloaded += length as u32;
                self.download_block += 1;
                self.pending = true;
                self.state = State::DownloadSync;
            }
            Action::Upload { length, requested, first } => {
                if first {
                    self.upload_address = APP_BASE;
                    self.upload_block = 0;
                }
                self.upload_address += length;
                self.upload_block += 1;
                self.state = if length < requested { State::Idle } else { State::UploadIdle };
            }
            Action::Manifest => {
                self.manifest = true;
                self.state = State::ManifestSync;
            }
            Action::PollWrite => {
                self.ready = true;
                self.poll_arm = true;
                self.state = State::DownloadBusy;
            }
            Action::FinishWrite => self.state = State::DownloadIdle,
            Action::PollManifest => {
                if self.state != State::ManifestWaitReset {
                    self.ready = true;
                    self.state = State::Manifest;
                }
                self.poll_arm = true;
            }
            Action::Idle => {
                if self.state == State::ManifestWaitReset {
                    return;
                }
                self.cancel();
                self.status = Status::Ok;
                self.state = State::Idle;
            }
        }
    }

    fn tick(&mut self, now_ms: u32) {
        // A failed final-SP command might leave plausible vectors. Invalidate them on a
        // separate tick, retaining this obligation across USB resets and CLRSTATUS.
        // A failed invalidation latches an error rather than issuing an unbounded retry loop.
        if self.cleanup_required {
            if !self.cleanup_attempted {
                self.cleanup_attempted = true;
                let erased = self.flash.erase_page(APP_BASE).is_ok()
                    && (APP_BASE..APP_BASE + PAGE_SIZE).step_by(4)
                        .all(|address| self.flash.read_word(address) == u32::MAX);
                if erased {
                    self.cleanup_required = false;
                }
            }
            return;
        }
        if self.poll_arm {
            self.poll_arm = false;
            self.poll_started = Some(now_ms);
        }
        if self.poll_started.is_some_and(|start| now_ms.wrapping_sub(start) >= POLL_MS) {
            self.poll_started = None;
            self.state = match self.state {
                State::DownloadBusy => State::DownloadSync,
                State::Manifest => State::ManifestSync,
                other => other,
            };
        }
        if !self.ready {
            return;
        }
        match self.stream.step(&mut self.flash, self.manifest) {
            Ok(false) => {}
            Ok(true) => {
                self.ready = false;
                self.pending = false;
                if self.manifest {
                    self.manifest = false;
                    self.state = State::ManifestWaitReset;
                }
            }
            Err(status) => {
                self.cleanup_required = self.stream.sp_attempted;
                self.cleanup_attempted = false;
                let poll_arm = self.poll_arm;
                let poll_started = self.poll_started;
                self.cancel();
                self.poll_arm = poll_arm;
                self.poll_started = poll_started;
                self.status = status;
                self.state = State::Error;
            }
        }
    }
}

#[cfg(test)]
mod tests;
