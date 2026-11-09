use super::{vectors_valid, Flash, Status, APP_BASE, FLASH_END, PAGE_SIZE, TRANSFER_SIZE};

/// A block buffer plus a four-byte carry: block numbers never imply flash addresses.
/// The C reference is ht32-usb-dfu/source/usbdfu.c (write_block/manifest_image).
pub(crate) struct Stream {
    pub buffer: [u8; TRANSFER_SIZE],
    length: u32,
    vectors: [u8; 8],
    word: u32,
    word_bytes: u8,
    word_pending: bool,
    erased_page: Option<u32>,
    block_len: usize,
    cursor: usize,
    body_crc: u32,
    phase: Phase,
    verify_address: u32,
    verify_crc: u32,
    pub sp_attempted: bool,
}

#[derive(Clone, Copy)]
enum Phase {
    Validate,
    Tail,
    Verify,
    ResetVector,
    StackPointer,
    Done,
}

impl Stream {
    pub fn new() -> Self {
        Self {
            buffer: [0; TRANSFER_SIZE],
            length: 0,
            vectors: [0; 8],
            word: u32::MAX,
            word_bytes: 0,
            word_pending: false,
            erased_page: None,
            block_len: 0,
            cursor: 0,
            body_crc: u32::MAX,
            phase: Phase::Validate,
            verify_address: APP_BASE + 8,
            verify_crc: u32::MAX,
            sp_attempted: false,
        }
    }

    pub fn reset(&mut self) {
        // Preserve the just-received buffer when block zero is committed.
        self.length = 0;
        self.vectors = [0; 8];
        self.word = u32::MAX;
        self.word_bytes = 0;
        self.word_pending = false;
        self.erased_page = None;
        self.block_len = 0;
        self.cursor = 0;
        self.body_crc = u32::MAX;
        self.phase = Phase::Validate;
        self.verify_address = APP_BASE + 8;
        self.verify_crc = u32::MAX;
        self.sp_attempted = false;
    }

    pub fn block(&mut self, length: usize) {
        self.block_len = length;
        self.cursor = 0;
    }

    fn erase<F: Flash>(&mut self, flash: &mut F, page: u32) -> Result<(), Status> {
        if page < APP_BASE || page + PAGE_SIZE > FLASH_END || page % PAGE_SIZE != 0 {
            return Err(Status::ErrAddress);
        }
        flash.erase_page(page)?;
        for address in (page..page + PAGE_SIZE).step_by(4) {
            if flash.read_word(address) != u32::MAX {
                return Err(Status::ErrCheckErased);
            }
        }
        self.erased_page = Some(page);
        Ok(())
    }

    fn program<F: Flash>(flash: &mut F, address: u32, word: u32) -> Result<(), Status> {
        if address < APP_BASE || address > FLASH_END - 4 || address % 4 != 0 {
            return Err(Status::ErrAddress);
        }
        flash.program_word(address, word)?;
        if flash.read_word(address) != word {
            return Err(Status::ErrVerify);
        }
        Ok(())
    }

    /// Issues at most one flash command; even page erase and its first write are separate ticks.
    fn flush_word<F: Flash>(&mut self, flash: &mut F) -> Result<(), Status> {
        let address = APP_BASE + self.length - u32::from(self.word_bytes);
        let page = address & !(PAGE_SIZE - 1);
        if self.erased_page != Some(page) {
            self.erase(flash, page)?;
            return Ok(());
        }
        Self::program(flash, address, self.word)?;
        self.word = u32::MAX;
        self.word_bytes = 0;
        self.word_pending = false;
        Ok(())
    }

    pub fn step<F: Flash>(&mut self, flash: &mut F, manifest: bool) -> Result<bool, Status> {
        // Invalidate the previous image before consuming the first block, including vector-only
        // blocks. Neither vector is ever programmed during download.
        if self.erased_page.is_none() {
            self.erase(flash, APP_BASE)?;
            return Ok(false);
        }
        if self.word_pending {
            self.flush_word(flash)?;
            return Ok(false);
        }
        while self.cursor < self.block_len {
            if APP_BASE + self.length >= FLASH_END {
                return Err(Status::ErrAddress);
            }
            let byte = self.buffer[self.cursor];
            self.cursor += 1;
            if self.length < 8 {
                self.vectors[self.length as usize] = byte;
            } else {
                let shift = u32::from(self.word_bytes) * 8;
                self.word = (self.word & !(0xff << shift)) | (u32::from(byte) << shift);
                self.word_bytes += 1;
                self.body_crc = crc_byte(self.body_crc, byte);
            }
            self.length += 1;
            if self.word_bytes == 4 {
                self.word_pending = true;
                self.flush_word(flash)?;
                return Ok(false);
            }
        }
        if !manifest {
            return Ok(true);
        }
        match self.phase {
            Phase::Validate => {
                if self.length < 8 || !vectors_valid(self.sp(), self.reset_vector(), APP_BASE + self.length) {
                    return Err(Status::ErrFile);
                }
                self.phase = Phase::Tail;
            }
            Phase::Tail => {
                if self.word_bytes != 0 {
                    self.word_pending = true;
                    self.flush_word(flash)?;
                } else {
                    self.phase = Phase::Verify;
                }
            }
            Phase::Verify => {
                // Read back the entire body, not just the last word. Keep verification bounded
                // to 64 bytes per tick, so USB polling is not starved by a maximum-size image.
                let end = APP_BASE + self.length;
                let stop = core::cmp::min(self.verify_address + 64, end);
                while self.verify_address < stop {
                    let word = flash.read_word(self.verify_address);
                    for byte in word.to_le_bytes() {
                        if self.verify_address >= end {
                            break;
                        }
                        self.verify_crc = crc_byte(self.verify_crc, byte);
                        self.verify_address += 1;
                    }
                }
                if self.verify_address == end {
                    if self.verify_crc != self.body_crc {
                        return Err(Status::ErrVerify);
                    }
                    self.phase = Phase::ResetVector;
                }
            }
            Phase::ResetVector => {
                Self::program(flash, APP_BASE + 4, self.reset_vector())?;
                self.phase = Phase::StackPointer;
            }
            Phase::StackPointer => {
                self.sp_attempted = true;
                Self::program(flash, APP_BASE, self.sp())?;
                self.phase = Phase::Done;
                return Ok(true);
            }
            Phase::Done => return Ok(true),
        }
        Ok(false)
    }

    fn sp(&self) -> u32 {
        u32::from_le_bytes(self.vectors[..4].try_into().unwrap())
    }

    fn reset_vector(&self) -> u32 {
        u32::from_le_bytes(self.vectors[4..].try_into().unwrap())
    }
}

fn crc_byte(mut crc: u32, byte: u8) -> u32 {
    crc ^= u32::from(byte);
    for _ in 0..8 {
        crc = (crc >> 1) ^ (0xedb8_8320 & 0u32.wrapping_sub(crc & 1));
    }
    crc
}
