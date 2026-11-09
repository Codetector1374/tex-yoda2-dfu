//! Board-specific clock, flash and application handoff support.
//!
//! Clock/FMC sequencing and handoff are adapted from ht32-usb-dfu/main.c,
//! source/usbdfu.c and ChibiOS HT32F165x/hal_lld.c. The ChibiOS portions are
//! Copyright (C) 2006..2018 Giovanni Di Sirio, (C) 2020 Yaotian Feng,
//! licensed under Apache-2.0: https://www.apache.org/licenses/LICENSE-2.0.
//! Register encodings follow HT32F1653/1654 User Manual Rev. 1.01.

use bootloader::dfu::{Flash, Status, APP_BASE, FLASH_END};
use core::ptr::{read_volatile, write_volatile};
use cortex_m::{peripheral::DWT, Peripherals};
use ht32_pac::{CKCU, FMC, PWRCU, RSTCU, USB};

const CYCLES_PER_MS: u32 = 72_000;
const STARTUP_POLLS: u32 = 1_000_000;
const FLASH_TIMEOUT_CYCLES: u32 = 100 * CYCLES_PER_MS;
const PAGE_SIZE: u32 = 1024;
// Logical field values: OPM occupies bits 4:1 in the hardware register.
const FMC_IDLE: u8 = 0x6;
const FMC_COMMIT: u8 = 0xa;
const FMC_FINISHED: u8 = 0xe;
const FMC_PROGRAM: u8 = 4;
const FMC_ERASE: u8 = 8;
// ITADF, OBEF, IOCMF, OREF and PPEF; bit 0 is the completion flag.
const FMC_ERRORS: u32 = 0x0002_001e;
const FMC_CLEAR: u32 = FMC_ERRORS | 1;


fn wait_for(mut ready: impl FnMut() -> bool) -> Result<(), ()> {
    for _ in 0..STARTUP_POLLS {
        if ready() {
            return Ok(());
        }
        cortex_m::asm::nop();
    }
    Err(())
}

/// Establish 8MHz HSE -> 144MHz PLL -> 72MHz HCLK / 48MHz USB.
/// Each hardware wait is finite; callers must not instantiate flash or USB
/// after any failure. HSI remains enabled because FMC requires it.
pub fn initialize(core: &mut Peripherals) -> Result<(), ()> {
    // Return to a known source before touching the PLL, including debug entry
    // after a previously running application. Preserve other GCCR fields.
    CKCU.gccr().modify(|r| {
        r.set_ckmen(false);
        r.set_hsien(true);
    });
    wait_for(|| CKCU.gcsr().read().hsirdy())?;
    CKCU.gccr().modify(|r| r.set_sw(3));
    wait_for(|| CKCU.ckst().read().ckswst() == 3)?;
    CKCU.gccr().modify(|r| {
        r.set_pllen(false);
        r.set_hseen(true);
    });
    wait_for(|| !CKCU.gcsr().read().pllrdy())?;
    wait_for(|| CKCU.gcsr().read().hserdy())?;

    // HSE source, USB /3, PLL x18 with output /1; bypass disabled.
    CKCU.gcfgr().modify(|r| {
        r.set_pllsrc(false);
        r.set_usbpre(2);
    });
    CKCU.pllcr().write(|r| r.set_pllbps(false));
    CKCU.pllcfgr().write(|r| {
        r.set_pfbd(18);
        r.set_potd(0);
    });
    CKCU.gccr().modify(|r| r.set_pllen(true));
    wait_for(|| CKCU.gcsr().read().pllrdy())?;

    // Two flash wait states at 72MHz. Force DCODE reads non-cacheable for
    // program/erase verification; preserve the instruction prefetch settings.
    FMC.cfcr().modify(|r| {
        r.set_wait(3);
        r.set_dcdb(true);
    });
    CKCU.ahbcfgr().modify(|r| r.set_ahbpre(1));
    CKCU.apbpcsr0().write_value(Default::default());
    CKCU.apbpcsr1().write_value(Default::default());
    CKCU.gccr().modify(|r| r.set_sw(1));
    wait_for(|| CKCU.ckst().read().ckswst() & 2 == 0)?;
    cortex_m::asm::dsb();
    cortex_m::asm::isb();

    // USB requires backup-domain access, even though no RTC is used.
    CKCU.lpcr().modify(|r| {
        r.set_usbsleep(false);
        r.set_bkiso(true);
    });
    // The SVD names the manual's BKPREN (backup-register access) RTCEN.
    CKCU.apbccr1().modify(|r| r.set_rtcen(true));
    wait_for(|| PWRCU.baktest().read().baktest() == 0x27)?;
    FMC.oier().write_value(Default::default());

    // CYCCNT is implemented by this Cortex-M3. Polling it avoids interrupts
    // and accounts for time spent inside synchronous bounded FMC commands.
    core.DCB.enable_trace();
    core.DWT.set_cycle_count(0);
    core.DWT.enable_cycle_counter();
    let before = DWT::cycle_count();
    cortex_m::asm::delay(16);
    if DWT::cycle_count() == before {
        return Err(());
    }
    Ok(())
}

pub struct Usb;

// SAFETY: the generated HT32F1654 USB handle has the register layout expected
// by ht32-usbd; its dedicated 1KiB endpoint SRAM is not otherwise accessed.
// Initialization above establishes the clock/power prerequisites, and this
// singleton is only instantiated once by main, with all NVIC lines disabled.
unsafe impl ht32_usbd::UsbPeripheral for Usb {
    const REGISTERS: *const () = USB.as_ptr().cast();
    const DP_PULL_UP_FEATURE: bool = true;
    const EP_MEMORY: *const () = 0x400a_a000 as *const ();
    const EP_MEMORY_SIZE: usize = 1024;

    fn enable(&self) {
        CKCU.ahbccr().modify(|r| r.set_usben(true));
        RSTCU.ahbprstr().write(|r| r.set_usbrst(true));
        cortex_m::asm::dsb();
        RSTCU.ahbprstr().write_value(Default::default());
        USB.csr().write_value(Default::default()); // Disconnect until enabled.
    }

    fn dp_pull_up(&mut self) {
        // The bus driver controls the internal D+ pull-up itself.
    }
}

/// Disconnect and quiesce USB before resetting or transferring execution.
pub fn stop_usb() {
    USB.ier().write_value(Default::default());
    USB.csr().write_value(Default::default());
    RSTCU.ahbprstr().write(|r| r.set_usbrst(true));
    cortex_m::asm::dsb();
    RSTCU.ahbprstr().write_value(Default::default());
    CKCU.ahbccr().modify(|r| r.set_usben(false));
    cortex_m::asm::dsb();
}

/// Wrapping CYCCNT accumulation. The maximum gap between samples is a 100ms
/// FMC command, well below CYCCNT's approximately 59-second wrap period.
pub struct Millis {
    previous: u32,
    remainder: u32,
    millis: u32,
}

impl Millis {
    pub fn new() -> Self {
        Self { previous: DWT::cycle_count(), remainder: 0, millis: 0 }
    }

    pub fn now(&mut self) -> u32 {
        let cycles = DWT::cycle_count();
        let delta = cycles.wrapping_sub(self.previous);
        self.previous = cycles;
        let elapsed = delta + self.remainder;
        self.millis = self.millis.wrapping_add(elapsed / CYCLES_PER_MS);
        self.remainder = elapsed % CYCLES_PER_MS;
        self.millis
    }
}

/// Sole FMC command issuer. A timeout/busy anomaly permanently poisons the
/// controller until reset: a potentially active command is never overlapped.
pub struct HardwareFlash {
    faulted: bool,
}

impl HardwareFlash {
    pub fn new() -> Self {
        Self { faulted: false }
    }

    fn command(&mut self, address: u32, command: u8, word: u32) -> Result<(), Status> {
        let erase = command == FMC_ERASE;
        let length = if erase { PAGE_SIZE } else { 4 };
        if address < APP_BASE || address > FLASH_END - length
            || address & (length - 1) != 0
        {
            return Err(Status::ErrAddress);
        }
        if self.faulted {
            return Err(Status::ErrUnknown);
        }
        let clocks = CKCU.gcsr().read();
        let mode = FMC.opcr().read().opm();
        if !clocks.hsirdy() || !clocks.hserdy() || !clocks.pllrdy()
            || CKCU.ckst().read().ckswst() & 2 != 0
            || (mode != FMC_IDLE && mode != FMC_FINISHED)
        {
            self.faulted = true;
            return Err(Status::ErrUnknown);
        }
        // The range/alignment checks above already cover these FMC accesses.
        if !erase && unsafe { read_volatile(address as *const u32) } & word != word {
            return Err(Status::ErrProg);
        }

        FMC.oisr().write_value(ht32_pac::fmc::regs::Oisr(FMC_CLEAR));
        FMC.tadr().write(|r| r.set_tadb(address));
        FMC.wrdr().write(|r| r.set_wrdb(word));
        FMC.ocmr().write(|r| r.set_cmd(command));
        cortex_m::asm::dsb();
        let start = DWT::cycle_count();
        FMC.opcr().write(|r| r.set_opm(FMC_COMMIT));
        while FMC.opcr().read().opm() != FMC_FINISHED {
            if DWT::cycle_count().wrapping_sub(start) >= FLASH_TIMEOUT_CYCLES {
                self.faulted = true;
                return Err(if erase { Status::ErrErase } else { Status::ErrWrite });
            }
        }
        cortex_m::asm::dsb();
        let errors = FMC.oisr().read().0 & FMC_ERRORS;
        FMC.oisr().write_value(ht32_pac::fmc::regs::Oisr(FMC_CLEAR));
        FMC.ocmr().write(|r| r.set_cmd(0)); // IDLE command, not OPCR operation.
        if errors != 0 {
            return Err(if erase { Status::ErrErase } else { Status::ErrProg });
        }
        if erase {
            for offset in (0..PAGE_SIZE).step_by(4) {
                if unsafe { read_volatile((address + offset) as *const u32) } != u32::MAX {
                    return Err(Status::ErrCheckErased);
                }
            }
        } else if unsafe { read_volatile(address as *const u32) } != word {
            return Err(Status::ErrVerify);
        }
        Ok(())
    }
}

impl Flash for HardwareFlash {
    fn read_word(&self, address: u32) -> u32 {
        // Invalid trait inputs must never dereference peripheral/option space.
        if address < APP_BASE || address > FLASH_END - 4 || address & 3 != 0 {
            return u32::MAX;
        }
        unsafe { read_volatile(address as *const u32) }
    }

    fn erase_page(&mut self, address: u32) -> Result<(), Status> {
        self.command(address, FMC_ERASE, 0)
    }

    fn program_word(&mut self, address: u32, word: u32) -> Result<(), Status> {
        self.command(address, FMC_PROGRAM, word)
    }
}

/// Caller must have validated the application's initial SP and Thumb reset
/// vector, and must guarantee no FMC command is outstanding.
pub unsafe fn jump_to_application() -> ! {
    let stack = read_volatile(APP_BASE as *const u32);
    let entry = read_volatile((APP_BASE + 4) as *const u32);
    cortex_m::interrupt::disable();
    stop_usb();
    // Architectural core registers, not a second vendor register mapping.
    write_volatile(0xe000_e010 as *mut u32, 0); // SysTick CTRL
    write_volatile(0xe000_e018 as *mut u32, 0); // SysTick VAL
    for bank in 0u32..8 {
        write_volatile((0xe000_e180 + bank * 4) as *mut u32, u32::MAX); // ICER
        write_volatile((0xe000_e280 + bank * 4) as *mut u32, u32::MAX); // ICPR
    }
    write_volatile(0xe000_ed04 as *mut u32, (1 << 25) | (1 << 27)); // ICSR
    write_volatile(0xe000_ed08 as *mut u32, APP_BASE); // VTOR
    cortex_m::asm::dsb();
    cortex_m::asm::isb();
    // No Rust, stack access or function epilogue after changing MSP/CONTROL.
    core::arch::asm!(
        "movs r2, #0",
        "msr basepri, r2",
        "msr faultmask, r2",
        "msr control, r2",
        "isb",
        "msr msp, r0",
        "cpsie i",
        "bx r1",
        in("r0") stack,
        in("r1") entry,
        options(noreturn)
    )
}

pub fn halt() -> ! {
    cortex_m::interrupt::disable();
    loop {
        cortex_m::asm::nop();
    }
}
