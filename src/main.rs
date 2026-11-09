//! HT32F1654 / TEX Yoda II polling USB DFU bootloader.
//!
//! Every reset opens a 1500ms USB boot window. A recognized DFU request latches
//! bootloader entry; without activity a valid application at 0x2000 is started.
//! Invalid applications always remain in DFU. No unverified keyboard matrix or
//! software-entry magic is used: start the host DFU tool before resetting.
//! Hardware assumptions: 8MHz HSE, internal USB D+ pull-up, 63KiB main flash,
//! 16KiB SRAM. Clock startup failure stops before USB or flash programming.

#![no_std]
#![no_main]

mod hardware;
mod usb_hw;

// Keep memory helpers small and prevent LLVM from replacing their loops with
// calls to generic compiler-builtins implementations.
core::arch::global_asm!(include_str!("memcpy.s"));
core::arch::global_asm!(include_str!("memclr.s"));

use bootloader::dfu::{application_valid, Dfu};
use cortex_m_rt::entry;
use hardware::{HardwareFlash, Millis};
use bootloader::usb::Device;
use usb_hw::Ht32Ep0;

const BOOT_WINDOW_MS: u32 = 1500;

extern "C" {
    fn DefaultHandler();
}

// HT32F1654 IRQ0..IRQ68 (EBI). All NVIC lines remain disabled; the USB
// peripheral's event flags are consumed exclusively by the polling driver.
#[used]
#[no_mangle]
#[link_section = ".vector_table.interrupts"]
pub static __INTERRUPTS: [unsafe extern "C" fn(); 69] = [DefaultHandler; 69];

#[entry]
fn main() -> ! {
    cortex_m::interrupt::disable();
    let mut core = cortex_m::Peripherals::take().unwrap();
    if hardware::initialize(&mut core).is_err() {
        hardware::halt();
    }

    let flash = HardwareFlash::new();
    let valid_application = application_valid(&flash);
    let bus = Ht32Ep0::new();
    let mut dfu = Dfu::new(flash);
    let mut device = Device::new(bus);
    let mut clock = Millis::new();

    loop {
        device.poll(&mut dfu);
        let now = clock.now();
        dfu.tick(now);
        if dfu.should_reset() {
            hardware::stop_usb();
            cortex_m::peripheral::SCB::sys_reset();
        }
        if now >= BOOT_WINDOW_MS && !dfu.has_activity() && valid_application {
            // No DFU request means no flash command can have changed the image
            // validated before moving flash ownership into the class.
            unsafe { hardware::jump_to_application() }
        }
    }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo<'_>) -> ! {
    hardware::halt()
}
