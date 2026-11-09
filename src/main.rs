//! HT32F1654 / TEX Yoda II polling USB DFU bootloader.
//!
//! A held PC15 -> diode -> switch -> PC4 key latches DFU entry at reset.
//! Otherwise a valid application at 0x2000 starts immediately.
//! Empty or invalid applications remain in DFU. PB1 indicates the DFU state;
//! the recovery key is sampled only for the boot-time application decision.
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

use bootloader::dfu::{application_valid, Dfu, State};
use cortex_m_rt::entry;
use hardware::{HardwareFlash, Millis};
use bootloader::usb::Device;
use usb_hw::Ht32Ep0;


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
    hardware::initialize_led();

    hardware::initialize_boot_key();
    let key_held = hardware::boot_key_held();
    let flash = HardwareFlash::new();
    if !key_held && application_valid(&flash) {
        unsafe { hardware::jump_to_application() }
    }
    let bus = Ht32Ep0::new();
    let mut dfu = Dfu::new(flash);
    let mut device = Device::new(bus);
    let mut clock = Millis::new();
    let mut led_changed = 0u32;
    let mut led_on = true;
    let mut led_mode = (Some(250u32), true);

    loop {
        device.poll(&mut dfu);
        let now = clock.now();
        dfu.tick(now);
        let mode = match dfu.state() {
            State::Idle => (Some(250), true),
            State::DownloadSync | State::DownloadBusy | State::DownloadIdle
            | State::UploadIdle => (Some(125), true),
            State::ManifestSync | State::Manifest => (Some(50), true),
            State::Error => (None, true),
            State::ManifestWaitReset => (None, false),
        };
        if mode != led_mode {
            led_mode = mode;
            led_changed = now;
            led_on = mode.1;
            hardware::set_led(led_on);
        } else if let Some(period) = mode.0 {
            if now.wrapping_sub(led_changed) >= period {
                led_changed = now;
                led_on = !led_on;
                hardware::set_led(led_on);
            }
        }
        if dfu.should_reset() {
            hardware::set_led(false);
            hardware::stop_usb();
            cortex_m::peripheral::SCB::sys_reset();
        }
    }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo<'_>) -> ! {
    hardware::halt()
}
