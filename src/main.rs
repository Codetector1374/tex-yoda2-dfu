//! HT32F1654 / TEX Yoda II polling USB DFU bootloader.
//!
//! A held PC15 -> diode -> switch -> PC4 key latches DFU entry at reset.
//! Otherwise a valid application at 0x2000 starts immediately.
//! Empty or invalid applications remain in DFU. PB1 blinks twice as fast while
//! the key is pressed in DFU and returns to normal speed when released.
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

const LED_HALF_PERIOD_MS: u32 = 250;

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

    loop {
        device.poll(&mut dfu);
        let now = clock.now();
        let led_half_period_ms = if hardware::boot_key_held() {
            LED_HALF_PERIOD_MS / 2
        } else {
            LED_HALF_PERIOD_MS
        };
        if now.wrapping_sub(led_changed) >= led_half_period_ms {
            led_changed = now;
            led_on = !led_on;
            hardware::set_led(led_on);
        }
        dfu.tick(now);
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
