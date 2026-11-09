# HT32F1654 Rust DFU bootloader

This targets **Holtek HT32F1654**, not WCH CH32.
A polling, `no_std` USB DFU bootloader, without a HAL, heap, RTOS, or async executor.

## Layout

- `ht32-pac`: full SVD-generated chiptool-style register API, with global peripheral constants and copyable register handles. No `Peripherals::take()` requirement. See its generation instructions and input notices.
- `src/dfu/`: DFU protocol, bounded streaming image programming, and host tests.
- `src/usb.rs`: compact EP0-only USB transport and packet-level host tests.
- `src/usb_hw.rs`: binary-only fixed EP0 hardware backend, using the same generated `ht32-pac` as clock and flash support.
- `src/main.rs`, `src/hardware.rs`: Cortex-M3 startup, clocks, FMC, and application handoff, built only as the firmware binary.
- `src/memcpy.s`, `src/memclr.s`: compact C/AEABI copy and zeroing helpers. Byte accesses support arbitrary alignment and avoid generic implementations; throughput is traded for flash size.
- Root `Cargo.toml`: the single `bootloader` package, with a host-testable library and the hardware binary.
- `../chiptool`: sibling local source fork used for PAC generation, retaining its Git history and upstream license. `../ht32-usbd` remains an unused reference fork, not a firmware dependency. No remote GitHub forks have been created (`gh` is unavailable). This firmware Git repository is `ht32-rs/usb_dfu_bootloader/`.

## Hardware and memory contract

- HT32F1654, 8 MHz external crystal, 72 MHz core, 48 MHz USB.
- Bootloader: `[0x00000000, 0x00002000)` (8 KiB), enforced by the linker.
- Application: `[0x00002000, 0x0000FC00)` (55 KiB).
- SRAM: `[0x20000000, 0x20004000)` (16 KiB).
- Erase pages: 1024 bytes; program units: aligned 32-bit words.
- USB endpoint SRAM: `0x400AA000`, 1024 bytes; internal D+ pull-up.
- PB1 drives an active-low DFU status LED. Idle: 250 ms on/off; download/upload: 125 ms on/off; manifestation: 50 ms on/off; error: solid on; completed/waiting for reset: off. The recovery key does not affect blinking. SysTick supplies polling time without interrupts or debug/trace dependencies.

The option-byte region and bootloader are never application download targets.
The application must be linked at **0x2000**, as the current Tex Yoda 2 QMK target is.
Recovery key: **PC15 -> diode -> switch -> PC4**, conducting from PC15 to PC4. PC4 is driven low and PC15 pulled up for a single sample after 1 ms settling. That sample decides application handoff. PC4 is then released to input and PC15's pull-up/input buffer disabled, including while staying in DFU.

## Build

Requires Rust with `thumbv7m-none-eabi` and GNU Arm binutils (`arm-none-eabi-size` and `arm-none-eabi-objcopy`). Run from this directory:

```sh
rustup target add thumbv7m-none-eabi
cargo test --lib
sh build.sh
```

Do not program this over a working recovery bootloader until you have a saved flash backup and working SWD recovery. Building and host tests do not establish hardware safety or USB enumeration.
The direct `src/usb_hw.rs` backend has been flashed and verified over SWD, enumerated as `feed:6969`, and uploaded the entire 56,320-byte application region over DFU with byte-for-byte agreement against SWD readback. Full QMK application download/programming is now verified through DFU; recovery under interruption remains untested on hardware.

All 28 DFU-core/USB packet library tests pass, including early status polling.
The status-LED/polling-fix binary is 7,292 bytes (900 bytes remain) and was programmed/readback-verified over SWD. A full 23,684-byte QMK application download then completed through dfu-util, reached MANIFEST-WAIT-RESET with status OK, and booted into enumerated keyboard/mouse HID collections. dfu-util returned 74 despite printing Done after reset; runtime enumeration independently confirms handoff.
The size guard is intentionally strict; compiler/dependency updates may change the footprint.
Optional manufacturer/product/serial strings are omitted; identify the device by VID:PID.

The linked Thumb memory helpers pass 3,712 copy cases and 233 zeroing cases under ARM emulation, covering boundaries, lengths 0–1024, alignment, termination, and callee-saved registers. To repeat after building, install `unicorn` in a Python environment and run `python3 tests/memory_helpers.py`.

## DFU host contract

USB VID:PID `feed:6969`, DFU 1.1 mode, alternate setting 0, 512-byte transfers. Images are raw binaries linked at 0x2000, not DfuSe address-command streams and not files including the bootloader.

```sh
dfu-util -d feed:6969 -a 0 -t 512 -D application.bin
dfu-util -d feed:6969 -a 0 -t 512 -U application-readback.bin
```

Hold the recovery key while resetting to remain in DFU. With the key released at boot, plausible stack/reset vectors at 0x2000 cause immediate application handoff before USB startup; empty or invalid vectors stay in DFU. After this one-time decision, the key has no effect on the bootloader. PB1 follows the DFU protocol state. There is no timed USB entry window. After successful manifestation, reset with the key released to start the application.

GETSTATUS/GETSTATE remain available during the advertised busy polling window.
An early status request reports the current busy state instead of latching
ERR_STALLEDPKT; download, abort and other state-changing requests remain gated.

The writer bounds every erase/program operation, verifies flash results, and defers both application vectors until manifestation. The initial stack pointer is the final commit word. An interrupted download is not made bootable by the bootloader. This is a recovery mechanism, **not signature authentication, anti-rollback, or a power-fail-atomic dual-image updater**.

## Sources and dependencies

- `src/usb_hw.rs` implements the published `usb-device` bus trait directly using the root generated `ht32-pac`; there is no second PAC or `ht32-usbd` dependency. EP0 SRAM is fixed at `0x400AA000`: 8 SETUP bytes at offset 0, 64 IN bytes at offset 8, and 64 OUT bytes at offset 72. The unused sibling `../ht32-usbd` reference is based on `https://github.com/ht32-rs/ht32-usbd`, commit `eb2b941742582fdd10affd72f461073d85e39d54` (MIT).
- `usb-device` 0.3.2 comes from crates.io without patches, providing bus traits and request/endpoint types. No USB resource allocator or generic device runtime is used. Interface 0 and EP0's 64-byte IN/OUT buffers are fixed; the transport owns the initialized bus directly.
- `../chiptool`: `https://github.com/embassy-rs/chiptool`, base `e8902440a6eee471a9b14b38ee1cd2e526d2c495` (see upstream licenses).

Register provenance and corrections live with `ht32-pac`. Flash boundaries, DFU sequencing, and vector-last commit follow the local `../../ht32-usb-dfu` reference; that project's notices still apply to adapted material.
