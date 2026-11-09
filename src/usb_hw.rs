//! Fixed EP0 backend, specialized from the MIT-licensed ht32-usbd driver.
//! USB SRAM is SETUP [0,8), IN [8,72), OUT [72,136); no buffers are allocated.

use cortex_m::interrupt;
use ht32_pac::{usb::regs::UsbIsr, USB};
use usb_device::bus::{PollResult, UsbBus};
use usb_device::endpoint::{EndpointAddress, EndpointType};
use usb_device::{Result, UsbDirection, UsbError};

/// The binary's sole USB owner. Compound register/SRAM operations stay inside
/// interrupt critical sections; no Rust references into peripheral SRAM escape.
pub struct Ht32Ep0 {
    _private: (),
}

impl Ht32Ep0 {
    pub fn new() -> Self {
        interrupt::free(|_| {
            crate::hardware::enable_usb();
            USB.ep0().cfgr().modify(|w| {
                w.set_epbufa(8);
                w.set_eplen(64);
            });
        });
        Self { _private: () }
    }
}

// Call only within a critical section. ESOFIF is W0C, unlike the W1C flags:
// writing one preserves it while acknowledging all endpoint summary flags.
fn clear_endpoint_flags() {
    USB.isr().write(|w| {
        w.0 = 0xff00;
        w.set_esofif(true);
    });
}

fn enable_events() {
    USB.ier().write(|w| {
        w.set_ugie(true);
        w.set_urstie(true);
        w.set_rsmie(true);
        w.set_suspie(true);
    });
}

// Callers hold a critical section and have checked packet lengths. Aligned
// word accesses never leave their fixed, word-sized buffer, including tails.
// Empty packets perform no SRAM access; partial words do not read user padding.
fn write_packet(mut buf: &[u8]) {
    let mut ptr = (0x400a_a000usize + 8) as *mut u32;
    while buf.len() >= 4 {
        let word = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
        unsafe { ptr.write_volatile(word); ptr = ptr.add(1); }
        buf = &buf[4..];
    }
    if !buf.is_empty() {
        let mut word = buf[0] as u32;
        if buf.len() > 1 { word |= (buf[1] as u32) << 8; }
        if buf.len() > 2 { word |= (buf[2] as u32) << 16; }
        unsafe { ptr.write_volatile(word); }
    }
}

// offset is either SETUP's 0 (at most 8 bytes) or OUT's 72 (at most 64).
fn read_packet(offset: usize, mut buf: &mut [u8]) {
    let mut ptr = (0x400a_a000usize + offset) as *const u32;
    while buf.len() >= 4 {
        let word = unsafe { let word = ptr.read_volatile(); ptr = ptr.add(1); word };
        buf[0] = word as u8;
        buf[1] = (word >> 8) as u8;
        buf[2] = (word >> 16) as u8;
        buf[3] = (word >> 24) as u8;
        buf = &mut buf[4..];
    }
    if !buf.is_empty() {
        let word = unsafe { ptr.read_volatile() };
        buf[0] = word as u8;
        if buf.len() > 1 { buf[1] = (word >> 8) as u8; }
        if buf.len() > 2 { buf[2] = (word >> 16) as u8; }
    }
}

impl UsbBus for Ht32Ep0 {
    fn alloc_ep(
        &mut self,
        ep_dir: UsbDirection,
        ep_addr: Option<EndpointAddress>,
        ep_type: EndpointType,
        max_packet_size: u16,
        _interval: u8,
    ) -> Result<EndpointAddress> {
        let address = EndpointAddress::from_parts(0, ep_dir);
        // Merely describe the preconfigured EP0; never allocate resources.
        if ep_type == EndpointType::Control && max_packet_size == 64
            && ep_addr.is_none_or(|requested| requested == address)
        {
            Ok(address)
        } else {
            Err(UsbError::InvalidEndpoint)
        }
    }

    fn enable(&mut self) {
        interrupt::free(|_| {
            USB.csr().write(|w| {
                w.set_dppuen(true);
                w.set_dpwken(true);
                w.set_pdwn(true);
                w.set_lpmode(true);
            });
            USB.isr().write_value(UsbIsr(!0));
            USB.csr().modify(|w| w.set_dpwken(false));
            enable_events();
        });
    }

    fn reset(&self) {
        interrupt::free(|_| {
            // Reset every CSR control bit except the existing internal pull-up.
            let pullup = USB.csr().read().dppuen();
            USB.csr().write(|w| w.set_dppuen(pullup));
            enable_events();
            USB.ep0().cfgr().modify(|w| {
                w.set_epadr(0);
                w.set_epen(true);
            });
            USB.ep0().ier().write(|w| {
                w.set_sdrxie(true);
                w.set_odrxie(true);
                w.set_idtxie(true);
            });
            USB.ier().modify(|w| w.set_ep0ie(true));
        });
    }

    fn set_device_address(&self, addr: u8) {
        // usb-device calls this after accepting SET_ADDRESS; leave ADRSET alone.
        interrupt::free(|_| USB.devar().modify(|w| w.set_deva(addr)));
    }

    fn poll(&self) -> PollResult {
        interrupt::free(|_| {
            let status = USB.isr().read();
            if status.rsmif() {
                USB.isr().write(|w| { w.set_esofif(true); w.set_rsmif(true); });
                PollResult::Resume
            } else if status.urstif() {
                USB.isr().write(|w| { w.set_esofif(true); w.set_urstif(true); });
                PollResult::Reset
            } else if status.suspif() {
                USB.isr().write(|w| { w.set_esofif(true); w.set_suspif(true); });
                PollResult::Suspend
            } else if status.0 & 0xff00 != 0 {
                let mut ep_out = 0;
                let mut ep_in_complete = 0;
                let mut ep_setup = 0;
                if status.ep0if() {
                    let ep_status = USB.ep0().isr().read();
                    ep_setup = u16::from(ep_status.sdrxif());
                    ep_out = u16::from(ep_status.odrxif());
                    ep_in_complete = u16::from(ep_status.idtxif());
                    if ep_status.idtxif() {
                        USB.ep0().isr().write(|w| w.set_idtxif(true));
                    }
                }
                clear_endpoint_flags();
                PollResult::Data { ep_out, ep_in_complete, ep_setup }
            } else {
                PollResult::None
            }
        })
    }

    fn write(&self, ep_addr: EndpointAddress, buf: &[u8]) -> Result<usize> {
        if !ep_addr.is_in() || ep_addr.index() != 0 {
            return Err(UsbError::InvalidEndpoint);
        }
        interrupt::free(|_| {
            if buf.len() > 64 { return Err(UsbError::BufferOverflow); }
            if USB.ep0().tcr().read().txcnt() != 0 { return Err(UsbError::WouldBlock); }
            write_packet(buf);
            USB.ep0().tcr().modify(|w| w.set_txcnt(buf.len() as u8));
            // Endpoint CSR bits toggle on one; never read-modify-write them.
            if USB.ep0().csr().read().naktx() {
                USB.ep0().csr().write(|w| w.set_naktx(true));
            }
            Ok(buf.len())
        })
    }

    fn read(&self, ep_addr: EndpointAddress, buf: &mut [u8]) -> Result<usize> {
        if !ep_addr.is_out() || ep_addr.index() != 0 {
            return Err(UsbError::InvalidEndpoint);
        }
        interrupt::free(|_| {
            let status = USB.ep0().isr().read();
            // SETUP must win if SETUP and OUT are both pending.
            let result = if status.sdrxif() {
                USB.ep0().isr().write(|w| w.set_sdrxif(true));
                if buf.len() < 8 { Err(UsbError::BufferOverflow) }
                else { read_packet(0, &mut buf[..8]); Ok(8) }
            } else if status.odrxif() {
                USB.ep0().isr().write(|w| w.set_odrxif(true));
                let count = USB.ep0().tcr().read().rxcnt() as usize;
                if count > buf.len() || count > 64 { Err(UsbError::BufferOverflow) }
                else { read_packet(72, &mut buf[..count]); Ok(count) }
            } else {
                Err(UsbError::WouldBlock)
            };
            if USB.ep0().csr().read().nakrx() {
                USB.ep0().csr().write(|w| w.set_nakrx(true));
            }
            clear_endpoint_flags();
            result
        })
    }

    fn set_stalled(&self, ep_addr: EndpointAddress, stalled: bool) {
        if ep_addr.index() != 0 { return; }
        interrupt::free(|_| {
            let status = USB.ep0().csr().read();
            let current = if ep_addr.is_out() { status.stlrx() } else { status.stltx() };
            if current != stalled {
                USB.ep0().csr().write(|w| {
                    if ep_addr.is_out() { w.set_stlrx(true); }
                    else { w.set_stltx(true); }
                });
            }
        });
    }

    fn is_stalled(&self, ep_addr: EndpointAddress) -> bool {
        if ep_addr.index() != 0 { return false; }
        interrupt::free(|_| {
            let status = USB.ep0().csr().read();
            if ep_addr.is_out() { status.stlrx() } else { status.stltx() }
        })
    }

    fn suspend(&self) {
        interrupt::free(|_| USB.csr().modify(|w| w.set_lpmode(true)));
    }

    fn resume(&self) {
        interrupt::free(|_| USB.csr().modify(|w| w.set_lpmode(false)));
    }
}
