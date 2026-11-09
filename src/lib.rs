//! Host-testable DFU protocol and USB transport for the HT32F1654 bootloader.
#![cfg_attr(not(test), no_std)]

pub mod dfu;
pub mod usb;
