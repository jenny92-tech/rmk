//! Vendor data channel (k9pad).
//!
//! A 64-byte bidirectional payload pipe exposed alongside the regular
//! keyboard interface so a companion app can stream control packets to the
//! firmware and receive packets back. Available over both BLE (vendor GATT
//! service, UUID base `e9dc0000-7374-7265-616d-6b3970616400`) and USB
//! (vendor HID interface, usage page `0xFF61`).
//!
//! The shared queue statics live in [`crate::channel`] (`DATA_CHANNEL_RX` /
//! `DATA_CHANNEL_TX`); transport-side runners are in [`ble`] and [`usb`].

#[cfg(feature = "_ble")]
pub(crate) mod ble;
#[cfg(not(feature = "_no_usb"))]
pub(crate) mod usb;

pub use crate::channel::{DATA_CHANNEL_RX, DATA_CHANNEL_TX};
