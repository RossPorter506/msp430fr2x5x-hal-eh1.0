// This file re-exports modules that may be unique to a particular device model.
// e.g. the clock system being either the basic or enhanced version
// Business logic for device-specific implementations are handled in the device_specific/ folder

#[cfg(feature = "2x5x")]
pub mod msp430fr2x5x;

#[cfg(feature = "2x5x")]
pub use msp430fr2355 as pac;