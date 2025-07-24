pub trait Steal {
    unsafe fn steal() -> Self;
}

pub mod eusci;
pub mod gpio;
pub mod timerb;
#[cfg(feature = "sac")]
pub mod sac;

pub mod ecomp;