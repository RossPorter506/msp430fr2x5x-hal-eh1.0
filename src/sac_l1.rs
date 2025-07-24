//! Smart Analog Combo L1 (SAC-L1)
//!
//! The Smart Analog Combo L1 (SAC-L1) integrates an operational amplifier.
//! TODO.

use core::marker::PhantomData;

use crate::{
    hw_traits::sac::{MSel, NSel, SacPeriph}, pmm::InternalVRef, pwm::{CCR1, CCR2}, timer::SubTimer,
    pac::TB2,
};

/// A builder for configuring a Smart Analog Combo (SAC) unit
pub struct SacConfig;
impl SacConfig {
    /// Begin configuration of a Smart Analog Combo (SAC) unit.
    #[inline(always)]
    pub fn begin<SAC: SacPeriph>(_reg: SAC, pos_in: PositiveInput<SAC>, power_mode: PowerMode) -> Self {
        SAC::configure_sacoa(pos_in.psel(), neg_in.nsel(), power_mode);
        AmpConfig{mode: PhantomData, reg: PhantomData}
    }
    /// Route the output of the amplifier to the GPIO pin
    #[inline(always)]
    pub fn output_pin(self, _output_pin: impl Into<SAC::OutputPin>) -> Amplifier<SAC> {
        Amplifier(PhantomData)
    }
    /// Do not route the amplifier output to a GPIO pin.
    /// Useful if you only need the signal internally and don't want to give up a GPIO pin.
    #[inline(always)]
    pub fn no_output_pin(self) -> Amplifier<SAC> {
        Amplifier(PhantomData)
    }
}

/// List of possible sources for the amplifier's non-inverting input
#[derive(Debug)]
pub enum PositiveInput<'a, SAC: SacPeriph> {
    /// Use the GPIO pin labelled as OA+ as this amplifier's non-inverting input
    ExtPin0(SAC::PosInputPin0),
    /// Use the GPIO pin labelled as OA+ as this amplifier's non-inverting input
    ExtPin1(SAC::PosInputPin1),
}
impl<SAC: SacPeriph> PositiveInput<'_, SAC> {
    #[inline(always)]
    fn psel(&self) -> u8 {
        match self {
            PositiveInput::ExtPin(_)   => 0b00,
            PositiveInput::Dac(_)      => 0b01,
            PositiveInput::PairedOpamp => 0b10,
        }
    }
}

/// Power mode setting for the SAC. Controls power consumption and opamp slew rate
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum PowerMode {
    /// High slew rate, high power consumption - 3 V/us @ 1mA
    HighPerformance,
    /// Low slew rate, low power consumption - 1 V/us @ 200uA
    LowPower,
}
impl From<PowerMode> for bool {
    #[inline(always)]
    fn from(value: PowerMode) -> Self {
        match value {
            PowerMode::HighPerformance => false,
            PowerMode::LowPower => true,
        }
    }
}

/// Represents an amplifier inside a Smart Analog Combo (SAC) that has been configured
pub struct Amplifier<SAC: SacPeriph>(PhantomData<SAC>);

/// Typestate for a SacConfig that has not been configured yet
pub struct NoModeSet;
/// Typestate for a SacConfig that has been configured for a particular mode
pub struct ModeSet;
