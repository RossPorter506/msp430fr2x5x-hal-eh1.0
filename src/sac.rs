//! Smart Analog Combo (SAC)
//!
//! The Smart Analog Combo (SAC) integrates an operational amplifier, a programmable gain amplifier with up to 33x
//! gain, and a 12-bit Digital to Analog Converter (DAC) core. The SAC can be used for signal
//! conditioning for either the input or output path.
//!
//! Only available on the MSP430FR235x.
//! 
//! There are four SAC modules, forming two pairs (SAC0 and SAC2, SAC1 and SAC3). 
//! The amplifiers in each pair can be fed the output of the other (e.g. SAC0 may use the output of SAC2, 
//! and SAC2 may use the output of SAC0). Both amplifiers can be fed into the respective 
//! enhanced comparator module (eCOMP) - SAC0 and SAC2 into eCOMP0, and SAC1 and SAC3 into eCOMP1.
//! 
//! Each SAC can be put into one of four modes: 
//! - An open-loop operational amplifier (no internal feedback):
#![allow(rustdoc::bare_urls)] // SVG files trigger false positives
#![doc= include_str!("../docs/sac_open_loop.svg")]
//! - An inverting amplifier with programmable gain/feedback:
#![doc= include_str!("../docs/sac_inverting.svg")]
//! - A non-inverting amplifier with programmable gain/feedback:
#![doc= include_str!("../docs/sac_noninverting.svg")]
//! - A unity-gain buffer / voltage follower:
#![doc= include_str!("../docs/sac_buffer.svg")]
//! 
//! In all modes the positive input to the amplifier can be connected to:
//! - The external pin OA+,
//! - An internal 12-bit DAC,
//! - The output of the paired SAC opamp.
//! 
//! In the open-loop or inverting configuration, the negative input of the amplifier can be connected to:
//! - The external pin OA-,
//! - The output of the paired SAC opamp.
//! 
//! The output of the amplifier can either be routed to the external pin OAO, 
//! or used internally with the enhanced comparator module.
//! 
//! To begin configuration, call [`SacConfig::begin()`]. This returns configuration objects for the DAC 
//! and for the amplifier. If the DAC is not used then it need not be configured.
//! 
//! Pins used:
//! 
//! |        |   OA+  |  OA--  |   OAO   |
//! |:------:|:------:|:------:|:-------:|
//! | SAC0   | `P1.3` | `P1.2` | `P1.1`  |
//! | SAC1   | `P1.7` | `P1.6` | `P1.5`  |
//! | SAC2   | `P3.3` | `P3.2` | `P3.1`  |
//! | SAC3   | `P3.7` | `P3.6` | `P3.5`  |
//! 

use core::marker::PhantomData;
use msp430fr2355::TB2;

use crate::{hw_traits::sac::{MSel, NSel, SacPeriph}, pmm::InternalVRef, pwm::{CCR1, CCR2}, timer::SubTimer};

/// A builder for configuring a Smart Analog Combo (SAC) unit
pub struct SacConfig;
impl SacConfig {
    /// Begin configuration of a Smart Analog Combo (SAC) unit.
    #[inline(always)]
    pub fn begin<SAC: SacPeriph>(_reg: SAC) -> (DacConfig<SAC>, AmpConfig<NoModeSet, SAC>) {
        (DacConfig(PhantomData), AmpConfig{mode: PhantomData, reg: PhantomData})
    }
}

/// Struct representing a configuration for a DAC inside this Smart Analog Combo (SAC) unit.
pub struct DacConfig<SAC: SacPeriph>(PhantomData<SAC>);
impl<SAC: SacPeriph> DacConfig<SAC> {
    /// Initialise the DAC within this SAC with the provided values.
    #[inline(always)]
    pub fn configure<'a>(self, vref: VRef<'a>, load_trigger: LoadTrigger<'_>) -> Dac<'a, SAC> {
        SAC::configure_dac(load_trigger, vref);
        Dac{sac: PhantomData, vref_lifetime: PhantomData}
    }
}

#[derive(Copy, Clone)]
/// Options for when the DAC loads in a new value placed in the DAC data register.
pub enum LoadTrigger<'a> {
    /// The DAC loads the new value as soon as the register is written to.
    Immediate,
    /// The DAC loads the new value when TB2.1 exhibits a rising edge.
    TB2_1(&'a SubTimer<TB2, CCR1>),
    /// The DAC loads the new value when TB2.2 exhibits a rising edge.
    TB2_2(&'a SubTimer<TB2, CCR2>),
}
impl From<LoadTrigger<'_>> for u8 {
    #[inline(always)]
    fn from(value: LoadTrigger) -> Self {
        match value {
            LoadTrigger::Immediate => 0b00,
            //           Reserved:    0b01,
            LoadTrigger::TB2_1(_)  => 0b10,
            LoadTrigger::TB2_2(_)  => 0b11,
        }
    }
}

/// Defines which voltage reference the DAC uses
#[derive(Debug, Copy, Clone)]
pub enum VRef<'a> {
    /// Use VCC as the DAC reference voltage.
    Vcc,
    /// Use the shared internal reference as the DAC reference voltage
    Internal(&'a InternalVRef),
}
impl From<VRef<'_>> for bool {
    #[inline(always)]
    fn from(value: VRef) -> Self {
        match value {
            VRef::Vcc         => false,
            VRef::Internal(_) => true,
        }
    }
}

/// The Digital to Analog Converter (DAC) inside this Smart Analog Combo (SAC) module.
#[derive(Debug)]
pub struct Dac<'a, SAC: SacPeriph>{
    sac: PhantomData<SAC>, 
    vref_lifetime: PhantomData<VRef<'a>>, // If we use the internal reference, ensure it stays enabled for the life of the DAC.
}
impl<SAC: SacPeriph> Dac<'_, SAC> {
    /// Set the DAC count. This should be a value between 0 and 4095, where 0 is 0V, and 4095 is (just below) the DAC reference voltage.
    /// The value is masked with `0xFFF` before being written to the register.
    #[inline(always)]
    pub fn set_count(&mut self, count: u16) {
        SAC::set_dac_count(count);
    }
}

/// A builder for configuring a Smart Analog Combo (SAC) unit's amplifier
pub struct AmpConfig<MODE, SAC> {
    mode: PhantomData<MODE>,
    reg: PhantomData<SAC>,
}
impl<SAC:SacPeriph> AmpConfig<NoModeSet, SAC> {
    /// Begin configuring this SAC as an open-loop operational amplifier (no internal feedback).
    #[inline(always)]
    pub fn opamp(self, pos_in: PositiveInput<SAC>, neg_in: NegativeInput<SAC>, power_mode: PowerMode) -> AmpConfig<ModeSet, SAC> {
        SAC::configure_sacoa(pos_in.psel(), neg_in.nsel(), power_mode);
        AmpConfig{ mode: PhantomData, reg: PhantomData }
    }

    /// Begin configuring this SAC as an inverting amplifier.
    #[inline(always)]
    pub fn inverting_amplifier(self, pos_in: PositiveInput<SAC>, neg_in: NegativeInput<SAC>, gain: InvertingGain, power_mode: PowerMode) -> AmpConfig<ModeSet, SAC> {
        SAC::configure_sacpga(gain as u8, neg_in.msel());
        SAC::configure_sacoa(pos_in.psel(), NSel::Feedback, power_mode);
        AmpConfig{ mode: PhantomData, reg: PhantomData }
    }

    /// Begin configuring this SAC as a non-inverting amplifier.
    #[inline(always)]
    pub fn noninverting_amplifier(self, pos_in: PositiveInput<SAC>, gain: NoninvertingGain, power_mode: PowerMode) -> AmpConfig<ModeSet, SAC> {
        SAC::configure_sacpga(gain as u8, MSel::NonInverting);
        SAC::configure_sacoa(pos_in.psel(), NSel::Feedback, power_mode);
        AmpConfig{ mode: PhantomData, reg: PhantomData }
    }

    /// Begin configuring this SAC as a unity-gain buffer / voltage follower.
    #[inline(always)]
    pub fn buffer(self, source: PositiveInput<SAC>, power_mode: PowerMode) -> AmpConfig<ModeSet, SAC> {
        SAC::configure_sacpga(0, MSel::Follower);
        SAC::configure_sacoa(source.psel(), NSel::Feedback, power_mode);
        AmpConfig{ mode: PhantomData, reg: PhantomData }
    }
}
impl<SAC:SacPeriph> AmpConfig<ModeSet, SAC> {
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
    ExtPin(SAC::PosInputPin),
    /// Use the SAC's Internal DAC as the amplifier's non-inverting input
    Dac(&'a Dac<'a, SAC>),
    /// Use the output of the paired SAC amplifier as this amplifier's non-inverting input. 
    /// It is your responsibility to ensure this amplifier has been configured.
    // We can't require a reference to this Amplifier, as they could both refer to the other which would be impossible to instantiate
    PairedOpamp,
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

/// List of possible sources for the SAC amplifier's inverting input
// Note that this corresponds to a combination of NSEL and MSEL. In modes with feedback NSEL is always 0b01, so MSEL varies the negative input.
#[derive(Debug)]
pub enum NegativeInput<SAC: SacPeriph> {
    /// Use the GPIO pin labelled as OA- as the amplifier's inverting input
    ExtPin(SAC::NegInputPin),
    /// Use the output of the paired SAC amplifier as this amplifier's inverting input
    PairedOpamp,
}
impl<SAC: SacPeriph> NegativeInput<SAC> {
    /// This corresponds to the input to the inverting opamp input when in open-loop mode
    #[inline(always)]
    fn nsel(&self) -> NSel {
        match self {
            NegativeInput::ExtPin(_)      => NSel::ExtPinMinus,
            NegativeInput::PairedOpamp    => NSel::PairedOpamp,
        }
    }
    /// In modes with feedback this corresponds to whether the feedback divider is connected to OA- or the paired opamp output
    #[inline(always)]
    fn msel(&self) -> MSel {
        match self {
            NegativeInput::ExtPin(_)      => MSel::Inverting,
            NegativeInput::PairedOpamp    => MSel::Cascade,
        }
    }
}

/// List of possible gain values when the SAC is in the inverting amplifier mode
#[derive(Debug, Copy, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub enum InvertingGain {
    //    0b000 is not a valid value
    /// 1x gain
    _1  = 0b001,
    /// 2x gain
    _2  = 0b010,
    /// 4x gain
    _4  = 0b011,
    /// 8x gain
    _8  = 0b100,
    /// 16x gain
    _16 = 0b101,
    /// 25x gain
    _25 = 0b110,
    /// 32x gain
    _32 = 0b111,
}

/// List of possible gain values when the SAC is in the non-inverting amplifier mode
#[derive(Debug, Copy, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub enum NoninvertingGain {
    /// 1x gain
    _1  = 0b000,
    /// 2x gain
    _2  = 0b001,
    /// 3x gain
    _3  = 0b010,
    /// 5x gain
    _5  = 0b011,
    /// 9x gain
    _9  = 0b100,
    /// 17x gain
    _17 = 0b101,
    /// 26x gain
    _26 = 0b110,
    /// 33x gain
    _33 = 0b111,
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
