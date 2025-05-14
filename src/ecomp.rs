//! Enhanced Comparator (eCOMP) 
//! 
//! The enhanced comparator peripheral consists of a comparator with configurable inputs - including
//! a pair of GPIO pins, a low power 1.2V reference, the outputs of two separate Smart Analog Combo 
//! (SAC) amplifiers, and a 6-bit DAC. The comparator output can be read by software and/or routed 
//! to a GPIO pin.
//! 
//! Start by calling [`ECompConfig::begin()`], which returns two configuration objects: One for the
//! internal DAC: [`ComparatorDacConfig`], and the other for the comparator itself: [`ComparatorConfig`]. 
//! If the DAC is not used then it need not be configured.
//! 
//! The internal DAC contains dual input buffers, where either buffer can be set as the input value 
//! to the DAC. The buffer selection can either be done by software (software mode), 
//! or selected automatically by the output value of the comparator (hardware mode). 
//! The voltage reference of the DAC can be chosen as either VCC or the internal shared reference 
//! (provided it has been previously configured).
//! 
//! The comparator has a pair of inputs. In normal operation, when the positive input is larger than
//! the negative input the output is high, otherwise it is low. This behaviour can be inverted by
//! selecting the inverted output polarity mode. The comparator also features a selectable power mode 
//! ('high speed' or 'low power'), configurable hysteresis levels and an optional, variable-strength 
//! analog low pass filter on the output. 
//! 
//! Interrupts can be triggered on rising, falling, or all edges of the comparator output.

use core::marker::PhantomData;
use crate::{hw_traits::ecomp::{CompDacPeriph, DacBufferMode, ECompPeriph}, pmm::InternalVRef};

/// List of possible inputs to the positive input of an eCOMP comparator.
/// The amplifier output and DAC options take a reference to ensure they have been configured.
#[allow(non_camel_case_types)]
pub enum PositiveInput<'a, COMP: ECompPeriph> {
    /// COMPx.0. P1.0 for COMP0, P2.5 for COMP1
    COMPx_0(COMP::COMPx_0),
    /// COMPx.1. P1.1 for COMP0, P2.4 for COMP1
    COMPx_1(COMP::COMPx_1),
    /// Internal 1.2V reference
    _1V2,
    /// Output of amplifier SAC0 for eCOMP0, SAC2 for eCOMP1. 
    /// 
    /// Requires a reference to ensure that it has been configured.
    OAxO(&'a COMP::SACp),
    /// This eCOMP's internal 6-bit DAC
    /// 
    /// Requires a reference to ensure that it has been configured.
    Dac(&'a dyn CompDacPeriph<COMP>),
}
impl<COMP: ECompPeriph> From<PositiveInput<'_, COMP >> for u8 {
    fn from(value: PositiveInput<'_, COMP >) -> Self {
        match value {
            PositiveInput::COMPx_0(_)   => 0b000,
            PositiveInput::COMPx_1(_)   => 0b001,
            PositiveInput::_1V2         => 0b010,
            PositiveInput::OAxO(_)      => 0b101,
            PositiveInput::Dac(_)       => 0b110,
        }
    }
}

/// List of possible inputs to the negative input of an eCOMP comparator.
/// The amplifier output and DAC options take a reference to ensure they have been configured.
#[allow(non_camel_case_types)]
pub enum NegativeInput<'a, COMP: ECompPeriph> {
    /// COMPx.0. P1.0 for COMP0, P2.5 for COMP1
    COMPx_0(COMP::COMPx_0),
    /// COMPx.1. P1.1 for COMP0, P2.4 for COMP1
    COMPx_1(COMP::COMPx_1),
    /// Internal 1.2V reference
    _1V2,
    /// Output of amplifier SAC1 for eCOMP0, SAC3 for eCOMP1. 
    OAxO(&'a COMP::SACn),
    /// This eCOMP's internal 6-bit DAC
    Dac(&'a dyn CompDacPeriph<COMP>),
}
impl<COMP: ECompPeriph> From<NegativeInput<'_, COMP >> for u8 {
    fn from(value: NegativeInput<'_, COMP >) -> Self {
        match value {
            NegativeInput::COMPx_0(_)   => 0b000,
            NegativeInput::COMPx_1(_)   => 0b001,
            NegativeInput::_1V2         => 0b010,
            NegativeInput::OAxO(_)      => 0b101,
            NegativeInput::Dac(_)       => 0b110,
        }
    }
}

/// Represents an eCOMP DAC that has been configured
pub struct ComparatorDac<COMP: ECompPeriph, MODE> {
    phantom: PhantomData<COMP>,
    mode: PhantomData<MODE>,
}

/// List of possible reference voltages for eCOMP DACs
#[derive(Debug, Copy, Clone)]
pub enum DacVRef<'a> {
    /// Use VCC as the reference voltage for this eCOMP DAC
    Vcc,
    /// Use the internal shared voltage reference for this eCOMP DAC
    InternalRef(&'a InternalVRef),
}
impl From<DacVRef<'_>> for bool {
    fn from(value: DacVRef) -> Self {
        match value {
            DacVRef::Vcc            => false,
            DacVRef::InternalRef(_) => true,
        }
    }
}

/// Possible buffers used by the eCOMP DAC
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum DacBufferSel {
    /// CPDACBUF1
    Buffer1,
    /// CPDACBUF2
    Buffer2,
}
impl From<DacBufferSel> for bool {
    fn from(value: DacBufferSel) -> Self {
        match value {
            DacBufferSel::Buffer1 => false,
            DacBufferSel::Buffer2 => true,
        }
    }
}

/// Typestate for a eCOMP DAC that is set in the hardware dual buffering mode.
pub struct HwDualBuffer;
/// Typestate for a eCOMP DAC that is set in the software dual buffering mode.
pub struct SwDualBuffer;

/// Represents a configuration for the DAC in an eCOMP peripheral
pub struct ComparatorDacConfig<COMP>(PhantomData<COMP>);
impl<COMP: ECompPeriph> ComparatorDacConfig<COMP> {
    /// Initialise the DAC in this eCOMP peripheral in software dual buffering mode.
    /// 
    /// The DAC value is determined by one of two buffers. In software mode this is selectable at will.
    pub fn new_sw_dac(self, vref: DacVRef, buf: DacBufferSel) -> ComparatorDac<COMP, SwDualBuffer> {
        COMP::cpxdacctl(true, vref, DacBufferMode::Software, buf);
        ComparatorDac { phantom: PhantomData, mode: PhantomData }
    }
    /// Initialise the DAC in this eCOMP peripheral in hardware dual buffering mode.
    /// 
    /// The DAC value is determined by one of two buffers. In hardware mode the comparator output value selects the buffer.
    pub fn new_hw_dac(self, vref: DacVRef) -> ComparatorDac<COMP, HwDualBuffer> {
        COMP::cpxdacctl(true, vref, DacBufferMode::Hardware, DacBufferSel::Buffer1);
        ComparatorDac { phantom: PhantomData, mode: PhantomData }
    }
}

impl<COMP: ECompPeriph, MODE> ComparatorDac<COMP, MODE> {
    /// Set the value in buffer 1 (CPDACBUF1)
    pub fn set_buffer_1(&mut self, count: u8) {
        COMP::set_buf1_val(count);
    }
    /// Set the value in buffer 2 (CPDACBUF2)
    pub fn set_buffer_2(&mut self, count: u8) {
        COMP::set_buf2_val(count);
    }
}

impl<COMP: ECompPeriph> ComparatorDac<COMP, SwDualBuffer> {
    /// Consume this DAC and return a DAC in the hardware dual buffer mode
    pub fn into_hw_buffer_mode(self) -> ComparatorDac<COMP, HwDualBuffer> {
        COMP::set_dac_buffer_mode(DacBufferMode::Hardware);
        ComparatorDac{ phantom: PhantomData, mode: PhantomData }
    }
    /// Select which buffer is passed to the DAC
    pub fn select_buffer(&mut self, buf: DacBufferSel) {
        COMP::select_buffer(buf);
    }
}

impl<COMP: ECompPeriph> ComparatorDac<COMP, HwDualBuffer> {
    /// Consume this DAC and return a DAC in the software dual buffer mode
    pub fn into_sw_buffer_mode(self) -> ComparatorDac<COMP, SwDualBuffer> {
        COMP::set_dac_buffer_mode(DacBufferMode::Software);
        ComparatorDac{ phantom: PhantomData, mode: PhantomData }
    }
}

/// Struct representing a configuration for an enhanced comparator (eCOMP) module.
pub struct ECompConfig<COMP: ECompPeriph>(PhantomData<COMP>);
impl<COMP: ECompPeriph> ECompConfig<COMP> {
    /// Begin configuration of an enhanced comparator (eCOMP) module.
    pub fn begin(_reg: COMP) -> (ComparatorDacConfig<COMP>, ComparatorConfig<COMP, NoModeSet>) {
        (ComparatorDacConfig(PhantomData), ComparatorConfig(PhantomData, PhantomData))
    }
}

/// Marker struct for a Comparator that has not been configured
pub struct NoModeSet;
/// Marker struct for a Comparator that is being configured
pub struct ModeSet;

/// A configuration for the comparator in an eCOMP module
pub struct ComparatorConfig<COMP: ECompPeriph, MODE>(PhantomData<COMP>, PhantomData<MODE>);
impl<COMP: ECompPeriph> ComparatorConfig<COMP, NoModeSet> {
    /// Configure the comparator with the provided settings and turn it on 
    pub fn configure(&mut self, pos_in: PositiveInput<COMP>, neg_in: NegativeInput<COMP>, pol: OutputPolarity, pwr: PowerMode, hstr: Hysteresis, fltr: FilterStrength) -> ComparatorConfig<COMP, ModeSet> {
        COMP::cpxctl0(pos_in, neg_in);
        COMP::configure_comparator(pol, pwr, hstr, fltr);
        ComparatorConfig(PhantomData, PhantomData)
    }
}
impl<COMP: ECompPeriph> ComparatorConfig<COMP, ModeSet> {
    /// Route the comparator output to its GPIO pin (P2.0 for COMP0, P2.1 for COMP1).
    pub fn with_output_pin(&mut self, _pin: COMP::COMPx_Out) -> Comparator<COMP> {
        Comparator(PhantomData)
    }
    /// Do not route the comparator output to its GPIO pin
    pub fn no_output_pin(&mut self) -> Comparator<COMP> {
        Comparator(PhantomData)
    }
}

/// Possible hysteresis value for an eCOMP comparator. Larger hysteresis values require a larger voltage 
/// difference between the input signals before a comparator output transition occurs. 
/// 
/// Larger values reduce spurious output transitions when the two inputs are very close together, effectively 
/// making the comparator less sensitive - both to noise but also to the input signals themselves.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Hysteresis {
    /// No hysteresis.
    Off   = 0b00,
    /// 10mV of hysteresis.
    _10mV = 0b01,
    /// 20mV of hysteresis.
    _20mV = 0b10,
    /// 30mV of hysteresis.
    _30mV = 0b11,
}

/// Possible comparator power modes. Controls the power consumption and propogation delay of the comparator.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum PowerMode {
    /// eCOMP0: 1us @ 24 uA.
    /// 
    /// eCOMP1: 100ns @ 162 uA
    HighSpeed,
    /// eCOMP0: 3.2us @ 1.6uA. 
    /// 
    /// eCOMP1: 320ns @ 10uA 
    LowPower,
}
impl From<PowerMode> for bool {
    fn from(value: PowerMode) -> Self {
        match value {
            PowerMode::HighSpeed => false,
            PowerMode::LowPower => true,
        }
    }
}

/// The possible values for the strength of the low pass filter in the eCOMP module.
/// 
/// Higher values will mroe aggressively filter out high-frequency components from the output, 
/// but also delays the output signal.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum FilterStrength {
    /// Typical delay of 450 ns (in high speed mode).
    Low         = 0b00,
    /// Typical delay of 900 ns (in high speed mode).
    Medium      = 0b01,
    /// Typical delay of 1800 ns (in high speed mode).
    High        = 0b10,
    /// Typical delay of 3600 ns (in high speed mode).
    VeryHigh    = 0b11,
    /// Do not use the low pass filter.
    Off         = 0b100, 
}

/// Struct representing a configured eCOMP comparator.
pub struct Comparator<COMP: ECompPeriph>(PhantomData<COMP>);

impl<COMP: ECompPeriph> Comparator<COMP> {
    /// The current value of the comparator output
    pub fn value(&mut self) -> bool {
        COMP::value()
    }
    /// Whether the current value of the comparator output is high
    pub fn is_high(&mut self) -> bool {
        COMP::value()
    }
    /// Whether the current value of the comparator output is low
    pub fn is_low(&mut self) -> bool {
        !COMP::value()
    }
    /// Enable rising-edge interrupts (CPIFG).
    pub fn enable_rising_interrupts(&mut self) {
        COMP::en_cpie();
    }
    /// Disable rising-edge interrupts (CPIFG).
    pub fn disable_rising_interrupts(&mut self) {
        COMP::dis_cpie();
    }
    /// Enable falling-edge interrupts (CPIIFG).
    pub fn enable_falling_interrupts(&mut self) {
        COMP::en_cpiie();
    }
    /// Disable falling-edge interrupts (CPIIFG).
    pub fn disable_falling_interrupts(&mut self) {
        COMP::dis_cpiie();
    }
}

/// Output polarity of the comparator
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum OutputPolarity {
    /// Non-inverted output: When V+ is larger than V- the output is high.
    Normal,
    /// Inverted output: When V+ is larger than V- the output is low.
    Inverted,
}
impl From<OutputPolarity> for bool {
    fn from(value: OutputPolarity) -> Self {
        match value {
            OutputPolarity::Normal => false,
            OutputPolarity::Inverted => true,
        }
    }
}