use core::convert::Infallible;

use crate::gpio::{Alternate2, Floating, Input, Output, Pin, Pin0, Pin1, Pin4, Pin5};
use crate::hw_traits::{Steal, ecomp::{CompDacPeriph, ECompInputs, ECompPeriph}};
use crate::pac::{P1, P2, P3, E_COMP0, E_COMP1};
#[cfg(feature = "sac")]
use crate::{sac_l3::Amplifier, pac::{SAC0, SAC1, SAC2, SAC3}};

/********** eCOMP ***********/

/// List of possible inputs to the positive input of an eCOMP comparator.
/// The amplifier output and DAC options take a reference to ensure they have been configured.
#[allow(non_camel_case_types)]
pub enum PositiveInput<'a, COMP: ECompInputs> {
    /// COMPx.0. P1.0 for COMP0, P2.5 for COMP1
    COMPx_0(COMP::COMPx_0),

    /// COMPx.1. P1.1 for COMP0, P2.4 for COMP1
    COMPx_1(COMP::COMPx_1),

    /// Internal 1.2V reference
    _1V2,

    /// Output of amplifier SAC0 for eCOMP0, SAC2 for eCOMP1. 
    /// 
    /// Requires a reference to ensure that it has been configured.
    #[cfg(feature = "sac")]
    OAxO(&'a COMP::SACp),

    /// P1.1 for eCOMP0, P1.5 for eCOMP1.
    OtherPin(COMP::DeviceSpecific3Pos),

    /// This eCOMP's internal 6-bit DAC
    /// 
    /// Requires a reference to ensure that it has been configured.
    Dac(&'a dyn CompDacPeriph<COMP>),
}
impl<COMP: ECompInputs> PositiveInput<'_, COMP>  {
    #[inline(always)]
    pub(crate) fn cppsel(&self) -> u8 {
        match self {
            PositiveInput::COMPx_0(_)   => 0b000,
            PositiveInput::COMPx_1(_)   => 0b001,
            PositiveInput::_1V2         => 0b010,
            #[cfg(feature = "sac")]
            PositiveInput::OAxO(_)      => 0b101,
            PositiveInput::OtherPin(_)  => 0b101,
            PositiveInput::Dac(_)       => 0b110,
        }
    }
}

/// List of possible inputs to the negative input of an eCOMP comparator.
/// The amplifier output and DAC options take a reference to ensure they have been configured.
#[allow(non_camel_case_types)]
pub enum NegativeInput<'a, COMP: ECompInputs> {
    /// COMPx.0. P1.0 for COMP0, P2.5 for COMP1
    COMPx_0(COMP::COMPx_0),

    /// COMPx.1. P1.1 for COMP0, P2.4 for COMP1
    COMPx_1(COMP::COMPx_1),

    /// Internal 1.2V reference
    _1V2,

    /// Output of amplifier SAC1 for eCOMP0, SAC3 for eCOMP1. 
    /// 
    /// Requires a reference to ensure that it has been configured.
    #[cfg(feature = "sac")]
    OAxO(&'a COMP::SACn),

    /// P1.5 for eCOMP0, P3.5 for eCOMP1.
    OtherPin(COMP::DeviceSpecific3Neg),

    /// This eCOMP's internal 6-bit DAC
    /// 
    /// Requires a reference to ensure that it has been configured.
    Dac(&'a dyn CompDacPeriph<COMP>),
}
impl<COMP: ECompInputs> NegativeInput<'_, COMP> {
    #[inline(always)]
    pub(crate) fn cpnsel(&self) -> u8 {
        match self {
            NegativeInput::COMPx_0(_)   => 0b000,
            NegativeInput::COMPx_1(_)   => 0b001,
            NegativeInput::_1V2         => 0b010,
            #[cfg(feature = "sac")]
            NegativeInput::OAxO(_)      => 0b101,
            NegativeInput::OtherPin(_)  => 0b101,
            NegativeInput::Dac(_)       => 0b110,
        }
    }
}

impl ECompInputs for E_COMP0 {
    type COMPx_0            = Pin<P1, Pin0, Alternate2<Input<Floating>>>;
    type COMPx_1            = Pin<P1, Pin1, Alternate2<Input<Floating>>>;
    type COMPx_Out          = Pin<P2, Pin0, Alternate2<Output>>;
    
    type DeviceSpecific0    = (); // Internal 1.2V reference. No type required.
    type DeviceSpecific1    = Infallible; // N/A
    type DeviceSpecific2Pos = Infallible; // N/A
    type DeviceSpecific2Neg = Infallible; // N/A
    type DeviceSpecific3Pos = Pin<P1, Pin1, Alternate2<Input<Floating>>>;
    type DeviceSpecific3Neg = Pin<P3, Pin1, Alternate2<Input<Floating>>>;

    #[cfg(feature = "sac")]
    type SACp = Amplifier<SAC0>;
    #[cfg(feature = "sac")]
    type SACn = Amplifier<SAC2>;
}

impl ECompInputs for E_COMP1 {
    type COMPx_0            = Pin<P2, Pin5, Alternate2<Input<Floating>>>;
    type COMPx_1            = Pin<P2, Pin4, Alternate2<Input<Floating>>>;
    type COMPx_Out          = Pin<P2, Pin1, Alternate2<Output>>;

    type DeviceSpecific0    = (); // Internal 1.2V reference. No type required.
    type DeviceSpecific1    = Infallible; // N/A
    type DeviceSpecific2Pos = Infallible; // N/A
    type DeviceSpecific2Neg = Infallible; // N/A
    type DeviceSpecific3Pos = Pin<P1, Pin5, Alternate2<Input<Floating>>>;
    type DeviceSpecific3Neg = Pin<P3, Pin5, Alternate2<Input<Floating>>>;

    #[cfg(feature = "sac")]
    type SACp = Amplifier<SAC1>;
    #[cfg(feature = "sac")]
    type SACn = Amplifier<SAC3>;
}

use crate::hw_traits::ecomp::impl_ecomp;
use crate::ecomp::{BufferSel, DacBufferMode, DacVRef, FilterStrength, Hysteresis, OutputPolarity, PowerMode};

crate::hw_traits::ecomp::impl_ecomp!(
    E_COMP0,
    cpctl0, cpctl1,
    cpdacctl, cpdacdata,
    cpint, cpiv
);

impl_ecomp!(
    E_COMP1,
    cp1ctl0, cp1ctl1,
    cp1dacctl, cp1dacdata,
    cp1int, cp1iv
);

/********** SAC **********/

impl_sac_periph!(
    SAC0, P1, Pin3, Pin2, Pin1, // Register block, port, pos_in, neg_in, out
    sac0oa, sac0pga, sac0dac, sac0dat
);
impl_sac_periph!(
    SAC1, P1, Pin7, Pin6, Pin5,
    sac1oa, sac1pga, sac1dac, sac1dat
);
impl_sac_periph!(
    SAC2, P3, Pin3, Pin2, Pin1,
    sac2oa, sac2pga, sac2dac, sac2dat
);
impl_sac_periph!(
    SAC3, P3, Pin7, Pin6, Pin5,
    sac3oa, sac3pga, sac3dac, sac3dat
);
