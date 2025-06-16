//! Power management module

use core::marker::PhantomData;

use msp430fr2355::PMM;

/// PMM type
pub struct Pmm(PMM);

/// Struct indicating that the internal voltage reference has been enabled and configured.
/// This can be passed to the ADC to read the reference voltage.
#[derive(Debug)]
pub struct InternalVRef(ReferenceVoltage);
impl InternalVRef {
    /// Get the requested internal reference voltage
    pub fn voltage(&self) -> ReferenceVoltage {
        self.0
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, PartialOrd, Ord)]
/// A list of possible internal reference voltages
pub enum ReferenceVoltage {
    /// 1.5V
    _1V5 = 0b00,
    /// 2.0V
    _2V0 = 0b01,
    /// 2.5V
    _2V5 = 0b10,
}

/// Token indicating that the internal temperature sensor has been enabled.
/// This can be passed to the ADC to read the temperature sensor voltage.
#[derive(Debug)]
pub struct InternalTempSensor<'a>(PhantomData<&'a InternalVRef>);

impl Pmm {
    /// Sets the LOCKLPM5 bit and returns a `Pmm`
    pub fn new(pmm: PMM) -> Pmm {
        pmm.pm5ctl0.write(|w| w.locklpm5().locklpm5_0());
        Pmm(pmm)
    }

    /// Configures the internal voltage reference to the specified voltage and enables it.
    /// Returns a token signifying that the voltage reference has been enabled.
    pub fn enable_internal_reference(&mut self, vref: ReferenceVoltage) -> InternalVRef {
        self.0.pmmctl2.modify(|_, w| w
            .refvsel().bits(vref as u8)
            .intrefen().intrefen_1()
        );
        InternalVRef(vref)
    }

    /// Disables the internal reference voltage
    pub fn disable_internal_reference(&mut self, _vref: InternalVRef) {
        unsafe { self.0.pmmctl2.clear_bits(|w| w.intrefen().clear_bit()); }
    }

    /// Enables the internal temperature sensor.
    /// Returns a token signifying that the temp sensor has been enabled.
    pub fn enable_internal_temp_sensor<'a, 'b:'a>(&'a mut self, _vref: &'b InternalVRef) -> InternalTempSensor<'b> {
        unsafe { self.0.pmmctl2.set_bits(|w| w.tsensoren().set_bit()); }
        InternalTempSensor(PhantomData)
    }

    /// Disables the internal temperature sensor
    pub fn disable_internal_temp_sensor(&mut self, _tsense: InternalTempSensor) {
        unsafe { self.0.pmmctl2.clear_bits(|w| w.tsensoren().clear_bit()); }
    }
}
