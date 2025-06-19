//! SPI
//! 
//! Peripherals eUSCI_A0, eUSCI_A1, eUSCI_B0 and eUSCI_B1 can be used for SPI communication.
//! Currently there is only support for the MSP430 to act as the master.
//!
//! Begin by calling [`SpiConfig::new()`]. Once configured an [`Spi`] will be returned. 
//!
//! Note that even if you are only using the legacy embedded-hal 0.2.7 trait implementations, configuration of the SPI bus uses the embedded-hal 1.0 versions of types (e.g. [`Mode`]).
//! 
//! [`Spi`] implements the embedded-hal [`SpiBus`](embedded_hal::spi::SpiBus) trait. 
//! 
//! [`Spi`] also provides a non-blocking implementation through [`embedded-hal-nb`](embedded_hal_nb)'s 
//! [`FullDuplex`](embedded_hal_nb::spi::FullDuplex) trait.
//!
//! Pins used:
//!
//! eUSCI_A0: {MISO: `P1.7`, MOSI: `P1.6`, SCLK: `P1.5`}.
//!
//! eUSCI_A1: {MISO: `P4.3`, MOSI: `P4.2`, SCLK: `P4.1`}.
//!
//! eUSCI_B0: {MISO: `P1.3`, MOSI: `P1.2`, SCLK: `P1.1`}.
//!
//! eUSCI_B1: {MISO: `P4.7`, MOSI: `P4.6`, SCLK: `P4.5`}.
use crate::{
    clock::{Aclk, Smclk}, 
    gpio::{Alternate1, Pin, Pin0, Pin1, Pin2, Pin3, Pin4, Pin5, Pin6, Pin7, P1, P4}, 
    hw_traits::eusci::{EusciSPI, Ucmode, Ucssel, UcxSpiCtw0},
};
use core::marker::PhantomData;
use msp430fr2355 as pac;
use nb::Error::WouldBlock;

use embedded_hal::spi::{Mode, Phase, Polarity};

/// Marks a eUSCI capable of SPI communication (in this case, all euscis do)
pub trait SpiUsci: EusciSPI {
    /// Master In Slave Out (refered to as SOMI in datasheet)
    type MISO;
    /// Master Out Slave In (refered to as SIMO in datasheet)
    type MOSI;
    /// Serial Clock
    type SCLK;
    /// Slave Transmit Enable (acts like CS)
    type STE;
}

impl SpiUsci for pac::E_USCI_A0 {
    type MISO = UsciA0MISOPin;
    type MOSI = UsciA0MOSIPin;
    type SCLK = UsciA0SCLKPin;
    type STE = UsciA0STEPin;
}

impl SpiUsci for pac::E_USCI_A1 {
    type MISO = UsciA1MISOPin;
    type MOSI = UsciA1MOSIPin;
    type SCLK = UsciA1SCLKPin;
    type STE = UsciA1STEPin;
}

impl SpiUsci for pac::E_USCI_B0 {
    type MISO = UsciB0MISOPin;
    type MOSI = UsciB0MOSIPin;
    type SCLK = UsciB0SCLKPin;
    type STE = UsciB0STEPin;
}

impl SpiUsci for pac::E_USCI_B1 {
    type MISO = UsciB1MISOPin;
    type MOSI = UsciB1MOSIPin;
    type SCLK = UsciB1SCLKPin;
    type STE = UsciB1STEPin;
}

// Allows a GPIO pin to be converted into an SPI object
macro_rules! impl_spi_pin {
    ($struct_name: ident, $port: ty, $pin: ty) => {
        impl<DIR> From<Pin<$port, $pin, Alternate1<DIR>>> for $struct_name {
            #[inline(always)]
            fn from(_val: Pin<$port, $pin, Alternate1<DIR>>) -> Self {
                $struct_name
            }
        }
    };
}

/// SPI MISO pin for eUSCI A0
pub struct UsciA0MISOPin;
impl_spi_pin!(UsciA0MISOPin, P1, Pin6);

/// SPI MOSI pin for eUSCI A0
pub struct UsciA0MOSIPin;
impl_spi_pin!(UsciA0MOSIPin, P1, Pin7);

/// SPI SCLK pin for eUSCI A0
pub struct UsciA0SCLKPin;
impl_spi_pin!(UsciA0SCLKPin, P1, Pin5);

/// SPI STE pin for eUSCI A0
pub struct UsciA0STEPin;
impl_spi_pin!(UsciA0STEPin, P1, Pin4);

/// SPI MISO pin for eUSCI A1
pub struct UsciA1MISOPin;
impl_spi_pin!(UsciA1MISOPin, P4, Pin2);

/// SPI MOSI pin for eUSCI A1
pub struct UsciA1MOSIPin;
impl_spi_pin!(UsciA1MOSIPin, P4, Pin3);

/// SPI SCLK pin for eUSCI A1
pub struct UsciA1SCLKPin;
impl_spi_pin!(UsciA1SCLKPin, P4, Pin1);
/// SPI STE pin for eUSCI A1
pub struct UsciA1STEPin;
impl_spi_pin!(UsciA1STEPin, P4, Pin0);

/// SPI MISO pin for eUSCI B0
pub struct UsciB0MISOPin;
impl_spi_pin!(UsciB0MISOPin, P1, Pin3);

/// SPI MOSI pin for eUSCI B0
pub struct UsciB0MOSIPin;
impl_spi_pin!(UsciB0MOSIPin, P1, Pin2);

/// SPI SCLK pin for eUSCI B0
pub struct UsciB0SCLKPin;
impl_spi_pin!(UsciB0SCLKPin, P1, Pin1);

/// SPI STE pin for eUSCI B0
pub struct UsciB0STEPin;
impl_spi_pin!(UsciB0STEPin, P1, Pin0);

/// SPI MISO pin for eUSCI B1
pub struct UsciB1MISOPin;
impl_spi_pin!(UsciB1MISOPin, P4, Pin7);

/// SPI MOSI pin for eUSCI B1
pub struct UsciB1MOSIPin;
impl_spi_pin!(UsciB1MOSIPin, P4, Pin6);

/// SPI SCLK pin for eUSCI B1
pub struct UsciB1SCLKPin;
impl_spi_pin!(UsciB1SCLKPin, P4, Pin5);

/// SPI STE pin for eUSCI B1
pub struct UsciB1STEPin;
impl_spi_pin!(UsciB1STEPin, P4, Pin4);

/// Configuration object for an eUSCI peripheral being set up for SPI mode.
pub struct SpiConfig<USCI: SpiUsci, ROLE>{
    usci: USCI, 
    ctlw0: UcxSpiCtw0,
    prescaler: u16,
    _phantom: PhantomData<ROLE>,
}

impl<USCI: SpiUsci> SpiConfig<USCI, RoleNotSet> {
    /// Begin configuring an EUSCI peripheral for SPI mode.
    pub fn new(usci: USCI, mode: Mode, msb_first: bool) -> Self {
        let ctlw0 = UcxSpiCtw0{ 
            ucckph: match mode.phase {
                Phase::CaptureOnFirstTransition => true,
                Phase::CaptureOnSecondTransition => false,
            },
            ucckpl: match mode.polarity {
                Polarity::IdleLow => false,
                Polarity::IdleHigh => true,
            },
            ucmsb: msb_first,
            ucsync: true,
            ucswrst: true,
            // UCSTEM = 1 isn't useful for us, since the STE acts like a CS pin in this case, but 
            // it asserts and de-asserts after each byte automatically, and unfortunately 
            // ehal::SpiBus requires support for multi-byte transactions. 
            ucstem: false, 
            uc7bit: false, // Not supported
            ..Default::default()
        };

        Self { usci, ctlw0, prescaler: 0, _phantom: PhantomData }
    }
    /// This device will act as a slave on the SPI bus.
    pub fn as_slave(mut self) -> SpiConfig<USCI, Slave> {
        self.ctlw0.ucmst = false;
        // UCSSEL is 'don't care' in slave mode
        SpiConfig {usci: self.usci, ctlw0: self.ctlw0, prescaler: self.prescaler, _phantom: PhantomData}
    }
    /// This device will act as a master on the SPI bus, deriving SCLK from SMCLK.
    pub fn as_master_using_smclk(mut self, _smclk: &Smclk, clk_div: u16) -> SpiConfig<USCI, Master> {
        self.ctlw0.ucmst = true;
        self.ctlw0.ucssel = Ucssel::Smclk;
        self.prescaler = clk_div;
        SpiConfig {usci: self.usci, ctlw0: self.ctlw0, prescaler: self.prescaler, _phantom: PhantomData}
    }
    /// This device will act as a master on the SPI bus, deriving SCLK from ACLK.
    pub fn as_master_using_aclk(mut self, _aclk: &Aclk, clk_div: u16) -> SpiConfig<USCI, Master> {
        self.ctlw0.ucmst = true;
        self.ctlw0.ucssel = Ucssel::Aclk;
        self.prescaler = clk_div;
        SpiConfig {usci: self.usci, ctlw0: self.ctlw0, prescaler: self.prescaler, _phantom: PhantomData}
    }
}
impl<USCI: SpiUsci> SpiConfig<USCI, Master> {
    /// For an SPI bus with more than one master. 
    /// The STE pin is used by the other master to turn SCLK and MOSI high impedance, so the other master can talk on the bus.
    pub fn multi_master_bus<MOSI, MISO, SCLK, STE>(mut self, _miso: MISO, _mosi: MOSI, _sclk: SCLK, _ste: STE, ste_pol: StePolarity) -> Spi<USCI> 
    where MOSI: Into<USCI::MOSI>, MISO: Into<USCI::MISO>, SCLK: Into<USCI::SCLK>, STE: Into<USCI::STE> {
        self.ctlw0.ucmode = ste_pol.into();
        self.configure_hw();
        Spi(PhantomData)
    }
    /// For an SPI bus with a single master. 
    /// SCLK and MOSI are always outputs. The STE pin is not required.
    pub fn single_master_bus<MOSI, MISO, SCLK>(mut self, _miso: MISO, _mosi: MOSI, _sclk: SCLK) -> Spi<USCI>
    where MOSI: Into<USCI::MOSI>, MISO: Into<USCI::MISO>, SCLK: Into<USCI::SCLK> {
        self.ctlw0.ucmode = Ucmode::ThreePinSPI;
        self.configure_hw();
        Spi(PhantomData)
    }
}
impl<USCI: SpiUsci> SpiConfig<USCI, Slave> {
    /// For an SPI bus with more than one slave. 
    /// The STE pin is used to turn MISO high impedance, so other slaves can talk on the bus.
    pub fn shared_bus<MOSI, MISO, SCLK, STE>(mut self, _miso: MISO, _mosi: MOSI, _sclk: SCLK, _ste: STE, ste_pol: StePolarity) -> SpiSlave<USCI> 
    where MOSI: Into<USCI::MOSI>, MISO: Into<USCI::MISO>, SCLK: Into<USCI::SCLK>, STE: Into<USCI::STE> {
        self.ctlw0.ucmode = ste_pol.into();
        self.configure_hw();
        SpiSlave(self.usci)
    }
    /// For an SPI bus where this device is the only slave.
    /// MOSI is always an output. 
    pub fn exclusive_bus<MOSI, MISO, SCLK>(mut self, _miso: MISO, _mosi: MOSI, _sclk: SCLK) -> SpiSlave<USCI> 
    where MOSI: Into<USCI::MOSI>, MISO: Into<USCI::MISO>, SCLK: Into<USCI::SCLK> {
        self.ctlw0.ucmode = Ucmode::ThreePinSPI;
        self.configure_hw();
        SpiSlave(self.usci)
    }
}
impl<USCI: SpiUsci, ROLE> SpiConfig<USCI, ROLE> {
    #[inline]
    fn configure_hw(&self) {
        self.usci.ctw0_set_rst();

        self.usci.ctw0_wr(&self.ctlw0);
        self.usci.brw_wr(self.prescaler);
        self.usci.uclisten_clear();

        self.usci.ctw0_clear_rst();

        self.usci.clear_transmit_interrupt();
        self.usci.clear_receive_interrupt();
    }
}

/// Typestate for an SPI bus whose role has not yet been chosen.
pub struct RoleNotSet;
/// Typestate for an SPI bus being configured as a master device.
pub struct Master;
/// Typestate for an SPI bus being configured as a slave device.
pub struct Slave;

/// The polarity of the STE pin.
pub enum StePolarity {
    /// This device is enabled when STE is high.
    EnabledWhenHigh = 0b01,
    /// This device is enabled when STE is low.
    EnabledWhenLow = 0b10,
}
impl From<StePolarity> for Ucmode {
    fn from(value: StePolarity) -> Self {
        match value {
            StePolarity::EnabledWhenHigh => Ucmode::FourPinSPI1,
            StePolarity::EnabledWhenLow  => Ucmode::FourPinSPI0,
        }
    }
}

/// An eUSCI peripheral that has been configured into an SPI slave.
pub struct SpiSlave<USCI: SpiUsci>(USCI);
impl<USCI: SpiUsci> SpiSlave<USCI> {
    /// Enable Rx interrupts, which fire when a byte is ready to be read
    #[inline(always)]
    pub fn set_rx_interrupt(&mut self) {
        self.0.set_receive_interrupt();
    }

    /// Disable Rx interrupts, which fire when a byte is ready to be read
    #[inline(always)]
    pub fn clear_rx_interrupt(&mut self) {
        self.0.clear_receive_interrupt();
    }

    /// Enable Tx interrupts, which fire when the transmit buffer is empty
    #[inline(always)]
    pub fn set_tx_interrupt(&mut self) {
        self.0.set_transmit_interrupt();
    }

    /// Disable Tx interrupts, which fire when the transmit buffer is empty
    #[inline(always)]
    pub fn clear_tx_interrupt(&mut self) {
        self.0.clear_transmit_interrupt();
    }

    /// Read the byte in the Rx buffer, waiting if necessary.
    #[inline]
    pub fn read(&mut self) -> Result<u8, SpiReadErr> {
        while !self.0.receive_flag() {
            msp430::asm::nop();
        }
        if self.0.overrun_flag() {
            return Err(SpiReadErr::Overrun(self.0.rxbuf_rd()));
        }
        Ok(self.0.rxbuf_rd())
    }

    /// Read the byte in the Rx buffer, without checking if the Rx buffer is ready.
    /// Useful when you already know the buffer is ready (e.g. an Rx interrupt was triggered).
    /// # Safety
    /// May read invalid data if RXIFG bit is not ready.
    #[inline]
    pub unsafe fn read_unchecked(&mut self) -> Result<u8, SpiReadErr> {
        if self.0.overrun_flag() {
            return Err(SpiReadErr::Overrun(self.0.rxbuf_rd()));
        }
        Ok(self.0.rxbuf_rd())
    }

    /// Write a byte into the Tx buffer, waiting if necessary. Returns immediately.
    #[inline]
    pub fn write(&mut self, byte: u8) {
        while !self.0.transmit_flag() {
            msp430::asm::nop();
        }
        self.0.txbuf_wr(byte);
    }

    /// Write a byte into the Tx buffer, without checking if the Tx buffer is empty. Returns immediately.
    /// Useful if you already know the buffer is empty (e.g. a Tx interrupt was triggered)
    /// # Safety
    /// May clobber previous unsent data if the TXIFG bit is not set.
    #[inline(always)]
    pub unsafe fn write_unchecked(&mut self, byte: u8) {
        self.0.txbuf_wr(byte);
    }

    /// Get the source of the interrupt currently being serviced.
    #[inline]
    pub fn interrupt_source(&mut self) -> SpiVector {
        let iv: u16 = self.0.iv_rd();
        match iv {
            0 => SpiVector::None,
            2 => SpiVector::RxBufferFull,
            4 => SpiVector::TxBufferEmpty, 
            _ => unsafe { core::hint::unreachable_unchecked() }, 
        }
    }
}

/// Possible sources for an eUSCI SPI interrupt
#[derive(Debug, Copy, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub enum SpiVector {
    /// No interrupt is currently being serviced.
    None = 0,
    /// The interrupt was caused by the Rx buffer being full.
    RxBufferFull = 2,
    /// The interrupt was caused by the Tx buffer being empty.
    TxBufferEmpty = 4,
}

/// Represents a group of pins configured for SPI communication
pub struct Spi<USCI: SpiUsci>(PhantomData<USCI>);

impl<USCI: SpiUsci> Spi<USCI> {
    /// Enable Rx interrupts, which fire when a byte is ready to be read
    #[inline(always)]
    pub fn set_rx_interrupt(&mut self) {
        let usci = unsafe { USCI::steal() };
        usci.set_receive_interrupt();
    }

    /// Disable Rx interrupts, which fire when a byte is ready to be read
    #[inline(always)]
    pub fn clear_rx_interrupt(&mut self) {
        let usci = unsafe { USCI::steal() };
        usci.clear_receive_interrupt();
    }

    /// Enable Tx interrupts, which fire when the transmit buffer is empty
    #[inline(always)]
    pub fn set_tx_interrupt(&mut self) {
        let usci = unsafe { USCI::steal() };
        usci.set_transmit_interrupt();
    }

    /// Disable Tx interrupts, which fire when the transmit buffer is empty
    #[inline(always)]
    pub fn clear_tx_interrupt(&mut self) {
        let usci = unsafe { USCI::steal() };
        usci.clear_transmit_interrupt();
    }

    /// Writes raw value to Tx buffer with no checks for validity
    /// # Safety
    /// May clobber unsent data still in the buffer
    #[inline(always)]
    pub unsafe fn write_no_check(&mut self, val: u8) {
        let usci = unsafe { USCI::steal() };
        usci.txbuf_wr(val)
    }

    #[inline(always)]
    /// Reads a raw value from the Rx buffer with no checks for validity
    /// # Safety
    /// May read duplicate data
    pub unsafe fn read_no_check(&mut self) -> u8 {
        let usci = unsafe { USCI::steal() };
        usci.rxbuf_rd()
    }

    #[inline(always)]
    /// Change the SPI mode
    pub fn change_mode(&mut self, mode: Mode) {
        let usci = unsafe { USCI::steal() };
        usci.ctw0_set_rst();
        usci.set_spi_mode(mode);
        usci.ctw0_clear_rst();
    }

    fn recv_byte(&mut self) -> nb::Result<u8, SpiErr> {
        let usci = unsafe { USCI::steal() };
        
        if usci.receive_flag() {
            if usci.overrun_flag() {
                Err(nb::Error::Other(SpiErr::Overrun(usci.rxbuf_rd())))
            }
            else {
                Ok(usci.rxbuf_rd())
            }
        } else {
            Err(WouldBlock)
        }
    }

    fn send_byte(&mut self, word: u8) -> nb::Result<(), SpiErr> {
        let usci = unsafe { USCI::steal() };
        if usci.transmit_flag() {
            if usci.framing_flag() {
                return Err(nb::Error::Other(SpiErr::BusConflict));
            }
            usci.txbuf_wr(word);
            Ok(())
        } else {
            Err(WouldBlock)
        }
    }
}

// Embedded-hal requires the error type to be the same between the send and recieve fns...
/// SPI transmit/receive errors
#[derive(Clone, Copy, Debug)]
#[non_exhaustive]
pub enum SpiErr {
    /// Data in the recieve buffer was overwritten before it was read. The contained data is the new contents of the recieve buffer.
    Overrun(u8),
    /// Another master interrupted our previous transaction using the STE pin. Only occurs in 4-pin master mode.
    BusConflict,
}

// ...But for the SpiSlave we can separate them to make error handling easier.
/// SPI receive errors
#[derive(Clone, Copy, Debug)]
pub enum SpiReadErr {
    /// Data in the recieve buffer was overwritten before it was read. The contained data is the new contents of the recieve buffer.
    Overrun(u8),
}
impl From<SpiReadErr> for SpiErr {
    fn from(err: SpiReadErr) -> Self {
        match err {
            SpiReadErr::Overrun(byte) => SpiErr::Overrun(byte),
        }
    }
}

mod ehal1 {
    use embedded_hal::spi::{Error, ErrorType, SpiBus};
    use nb::block;
    use super::*;

    impl Error for SpiErr {
        fn kind(&self) -> embedded_hal::spi::ErrorKind {
            match self {
                SpiErr::Overrun(_) => embedded_hal::spi::ErrorKind::Overrun,
                SpiErr::BusConflict => embedded_hal::spi::ErrorKind::ModeFault,
            }
        }
    }

    impl<USCI: SpiUsci> ErrorType for Spi<USCI> {
        type Error = SpiErr;
    }

    impl<USCI: SpiUsci> SpiBus for Spi<USCI> {
        /// Send dummy packets (`0x00`) on MOSI so the slave can respond on MISO. Store the response in `words`.
        /// 
        /// ## Errors
        /// - Returns `SpiErr::OverrunError` if there was already a byte in the Rx buffer that was not read.
        /// - Returns `SpiErr::BusConflict` if another master interrupted our transmission. Only occurs on a multi-master bus.
        fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
            for word in words {
                block!(self.send_byte(0x00))?;
                *word = block!(self.recv_byte())?;
            }
            Ok(())
        }
    
        /// Write `words` to the slave, ignoring all the incoming words.
        ///
        /// ## Errors
        /// - Returns `SpiErr::BusConflict` if another master interrupted our transmission. Only occurs on a multi-master bus.
        fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
            for word in words {
                block!(self.send_byte(*word))?;
                let _ = block!(self.recv_byte());
            }
            Ok(())
        }
    
        /// Write and read simultaneously. `write` is written to the slave on MOSI and
        /// words received on MISO are stored in `read`.
        ///
        /// If `write` is longer than `read`, then after `read` is full any subsequent incoming words will be discarded. 
        /// If `read` is longer than `write`, then dummy packets of `0x00` are sent until `read` is full.
        /// ## Errors
        /// - Returns `SpiErr::OverrunError` if there was already a byte in the Rx buffer that was not read.
        /// - Returns `SpiErr::BusConflict` if another master interrupted our transmission. Only occurs on a multi-master bus.
        fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
            let mut read_bytes = read.iter_mut();
            let mut write_bytes = write.iter();
            const DUMMY_WRITE: u8 = 0x00;
            let mut dummy_read = 0;

            // Pair up read and write bytes (inserting dummy values as necessary) until everything's sent
            loop {
                let (rd, wr) = match (read_bytes.next(), write_bytes.next()) {
                    (Some(rd), Some(wr)) => (rd, wr),
                    (Some(rd), None    ) => (rd, &DUMMY_WRITE),
                    (None,     Some(wr)) => (&mut dummy_read, wr),
                    (None,     None    ) => break,
                };

                block!(self.send_byte(*wr))?;
                *rd = block!(self.recv_byte())?;
            }
            Ok(())
        }
    
        /// Write and read simultaneously. The contents of `words` are
        /// written to the slave, and the received words are stored into the same
        /// `words` buffer, overwriting it.
        /// ## Errors
        /// - Returns `SpiErr::OverrunError` if there was already a byte in the Rx buffer that was not read.
        /// - Returns `SpiErr::BusConflict` if another master interrupted our transmission. Only occurs on a multi-master bus.
        fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
            for word in words {
                block!(self.send_byte(*word))?;
                *word = block!(self.recv_byte())?;
            }
            Ok(())
        }
    
        /// ## Errors
        /// - Returns `SpiErr::BusConflict` if another master interrupted a transmission. Only occurs on a multi-master bus.
        fn flush(&mut self) -> Result<(), Self::Error> {
            // I would usually do this by checking the UCBUSY bit, but 
            // it seems to be missing from the (SPI version of the) PAC...
            let usci = unsafe { USCI::steal() };
            while usci.is_busy() {}
            if usci.framing_flag() {
                return Err(SpiErr::BusConflict);
            }
            Ok(())
        }
    }
}

mod ehal_nb1 {
    use embedded_hal_nb::{nb, spi::FullDuplex};
    use super::*;

    impl<USCI: SpiUsci> FullDuplex<u8> for Spi<USCI> {
        fn read(&mut self) -> nb::Result<u8, Self::Error> {
            self.recv_byte()
        }
    
        fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
            self.send_byte(word)
        }
    }
}

#[cfg(feature = "embedded-hal-02")]
mod ehal02 {
    use embedded_hal_02::spi::FullDuplex;
    use super::*;

    impl<USCI: SpiUsci> FullDuplex<u8> for Spi<USCI> {
        type Error = SpiErr;
        fn read(&mut self) -> nb::Result<u8, Self::Error> {
            self.recv_byte()
        }

        fn send(&mut self, word: u8) -> nb::Result<(), Self::Error> {
            self.send_byte(word)
        }
    }

    // Implementing FullDuplex above gets us a blocking write and transfer implementation for free
    impl<USCI: SpiUsci> embedded_hal_02::blocking::spi::write::Default<u8> for Spi<USCI> {}
    impl<USCI: SpiUsci> embedded_hal_02::blocking::spi::transfer::Default<u8> for Spi<USCI> {}
}
