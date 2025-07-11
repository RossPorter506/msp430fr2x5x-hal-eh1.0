//! I2C
//!
//! Peripherals eUSCI_B0 and eUSCI_B1 can be used for I2C communication.
//!
//! Begin by calling [`I2cConfig::new()`]. Once configured an [`I2cDevice`] will be returned.
//! 
//! [`I2cDevice`] supports three different modes - Slave, Single Master (suitable for I2C buses with no other master devices), 
//! and Multi-Master (suitable for buses with multiple masters, or when a device must act as both master and slave). 
//! 
//! ## Slave Modes
//! In slave mode the peripheral responds to requests from master devices. In slave and multi-master modes the 'own address' is 
//! treated as 7-bit should a `u8` be provided, and 10-bit if a `u16` is provided. Both polling and interrupt-based methods are 
//! available, though interrupt-based is recommended for slave devices, as the slave can 'fall behind' and lose information if 
//! polling is not done frequently enough. Both methods use [`read_rx_buf()`](I2cDevice<Slave>::read_rx_buf()) and 
//! [`write_tx_buf()`](I2cDevice<MultiMaster>::write_tx_buf()) to write and read from the bus. Interrupt-based implementations 
//! rely on interrupts and [`interrupt_source()`](I2cDevice::interrupt_source()) to inform about events on the bus. 
//! A polling-based implementation instead uses periodic calls to [`poll()`](I2cDevice<Slave>::poll()) to monitor for bus events.
//! 
//! ## Master Modes
//! Both Single Master and Multi-Master modes implement a blocking master implementation through the embedded-hal 
//! [`I2c`](embedded_hal::i2c::I2c) trait. Passing a `u8` address to these methods uses 7-bit addressing, passing a `u16` uses 
//! 10-bit addressing. Interrupt-based methods are also provided using the same [`read_rx_buf()`](I2cDevice<Slave>::read_rx_buf()), 
//! [`write_tx_buf()`](I2cDevice<MultiMaster>::write_tx_buf()), and [`interrupt_source()`](I2cDevice::interrupt_source()) as above.
//! 
//! ### Single Master Mode
//! Single master mode provides simplified error handling and ergonomics at the cost of being unsuitable for multi-master buses. 
//! Single master mode does not handle bus arbitration, so even if the device is not expected to be addressed as a slave it is not
//! suitable for use on an I2C bus with multiple masters.
//! 
//! ### Multi-Master Mode
//! Multi-Master mode handles both bus arbitration, and optionally being addressed as a slave device. 
//! If the device is addressed as a slave the hardware automatically fails over into slave mode, and the slave transaction must 
//! be concluded before the device is returned to master mode with [`return_to_master()`](`I2cDevice<MultiMaster>::return_to_master()`). 
//! If the device merely loses arbitration [`return_to_master()`](`I2cDevice<MultiMaster>::return_to_master()`) may be called immediately.
//! 
//! Pins used:
//!
//! eUSCI_B0: {SCL: `P1.3`, SDA: `P1.2`}. `P1.1` can optionally be used as an external clock source in master modes.
//!
//! eUSCI_B1: {SCL: `P4.7`, SDA: `P4.6`}. `P4.5` can optionally be used as an external clock source in master modes.
//!

use crate::clock::{Aclk, Smclk};
use crate::gpio::{Pin1, Pin5};
use crate::hw_traits::eusci::I2CUcbIfgOut;
use crate::{
    gpio::{Alternate1, Pin, Pin2, Pin3, Pin6, Pin7, P1, P4},
    hw_traits::eusci::{EUsciI2C, UcbCtlw0, UcbCtlw1, UcbI2coa, UcbIFG, UcbIe, Ucmode, Ucssel},
    pac,
};

use core::convert::Infallible;
use core::marker::PhantomData;
use embedded_hal::i2c::{AddressMode, Operation, SevenBitAddress, TenBitAddress};
use msp430::asm;
use nb::Error::{WouldBlock, Other};
/// Enumerates the two I2C addressing modes: 7-bit and 10-bit.
/// 
/// Used internally by the HAL.
#[derive(Clone, Copy)]
pub enum AddressingMode {
    /// 7-bit addressing mode
    SevenBit = 0,
    /// 10-bit addressing mode
    TenBit = 1,
}
impl From<AddressingMode> for bool {
    #[inline(always)]
    fn from(f: AddressingMode) -> bool {
        match f {
            AddressingMode::SevenBit => false,
            AddressingMode::TenBit => true,
        }
    }
}

/// Configure between master receiver and master transmitter modes
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum TransmissionMode {
    /// master receiver mode
    Receive = 0,
    /// master transmitter mode
    Transmit = 1,
}
impl From<TransmissionMode> for bool {
    #[inline(always)]
    fn from(f: TransmissionMode) -> bool {
        match f {
            TransmissionMode::Receive => false,
            TransmissionMode::Transmit => true,
        }
    }
}

pub use crate::hw_traits::eusci::Ucglit as GlitchFilter;

///Struct used to configure a I2C bus
pub struct I2cConfig<USCI: I2cUsci, CLKSRC, ROLE> {
    usci: USCI,
    divisor: u16,

    // Register configs
    ctlw0: UcbCtlw0,
    ctlw1: UcbCtlw1,
    i2coa0: UcbI2coa,
    i2coa1: UcbI2coa,
    i2coa2: UcbI2coa,
    i2coa3: UcbI2coa,
    ie: UcbIe,
    ifg: UcbIFG,
    clk_src: PhantomData<CLKSRC>,
    role: PhantomData<ROLE>,
}

/// Marks a usci capable of I2C communication
pub trait I2cUsci: EUsciI2C {
    /// I2C SCL pin
    type ClockPin;
    /// I2C SDA pin
    type DataPin;
    /// I2C external clock source pin. Only necessary if UCLKI is selected as a clock source.
    type ExternalClockPin;
}
impl I2cUsci for pac::E_USCI_B0 {
    type ClockPin = UsciB0SCLPin;
    type DataPin = UsciB0SDAPin;
    type ExternalClockPin = UsciB0UCLKIPin;
}
impl I2cUsci for pac::E_USCI_B1 {
    type ClockPin = UsciB1SCLPin;
    type DataPin = UsciB1SDAPin;
    type ExternalClockPin = UsciB1UCLKIPin;
}

// Allows a GPIO pin to be converted into an I2C object
macro_rules! impl_i2c_pin {
    ($struct_name: ident, $port: ty, $pin: ty) => {
        impl<DIR> From<Pin<$port, $pin, Alternate1<DIR>>> for $struct_name {
            #[inline(always)]
            fn from(_val: Pin<$port, $pin, Alternate1<DIR>>) -> Self {
                $struct_name
            }
        }
    };
}

/// I2C SCL pin for eUSCI B0
pub struct UsciB0SCLPin;
impl_i2c_pin!(UsciB0SCLPin, P1, Pin3);

/// I2C SDA pin for eUSCI B0
pub struct UsciB0SDAPin;
impl_i2c_pin!(UsciB0SDAPin, P1, Pin2);

/// UCLKI pin for eUSCI B0. Used as an external clock source.
pub struct UsciB0UCLKIPin;
impl_i2c_pin!(UsciB0UCLKIPin, P1, Pin1);

/// I2C SCL pin for eUSCI B1
pub struct UsciB1SCLPin;
impl_i2c_pin!(UsciB1SCLPin, P4, Pin7);

/// I2C SDA pin for eUSCI B1
pub struct UsciB1SDAPin;
impl_i2c_pin!(UsciB1SDAPin, P4, Pin6);

/// UCLKI pin for eUSCI B1. Used as an external clock source.
pub struct UsciB1UCLKIPin;
impl_i2c_pin!(UsciB1UCLKIPin, P4, Pin5);

/// Typestate for an I2C bus configuration with no clock source selected
pub struct NoClockSet;
/// Typestate for an I2C bus configuration with a clock source selected
pub struct ClockSet;

/// Typestate for an I2C bus that has not yet been assigned a role.
pub struct NoRoleSet;

// Marker trait for I2C roles.
trait I2cRole {
    type ErrType;
}
trait RoleMaster: I2cRole {}
trait RoleSlave: I2cRole {}
/// Typestate for an I2C bus being configured as a master on a bus with no other master devices present.
pub struct SingleMaster;
impl I2cRole for SingleMaster {
    type ErrType = I2cSingleMasterErr;
}
impl RoleMaster for SingleMaster {}

/// Typestate for an I2C bus being configured as a slave.
pub struct Slave;
impl I2cRole for Slave {
    type ErrType = Infallible;
}
impl RoleSlave for Slave {}

/// Typestate for an I2C bus being configured as a master on a bus that has other master devices present.
pub struct MultiMaster;
impl I2cRole for MultiMaster {
    type ErrType = I2cMultiMasterErr;
}
impl RoleMaster for MultiMaster {}
impl RoleSlave for MultiMaster {}

macro_rules! return_self_config {
    ($self: ident) => {
        I2cConfig {
            usci:    $self.usci,
            divisor: $self.divisor,
            ctlw0:   $self.ctlw0,
            ctlw1:   $self.ctlw1,
            i2coa0:  $self.i2coa0,
            i2coa1:  $self.i2coa1,
            i2coa2:  $self.i2coa2,
            i2coa3:  $self.i2coa3,
            ie:      $self.ie,
            ifg:     $self.ifg,
            clk_src: PhantomData,
            role: PhantomData,
        }
    };
}

impl<USCI: I2cUsci> I2cConfig<USCI, NoClockSet, NoRoleSet> {
    /// Begin configuration of an eUSCI peripheral as an I2C device.
    pub fn new(usci: USCI, deglitch_time: GlitchFilter) -> I2cConfig<USCI, NoClockSet, NoRoleSet> {
        let ctlw0 = UcbCtlw0 {
            ucsync: true,
            ucswrst: true,
            ucmode: Ucmode::I2CMode,
            ..Default::default()
        };

        let ctlw1 = UcbCtlw1 {
            ucglit: deglitch_time,
            ..Default::default()
        };

        let i2coa0 = UcbI2coa::default();
        let i2coa1 = UcbI2coa::default();
        let i2coa2 = UcbI2coa::default();
        let i2coa3 = UcbI2coa::default();
        let ie = UcbIe::default();
        let ifg = UcbIFG::default();

        I2cConfig {
            usci,
            divisor: 1,
            ctlw0,
            ctlw1,
            i2coa0,
            i2coa1,
            i2coa2,
            i2coa3,
            ie,
            ifg,
            clk_src: PhantomData,
            role: PhantomData,
        }
    }
    /// Configure this EUSCI peripheral as an I2C master on a bus with no other master devices.
    pub fn as_single_master(mut self) -> I2cConfig<USCI, NoClockSet, SingleMaster> {
        self.ctlw0.ucmst = true;

        return_self_config!(self)
    }

    /// Configure this EUSCI peripheral as an I2C master on a bus with other master devices. 
    /// These masters may contest the bus and/or address this device as a slave if an address is provided.
    pub fn as_multi_master<TenOrSevenBit>(mut self, own_address: Option<TenOrSevenBit>) -> I2cConfig<USCI, NoClockSet, MultiMaster> 
    where TenOrSevenBit: AddressType {
        self.ctlw0 = UcbCtlw0 { 
            uca10: TenOrSevenBit::addr_type().into(), 
            ucmst: true, 
            ucmm: true,
            ..self.ctlw0
        };

        // Enable address checking only if address provided
        // Note: If you add support for the other 3 own addresses (or the mask) you will also have to upgrade the logic for checking 
        // that the peripheral isn't addressing itself, i.e. I2cMultiMasterErr::TriedAddressingSelf
        if let Some(addr) = own_address {
            self.i2coa0 = UcbI2coa {
                ucgcen: false, // Not yet implemented
                ucoaen: true,
                i2coa0: addr.into(),
            };
        }

        return_self_config!(self)
    }

    /// Configure this eUSCI peripheral as an I2C slave.
    pub fn as_slave<TenOrSevenBit>(mut self, own_address: TenOrSevenBit) -> I2cConfig<USCI, ClockSet, Slave> 
    where TenOrSevenBit: AddressType {
        self.ctlw0.uca10 = TenOrSevenBit::addr_type().into();

        self.i2coa0 = UcbI2coa {
            ucgcen: false, // Not yet implemented
            ucoaen: true,
            i2coa0: own_address.into(),
        };

        return_self_config!(self)
    }
}

#[allow(private_bounds)]
impl<USCI: I2cUsci, ROLE: I2cRole> I2cConfig<USCI, NoClockSet, ROLE> {
    /// Configures this peripheral to use SMCLK
    #[inline]
    pub fn use_smclk(mut self, _smclk: &Smclk, clk_divisor: u16) -> I2cConfig<USCI, ClockSet, ROLE> {
        self.ctlw0.ucssel = Ucssel::Smclk;
        self.divisor = clk_divisor;
        return_self_config!(self)
    }
    /// Configures this peripheral to use ACLK
    #[inline]
    pub fn use_aclk(mut self, _aclk: &Aclk, clk_divisor: u16) -> I2cConfig<USCI, ClockSet, ROLE> {
        self.ctlw0.ucssel = Ucssel::Aclk;
        self.divisor = clk_divisor;
        return_self_config!(self)
    }
    /// Configures this peripheral to use UCLK
    #[inline]
    pub fn use_uclk<Pin: Into<USCI::ExternalClockPin> >(mut self, _uclk: Pin, clk_divisor: u16) -> I2cConfig<USCI, ClockSet, ROLE> {
        self.ctlw0.ucssel = Ucssel::Uclk;
        self.divisor = clk_divisor;
        return_self_config!(self)
    }
}

#[allow(private_bounds)]
impl<USCI: I2cUsci, RoleSet: I2cRole> I2cConfig<USCI, ClockSet, RoleSet> {
    /// Performs hardware configuration
    #[inline]
    fn configure_regs(&self) {
        self.usci.ctw0_set_rst();

        self.usci.ctw0_wr(&self.ctlw0);
        self.usci.ctw1_wr(&self.ctlw1);
        self.usci.i2coa_wr(0, &self.i2coa0);
        self.usci.i2coa_wr(1, &self.i2coa1);
        self.usci.i2coa_wr(2, &self.i2coa2);
        self.usci.i2coa_wr(3, &self.i2coa3);
        self.usci.ie_wr(&self.ie);
        self.usci.ifg_wr(&self.ifg);

        self.usci.brw_wr(self.divisor);
        self.usci.tbcnt_wr(0);

        self.usci.ctw0_clear_rst();
    }
}

impl<USCI: I2cUsci> I2cConfig<USCI, ClockSet, SingleMaster> {
    /// Performs hardware configuration and creates the I2C bus
    #[inline(always)]
    pub fn configure<SCL, SDA>(self, _scl: SCL, _sda: SDA) -> I2cDevice<USCI, SingleMaster> 
    where SCL: Into<USCI::ClockPin>, SDA: Into<USCI::DataPin> {
        self.configure_regs();
        I2cDevice{ usci: self.usci, role: PhantomData }
    }
}
impl<USCI: I2cUsci> I2cConfig<USCI, ClockSet, MultiMaster> {
    /// Performs hardware configuration and creates the I2C bus
    #[inline(always)]
    pub fn configure<SCL, SDA>(self, _scl: SCL, _sda: SDA) -> I2cDevice<USCI, MultiMaster> 
    where SCL: Into<USCI::ClockPin>, SDA: Into<USCI::DataPin> {
        self.configure_regs();
        I2cDevice{ usci: self.usci, role: PhantomData }
    }
}
impl<USCI: I2cUsci> I2cConfig<USCI, ClockSet, Slave> {
    /// Performs hardware configuration and creates the I2C bus
    #[inline(always)]
    pub fn configure<SCL, SDA>(self, _scl: SCL, _sda: SDA) -> I2cDevice<USCI, Slave> 
    where SCL: Into<USCI::ClockPin>, SDA: Into<USCI::DataPin> {
        self.configure_regs();
        I2cDevice{ usci: self.usci, role: PhantomData }
    }
}

/// An eUSCI peripheral that has been configured as an I2C device. 
/// `ROLE` determines whether it acts as a slave, single master, or multi-master device.
pub struct I2cDevice<USCI, ROLE> {
    usci: USCI,
    role: PhantomData<ROLE>,
}
impl<USCI: I2cUsci, ROLE> I2cDevice<USCI, ROLE> {
    /// Get the event that triggered the current interrupt. Used as part of the interrupt-based interface.
    pub fn interrupt_source(&mut self) -> I2cVector {
        use I2cVector::*;
        match self.usci.iv_rd() {
            0x00 => None,
            0x02 => ArbitrationLost,
            0x04 => NackReceived,
            0x06 => StartReceived,
            0x08 => StopReceived,
            0x0A => Slave3RxBufFull,
            0x0C => Slave3TxBufEmpty,
            0x0E => Slave2RxBufFull,
            0x10 => Slave2TxBufEmpty,
            0x12 => Slave1RxBufFull,
            0x14 => Slave1TxBufEmpty,
            0x16 => RxBufFull,
            0x18 => TxBufEmpty,
            0x1A => ByteCounterZero,
            0x1C => ClockLowTimeout,
            0x1E => NinthBitReceived,
            _ => unsafe{ core::hint::unreachable_unchecked() }
        }
    }

    /// Send a NACK on the I2C bus. Only use during a receive operation. Used as part of the non-blocking / interrupt-based interface.
    #[inline(always)]
    pub fn send_nack(&mut self) {
        self.usci.transmit_nack();
    }

    /// Get the number of bytes received/transmitted since the last Start or Repeated Start condition.
    #[inline(always)]
    pub fn byte_count(&mut self) -> u8 {
        self.usci.byte_count()
    }

    /// Set the bits in the interrupt enable register that correspond to the bits set in `intrs`. 
    /// 
    /// This bitmask can be generated using [`I2cInterruptBits`].
    #[inline(always)]
    pub fn set_interrupts<I2cInterruptBits>(&mut self, intrs: I2cInterruptBits) 
    where I2cInterruptBits: Into<u16> {
        self.usci.ie_set(intrs.into())
    }
    /// Clear the bits in the interrupt enable register that correspond to the bits *set* in `intrs`. 
    /// 
    /// This bitmask can be generated using [`I2cInterruptBits`].
    #[inline(always)]
    pub fn clear_interrupts<I2cInterruptBits>(&mut self, intrs: I2cInterruptBits) 
    where I2cInterruptBits: Into<u16> {
        self.usci.ie_clr(!(intrs.into()))
    }
}
#[allow(private_bounds)]
impl<USCI: I2cUsci, MASTERS: RoleMaster> I2cDevice<USCI, MASTERS> {
    /// Set the address of the slave you want to talk to. Used as part of the non-blocking interface. 
    /// If a `u8` address is provided then the peripheral is put into 7-bit addressing mode. 
    /// A `u16` address puts the peripheral into 10-bit addressing mode. 
    #[inline]
    pub fn set_target_address<TenOrSevenBit: AddressType>(&mut self, address: TenOrSevenBit) {
        self.set_addressing_mode(TenOrSevenBit::addr_type());
        self.usci.i2csa_wr(address.into());
    }

    #[inline(always)]
    fn set_addressing_mode(&mut self, mode: AddressingMode) {
        self.usci.set_ucsla10(mode.into())
    }

    /// Manually set the transmission mode. Used as part of the non-blocking interface.
    #[inline(always)]
    pub fn set_transmission_mode(&mut self, mode: TransmissionMode) {
        self.usci.set_uctr(mode.into())
    }

    /// Manually send a start condition and address byte. Used as part of the non-blocking interface.
    #[inline(always)]
    pub fn send_start(&mut self) {
        self.usci.transmit_start();
    }

    /// Manually schedule a stop condition to be sent. Used as part of the non-blocking interface.
    /// 
    /// The stop will be sent after the current byte operation. If the bus stalls waiting for the Rx or Tx buffer then the stop won't be sent until that condition is dealt with. 
    #[inline(always)]
    pub fn schedule_stop(&mut self) {
        self.usci.transmit_stop();
        self.usci.ifg_rst(); // For some reason the TXIFG flag needs to be cleared between transactions
    }

    /// Blocking read
    // TODO: Arbitration and slave addressing
    fn __read(&mut self, address: u16, buffer: &mut [u8], send_start: bool, send_stop: bool) -> Result<(), I2cMultiMasterErr> {
        // Hardware doesn't support zero byte reads.
        if buffer.is_empty() { return Ok(()) }

        let usci = &mut self.usci;

        if !usci.is_master() {
            return Err(I2cMultiMasterErr::ArbitrationLost);
        }
        let ifg = usci.ifg_rd();
        if ifg.ucalifg() {
            return match ifg.ucsttifg() {
                false => Err(I2cMultiMasterErr::ArbitrationLost),
                true  => Err(I2cMultiMasterErr::AddressedAsSlave),
            }
        }

        // Check if the eUSCI is addressing itself. The hardware isn't capable of this.
        let own_addr_reg = usci.i2coa_rd(0);
        if own_addr_reg.ucoaen && own_addr_reg.i2coa0 == address {
            return Err(I2cMultiMasterErr::TriedAddressingSelf);
        }

        // Clear any flags from previous transactions
        usci.ifg_rst();
        
        usci.i2csa_wr(address);

        if send_start {
            usci.transmit_start();
            // Wait for initial address byte and (N)ACK to complete.
            while usci.uctxstt_rd() {
                asm::nop();
            }
        }

        let len = buffer.len();
        for (idx, byte) in buffer.iter_mut().enumerate() {
            if send_stop && (idx == len - 1) {
                usci.transmit_stop();
            }
            loop {
                let ifg = usci.ifg_rd();
                // If NACK (from initial address packet), send STOP and abort
                if ifg.ucnackifg() {
                    usci.transmit_stop();
                    while usci.uctxstp_rd() {
                        asm::nop();
                    }
                    return Err(I2cMultiMasterErr::GotNACK(idx));
                }
                // If byte recieved
                if ifg.ucrxifg0() {
                    break;
                }
            }
            *byte = usci.ucrxbuf_rd();
        }

        if send_stop {
            while usci.uctxstp_rd() {
                asm::nop();
            }
        }

        Ok(())
    }

    /// Blocking write
    // TODO: Arbitration and slave addressing
    fn __write(&mut self, address: u16, bytes: &[u8], mut send_start: bool, mut send_stop: bool) -> Result<(), I2cMultiMasterErr> {
        // The only way to perform a zero byte write is with a start + stop
        if bytes.is_empty() {
            send_start = true;
            send_stop = true;
        }

        let usci = &mut self.usci;

        if !usci.is_master() {
            return Err(I2cMultiMasterErr::ArbitrationLost);
        }
        let ifg = usci.ifg_rd();
        if ifg.ucalifg() {
            return match ifg.ucsttifg() {
                false => Err(I2cMultiMasterErr::ArbitrationLost),
                true  => Err(I2cMultiMasterErr::AddressedAsSlave),
            }
        }

        // Check if the eUSCI is addressing itself. The hardware isn't capable of this.
        let own_addr_reg = usci.i2coa_rd(0);
        if own_addr_reg.ucoaen && own_addr_reg.i2coa0 == address {
            return Err(I2cMultiMasterErr::TriedAddressingSelf);
        }

        // Clear any flags from previous transactions
        usci.ifg_rst();

        usci.i2csa_wr(address);

        if send_start {
            usci.transmit_start();
        }

        for (idx, &byte) in bytes.iter().enumerate() {
            loop {
                let ifg = usci.ifg_rd();
                if ifg.ucnackifg() {
                    usci.transmit_stop();
                    while usci.uctxstp_rd() {
                        asm::nop();
                    }
                    return Err(I2cMultiMasterErr::GotNACK(idx));
                }
                if ifg.ucalifg() {
                    return match ifg.ucsttifg() {
                        false => Err(I2cMultiMasterErr::ArbitrationLost), // Lost arbitration
                        true  => Err(I2cMultiMasterErr::AddressedAsSlave),// Lost arbitration and the slave address was us
                    }
                }
                if ifg.uctxifg0() {
                    break;
                }
            }
            usci.uctxbuf_wr(byte);
        } 

        while !usci.ifg_rd().uctxifg0() {
            asm::nop();
        }

        if send_stop {
            usci.transmit_stop();
            while usci.uctxstp_rd() {
                // This is mainly for catching NACKs in a zero-byte write
                if usci.ifg_rd().ucnackifg() {
                    return Err(I2cMultiMasterErr::GotNACK(bytes.len()));
                }
            }
        }

        Ok(())
    }

    #[inline]
    fn _is_slave_present<TenOrSevenBit>(&mut self, address: TenOrSevenBit) -> Result<bool, I2cMultiMasterErr> 
    where TenOrSevenBit: AddressType {
        self.set_addressing_mode(TenOrSevenBit::addr_type());
        self.set_transmission_mode(TransmissionMode::Transmit);
        match self.__write(address.into(), &[], true, true) {
            Ok(_) => Ok(true),
            Err(I2cMultiMasterErr::GotNACK(_)) => Ok(false),
            Err(e) => Err(e),
        }
    }
}
#[allow(private_bounds)]
impl<USCI: I2cUsci, SLAVES: RoleSlave> I2cDevice<USCI, SLAVES> {
    /// Check the I2C bus flags for any events that should be dealt with. Returns `Err(WouldBlock)` if no events have occurred yet, otherwise `Ok(I2cEvent)`.
    pub fn poll(&mut self) -> nb::Result<I2cEvent, Infallible> {
        if self.usci.stop_received() {
            self.usci.clear_start_stop_flags();
            return Ok(I2cEvent::Stop);
        }

        match (self.usci.start_received(), self.usci.rxifg0_rd(), self.usci.is_transmitter() & self.usci.txifg0_rd()) {
            (true,  true,  false) => {self.usci.clear_start_flag(); Ok(I2cEvent::WriteStart)},
            (true,  false, true ) => {self.usci.clear_start_flag(); Ok(I2cEvent::ReadStart)},
            (false, true,  false) => Ok(I2cEvent::Write),
            (false, false, true ) => Ok(I2cEvent::Read),
            // Rx buffer filled, then repeated start then Tx buffer empty. (Can't be reverse because empty Tx buf stalls the bus).
            (true,  true,  true ) => Ok(I2cEvent::OverrunWrite), // Don't clear the start flag yet.
            // Start flag but no Rx / Tx events yet. Don't clear the flag yet.
            (_,     false, false) => Err(WouldBlock), 
            // I don't believe this is ever reachable. 
            (false, true,  true ) => unreachable!(), // TODO: Test and replace with unchecked
        }
    }
}
impl<USCI: I2cUsci> I2cDevice<USCI, SingleMaster> {
    /// Check if the Rx buffer is full, if so read it. Used as part of the non-blocking / interrupt-based interface.
    /// 
    /// Returns `Err(WouldBlock)` if the Rx buffer is empty, `Err(GotNACK(n))` if a NACK was received 
    /// (will prevent the Rx buffer from filling), where `n` is the number of 
    /// bytes since the latest Start or Repeated Start condition. otherwise `Ok(n)`.
    #[inline]
    pub fn read_rx_buf(&mut self) -> nb::Result<u8, I2cSingleMasterErr> {
        let ifg = self.usci.ifg_rd();
        if ifg.ucnackifg() {
            return Err(Other(I2cSingleMasterErr::GotNACK(self.usci.byte_count() as usize)));
        }
        if !ifg.ucrxifg0() {
            return Err(WouldBlock);
        }
        Ok(self.usci.ucrxbuf_rd())
    }

    /// Check if the Tx buffer is empty, if so write to it. Used as part of the non-blocking / interrupt-based interface.
    /// 
    /// Returns `Err(WouldBlock)` if the Tx buffer is still full, `Err(GotNACK(n))` if a NACK was received 
    /// (will prevent the Tx buffer from emptying), where `n` is the number of 
    /// bytes since the latest Start or Repeated Start condition. Otherwise returns `Ok(())`.
    #[inline]
    pub fn write_tx_buf(&mut self, byte: u8) -> nb::Result<(), I2cSingleMasterErr>{
        let ifg = self.usci.ifg_rd();
        if ifg.ucnackifg() {
            return Err(Other(I2cSingleMasterErr::GotNACK(self.usci.byte_count() as usize)));
        }
        if !ifg.uctxifg0() {
            return Err(WouldBlock);
        }
        self.usci.uctxbuf_wr(byte);
        Ok(())
    }

    /// Discard impossible to reach errors
    #[inline]
    fn multi_to_single_err(err: I2cMultiMasterErr) -> I2cSingleMasterErr {
        match err {
            I2cMultiMasterErr::GotNACK(n) => I2cSingleMasterErr::GotNACK(n),
            _ => unreachable!(), // TODO: Replace with unchecked after some testing
        }
    }

    #[inline(always)]
    fn _write(&mut self, address: u16, bytes: &[u8], send_start: bool, send_stop: bool) -> Result<(), I2cSingleMasterErr> {
        self.__write(address, bytes, send_start, send_stop).map_err(Self::multi_to_single_err)
    }

    #[inline(always)]
    fn _read(&mut self, address: u16, buffer: &mut [u8], send_start: bool, send_stop: bool) -> Result<(), I2cSingleMasterErr> {
        self.__read(address, buffer, send_start, send_stop).map_err(Self::multi_to_single_err)
    }
    
    /// Checks whether a slave with the specified address is present on the I2C bus.
    /// Sends a zero-byte write and records whether the slave sends an ACK or not.
    /// 
    /// A u8 address will use the 7-bit addressing mode, a u16 address uses 10-bit addressing.
    #[inline(always)]
    pub fn is_slave_present<TenOrSevenBit>(&mut self, address: TenOrSevenBit) -> Result<bool, I2cSingleMasterErr> 
    where TenOrSevenBit: AddressType {
        self._is_slave_present(address).map_err(Self::multi_to_single_err)
    }

    /// blocking write then blocking read
    fn _write_read(&mut self, address: u16, bytes: &[u8], buffer: &mut [u8]) -> Result<(), I2cSingleMasterErr> {
        self.set_transmission_mode(TransmissionMode::Transmit);
        self._write(address, bytes, true, false)?;
        self.set_transmission_mode(TransmissionMode::Receive);
        self._read(address, buffer, true, true)
            .map_err(|e| Self::add_nack_count(e, bytes.len()))
    }

    /// In multi-operation transactions update the NACK byte error count to match *total* bytes sent 
    #[inline]
    fn add_nack_count(err: I2cSingleMasterErr, bytes_already_sent: usize) -> I2cSingleMasterErr {
        match err {
            I2cSingleMasterErr::GotNACK(n) => I2cSingleMasterErr::GotNACK(n + bytes_already_sent),
            e => e,
        }
    }
}
impl<USCI: I2cUsci> I2cDevice<USCI, MultiMaster> {
    /// Check if the Rx buffer is full, if so read it. Used as part of the non-blocking / interrupt-based interface.
    /// 
    /// Returns `Err(WouldBlock)` if the buffer is still full, 
    /// `Err(Other(I2cMultiMasterErr))` if any bus conditions occur that would impede regular operation, or 
    /// `Ok(n)` if data was successfully retreived from the Rx buffer.  
    pub fn read_rx_buf_as_master(&mut self) -> nb::Result<u8, I2cMultiMasterErr> {
        let ifg = self.usci.ifg_rd();
        if ifg.ucalifg() {
            return match ifg.ucsttifg() {
                false => Err(Other(I2cMultiMasterErr::ArbitrationLost)), // Lost arbitration
                true  => Err(Other(I2cMultiMasterErr::AddressedAsSlave)),// Lost arbitration and the slave address was us
            }
        }
        if ifg.ucnackifg() {
            return Err(Other(I2cMultiMasterErr::GotNACK(self.usci.byte_count() as usize)));
        }
        self._read_rx_buf(ifg).map_err(|e| match e {WouldBlock => WouldBlock})
    }

    /// Check if the Rx buffer is full, if so read it. Used as part of the non-blocking / interrupt-based interface.
    /// 
    /// Returns `Err(WouldBlock)` if the buffer is still full, 
    /// `Err(Other(I2cMultiMasterErr))` if any bus conditions occur that would impede regular operation, or 
    /// `Ok(n)` if data was successfully retreived from the Rx buffer.  
    #[inline(always)]
    pub fn read_rx_buf_as_slave(&mut self) -> nb::Result<u8, Infallible> {
        self._read_rx_buf(self.usci.ifg_rd())
    }

    #[inline]
    fn _read_rx_buf(&mut self, ifg: USCI::IfgOut) -> nb::Result<u8, Infallible> {
        if !ifg.ucrxifg0() {
            return Err(WouldBlock);
        }
        Ok(self.usci.ucrxbuf_rd())
    }

    /// Read the Rx buffer without checking if it's ready. Should only be used if the peripheral is in slave mode.
    /// 
    /// Useful in cases where you already know the Rx buffer is ready (e.g. an Rx interrupt occurred).
    /// Used as part of the non-blocking / interrupt-based interface.
    /// # Safety
    /// If the buffer is not ready then the data will be invalid.
    #[inline(always)]
    pub unsafe fn read_rx_buf_as_slave_unchecked(&mut self) -> u8 {
        self.usci.ucrxbuf_rd()
    }
    
    /// Check if the Tx buffer is empty, if so write to it. Used as part of the non-blocking / interrupt-based interface.
    /// First checks if the peripheral is still in master mode, if not returns an error.
    /// 
    /// Returns `Err(WouldBlock)` if the buffer is still full, 
    /// `Err(Other(I2cMultiMasterErr))` if any bus conditions occur that would impede regular operation, or 
    /// `Ok(())` if data was successfully loaded into the Tx buffer.  
    pub fn write_tx_buf_as_master(&mut self, byte: u8) -> nb::Result<(), I2cMultiMasterErr>{
        let ifg = self.usci.ifg_rd();
        if ifg.ucalifg() {
            return match ifg.ucsttifg() {
                false => Err(Other(I2cMultiMasterErr::ArbitrationLost)), // Lost arbitration
                true  => Err(Other(I2cMultiMasterErr::AddressedAsSlave)),// Lost arbitration and the slave address was us
            }
        }
        if ifg.ucnackifg() {
            return Err(Other(I2cMultiMasterErr::GotNACK(self.usci.byte_count() as usize)));
        }
        self._write_tx_buf(byte, ifg).map_err(|e| match e {WouldBlock => WouldBlock})
    }


    /// Check if the Tx buffer is empty, if so write to it. Used as part of the non-blocking / interrupt-based interface.
    /// Does not check if the peripheral is in master mode.
    /// 
    /// Returns `Err(WouldBlock)` if the buffer is still full, 
    /// `Err(Other(I2cMultiMasterErr))` if any bus conditions occur that would impede regular operation, or 
    /// `Ok(())` if data was successfully loaded into the Tx buffer.  
    #[inline(always)]
    pub fn write_tx_buf_as_slave(&mut self, byte: u8) -> nb::Result<(), Infallible>{
        self._write_tx_buf(byte, self.usci.ifg_rd())
    }

    #[inline]
    fn _write_tx_buf(&mut self, byte: u8, ifg: USCI::IfgOut) -> nb::Result<(), Infallible> {
        if !ifg.uctxifg0() {
            return Err(WouldBlock);
        }
        self.usci.uctxbuf_wr(byte);
        Ok(())
    }

    /// Write to the Tx buffer without checking if it's ready. Should only be used if the peripheral is in slave mode.
    /// Useful in cases where you already know the Tx buffer is ready (e.g. a Tx interrupt occurred). 
    /// 
    /// Used as part of the non-blocking / interrupt-based interface.
    /// # Safety
    /// If the buffer is not ready then previous data may be clobbered.
    #[inline(always)]
    pub unsafe fn write_tx_buf_as_slave_unchecked(&mut self, byte: u8) {
        self.usci.uctxbuf_wr(byte);
    }

    #[inline(always)]
    fn _write(&mut self, address: u16, bytes: &[u8], send_start: bool, send_stop: bool) -> Result<(), I2cMultiMasterErr> {
        self.__write(address, bytes, send_start, send_stop)
    }

    #[inline(always)]
    fn _read(&mut self, address: u16, buffer: &mut [u8], send_start: bool, send_stop: bool) -> Result<(), I2cMultiMasterErr> {
        self.__read(address, buffer, send_start, send_stop)
    }

    /// Checks whether a slave with the specified address is present on the I2C bus.
    /// Sends a zero-byte write and records whether the slave sends an ACK or not.
    /// 
    /// A u8 address will use the 7-bit addressing mode, a u16 address uses 10-bit addressing.
    #[inline(always)]
    pub fn is_slave_present<TenOrSevenBit>(&mut self, address: TenOrSevenBit) -> Result<bool, I2cMultiMasterErr> 
    where TenOrSevenBit: AddressType {
        self._is_slave_present(address)
    }

    /// After losing arbitration (or after being addressed as a slave) call this method to return the peripheral to master mode.
    #[inline(always)]
    pub fn return_to_master(&mut self) {
        self.usci.set_master();
    }

    /// In multi-operation transactions update the NACK byte error count to match *total* bytes sent 
    #[inline]
    fn add_nack_count(err: I2cMultiMasterErr, bytes_already_sent: usize) -> I2cMultiMasterErr {
        match err {
            I2cMultiMasterErr::GotNACK(n) => I2cMultiMasterErr::GotNACK(n + bytes_already_sent),
            e => e,
        }
    }
    
    /// blocking write then blocking read
    fn _write_read(&mut self, address: u16, bytes: &[u8], buffer: &mut [u8]) -> Result<(), I2cMultiMasterErr> {
        self.set_transmission_mode(TransmissionMode::Transmit);
        self._write(address, bytes, true, false)?;
        self.set_transmission_mode(TransmissionMode::Receive);
        self._read(address, buffer, true, true)
            .map_err(|e| Self::add_nack_count(e, bytes.len()))
    }
}
impl<USCI: I2cUsci> I2cDevice<USCI, Slave> {
    /// Read the Rx buffer without checking if it's ready. 
    /// Useful in cases where you already know the Rx buffer is ready (e.g. an Rx interrupt occurred).
    /// Used as part of the non-blocking / interrupt-based interface.
    /// # Safety
    /// If the buffer is not ready then the data will be invalid.
    #[inline(always)]
    pub unsafe fn read_rx_buf_unchecked(&mut self) -> u8 {
        self.usci.ucrxbuf_rd()
    }

    /// Write to the Tx buffer without checking if it's ready. 
    /// Useful in cases where you already know the Tx buffer is ready (e.g. a Tx interrupt occurred). 
    /// Used as part of the non-blocking / interrupt-based interface.
    /// # Safety
    /// If the buffer is not ready then previous data may be clobbered.
    #[inline(always)]
    pub unsafe fn write_tx_buf_unchecked(&mut self, byte: u8) {
        self.usci.uctxbuf_wr(byte);
    }

    /// Check if the Rx buffer is full, if so read it. Used as part of the non-blocking / interrupt-based interface.
    /// Returns `Err(WouldBlock)` if the Rx buffer is empty, otherwise `Ok(n)`.
    #[inline]
    pub fn read_rx_buf(&mut self) -> nb::Result<u8, Infallible> {
        if !self.usci.ifg_rd().ucrxifg0() {
            return Err(WouldBlock);
        }
        Ok(self.usci.ucrxbuf_rd())
    }

    /// Check if the Tx buffer is empty, if so write to it. Used as part of the non-blocking / interrupt-based interface.
    /// Returns `Err(WouldBlock)` if the Tx buffer is still full, otherwise `Ok(())`.
    #[inline]
    pub fn write_tx_buf(&mut self, byte: u8) -> nb::Result<(), Infallible>{
        if !self.usci.ifg_rd().uctxifg0() {
            return Err(WouldBlock);
        }
        self.usci.uctxbuf_wr(byte);
        Ok(())
    }
}

/// I2C transmit/receive errors on a single master I2C bus.
#[derive(Clone, Copy, Debug)]
#[non_exhaustive]
pub enum I2cSingleMasterErr {
    /// Received a NACK. The contained value denotes the byte where the NACK occurred. 
    /// 
    /// In the blocking methods the contained value counts up from the initial start condition 
    /// (byte 0 is the address byte, byte 1 is the first data byte, etc.). When using the non-blocking 
    /// methods this value counts up from the most recent Start or Repeated Start condition.
    GotNACK(usize),
    // Other errors like the 'clock low timeout' UCCLTOIFG may appear here in future.
}

/// I2C transmit/receive errors on a multi-master I2C bus.
#[derive(Clone, Copy, Debug)]
#[non_exhaustive]
pub enum I2cMultiMasterErr {
    /// Received a NACK. The contained value denotes the byte where the NACK occurred. 
    /// 
    /// In the blocking implementation the contained value counts since the initial start condition 
    /// (byte 0 is the address byte, byte 1 is the first data byte, etc.). When using the non-blocking 
    /// methods this value counts up from the most recent Start or Repeated Start condition.
    GotNACK(usize),
    /// Another master on the bus talked over us, so the transaction was aborted. 
    /// The peripheral has been forced into slave mode. 
    /// Call [`return_to_master()`](I2cDevice<MultiMaster>::return_to_master) to resume the master role.
    ArbitrationLost,
    /// Another master on the bus addressed us as a slave device. The peripheral has been forced into slave mode.
    /// The slave transaction *must* be completed before master operations can be resumed with
    /// [`return_to_master()`](I2cDevice<MultiMaster>::return_to_master).
    AddressedAsSlave,
    /// The eUSCI peripheral attempted to address itself. The hardware does not support this operation.
    TriedAddressingSelf,
    // Other errors like the 'clock low timeout' UCCLTOIFG may appear here in future.
}

/// A list of events that may occur on the I2C bus.
#[derive(Debug, Copy, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub enum I2cEvent {
    /// The master sent a (repeated) start and wants to read from us. Write to the Tx buffer to clear this event.
    ReadStart,
    /// The master continues to read from us. Write to the Tx buffer to clear this event.
    Read,
    /// The master sent a (repeated) start and wants to write to us. Read from the Rx buffer to clear this event.
    WriteStart,
    /// The master continues to write to us. Read from the Rx buffer to clear this event.
    Write,
    /// We have fallen behind. The master sent a write (filled the Rx buffer), then a repeated start and a read (currently stalled).
    /// 
    /// The repeated start after the write means we can no longer tell if the initial write was a `WriteStart` or a `Write`.
    /// 
    /// If you have more information about the expected format of the transaction you may be able to deduce which of the two it was. 
    /// 
    /// Read from the Rx buffer to clear this event.
    OverrunWrite,
    /// The master has ended the transaction. This event is automatically cleared.
    Stop,
}

/// List of possible I2C interrupt sources. Used when reading from the 'interrupt vector' register
#[derive(Debug, Copy, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub enum I2cVector {
    /// No interrupt.
    None                = 0x00,
    /// Arbitration was lost during an attempted transmission.
    ArbitrationLost     = 0x02,
    /// Received a NACK.
    NackReceived        = 0x04,
    /// Received a Start condition on the I2C bus along with one of our own addresses.
    StartReceived       = 0x06,
    /// Received a Stop condition on the I2C bus. 
    /// This is usually set when acting as an I2C slave, but that this can occur as an I2C master during a zero byte write.
    StopReceived        = 0x08,
    /// Slave address 3 received a data byte.
    Slave3RxBufFull     = 0x0A,
    /// The Tx buffer is empty and slave address 3 was on the I2C bus when this occurred.
    Slave3TxBufEmpty    = 0x0C,
    /// Slave address 2 received a data byte.
    Slave2RxBufFull     = 0x0E,
    /// The Tx buffer is empty and slave address 2 was on the I2C bus when this occurred.
    Slave2TxBufEmpty    = 0x10,
    /// Slave address 1 received a data byte.
    Slave1RxBufFull     = 0x12,
    /// The Tx buffer is empty and slave address 1 was on the I2C bus when this occurred.
    Slave1TxBufEmpty    = 0x14,
    /// Data is waiting in the Rx buffer. In slave mode slave address 0 was on the I2C bus when this occurred.
    RxBufFull           = 0x16,
    /// The Tx buffer is empty. In slave mode slave address 0 was on the I2C bus when this occurred.
    TxBufEmpty          = 0x18,
    /// The target byte count has been reached.
    ByteCounterZero     = 0x1A,
    /// The SCL line has been held low longer than the Clock Low Timeout value.
    ClockLowTimeout     = 0x1C,
    /// The 9th bit of an I2C data packet has been completed.
    NinthBitReceived    = 0x1E,
}

/// Human-friendly list of possible I2C interrupt source flags. Used for writing to the 'interrupt enable' register.
#[derive(Debug, Copy, Clone, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u16)]
pub enum I2cInterruptBits {
    /// UCRXIE0. Trigger an interrupt when data is waiting in the Rx buffer. In slave mode slave address 0 must be on the I2C bus when this occurred.
    RxBufFull           = 1 << 0,
    /// UCTXIE0. Trigger an interrupt when the Tx buffer is empty. In slave mode slave address 0 must be on the I2C bus when this occurred.
    TxBufEmpty          = 1 << 1,
    /// UCSTTIE. Trigger an interrupt when a Start condition is received on the I2C bus along with one of our own addresses.
    StartReceived       = 1 << 2,
    /// UCSTPIE. Trigger an interrupt when a Stop condition is received on the I2C bus in a transaction we are a part of. 
    /// Typically this triggers when acting as an I2C slave, but this also triggers as an I2C master during a zero byte write.
    StopReceived        = 1 << 3,
    /// UCALIE. Trigger an interrupt when arbitration was lost during an attempted transmission.
    ArbitrationLost     = 1 << 4,
    /// UCNACKIE. Trigger an interrupt a NACK is received.
    NackReceived        = 1 << 5,
    /// UCBCNTIE. Trigger an interrupt when the target byte count has been reached.
    ByteCounterZero     = 1 << 6,
    /// UCCLTOIE. Trigger an interrupt when the SCL line has been held low longer than the Clock Low Timeout value.
    ClockLowTimeout     = 1 << 7,
    /// UCRXIE1. Trigger an interrupt when slave address 1 receives a data byte.
    Slave1RxBufFull     = 1 << 8,
    /// UCTXIE1. Trigger an interrupt when the Tx buffer is empty and slave address 1 was on the I2C bus when this occurred.
    Slave1TxBufEmpty    = 1 << 9,
    /// UCRXIE2. Trigger an interrupt when slave address 2 receives a data byte.
    Slave2RxBufFull     = 1 << 10,
    /// UCTXIE2. Trigger an interrupt when the Tx buffer is empty and slave address 2 was on the I2C bus when this occurred.
    Slave2TxBufEmpty    = 1 << 11,
    /// UCRXIE3. Trigger an interrupt when slave address 3 receives a data byte.
    Slave3RxBufFull     = 1 << 12,
    /// UCTXIE3. Trigger an interrupt when the Tx buffer is empty and slave address 3 was on the I2C bus when this occurred.
    Slave3TxBufEmpty    = 1 << 13,
    /// UCBIT9IE. Trigger an interrupt when the 9th bit of an I2C data packet we are involved in has been completed.
    NinthBitReceived    = 1 << 14,
}
impl From<I2cInterruptBits> for u16 {
    fn from(value: I2cInterruptBits) -> Self {
        value as u16
    }
}
impl core::ops::Add for I2cInterruptBits {
    type Output = u16;
    fn add(self, rhs: Self) -> Self::Output {
        (self as u16) + (rhs as u16)
    }
}
impl core::ops::Add<u16> for I2cInterruptBits {
    type Output = u16;
    fn add(self, rhs: u16) -> Self::Output {
        (self as u16) + rhs
    }
}
impl core::ops::Add<I2cInterruptBits> for u16 {
    type Output = u16;
    fn add(self, rhs: I2cInterruptBits) -> Self::Output {
        self + (rhs as u16)
    }
}

// Trait to link embedded-hal types to our addressing mode enum.
// Since SevenBitAddress and TenBitAddress are just aliases for u8 and u16 in both ehal 1.0 and 0.2.7, this works for both!
/// A trait marking types that can be used as I2C addresses. Namely `u8` for 7-bit addresses and `u16` for 10-bit addresses.
/// 
/// Used internally by the HAL.
pub trait AddressType: AddressMode + Into<u16> + Copy {
    /// Return the `AddressingMode` that relates to this type: `SevenBit` for `u8`, `TenBit` for `u16`.
    fn addr_type() -> AddressingMode;
}
impl AddressType for SevenBitAddress {
    fn addr_type() -> AddressingMode {
        AddressingMode::SevenBit
    }
}
impl AddressType for TenBitAddress {
    fn addr_type() -> AddressingMode {
        AddressingMode::TenBit
    }
}

/// Implement embedded-hal's [`I2c`](embedded_hal::i2c::I2c) trait
macro_rules! impl_ehal_i2c {
    ($role: ty) => {
        impl<USCI: I2cUsci, TenOrSevenBit> I2c<TenOrSevenBit> for I2cDevice<USCI, $role> 
        where TenOrSevenBit: AddressType {
            fn transaction(&mut self, address: TenOrSevenBit, ops: &mut [Operation<'_>]) -> Result<(), Self::Error> {
                self.set_addressing_mode(TenOrSevenBit::addr_type());
                
                let mut prev_discr = None;
                let mut bytes_sent = 0;
                let len = ops.len();
                for (i, op) in ops.iter_mut().enumerate() {
                    // Send a start if this is the first operation, 
                    // or if the previous operation was a different type (e.g. Read and Write)
                    let send_start = match prev_discr {
                        None => true,
                        Some(prev) => prev != core::mem::discriminant(op),
                    };
                    // Send a stop only if this is the last operation
                    let send_stop = i == (len - 1);
                    
                    match op {
                        Operation::Read(ref mut items) => {
                            self.set_transmission_mode(TransmissionMode::Receive);
                            self._read(address.into(), items, send_start, send_stop)
                                .map_err(|e| Self::add_nack_count(e, bytes_sent))?;
                            bytes_sent += items.len();
                        }
                        Operation::Write(items) => {
                            self.set_transmission_mode(TransmissionMode::Transmit);
                            self._write(address.into(), items, send_start, send_stop)
                                .map_err(|e| Self::add_nack_count(e, bytes_sent))?;
                            bytes_sent += items.len();
                        }
                    }
                    prev_discr = Some(core::mem::discriminant(op));
                }
                Ok(())
            }
        }
    };
}

mod ehal1 {
    use embedded_hal::i2c::{Error, ErrorKind, ErrorType, I2c, Operation, NoAcknowledgeSource};
    use super::*;

    impl Error for I2cSingleMasterErr {
        fn kind(&self) -> ErrorKind {
            match self {
                I2cSingleMasterErr::GotNACK(0)  => ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address),
                I2cSingleMasterErr::GotNACK(_)  => ErrorKind::NoAcknowledge(NoAcknowledgeSource::Data),
            }
        }
    }
    impl<USCI: I2cUsci> ErrorType for I2cDevice<USCI, SingleMaster> {
        type Error = I2cSingleMasterErr;
    }
    impl_ehal_i2c!(SingleMaster);

    impl Error for I2cMultiMasterErr {
        fn kind(&self) -> ErrorKind {
            match self {
                I2cMultiMasterErr::GotNACK(0)           => ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address),
                I2cMultiMasterErr::GotNACK(_)           => ErrorKind::NoAcknowledge(NoAcknowledgeSource::Data),
                I2cMultiMasterErr::ArbitrationLost      => ErrorKind::ArbitrationLoss,
                I2cMultiMasterErr::AddressedAsSlave     => ErrorKind::ArbitrationLoss,
                I2cMultiMasterErr::TriedAddressingSelf  => ErrorKind::Other,
            }
        }
    }
    impl<USCI: I2cUsci> ErrorType for I2cDevice<USCI, MultiMaster> {
        type Error = I2cMultiMasterErr;
    }
    impl_ehal_i2c!(MultiMaster);
}

#[cfg(feature = "embedded-hal-02")]
mod ehal02 {
    use embedded_hal_02::blocking::i2c::{AddressMode, Read, Write, WriteRead};
    use super::*;

    macro_rules! impl_ehal02_i2c {
        ($type: ty, $err_type: ty) => {
            impl<USCI: I2cUsci, SevenOrTenBit> Read<SevenOrTenBit> for $type
            where SevenOrTenBit: AddressMode + AddressType {
                type Error = $err_type;
        fn read(&mut self, address: SevenOrTenBit, buffer: &mut [u8]) -> Result<(), Self::Error> {
            self.set_addressing_mode(SevenOrTenBit::addr_type());
            self.set_transmission_mode(TransmissionMode::Receive);
            self._read(address.into(), buffer, true, true)
        }
    }
            impl<USCI: I2cUsci, SevenOrTenBit> Write<SevenOrTenBit> for $type 
            where SevenOrTenBit: AddressMode + AddressType {
                type Error = $err_type;
        fn write(&mut self, address: SevenOrTenBit, bytes: &[u8]) -> Result<(), Self::Error> {
            self.set_addressing_mode(SevenOrTenBit::addr_type());
            self.set_transmission_mode(TransmissionMode::Transmit);
            self._write(address.into(), bytes, true, true)
        }
    }
            impl<USCI: I2cUsci, SevenOrTenBit> WriteRead<SevenOrTenBit> for $type  
            where SevenOrTenBit: AddressMode + AddressType {
                type Error = $err_type;
        fn write_read(
            &mut self,
            address: SevenOrTenBit,
            bytes: &[u8],
            buffer: &mut [u8],
        ) -> Result<(), Self::Error> {
            self.set_addressing_mode(SevenOrTenBit::addr_type());
            self._write_read(address.into(), bytes, buffer)
        }
            }
        };
    }

    impl_ehal02_i2c!(I2cDevice<USCI, SingleMaster>, I2cSingleMasterErr);
    impl_ehal02_i2c!(I2cDevice<USCI, MultiMaster>,  I2cMultiMasterErr);
}