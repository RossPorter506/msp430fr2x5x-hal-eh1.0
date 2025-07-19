#![no_main]
#![no_std]
#![feature(abi_msp430_interrupt)]

// This example demonstrates an SPI slave using interrupts. 
// Another eUSCI peripheral is configured as an SPI master to drive the bus.

// Connect:
// P1.7 <--> P4.6, 
// P1.6 <--> P4.7, 
// P1.5 <--> P4.5,
// P1.4 <--> P4.4

use core::cell::RefCell;

use critical_section::Mutex;
use embedded_hal::digital::{OutputPin, StatefulOutputPin};
use embedded_hal::spi::{SpiBus, MODE_0};
use embedded_hal::delay::DelayNs;
use msp430_rt::entry;
use msp430fr2355::{interrupt, E_USCI_A0};
use msp430fr2x5x_hal::spi::{SpiConfig, SpiErr, SpiSlave, StePolarity};
use msp430fr2x5x_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv}, fram::Fram, gpio::Batch, pmm::Pmm, watchdog::Wdt
};

use panic_msp430 as _;

#[entry]
fn main() -> ! {
    let periph = msp430fr2355::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.FRCTL);
    let _wdt = Wdt::constrain(periph.WDT_A);

    let pmm = Pmm::new(periph.PMM);
    let p1 = Batch::new(periph.P1)
        .split(&pmm);
    let sl_mosi = p1.pin7.to_alternate1();
    let sl_miso = p1.pin6.to_alternate1();
    let sl_sclk = p1.pin5.to_alternate1();
    let sl_ste  = p1.pin4.to_alternate1();

    let p4 = Batch::new(periph.P4)
        .split(&pmm);
    let mosi = p4.pin6.to_alternate1();
    let miso = p4.pin7.to_alternate1();
    let sclk = p4.pin5.to_alternate1();
    let mut ste = p4.pin4.to_output();
    ste.set_high().ok();

    let mut red_led = p1.pin0.to_output();
    let mut green_led = Batch::new(periph.P6).split(&pmm).pin6.to_output();

    let (smclk, _aclk, mut delay) = ClockConfig::new(periph.CS)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_vloclk()
        .freeze(&mut fram);

    // Configure a peripheral as an SPI slave. 
    // It can be configured for either a shared or exclusive bus depending on whether
    // there are other slaves on the bus. On an eclusive bus MISO is always an output.
    // On a shared bus the STE pin is used to control whether this slave's MISO is an output or high impedance pin.
    let mut spi_slave = SpiConfig::new(periph.E_USCI_A0, MODE_0, true)
        .as_slave()
        .shared_bus(sl_miso, sl_mosi, sl_sclk, sl_ste, StePolarity::EnabledWhenLow);

    // Configure another as an SPI master to drive the bus.
    let mut spi = SpiConfig::new(periph.E_USCI_B1, MODE_0, true)
        .as_master_using_smclk(&smclk, 800) // 8MHz / 80 = 100kHz
        .single_master_bus(miso, mosi, sclk);

    critical_section::with(|cs| {
        spi_slave.set_rx_interrupt();
        SPI_SLAVE.replace(cs, Some(spi_slave));
    });

    unsafe { msp430::interrupt::enable() };

    loop {
        let mut recv_buf = [0; 4];
        let send_buf = [12, 14, 0xFF];
        
        ste.set_low().ok(); // Enable slave MISO
        
        // Can return Err, but both error types aren't relevant here.
        let _ = spi.transfer(&mut recv_buf, &send_buf);
        let _ = spi.flush();

        ste.set_high().ok(); // Make slave MISO high impedance

        if recv_buf[1..] == [13, 15, 00] {
            green_led.toggle().ok();
        }

        red_led.toggle().ok();
        delay.delay_ms(1000);
    }
}

static SPI_SLAVE: Mutex<RefCell<Option< SpiSlave<E_USCI_A0> >>> = Mutex::new(RefCell::new(None));

#[interrupt]
fn EUSCI_A0() {
    critical_section::with(|cs| {
        let Some(ref mut spi_slave) = *SPI_SLAVE.borrow_ref_mut(cs) else {return};
        // If you have multiple interrupts enabled you can use .interrupt_source() to determine which one caused this interrupt
        let byte = match unsafe{spi_slave.read_unchecked()} {
            Ok(b) => b,
            Err(SpiErr::Overrun(b)) => b,
        };
        spi_slave.write(byte.wrapping_add(1));
    });
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
