#![no_main]
#![no_std]

// This example uses the SpiBus embedded-hal interface, with a software controlled CS pin.
// TODO: Undo changes after multi-master test 
use embedded_hal::digital::StatefulOutputPin;
use embedded_hal::spi::SpiBus;
use embedded_hal::{digital::OutputPin, spi::MODE_0};
use embedded_hal::delay::DelayNs;
use msp430_rt::entry;
use msp430fr2x5x_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv}, fram::Fram, gpio::Batch, pmm::Pmm, spi::SpiConfig, watchdog::Wdt
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
    let mosi = p1.pin7.to_alternate1();
    let miso = p1.pin6.to_alternate1();
    let sck  = p1.pin5.to_alternate1();
    let ste  = p1.pin4.pulldown().to_alternate1();
    let mut cs   = p1.pin3.to_output();
    let mut red_led = p1.pin0.to_output(); 
    cs.set_high().ok();

    let p6 = Batch::new(periph.P6).split(&pmm);
    let mut green_led = p6.pin6.to_output();

    let (smclk, _aclk, mut delay) = ClockConfig::new(periph.CS)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_vloclk()
        .freeze(&mut fram);

    // In single master mode SCK and MOSI are always outputs.
    // Multi-master mode allows another master to control whether this device's SCK 
    // and MOSI pins are outputs or high impedance via the STE pin.
    let mut spi = SpiConfig::new(periph.E_USCI_A0, MODE_0, true)
        .as_master_using_smclk(&smclk, 16) // 8MHz / 16_000 = 500Hz
        .multi_master_bus(miso, mosi, sck, ste, msp430fr2x5x_hal::spi::StePolarity::EnabledWhenLow);

    loop {
        // Perform the following transaction:
        // Send: 0x12, 0x00,    0x00,    0x34,    0x56,
        // Recv: N/A,  recv[0], recv[1], recv[2], N/A
        let mut recv = [0; 3];
        cs.set_low().ok();

        // These methods do return errors - Overrun and BusConflict.
        // Because this is a single-master bus we can ignore BusConflict, and because we are a master 
        // and haven't used the non-blocking API (embedded-hal-nb) the Rx buffer should never overrun. 
        let res = spi.write(&[0x12]);
        //spi.read(&mut recv[0..1]);
        //spi.transfer(&mut recv[2..], &[0x34, 0x56]);
        
        let res = res.and(spi.flush());
        cs.set_high().ok();

        green_led.set_state(res.is_ok().into()).ok();
        red_led.toggle().ok();

        // delay.delay_ms(1000);
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
