#![no_main]
#![no_std]
#![feature(abi_msp430_interrupt)]

// Demonstrates a blocking master implementation, and a blocking and interrupt-based multi-master. 
// The master sends a byte to the slave, then switches to read mode. The multi-master echoes the sent value back to the master.

// The non-blocking master interface is lower-level than the blocking version (the embedded-hal `I2c` trait) 
// and requires more careful usage.

// eUSCI B1 is configured as the slave. eUSCI B0 is configured as the master. Connect P1.2 to P4.6. Connect P1.3 to P4.7.

// TODO: Figure out what to do with write_buf and read_buf for multi-master. We can't just return 'hey you're a slave' if they're trying to be a slave.

use core::cell::RefCell;

use critical_section::Mutex;
use embedded_hal::{delay::DelayNs, digital::{OutputPin, StatefulOutputPin}, i2c::I2c};
use msp430::interrupt::enable as enable_interrupts;
use msp430_rt::{entry};
use msp430fr2355::{interrupt, E_USCI_B1};
use msp430fr2x5x_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv}, fram::Fram, gpio::Batch, 
    i2c::{GlitchFilter, I2cConfig, I2cDevice, I2cEvent, I2cInterruptBits, I2cVector, MultiMaster, TransmissionMode}, pmm::Pmm, watchdog::Wdt
};
use panic_msp430 as _;

static I2C_MULTI_MASTER: Mutex<RefCell<Option< I2cDevice<E_USCI_B1, MultiMaster> >>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let periph = msp430fr2355::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.FRCTL);
    let _wdt = Wdt::constrain(periph.WDT_A);

    let pmm = Pmm::new(periph.PMM);
    let p1 = Batch::new(periph.P1).split(&pmm);
    let mut red_led = p1.pin0.to_output();
    let mut green_led = Batch::new(periph.P6).split(&pmm).pin6.to_output();
    let p4 = Batch::new(periph.P4).split(&pmm);
    let mm_scl = p4.pin7.to_alternate1();
    let mm_sda = p4.pin6.to_alternate1();

    let scl = p1.pin3.to_alternate1();
    let sda = p1.pin2.to_alternate1();

    let (smclk, _aclk, mut delay) = ClockConfig::new(periph.CS)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_vloclk()
        .freeze(&mut fram);

    let mut i2c_master = I2cConfig::new(periph.E_USCI_B0, GlitchFilter::Max50ns)
    .as_single_master()
    .use_smclk(&smclk, 80) // 8MHz / 80 = 100kHz
    .configure(scl, sda);

    const MULTI_MASTER_ADDR: u8 = 0x1A;
    let mut i2c_multi_master = I2cConfig::new(periph.E_USCI_B1, GlitchFilter::Max50ns)
        .as_multi_master(Some(MULTI_MASTER_ADDR))
        .use_smclk(&smclk, 80) // 8MHz / 80 = 100kHz
        .configure(mm_scl, mm_sda);

    critical_section::with(|cs| {
        i2c_multi_master.set_interrupts(I2cInterruptBits::StartReceived);
        I2C_MULTI_MASTER.borrow_ref_mut(cs).replace(i2c_multi_master);
    });
    unsafe { enable_interrupts() };

    loop {
        // The master sends a byte then receives a byte from the multi-master. 
        // The multi-master echoes the master's byte back.
        let mut echo_rx = [0; 1];
        const ECHO_TX: [u8;1] = [128; 1];
        // Start a transaction with the multi-master. This will force the multi-master into slave mode.
        i2c_master.write_read(MULTI_MASTER_ADDR, &ECHO_TX, &mut echo_rx).unwrap(); // Safe, our slave never NACKs

        // The multi-master is set back into master mode in the StopReceived interrupt, so now we can just use it like a master.
        // Here we send a zero-byte write to check if a device with address 0x10 is on the bus
        critical_section::with(|cs| {
            let Some(ref mut i2c_multi_master) = *I2C_MULTI_MASTER.borrow_ref_mut(cs) else {return};
            match i2c_multi_master.is_slave_present(0x10_u8) {
                Ok(true) => green_led.set_high().ok(),
                // In this case we know the other master can't be addressing or contesting the bus, 
                // but in reality these conditions should (and should) be handled here
                _ => green_led.set_low().ok(),
            };
        });

        // If the I2C devices echoed correctly set the red LED
        red_led.set_state((ECHO_TX[0] == echo_rx[0]).into()).ok();
        delay.delay_ms(100);
    }
}

// Static mut variables defined inside an interrupt handler are safe. See: https://docs.rust-embedded.org/book/start/interrupts.html
#[allow(static_mut_refs)]
#[interrupt]
fn EUSCI_B1() {
    static mut TEMP_VAR: u8 = 0;
    critical_section::with(|cs| {
        let Some(ref mut i2c_multi_master) = *I2C_MULTI_MASTER.borrow_ref_mut(cs) else {return};
        match i2c_multi_master.interrupt_source() {
            I2cVector::StartReceived => {
                // Enable Rx, Tx and Stop interrupts.
                use I2cInterruptBits::*;
                i2c_multi_master.set_interrupts(TxBufEmpty + RxBufFull + StopReceived);
            },
            I2cVector::RxBufFull => {
                // Store the received value so we can echo it back later when the master switches to read mode
                *TEMP_VAR = unsafe{ i2c_multi_master.read_rx_buf_as_slave_unchecked() };
            },
            I2cVector::TxBufEmpty => {
                // Echo back the stored value
                unsafe{ i2c_multi_master.write_tx_buf_as_slave_unchecked(*TEMP_VAR) };
            },
            I2cVector::StopReceived => {
                // Disable Rx, Tx, and Stop interrupts. We don't want these to trigger when the multi-master acts as a blocking master.
                use I2cInterruptBits::*;
                i2c_multi_master.clear_interrupts(TxBufEmpty + RxBufFull + StopReceived);
                i2c_multi_master.return_to_master();
            },
            _ => unreachable!(),
        }
    })
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
