#![no_main]
#![no_std]
#![feature(abi_msp430_interrupt)]

// This I2C slave implements reading and writing from a 10-byte array.
// If a transaction begins with a write, the first byte is treated as the desired array index. 
// A subsequent write provides data to store at the specified index. Additional writes will be stored at the following indices, the index autoincrementing after each.
// After any number of writes the master may perform a Repeated Start and switch to reading in order to retrieve the value at the specified index. 
// Further reads will read the subsequent indices, autoincrementing.
// If a transaction does not begin with a write, then the index from the previous transaction is used. 
// If no previous transaction has been performed the index defaults to 0.

// eUSCI B0 is configured as the master. Connect P1.2 to P4.6. Connect P1.3 to P4.7.

use core::cell::RefCell;

use critical_section::Mutex;
use msp430::interrupt::enable as enable_interrupts;
use embedded_hal::{delay::DelayNs, digital::{OutputPin, StatefulOutputPin}, i2c::I2c};
use msp430_atomic::AtomicU8;
use msp430_rt::entry;
use msp430fr2355::{interrupt, E_USCI_B1};
use msp430fr2x5x_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv}, fram::Fram, gpio::Batch, i2c::{GlitchFilter, I2cDevice, I2cConfig, I2cInterruptBits, I2cVector, Slave}, pmm::Pmm, watchdog::Wdt
};
use panic_msp430 as _;

static I2C_SLAVE: Mutex<RefCell<Option< I2cDevice<E_USCI_B1, Slave> >>> = Mutex::new(RefCell::new(None));

// Store the exposed 'registers' as Atomic values, so they can be easily read/written to between the interrupt and main fn
const ARR_LEN: usize = 10;
static ARR:  [AtomicU8; ARR_LEN] = [const { AtomicU8::new(0) }; 10];

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
    let sl_scl = p4.pin7.to_alternate1();
    let sl_sda = p4.pin6.to_alternate1();

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

    const SLAVE_ADDR: u8 = 0x1A;
    let mut i2c_slave = I2cConfig::new(periph.E_USCI_B1, GlitchFilter::Max50ns)
        .as_slave(SLAVE_ADDR)
        .configure(sl_scl, sl_sda);

    use I2cInterruptBits::*;
    i2c_slave.set_interrupts(StopReceived + RxBufFull + TxBufEmpty);

    critical_section::with(|cs| {
        I2C_SLAVE.borrow_ref_mut(cs).replace(i2c_slave);
    });

    unsafe { enable_interrupts(); }

    let index = 0;
    let mut value = 0;
    loop {
        // Write a value between 0 and 9 to index 0.
        i2c_master.write(SLAVE_ADDR, &[index, value]).unwrap();
        value = (value+1) % 10;

        // Enable the green LED if the value at index 0 is 0.
        green_led.set_state((ARR[0].load() == 0).into()).ok();
        // Toggle the red LED after each
        red_led.toggle().ok();
        delay.delay_ms(100);
    }
}

// Static mut variables defined inside an interrupt handler are safe. See: https://docs.rust-embedded.org/book/start/interrupts.html
#[allow(static_mut_refs)]
#[interrupt]
fn EUSCI_B1() {
    static mut BYTE_COUNT: u8 = 0; // Bytes since the initial start condition
    static mut ARR_INDEX: usize = 0;

    critical_section::with(|cs| {
        let Some(ref mut i2c_slave) = *I2C_SLAVE.borrow_ref_mut(cs) else {return};
        match i2c_slave.interrupt_source() {
            I2cVector::RxBufFull => {
                // Safety: Rx interrupt triggered, so Rx buffer is ready.
                let val = unsafe{ i2c_slave.read_rx_buf_unchecked() };
                // If this is the first byte treat the I2C byte as the array index
                if *BYTE_COUNT == 0 {
                    *ARR_INDEX = val as usize % ARR_LEN;
                }
                else {
                    // Otherwise treat the I2C byte as data to be stored
                    ARR[*ARR_INDEX].store(val);
                    *ARR_INDEX = (*ARR_INDEX + 1) % ARR_LEN; // Autoincrement index
                }
            },
            I2cVector::TxBufEmpty => {
                // Safety: Tx interrupt triggered, so Tx buffer is ready.
                unsafe{ i2c_slave.write_tx_buf_unchecked(ARR[*ARR_INDEX].load()) };
                *ARR_INDEX = (*ARR_INDEX + 1) % ARR_LEN; // Autoincrement index
            },
            I2cVector::StopReceived => {
                *ARR_INDEX = (*ARR_INDEX).wrapping_sub(1).min(ARR_LEN - 1); // Undo last autoincrement
                *BYTE_COUNT = 0;
                return;
            },
            _ => unreachable!()
        }
        *BYTE_COUNT += 1;
    })
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
