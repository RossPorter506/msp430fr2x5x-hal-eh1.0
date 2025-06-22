#![no_main]
#![no_std]
#![feature(abi_msp430_interrupt)]

use core::marker::PhantomData;

use embedded_hal::{delay::DelayNs, digital::{OutputPin, StatefulOutputPin}, i2c::{I2c, Operation}};
use msp430_rt::entry;
use msp430fr2355::interrupt;
use msp430fr2x5x_hal::{
    clock::{ClockConfig, DcoclkFreqSel, MclkDiv, SmclkDiv}, fram::Fram, gpio::Batch, i2c::{GlitchFilter, I2cBus, I2cConfig}, pmm::Pmm, watchdog::Wdt
};
use panic_msp430 as _;

#[entry]
fn main() -> ! {
    let periph = msp430fr2355::Peripherals::take().unwrap();

    let mut fram = Fram::new(periph.FRCTL);
    let _wdt = Wdt::constrain(periph.WDT_A);

    let pmm = Pmm::new(periph.PMM);
    let mut red_led = Batch::new(periph.P1).split(&pmm).pin0.to_output();
    let mut green_led = Batch::new(periph.P6).split(&pmm).pin6.to_output();
    let p4 = Batch::new(periph.P4)
        .split(&pmm);
    let scl = p4.pin7.to_alternate1();
    let sda = p4.pin6.to_alternate1();

    let (smclk, _aclk, mut delay) = ClockConfig::new(periph.CS)
        .mclk_dcoclk(DcoclkFreqSel::_8MHz, MclkDiv::_1)
        .smclk_on(SmclkDiv::_1)
        .aclk_vloclk()
        .freeze(&mut fram);

    let mut i2c = I2cConfig::new(periph.E_USCI_B1, GlitchFilter::Max50ns)
        .use_smclk(&smclk, 80) // 8MHz / 80 = 100kHz
        .configure(scl, sda);
    
    let mut arr = [0,1,2,3,4,5,6,7,8,9];
    let mut index = 0;

    loop {
        // Below are examples of the various I2C methods provided for writing to / reading from the bus.
        // 7- and 10-bit addressing modes are controlled by passing the address as either a u8 or a u16.

        // match i2c.poll() {
        //     None => (),
        //     Some(Read) => handle_read(&mut i2c, &mut arr),
        //     Some(Write) => handle_write(&mut i2c, &mut arr),
        // }

        red_led.toggle().ok();
        delay.delay_ms(1000);
    }
}

// I2C contract:
// If a transaction begins with a write then the byte after the address indicates the array index in question.
// If the master wants to write a value to this byte the master continues with a second data packet that contains the value. Any subsequent writes will be written to the next index, wrapping around the array. The master can send a stop whenever.
// If instead the master wants to read an address, it should send a repeated start after the index byte. The slave will then send the value at that index, autoincrementing until the master sends a stop.

// If a transaction begins with a read, then the value at the previous index used is sent. If no previous index was ever sent, the index is 0.

// fn handle_read(mut i2c: I2cSlave, arr: &mut [u8;10], index: &mut usize) {
//     loop {
//         if let Packet::ContinueReadMode = i2c.send(arr[*index]) {
//             *index = (*index + 1) % 10;
//         }
//         else { i2c.abort(); return}
//     }
// }

// fn handle_write(mut i2c: I2cSlave, arr: &mut [u8;10], index: &mut usize) {
//     let Packet::ContinueWriteMode(idx) = i2c.recv()
//     else { i2c.abort(); return};
//     let mut idx = idx as usize;

//     match i2c.recv() {
//         // If the master continues in write mode it wants to write to the array
//         Packet::ContinueWriteMode(v) => {
//             let mut value = v;
//             loop {
//                 arr[idx] = value;
//                 idx = (idx + 1) % 10;
//                 if let Packet::ContinueWriteMode(val) = i2c.recv() {
//                     value = val;
//                 }
//                 else { i2c.abort(); break }
//             }
//         },
//         // If the master switches to read mode it wants to read from the array
//         Packet::RestartToReadMode => {
//             loop {
//                 if let Packet::ContinueReadMode = i2c.send(arr[idx]) {
//                     idx = (idx + 1) % 10;
//                 }
//                 else { i2c.abort(); break }
//             }
//         },
//         _ => { i2c.abort(); return; },
//     }
//     *index = idx;
// }

// struct I2cSlave(());
// impl I2cSlave {
//     /// If the master is in Read mode then send it the next packet.
//     /// This method should not be called if the master is in Write mode.
//     fn send(&mut self, byte: u8) -> Packet {
//         let new_data = 123;
//         match todo!() {
//             0 => Packet::ContinueReadMode,
//             1 => Packet::RestartToReadMode,
//             2 => Packet::RestartToWriteMode(new_data),
//             _ => Packet::Stop,
//         }
//     }
//     /// If the master is in Write mode then accept the next packet.
//     /// This method should not be called if the master is in Read mode.
//     fn recv(&mut self) -> Packet {
//         let new_data = 123;
//         match todo!() {
//             0 => Packet::ContinueWriteMode(new_data),
//             1 => Packet::RestartToReadMode,
//             2 => Packet::RestartToWriteMode(new_data),
//             _ => Packet::Stop,
//         }
//     }
//     /// End the transmission gracefully, if applicable.
//     /// If the master already sent a stop, this is a no-op.
//     fn abort(self) {
//         todo!()
//     }
// }

#[interrupt]
fn EUSCI_B0() {
    
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
