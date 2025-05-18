#![no_main]
#![no_std]

use embedded_hal::{digital::*};
use msp430_rt::entry;
use msp430fr2x5x_hal::{
    backup_mem::BackupMemory, gpio::Batch, pmm::Pmm
};
use panic_msp430 as _;

// Use the value of non-volatile backup memory to toggle the red onboard LED
// The red LED should flash rapidly.
#[entry]
fn main() -> ! {
    // Take peripherals
    let periph = msp430fr2355::Peripherals::take().unwrap();

    // DON'T disable the watchdog. It will reset us after a few ms.
    //let _wdt = Wdt::constrain(periph.WDT_A);

    // Configure GPIO
    let pmm = Pmm::new(periph.PMM);
    let mut led = Batch::new(periph.P1).split(&pmm).pin0.to_output();

    // Interpret register block as a &mut [u8;32]
    let nv_mem = BackupMemory::as_u8s(periph.BKMEM);

    // Toggle value of nv_mem[0] between 0 and 1
    nv_mem[0] = (nv_mem[0] + 1) % 2;

    // Set the output pin high if nv_mem[0] == 0
    led.set_state((nv_mem[0] == 0).into()).ok();
    
    // Loop until the watchdog resets us
    loop {
        msp430::asm::nop();
    }
}

// The compiler will emit calls to the abort() compiler intrinsic if debug assertions are
// enabled (default for dev profile). MSP430 does not actually have meaningful abort() support
// so for now, we create our own in each application where debug assertions are present.
#[no_mangle]
extern "C" fn abort() -> ! {
    panic!();
}
