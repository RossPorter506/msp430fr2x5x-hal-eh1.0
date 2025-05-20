//! Clock system for configuration of MCLK, SMCLK, and ACLK.
//!
//! Once configuration is complete, `Aclk` and `Smclk` clock objects are returned. The clock
//! objects are used to set the clock sources on other peripherals.
//! Configuration of MCLK and SMCLK *must* occur, though SMCLK can be disabled. In that case, only
//! `Aclk` is returned.
//!
//! DCO with FLL is supported on MCLK for select frequencies. Supporting arbitrary frequencies on
//! the DCO requires complex calibration routines not supported by the HAL.

use core::arch::asm;

use crate::delay::Delay;
use crate::fram::{Fram, WaitStates};
use msp430::asm;
use msp430fr2355 as pac;
use pac::cs::csctl1::DCORSEL_A;
use pac::cs::csctl4::{SELA_A, SELMS_A};
pub use pac::cs::csctl5::{DIVM_A as MclkDiv, DIVS_A as SmclkDiv};

/// REFOCLK frequency
pub const REFOCLK: u16 = 32768;
/// VLOCLK frequency
pub const VLOCLK: u16 = 10000;

enum MclkSel {
    Refoclk,
    Vloclk,
    DcoclkFactoryTrim(DcoclkFreqSel),
    DcoclkSoftwareTrim(u16),
}

impl MclkSel {
    #[inline]
    fn freq(&self) -> u32 {
        match self {
            MclkSel::Vloclk => VLOCLK as u32,
            MclkSel::Refoclk => REFOCLK as u32,
            MclkSel::DcoclkFactoryTrim(fsel) => REFOCLK as u32 * fsel.multiplier() as u32,
            MclkSel::DcoclkSoftwareTrim(freq_khz) => (*freq_khz) as u32 * 1000,
        }
    }

    #[inline(always)]
    fn selms(&self) -> SELMS_A {
        match self {
            MclkSel::Vloclk => SELMS_A::VLOCLK,
            MclkSel::Refoclk => SELMS_A::REFOCLK,
            MclkSel::DcoclkFactoryTrim(_) => SELMS_A::DCOCLKDIV,
            MclkSel::DcoclkSoftwareTrim(_) => SELMS_A::DCOCLKDIV,
        }
    }
}

#[derive(Clone, Copy)]
enum AclkSel {
    Vloclk,
    Refoclk,
}

impl AclkSel {
    #[inline(always)]
    fn sela(self) -> SELA_A {
        match self {
            AclkSel::Vloclk => SELA_A::VLOCLK,
            AclkSel::Refoclk => SELA_A::REFOCLK,
        }
    }

    #[inline(always)]
    fn freq(self) -> u16 {
        match self {
            AclkSel::Vloclk => VLOCLK,
            AclkSel::Refoclk => REFOCLK,
        }
    }
}

/// Selectable DCOCLK frequencies when using factory trim settings.
/// Actual frequencies may be slightly higher.
#[derive(Clone, Copy)]
pub enum DcoclkFreqSel {
    /// 16 MHz
    _16MHz,
    /// 24 MHz
    _24MHz,
}

impl DcoclkFreqSel {
    #[inline(always)]
    fn dcorsel(self) -> DCORSEL_A {
        match self {
            DcoclkFreqSel::_16MHz => DCORSEL_A::DCORSEL_5,
            DcoclkFreqSel::_24MHz => DCORSEL_A::DCORSEL_7,
        }
    }

    #[inline(always)]
    fn multiplier(self) -> u16 {
        match self {
            DcoclkFreqSel::_16MHz => 490,
            DcoclkFreqSel::_24MHz => 732,
        }
    }

    // Factory trimmed DCO value. See Table 6-70 from datasheet
    fn dco(self) -> u16 {
        match self {
            DcoclkFreqSel::_16MHz => unsafe{ *(0x1A2E as *const u16) },
            DcoclkFreqSel::_24MHz => unsafe{ *(0x1A30 as *const u16) },
        }
    }
    /// Numerical frequency
    #[inline]
    pub fn freq(self) -> u32 {
        (self.multiplier() as u32) * (REFOCLK as u32)
    }
}

/// Typestate for `ClockConfig` that represents unconfigured clocks
pub struct NoClockDefined;
/// Typestate for `ClockConfig` that represents a configured MCLK
pub struct MclkDefined(MclkSel);
/// Typestate for `ClockConfig` that represents a configured SMCLK
pub struct SmclkDefined(SmclkDiv);
/// Typestate for `ClockConfig` that represents disabled SMCLK
pub struct SmclkDisabled;

// Using SmclkState as a trait bound outside the HAL will never be useful, since we only configure
// the clock once, so just keep it hidden
#[doc(hidden)]
pub trait SmclkState {
    fn div(&self) -> Option<SmclkDiv>;
}

impl SmclkState for SmclkDefined {
    #[inline(always)]
    fn div(&self) -> Option<SmclkDiv> {
        Some(self.0)
    }
}

impl SmclkState for SmclkDisabled {
    #[inline(always)]
    fn div(&self) -> Option<SmclkDiv> {
        None
    }
}

/// Builder object that configures system clocks
///
/// Can only commit configurations to hardware if both MCLK and SMCLK settings have been
/// configured. ACLK configurations are optional, with its default source being REFOCLK.
pub struct ClockConfig<MCLK, SMCLK> {
    periph: pac::CS,
    mclk: MCLK,
    mclk_div: MclkDiv,
    aclk_sel: AclkSel,
    smclk: SMCLK,
}

macro_rules! make_clkconf {
    ($conf:expr, $mclk:expr, $smclk:expr) => {
        ClockConfig {
            periph: $conf.periph,
            mclk: $mclk,
            mclk_div: $conf.mclk_div,
            aclk_sel: $conf.aclk_sel,
            smclk: $smclk,
        }
    };
}

impl ClockConfig<NoClockDefined, NoClockDefined> {
    /// Converts CS into a fresh, unconfigured clock builder object
    pub fn new(cs: pac::CS) -> Self {
        ClockConfig {
            periph: cs,
            smclk: NoClockDefined,
            mclk: NoClockDefined,
            mclk_div: MclkDiv::_1,
            aclk_sel: AclkSel::Refoclk,
        }
    }
}

impl<MCLK, SMCLK> ClockConfig<MCLK, SMCLK> {
    /// Select REFOCLK for ACLK
    #[inline]
    pub fn aclk_refoclk(mut self) -> Self {
        self.aclk_sel = AclkSel::Refoclk;
        self
    }

    /// Select VLOCLK for ACLK
    #[inline]
    pub fn aclk_vloclk(mut self) -> Self {
        self.aclk_sel = AclkSel::Vloclk;
        self
    }

    /// Select REFOCLK for MCLK and set the MCLK divider. Frequency is `10000 / mclk_div` Hz.
    #[inline]
    pub fn mclk_refoclk(self, mclk_div: MclkDiv) -> ClockConfig<MclkDefined, SMCLK> {
        ClockConfig {
            mclk_div,
            ..make_clkconf!(self, MclkDefined(MclkSel::Refoclk), self.smclk)
        }
    }

    /// Select VLOCLK for MCLK and set the MCLK divider. Frequency is `32768 / mclk_div` Hz.
    #[inline]
    pub fn mclk_vcoclk(self, mclk_div: MclkDiv) -> ClockConfig<MclkDefined, SMCLK> {
        ClockConfig {
            mclk_div,
            ..make_clkconf!(self, MclkDefined(MclkSel::Vloclk), self.smclk)
        }
    }

    /// Select DCOCLK for MCLK with FLL for stabilization, using software trim to set DCOCLK to a custom frequency. 
    /// MCLK frequency is `target_freq / mclk_div` Hz. Software trim takes much longer than factory trim.
    /// 
    /// This option is recommeded only if you can't obtain your desired MCLK frequency using a 24 MHz DCOCLK and divider.
    #[inline]
    pub fn mclk_dcoclk_custom(
        self,
        target_freq_khz: u16,
        mclk_div: MclkDiv,
    ) -> ClockConfig<MclkDefined, SMCLK> {
        ClockConfig {
            mclk_div,
            ..make_clkconf!(self, MclkDefined(MclkSel::DcoclkSoftwareTrim(target_freq_khz)), self.smclk)
        }
    }

    /// Select DCOCLK for MCLK with FLL for stabilization, setting DCOCLK to the factory-trimmed 24 MHz setting. 
    /// MCLK frequency is `24 MHz / mclk_div`. This performs factory trim, which is much faster than software trim. 
    /// 
    /// This option is recommended provided you can obtain the MCLK speed you desire using the provided divider value.
    #[inline]
    pub fn mclk_dcoclk_factory(
        self,
        dco_freq: DcoclkFreqSel,
        mclk_div: MclkDiv,
    ) -> ClockConfig<MclkDefined, SMCLK> {
        ClockConfig {
            mclk_div,
            ..make_clkconf!(self, MclkDefined(MclkSel::DcoclkFactoryTrim(dco_freq)), self.smclk)
        }
    }

    /// Enable SMCLK and set SMCLK divider, which divides the MCLK frequency
    #[inline]
    pub fn smclk_on(self, div: SmclkDiv) -> ClockConfig<MCLK, SmclkDefined> {
        make_clkconf!(self, self.mclk, SmclkDefined(div))
    }

    /// Disable SMCLK
    #[inline]
    pub fn smclk_off(self) -> ClockConfig<MCLK, SmclkDisabled> {
        make_clkconf!(self, self.mclk, SmclkDisabled)
    }
}

#[inline(always)]
fn fll_off() {
    // 64 = 1 << 6, which is the 6th bit of SR
    unsafe { asm!("bis.b 64, SR", options(nomem, nostack)) };
}

#[inline(always)]
fn fll_on() {
    // 64 = 1 << 6, which is the 6th bit of SR
    unsafe { asm!("bic.b 64, SR", options(nomem, nostack)) };
}

impl<SMCLK: SmclkState> ClockConfig<MclkDefined, SMCLK> {
    #[inline]
    fn dco_factory_trim(&self, target_freq: DcoclkFreqSel) {
        // Run FLL configuration procedure from the user's guide if we are using DCO
        fll_off();
        msp430::asm::nop();
        msp430::asm::nop();
        msp430::asm::nop();
        msp430::asm::nop();
        msp430::asm::nop();
        msp430::asm::nop();

        self.periph.csctl3.write(|w| w.selref().refoclk());
        self.periph.csctl0.write(|w| unsafe { w.bits(target_freq.dco()) });
        self.periph
            .csctl1
            .write(|w| w.dcorsel().variant(target_freq.dcorsel()));
        self.periph.csctl2.write(|w| {
            unsafe { w.flln().bits(target_freq.multiplier() - 1) }
                .flld()
                ._1()
        });

        msp430::asm::nop();
        msp430::asm::nop();
        msp430::asm::nop();
        msp430::asm::nop();
        msp430::asm::nop();
        msp430::asm::nop();
        msp430::asm::nop();
        fll_on();

        while !self.periph.csctl7.read().fllunlock().is_fllunlock_0() {
            asm::nop();
        }
    }

    fn dco_software_trim(&self, dco_freq_khz: u16) {
            let dco_freq_hz = (dco_freq_khz as u32 * 1000).clamp(REFOCLK as u32, 24_000_000);

            // Calulate ratio assuming using REFOCLK
            self.periph.csctl3.modify(|_, w| w.selref().refoclk());
            let ratio: u16 = (dco_freq_hz / (REFOCLK as u32)) as u16;

            let x: u16 = ratio * 32;
        
            //Do not want the Oscillator Fault Flag to trigger during this routine.
            //So disable interrupt, save the state, and reapply later if necessary.
            //let ofie_was_on = unsafe{ pac::Peripherals::conjure().SFR.sfrie1.read().ofie().bit_is_set() };
            unsafe { pac::Peripherals::conjure().SFR.sfrie1.clear_bits(|w| w.ofie().clear_bit()) };
        
            // Disable FLL loop control. This is needed to prevent the FLL from acting as we are 
            // making fundamental modifications to the clock setup.
            fll_off();

            // Set DCO to lowest Tap
            self.periph.csctl0.write(|w| unsafe { w.bits(0) });

            // Set FLLD, FLLN
            unsafe { self.periph.csctl2.write(|w| w.flld()._1().flln().bits(ratio - 1)) };

            // Set DCORSEL
            let (starting_dcoftrim, dcorsel) = match dco_freq_hz {
                         0..= 1_500_000 => (3, DCORSEL_A::DCORSEL_0),
                 1_500_001..= 3_000_000 => (3, DCORSEL_A::DCORSEL_1),
                 3_000_001..= 6_000_000 => (3, DCORSEL_A::DCORSEL_2),
                 6_000_001..=10_000_000 => (3, DCORSEL_A::DCORSEL_3),
                10_000_001..=14_000_000 => (3, DCORSEL_A::DCORSEL_4),
                14_000_001..=18_000_000 => (3, DCORSEL_A::DCORSEL_5),
                18_000_001..=22_000_000 => (0, DCORSEL_A::DCORSEL_6),
                22_000_001..            => (0, DCORSEL_A::DCORSEL_7),
            };
            self.periph.csctl1.modify(|_,w| w.dcorsel().variant(dcorsel));

            // Re-enable FLL
            fll_on();

            // // Enable DCO frequency trim
            self.periph.csctl1.modify(|_, w| unsafe{ w.dcoftrim().bits(starting_dcoftrim).dcoftrimen().set_bit()} );
            //unsafe { self.periph.csctl1.set_bits(|w| w.dcoftrimen().set_bit()) };
        
            // Calculates DCO frequency trim values
            self.configure_dcoftrim(dco_freq_hz);
            
            while self.periph.csctl7.read().fllunlock().is_fllunlock_1() || 
                self.periph.csctl7.read().fllunlock().is_fllunlock_2() || 
                self.periph.csctl7.read().dcoffg().bit_is_set() {
                //Clear OSC fault flags
                unsafe { self.periph.csctl7.clear_bits(|w| w.dcoffg().clear_bit()) };
        
                //Clear OFIFG fault flag
                unsafe { pac::Peripherals::conjure().SFR.sfrifg1.clear_bits(|w| w.ofifg().clear_bit()) };
            }

            for _ in 0..x {
                for _ in 0..10 { // ~30 cycle delay
                    msp430::asm::nop();
                }
            }

            // Reapply Oscillator Fault Interrupt Enable if needed
            // if ofie_was_on {
            //     unsafe{ pac::Peripherals::conjure().SFR.sfrie1.set_bits(|w| w.ofie().set_bit()) };
            // }
    }

    fn configure_dcoftrim(&self, dco_freq_hz: u32) {
        let mut old_dco_tap;
        let mut new_dco_tap = 0xffff;
        let mut best_dco_delta = 0xffff;
        let mut best_csctl0 = 0;
        let mut best_csctl1 = 0;
        let mut found_best: bool = false;
        let mclk_freq_khz = (dco_freq_hz / 1_000) as u16;
        //let mut bak_writer = BakMemDebug::new();

        for _ in 0..=7 {
            if found_best {
                //bak_writer.write_to_next(0xFFFF);
                break;
            }
            //CSCTL0 = 0x100;                         // DCO Tap = 256
            unsafe { self.periph.csctl0.write(|w| w.dco().bits(256)); }
            //CSCTL7 &= ~DCOFFG;
            unsafe { self.periph.csctl7.clear_bits(|w| w.dcoffg().clear_bit()) };
            //while (CSCTL7 & DCOFFG) {               // Test DCO fault flag
            while self.periph.csctl7.read().dcoffg().bit_is_set() {
                //CSCTL7 &= ~DCOFFG;
                unsafe { self.periph.csctl7.clear_bits(|w| w.dcoffg().clear_bit()) };                  // Clear DCO fault flag
            };

            // Suggest to wait 24 cycles of divided FLL reference clock
            // __delay_cycles((unsigned int)3000 * MCLK_FREQ_MHZ);// Wait FLL lock status (FLLUNLOCK) to be stable
            for _ in 0..mclk_freq_khz {
                asm::nop();
            }
                                                            
            //while((CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1)) && ((CSCTL7 & DCOFFG) == 0)) {}
            while self.periph.csctl7.read().fllunlock().is_fllunlock_1() || self.periph.csctl7.read().fllunlock().is_fllunlock_2() {
                if self.periph.csctl7.read().dcoffg().bit_is_set() {
                    break;
                }
            }
            let is_err = self.periph.csctl7.read().dcoffg().bit_is_set();
            
            //csCtl0Read = CSCTL0;                   // Read CSCTL0
            let csctl0 = self.periph.csctl0.read().bits();
            //csCtl1Read = CSCTL1;                   // Read CSCTL1
            let csctl1 = self.periph.csctl1.read().bits();

            old_dco_tap = new_dco_tap;                 // Record DCOTAP value of last time
            //new_dco_tap = cs_ctl0_read & 0x01ff;       // Get DCOTAP value of this time
            new_dco_tap = self.periph.csctl0.read().dco().bits();       // Get DCOTAP value of this time
            //dco_freq_trim = (cs_ctl1_read & 0x0070)>>4;// Get DCOFTRIM value
            let dco_freq_trim = self.periph.csctl1.read().dcoftrim().bits();
            //bak_writer.write_to_next(((new_dco_tap) << 8) + ((is_err as u16) << 4) + (dco_freq_trim as u16));
            let new_dco_delta;
            if new_dco_tap < 256 {                   // DCOTAP < 256
                new_dco_delta = 256 - new_dco_tap;     // Delta value between DCPTAP and 256
                if (old_dco_tap != 0xffff) && (old_dco_tap >= 256) { // DCOTAP cross 256
                    found_best = true;
                }                   // Stop while loop
                else {
                    //CSCTL1 = (csCtl1Read & (~DCOFTRIM)) | (dcoFreqTrim<<4);
                    //unsafe {self.periph.csctl1.write(|w| w.bits( (cs_ctl1_read & !(0x0070)) | (dco_freq_trim<<4) ));}
                    if dco_freq_trim == 0 {
                        break;
                    }
                    unsafe {self.periph.csctl1.modify(|_, w| w.dcoftrim().bits(dco_freq_trim - 1))};
                }
            }
            else {                                 // DCOTAP >= 256
                    new_dco_delta = new_dco_tap - 256;     // Delta value between DCPTAP and 256
                    if old_dco_tap < 256 {              // DCOTAP cross 256
                    found_best = true;                   // Stop while loop
                }
                else {
                    //CSCTL1 = (csCtl1Read & (~DCOFTRIM)) | (dcoFreqTrim<<4);
                    //unsafe {self.periph.csctl1.write(|w| w.bits( (cs_ctl1_read & !(0x0070)) | (dco_freq_trim<<4) ));}
                    if dco_freq_trim == 7 {
                        break;
                    }
                    unsafe {self.periph.csctl1.modify(|_, w| w.dcoftrim().bits(dco_freq_trim + 1))};
                }
            }

            if new_dco_delta < best_dco_delta {        // Record DCOTAP closest to 256
                best_csctl0 = csctl0;
                best_csctl1 = csctl1;
                best_dco_delta = new_dco_delta;
            }
        }
        //bak_writer.write_to_next(0xFFFF);
        

        //CSCTL0 = csCtl0Copy;                         // Reload locked DCOTAP
        unsafe { self.periph.csctl0.write(|w| w.bits(best_csctl0)); }
        //CSCTL1 = csCtl1Copy;                         // Reload locked DCOFTRIM
        unsafe { self.periph.csctl1.write(|w| w.bits(best_csctl1)); }
        //while(CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1)) {} // Poll until FLL is locked
        while self.periph.csctl7.read().fllunlock().is_fllunlock_1() || self.periph.csctl7.read().fllunlock().is_fllunlock_2() {}
        //self.periph.csctl7.write(|w| w.dcoffg().clear_bit());
    }

    #[inline]
    fn configure_cs(&self) {
        // Configure clock selector and divisors
        self.periph.csctl4.write(|w| {
            w.sela()
                .variant(self.aclk_sel.sela())
                .selms()
                .variant(self.mclk.0.selms())
        });

        self.periph.csctl5.write(|w| {
            let w = w.vloautooff().set_bit().divm().variant(self.mclk_div);
            match self.smclk.div() {
                Some(div) => w.divs().variant(div),
                None => w.smclkoff().set_bit(),
            }
        });
    }

    #[inline]
    unsafe fn configure_fram(fram: &mut Fram, mclk_freq: u32) {
        if mclk_freq > 16_000_000 {
            fram.set_wait_states(WaitStates::Wait2);
        } else if mclk_freq > 8_000_000 {
            fram.set_wait_states(WaitStates::Wait1);
        } else {
            fram.set_wait_states(WaitStates::Wait0);
        }
    }
}

impl ClockConfig<MclkDefined, SmclkDefined> {
    /// Apply clock configuration to hardware and return SMCLK and ACLK clock objects.
    /// Also returns delay provider
    #[inline]
    pub fn freeze(self, fram: &mut Fram) -> (Smclk, Aclk, Delay) {
        let mclk_freq = self.mclk.0.freq() >> (self.mclk_div as u32);
        unsafe { Self::configure_fram(fram, mclk_freq) };
        match self.mclk.0 {
            MclkSel::Refoclk | MclkSel::Vloclk => (),
            MclkSel::DcoclkFactoryTrim(fsel) => self.dco_factory_trim(fsel),
            MclkSel::DcoclkSoftwareTrim(freq) => self.dco_software_trim(freq),
        };
        
        self.configure_cs();
        (
            Smclk(mclk_freq >> (self.smclk.0 as u32)),
            Aclk(self.aclk_sel.freq()),
            Delay::new(mclk_freq),
        )
    }
}

impl ClockConfig<MclkDefined, SmclkDisabled> {
    /// Apply clock configuration to hardware and return ACLK clock object, as SMCLK is disabled.
    /// Also returns delay provider.
    #[inline]
    pub fn freeze(self, fram: &mut Fram) -> (Aclk, Delay) {
        let mclk_freq = self.mclk.0.freq() >> (self.mclk_div as u32);
        unsafe { Self::configure_fram(fram, mclk_freq) };
        match self.mclk.0 {
            MclkSel::Refoclk | MclkSel::Vloclk  => (),
            MclkSel::DcoclkFactoryTrim(fsel)    => self.dco_factory_trim(fsel),
            MclkSel::DcoclkSoftwareTrim(freq)   => self.dco_software_trim(freq),
        };

        self.configure_cs();
        (Aclk(self.aclk_sel.freq()), Delay::new(mclk_freq))
    }
}

/// SMCLK clock object
pub struct Smclk(u32);
/// ACLK clock object
pub struct Aclk(u16);

/// Trait for configured clock objects
pub trait Clock {
    /// Type of the returned frequency value
    type Freq;

    /// Frequency of the clock
    fn freq(&self) -> Self::Freq;
}

impl Clock for Smclk {
    type Freq = u32;

    /// Returning a 32-bit frequency may seem suspect, since we're on a 16-bit system, but it is
    /// required as SMCLK can go up to 24 MHz. Clock frequencies are usually for initialization
    /// tasks such as computing baud rates, which should be optimized away, avoiding the extra cost
    /// of 32-bit computations.
    #[inline]
    fn freq(&self) -> u32 {
        self.0
    }
}

impl Clock for Aclk {
    type Freq = u16;

    #[inline]
    fn freq(&self) -> u16 {
        self.0
    }
}
