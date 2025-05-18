//! Backup Memory
//! 32 bytes of non-volatile memory.
//! 
//! The peripheral access crate exposes the backup memory as 16 individual 16-bit registers. 
//! This module provides helper functions for reinterpreting the backup memory as various array types.
//! 

use msp430fr2355::BKMEM;

/// Helper struct with static methods for interpreting the backup memory into more usable forms
pub struct BackupMemory;

macro_rules! as_x {
    ($fn_name: ident, $typ: ty) => {
        #[doc = "Interpret the backup memory as a `&mut"] #[doc = stringify!($typ)] #[doc = "`"]
        #[inline(always)]
        pub fn $fn_name(_reg: BKMEM) -> &'static mut $typ {
            let ptr = msp430fr2355::BKMEM::ptr();
            unsafe { &mut *(ptr as *mut $typ) }
        }
    };
}

impl BackupMemory {
    as_x!(as_u8s,   [u8; 32]);
    as_x!(as_u16s,  [u16;16]);
    as_x!(as_u32s,  [u32; 8]);
    as_x!(as_u64s,  [u64; 4]);
    as_x!(as_u128s, [u128;2]);
    
    as_x!(as_i8s,   [i8; 32]);
    as_x!(as_i16s,  [i16;16]);
    as_x!(as_i32s,  [i32; 8]);
    as_x!(as_i64s,  [i64; 4]);
    as_x!(as_i128s, [i128;2]);
}
