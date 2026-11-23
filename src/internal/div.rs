//! 64-bit division for configuration code.
//!
//! On RV32 a `u64` division that the compiler cannot simplify calls compiler-builtins'
//! `__udivdi3`, which is fast but takes several hundred bytes. The watchdogs and the software I2C
//! divide 64-bit values only while being configured, where speed does not matter: this division
//! finds one quotient bit per round, gives the same result, and takes 120 to 140 bytes. It always
//! runs 64 rounds, so it does not belong on a hot path.

// Which helpers a build uses depends on the chip's peripherals.
#![allow(dead_code)]

/// `n / d`, rounded up if `round_up`. Panics if `d` is zero, as `/` does.
///
/// One out-of-line core that returns a single `u64`: on RV32 a returned pair would go through
/// memory at every call.
#[inline(never)]
fn div_round(n: u64, d: u64, round_up: bool) -> u64 {
    assert!(d != 0, "attempt to divide by zero");
    // The dividend's bits leave `quotient` at the top while the quotient's bits enter at the
    // bottom; only constant shifts, which stay small on RV32.
    let (mut quotient, mut remainder) = (n, 0u64);
    for _ in 0..64 {
        // The remainder never exceeds the dividend bits moved in so far, at most 63 of them before
        // this shift, so doubling it cannot overflow.
        remainder = remainder << 1 | quotient >> 63;
        quotient <<= 1;
        if remainder >= d {
            remainder -= d;
            quotient |= 1;
        }
    }
    // A nonzero remainder means d >= 2, so the quotient is below u64::MAX and the + 1 fits.
    quotient + u64::from(round_up && remainder != 0)
}

/// `n / d`, rounded down.
pub(crate) fn div(n: u64, d: u64) -> u64 {
    div_round(n, d, false)
}

/// `n / d`, rounded up.
pub(crate) fn div_ceil(n: u64, d: u64) -> u64 {
    div_round(n, d, true)
}
