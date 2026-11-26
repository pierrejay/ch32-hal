//! The injector: the other USART, driven by registers, sends frames with exactly one bad
//! frame at a chosen position. Its own receiver is not looked at while it injects (polling its
//! STATR would end its own clear sequences), except in the break phases, which check that the
//! receiver it hosts keeps receiving cleanly.
//!
//! How each error is made, the receiver being at baud B:
//! - break: SBK, after the previous frame;
//! - framing: 9 data bits, no parity, against an 8N1 receiver, the 9th bit 0 where the
//!   receiver expects its stop bit; the 9th bit is 1 in the good frames;
//! - parity: 9 data bits, no parity, against an 8E1 receiver, the 9th bit being the parity
//!   bit, wrong in the bad frame;
//! - noise: two back-to-back 8N1 frames at 2B make one receiver frame whose bit b3 has an edge
//!   between its three samples (taken at 7, 8 and 9 sixteenths of the bit, so the first is in
//!   the first half): the value is the second half, with NE;
//! - overrun: before the bad position, a memory-to-memory DMA transfer at the highest priority
//!   stops the receiver's DMA channel for about a millisecond, while the frames keep coming.

use core::sync::atomic::{AtomicBool, Ordering::Relaxed};

use ch32_hal::pac;
use embassy_futures::yield_now;
use embassy_time::Timer;

use crate::{load, F_INJECTOR};

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    Break,
    Framing,
    Parity,
    Noise,
    Overrun,
}

/// The injector's CTLR1 just after it switched to 9 bits for a burst (for DIAG).
pub static NINE_CTLR1: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

pub struct Inj {
    r: pac::usart::Usart,
    brr: u32,
    /// After each frame of a burst: 0, yield (the receiver's task runs, the executor never
    /// sleeps); else sleep this many µs, a whole frame (`paced`).
    pace_us: u32,
}

/// Spin until `f` holds, about 15 ms at most.
fn wait(mut f: impl FnMut() -> bool) -> Result<(), u32> {
    for _ in 0..200_000 {
        if f() {
            return Ok(());
        }
    }
    Err(F_INJECTOR)
}

/// The two frames at twice the baud rate that a receiver reads as `v` with a noise error. `v`
/// must have b3 set (the bit with the edge reads as the first frame's stop bit) and b4 clear
/// (the second frame's start bit).
pub fn noise_frames(v: u8) -> (u8, u8) {
    let b = |i: u8| (v >> i) & 1;
    // Receiver bit j covers bits 2j and 2j+1 at 2B. First frame: start, d0 (0, clean start
    // bit), d1 = d2 = b0, d3 = d4 = b1, d5 = d6 = b2, d7 = 0 with its stop bit 1: b3, with NE.
    let first = b(0) << 1 | b(0) << 2 | b(1) << 3 | b(1) << 4 | b(2) << 5 | b(2) << 6;
    // Second frame: its start bit and e0 = 0 make b4, then e1 = e2 = b5, e3 = e4 = b6,
    // e5 = e6 = b7, e7 = 1 and its stop bit make the receiver's stop bit.
    let second = b(5) << 1 | b(5) << 2 | b(6) << 3 | b(6) << 4 | b(7) << 5 | b(7) << 6 | 1 << 7;
    (first, second)
}

/// The value a noise frame carries for `v`: b3 set, b4 clear.
pub fn noise_value(v: u8) -> u8 {
    (v | 0x08) & !0x10
}

fn even_parity(v: u8) -> u16 {
    (v.count_ones() & 1) as u16
}

impl Inj {
    pub fn new(r: pac::usart::Usart) -> Self {
        Inj { r, brr: r.brr().read().0, pace_us: 0 }
    }

    /// With the executor sleeping in WFI (`wfi`), the burst sleeps a whole frame (11 bits at
    /// `baud`, and 2 µs) after starting each frame instead of yielding, so that the frame ends,
    /// where its error is raised, while the executor sleeps.
    pub fn paced(mut self, baud: u32) -> Self {
        if crate::WFI {
            self.pace_us = 11_000_000 / baud + 2;
        }
        self
    }

    /// One frame (8 or 9 bits as configured) once the data register is free.
    pub fn frame(&self, v: u16) -> Result<(), u32> {
        wait(|| self.r.statr().read().txe())?;
        self.r.datar().write(|w| w.set_dr(v));
        Ok(())
    }

    /// Until the last frame has left the line.
    pub fn idle(&self) -> Result<(), u32> {
        wait(|| self.r.statr().read().tc())
    }

    /// A break after the frame being sent.
    pub fn brk(&self) -> Result<(), u32> {
        self.brk_start()?;
        self.brk_end()
    }

    fn brk_start(&self) -> Result<(), u32> {
        wait(|| self.r.statr().read().txe())?;
        critical_section::with(|_| self.r.ctlr1().modify(|w| w.set_sbk(true)));
        Ok(())
    }

    /// SBK is cleared by the hardware during the break's stop bit.
    fn brk_end(&self) -> Result<(), u32> {
        wait(|| !self.r.ctlr1().read().sbk())
    }

    /// 9 data bits, no parity (`true`), or back to the configuration it had (`false`).
    pub fn nine_bits(&self, on: bool, saved: &mut u32) -> Result<(), u32> {
        self.idle()?;
        critical_section::with(|_| {
            self.r.ctlr1().modify(|w| {
                if on {
                    *saved = w.0;
                    w.set_m(true);
                    w.set_pce(false);
                } else {
                    let s = pac::usart::regs::Ctlr1(*saved);
                    w.set_m(s.m());
                    w.set_pce(s.pce());
                }
            })
        });
        Ok(())
    }

    /// The injector's own receiver off (`false`) or back on. A format or baud rate change made
    /// while it is receiving (the other USART transmits to it) can still be the old one for the
    /// next frame sent: measured 17 times out of 20 for 8E1 to 9 bits on the CH32V203.
    fn receiver(&self, on: bool) {
        critical_section::with(|_| self.r.ctlr1().modify(|w| w.set_re(on)));
    }

    /// A frame read as `v` with a noise error (`v` as `noise_value` makes it).
    pub fn noise(&self, v: u8) -> Result<(), u32> {
        let started = self.noise_start(v);
        let ended = self.noise_end();
        started.and(ended)
    }

    /// Both frames of a noise frame back to back at 2B, nothing else in between: the second is
    /// queued while the first shifts out.
    fn noise_start(&self, v: u8) -> Result<(), u32> {
        let (first, second) = noise_frames(v);
        self.idle()?;
        critical_section::with(|_| {
            self.r.brr().write(|w| w.0 = self.brr / 2);
            self.frame(u16::from(first)).and_then(|_| self.frame(u16::from(second)))
        })
    }

    /// Once both have left the line, the rate back.
    fn noise_end(&self) -> Result<(), u32> {
        let idle = self.idle();
        self.r.brr().write(|w| w.0 = self.brr);
        idle
    }

    /// Frames `r`, the one at `k` bad as `kind` says. Yields (or sleeps, `paced`) after each
    /// frame so that the receiver's task runs meanwhile, and stops at a failure (DIAG set).
    /// `done` is set at the end, once the last frame has left the line.
    pub async fn burst(&self, kind: Option<Kind>, r: &[u8], k: usize, done: &AtomicBool) -> Result<(), u32> {
        let mut saved = 0;
        let nine = matches!(kind, Some(Kind::Framing | Kind::Parity));
        let reconfigures = nine || kind == Some(Kind::Noise);
        let receiving = self.r.ctlr1().read().re();
        if reconfigures {
            self.receiver(false);
        }
        let mut result = if nine { self.nine_bits(true, &mut saved) } else { Ok(()) };
        if nine {
            NINE_CTLR1.store(self.r.ctlr1().read().0, Relaxed);
        }
        for (i, &v) in r.iter().enumerate() {
            if result.is_err() || crate::DIAG_SET.load(Relaxed) {
                break;
            }
            let bad = i == k;
            result = match kind {
                Some(Kind::Break) if bad => self.brk_start(),
                // A break inserts a 0x00 frame: `r` has it at `k`.
                Some(Kind::Framing) => self.frame(u16::from(v) | u16::from(!bad) << 8),
                Some(Kind::Parity) => self.frame(u16::from(v) | (even_parity(v) ^ u16::from(bad)) << 8),
                Some(Kind::Noise) if bad => self.noise_start(v),
                Some(Kind::Overrun) if bad => {
                    load::m2m(true);
                    self.frame(u16::from(v))
                }
                _ => self.frame(u16::from(v)),
            };
            if self.pace_us == 0 {
                yield_now().await;
            } else {
                Timer::after_micros(u64::from(self.pace_us)).await;
            }
            // The break and the noise frame end here, after the sleep (or at once).
            match kind {
                Some(Kind::Break) if bad => result = result.and_then(|_| self.brk_end()),
                Some(Kind::Noise) if bad => {
                    let ended = self.noise_end();
                    result = result.and(ended);
                }
                _ => {}
            }
        }
        if result.is_ok() {
            result = self.idle();
        }
        if nine {
            let restored = self.nine_bits(false, &mut saved);
            result = result.and(restored);
        }
        if reconfigures {
            self.receiver(receiving);
        }
        done.store(true, Relaxed);
        result
    }
}
