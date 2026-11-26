//! General stress test of ch32-hal's USART receive paths, two USARTs wired across, A's TX to B's
//! RX and B's TX to A's RX: A = USART1, USART4 on the X035; B = USART2 (DMA1 channels 7 and 6).
//! Feature `v203` (CH32V203C8T6 at 144 MHz, up to 9 Mbaud), `v307` (the CH32V307VCT6, set up as
//! the V203), `x035` (CH32X035F8U6 at 48 MHz, up to 3 Mbaud) or `h417` (the CH32H417's V3F core at
//! 100 MHz, up to 6.25 Mbaud, DMA through the DMAMUX): pins, DMA channels, clock and rates in
//! chip.rs.
//!
//! Phases, in this order (decode.py has their names):
//! - 0..=9 controls by registers, before any DMA or interrupt use: a byte, a break, then the
//!   injector's forged framing, parity and noise errors, A to B (0..=4) then B to A (5..=9).
//!   The noise controls (4, 9) do not stop the run: if one fails, the noise phases in that
//!   direction are skipped (and decode.py says so). 14..=17 (measured, after each direction's
//!   controls): forged parity errors from an injector switched from 8E1 to 9 bits just before
//!   the frame.
//! - 90, 91 measured by registers: whether a parity error survives the DMA-like DATAR read after
//!   nothing, a STATR read, a STATR write or a byte sent (90); whether a DMA request the receive
//!   channel could not serve survives a pause, a full set-up or a CPU read (91). 92, 93: whether
//!   an overrun survives that read, and how long `start_uart()` takes.
//! - Feature `restart`, which leaves out everything below (the controls and 90..=93 run): 95,
//!   whether a DMA transfer progresses while the executor sleeps; 88, the DMA ring alone, and
//!   89, `read()`, their cost at 3 Mbaud; 94, restarts of a ring while bytes stream in: new bytes
//!   that follow on, stale ones, gaps or errors. 88 and 89 count bytes only, 94 counts its
//!   outcomes and a read timeout fails it; 88 and 89 also run on main (feature `main-hal`, which
//!   leaves out 94, 18 and 19: their restarts meet main's NDTR write with the channel running).
//! - 18, 19 measured (feature `restart`): parity errors into B with the bad frame first or second
//!   after a restart, B transmitting meanwhile or not; lost ones counted.
//! - 10..=13 `read_until_idle`: ten bytes, then a break, into B (10, 11) and into A (12, 13).
//! - 20..=30 the ring API on B (256 bytes), A sending: first reads, nothing arriving, echo,
//!   `start_uart()` dropping unread bytes, a buffer overrun, a restart with the lap's
//!   transfer-complete flag pending, breaks while waiting in `read()`, while not in `read()`
//!   and right after a burst, one-byte reads, a buffer larger than the ring.
//! - 31 wakes, in a task of its own (no timer of its own, no waker left registered by earlier
//!   reads): A's bytes sent from the timer interrupt, which wakes no task; each read() of B's
//!   ring must complete, so be woken by the driver, within its time (idle line, the DMA's half
//!   and full transfers, an error). Elsewhere the senders share the reader's task and poll it
//!   with their own wakes: the other phases do not check the driver's wakes.
//! - 40..=47 full duplex, both rings 256 bytes, each sender in random chunks with `flush()` after
//!   half of them and random gaps: four rates up to the fastest (chip.rs), each without load
//!   (even phase) then under load (odd phase: the timer interrupt, critical sections and the
//!   background DMA transfer of load.rs). 48: the timer did interrupt a USART handler in those
//!   phases. 49: rings of 255 and 100 bytes, at 1 Mbaud under load, overruns allowed (each
//!   checked real: a 100-byte ring is 1 ms there).
//! - 50..=55 errors into B, A injecting by registers (inject.rs) while B transmits to A: break
//!   (50, A's ring checking B's stream meanwhile), framing, parity, noise, overrun, then framing
//!   with B's transmitter calling `flush()` (55: the error may be lost, counted). Each round
//!   restarts B's ring and puts the bad frame 0, 1, 100, 127, 128, 254 or 255 bytes after the
//!   restart; at 115200 and 1 Mbaud. The error must be the one made, nothing from the bad frame on
//!   may come before it (for the overrun, the frame where the starvation begins: the USART flags
//!   a later one, and bytes in between are counted), and 16 frames sent after a restart must
//!   arrive alone. Noise: bit 3 of the noisy frame is read from three samples around the forged
//!   edge and either way is accepted; a round without an error (noise not made, or its error
//!   lost: they cannot be told apart) is counted and run again, five times at most, and the
//!   noise error must come at every position. The noise phases check the error's kind and
//!   order; lost errors are what the framing and parity phases check (the same flags, clear
//!   sequence and handler branch). 32 noise frames by registers at each rate are counted first,
//!   for the record (NOISE_CAL). With `wfi`, the injector sleeps a whole frame after starting
//!   each one, so that each frame ends, and raises its error, while the executor sleeps.
//!   60..=65: the same into A, B injecting.
//! - 70..=73 a slow reader on B: 32-byte ring at 115200, then 256 bytes at 2 Mbaud, first with
//!   delays well within a lap (no error allowed), then with delays of several laps (overruns
//!   required, and nothing wrong ever handed out).
//! - 80 (feature `long`): full duplex under load, 64 rounds of `chip::LONG_BYTES` each way. The
//!   load can hold a reader back more than a lap: overruns are allowed there.
//!
//! A stream phase fails on a stall (nothing sent or received for a second), not on its speed,
//! which is the chip's: `DUPLEX_MS` keeps how long each of 40..=47 took. Each sender must have
//! sent its whole stream.
//!
//! Overruns are allowed in 49, 71, 73 and 80 only. Each one the ring reports must be real: what
//! left the transmitter more than a ring ahead of the reader (bytes on their way and arriving
//! before the check widen this by a few: an overrun declared with the ring just full would
//! pass; one with the reader caught up, as main's lap-boundary defect, cannot). A USART overrun
//! (ORE) cannot be told from a ring overrun: with the reader more than a ring behind it passes
//! as one, otherwise it stops the run as a false one. After each one
//! the reception restarts, and the checker wants the stream to resume between what had left the
//! transmitter's DMA just before the restart, less the 3 frames on their way, and what had left
//! it just after: no older byte (one of those last 3 frames cannot be told apart), none skipped
//! after (bytes lost while the restarting read() call runs cannot be seen; on an idle line, 24
//! and the tails of 54 and 64 check restarts exactly). The sender adds 16 bytes as long as a
//! restart has not been followed by 8, so that a receiver that stops after an error fails.
//!
//! Feature `wfi` (executor sleeping in WFI between polls): on the V307 and the H417, whose DMA
//! keeps going while the core sleeps, everything above but the costs 88 and 89 (measured with
//! the executor spinning). On the V203 and the X035 the DMA stops while the core sleeps (#201):
//! a ring cannot keep up there (ch32-hal's README says so for the V203), and no phase streams
//! through one (93 times ring restarts on an idle line). The controls, 90, 92 and 93 (95 with `restart`) run, and on the V203, which ch32-hal
//! keeps awake during one-shot DMA transfers (#279), 10..=13.
//!
//! At startup, a chip left as a previous program had it (a probe restarting the core alone:
//! DMA channels or TIM2 still enabled, the system clock still configured) stops the program at
//! once, before any test; BOOT_DMA_ON says why.
//!
//! Every byte a stream phase receives is checked against the stream (check.rs). The first
//! unexpected result stops the senders (DIAG set) and the program at the end of its phase:
//! `STATE` 3, `PHASE` the phase, `RESULT` its failure code (low byte, the F_ constants; round in
//! bits 8..16, detail above), `DIAG` the first failure's details. `STATE` 2: all done. `RESULT`
//! 1: as expected; `DETAIL` counts. `BUILD` says what the image runs (decode.py reads it).
#![no_std]
#![no_main]
// Feature `restart` builds the probes only: the rest is compiled but unused.
#![cfg_attr(feature = "restart", allow(dead_code))]

mod check;
mod chip;
mod inject;
mod load;

use core::future::Future;
use core::ptr::{addr_of_mut, read_volatile, write_volatile};
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering::Relaxed};

use ch32_hal::interrupt::InterruptExt;
use ch32_hal::mode::Async;
use ch32_hal::usart::{self, Config, DataBits, Error, Instance, Parity, RingBufferedUartRx, Uart, UartRx, UartTx};
use ch32_hal::{self as hal, bind_interrupts, pac, peripherals, Peri};
use embassy_executor::Spawner;
use embassy_futures::join::{join, join3, join4};
use embassy_futures::select::{select, Either};
use embassy_time::{with_timeout, Duration, Timer};
use panic_halt as _;

use check::{Rng, Stream};

/// A UART constructor call with the remap parameter that every chip but the H417 takes (0, the
/// default pins): `remap0!(Uart::new(...))`.
#[cfg(not(feature = "h417"))]
macro_rules! remap0 {
    ($ty:ident :: $f:ident ( $($arg:expr),* $(,)? )) => {
        $ty::$f::<0>($($arg),*)
    };
}
#[cfg(feature = "h417")]
macro_rules! remap0 {
    ($ty:ident :: $f:ident ( $($arg:expr),* $(,)? )) => {
        $ty::$f($($arg),*)
    };
}
use inject::{Inj, Kind};

#[cfg(any(feature = "v203", feature = "v307", feature = "h417"))]
bind_interrupts!(struct Irqs {
    USART1 => load::Enter<0>, usart::InterruptHandler<peripherals::USART1>, load::Leave;
    USART2 => load::Enter<1>, usart::InterruptHandler<peripherals::USART2>, load::Leave;
    TIM2 => load::Disturb;
});
#[cfg(feature = "x035")]
bind_interrupts!(struct Irqs {
    USART4 => load::Enter<0>, usart::InterruptHandler<peripherals::USART4>, load::Leave;
    USART2 => load::Enter<1>, usart::InterruptHandler<peripherals::USART2>, load::Leave;
    TIM2_UP => load::Disturb;
});

type A = chip::A;
type B = peripherals::USART2;
type Tx<'a, T> = UartTx<'a, T, Async>;
type Ring<'a, T> = RingBufferedUartRx<'a, T>;

const WFI: bool = cfg!(feature = "wfi");
/// The whole suite runs, unless the executor sleeps on a chip whose DMA stops while the core
/// sleeps (V203, X035): a ring cannot keep up there (ch32-hal's README says so for the V203),
/// and the phases that use one are left out.
const FULL: bool = !WFI || chip::DMA_IN_SLEEP;
/// `read_until_idle` and one-shot writes: also on the V203 with the executor sleeping, which
/// ch32-hal keeps awake during one-shot DMA transfers there (#279); not on the X035.
const ONESHOTS: bool = FULL || cfg!(feature = "v203");

/// What this image runs, for decode.py: bit 0 `wfi`, 1 `long`, 2 `restart`, 3 `main-hal`, bits 8.. the chip
/// (`chip::ID`). In RAM (`mut`), so that it is in the dump with the results.
#[no_mangle]
#[used]
static mut BUILD: u32 = WFI as u32
    | (cfg!(feature = "long") as u32) << 1
    | (cfg!(feature = "restart") as u32) << 2
    | (cfg!(feature = "main-hal") as u32) << 3
    | chip::ID << 8;

// Failure codes (low byte of `RESULT`).
pub const F_TIMEOUT: u32 = 2;
pub const F_DATA: u32 = 3;
pub const F_ERROR: u32 = 4;
pub const F_MISSING: u32 = 5;
pub const F_KIND: u32 = 6;
pub const F_EXCESS: u32 = 7;
pub const F_WRITE: u32 = 8;
pub const F_RESYNC: u32 = 9;
pub const F_SHORT: u32 = 10;
pub const F_CONTROL: u32 = 11;
pub const F_PRECONDITION: u32 = 12;
pub const F_NO_OVERRUN: u32 = 13;
pub const F_NO_PREEMPTION: u32 = 14;
pub const F_INJECTOR: u32 = 15;
pub const F_CONFIG: u32 = 16;
pub const F_API: u32 = 17;
pub const F_PHASE_TIMEOUT: u32 = 18;
pub const F_UNEXPECTED_DATA: u32 = 19;
pub const F_SKIPPED: u32 = 20;
pub const F_TAIL: u32 = 21;
pub const F_ONESHOT: u32 = 22;
pub const F_STALE: u32 = 23;
pub const F_FALSE_OVERRUN: u32 = 24;
pub const F_ABORTED: u32 = 25;
pub const F_WAKE: u32 = 26;
pub const F_STACK: u32 = 27;

const PHASES: usize = 96;

#[no_mangle]
#[used]
static mut STATE: u32 = 0;
#[no_mangle]
#[used]
static mut PHASE: u32 = 0xff;
#[no_mangle]
#[used]
static mut RESULT: [u32; PHASES] = [0; PHASES];
#[no_mangle]
#[used]
static mut DETAIL: [[u32; 2]; PHASES] = [[0; 2]; PHASES];
#[no_mangle]
#[used]
static mut DIAG: [u32; 8] = [0; 8];
/// load.rs counters: timer runs, timer runs inside a USART handler, critical sections, DMA
/// transfers started.
#[no_mangle]
#[used]
static mut LOAD: [u32; 4] = [0; 4];
/// How long each full-duplex phase took, in ms (40..=47), whatever its result.
#[no_mangle]
#[used]
static mut DUPLEX_MS: [u32; 8] = [0; 8];
/// Probe 89, the cost of `read()`, in SysTick ticks: the phase, inside the `read()` polls, those
/// polls, the reads that returned bytes, those bytes; then SysTick's CTLR (bit 2: ticks at
/// HCLK, else HCLK / 8).
#[cfg(feature = "restart")]
#[no_mangle]
#[used]
static mut PROFILE: [u32; 8] = [0; 8];
#[cfg(feature = "restart")]
static PROFILING: AtomicBool = AtomicBool::new(false);
/// Probe 88, the ring alone, in SysTick ticks: the phase, inside the reads that returned bytes,
/// those reads, those bytes, inside the empty reads, those reads, overruns, SysTick's CTLR.
#[no_mangle]
#[used]
static mut RING_PROFILE: [u32; 8] = [0; 8];

/// The senders' chunks and the receivers' read buffers, one per USART (`slot`), outside their
/// futures: a future that holds them is copied whole on the stack where it is built and moved,
/// and with them the main task's frame and the full-duplex phase's took about 10 KiB of stack,
/// more than the 8 KiB the V203 and the X035 had left above their statics. Each USART has one
/// sender and one receiver at a time.
static mut CHUNK: [[u8; 256]; 2] = [[0; 256]; 2];
static mut READ: [[u8; 512]; 2] = [[0; 512]; 2];

/// 0 for USART A, 1 for B.
fn slot<T: Instance>() -> usize {
    if core::any::TypeId::of::<T>() == core::any::TypeId::of::<A>() {
        0
    } else {
        1
    }
}

/// The stack's free bytes that were never written, at worst: the RAM between the statics' end
/// (`_stack_end`, after `.uninit`) and 256 bytes below the stack at startup is painted, and each
/// phase's record looks for the deepest word overwritten. Below `STACK_MARGIN` the phase fails
/// (`F_STACK`): an overflow would overwrite the statics, the task's own state among them, and
/// fail anywhere, or hang. A frame can move the stack pointer past words it never writes: this
/// is what was written, not how deep the stack pointer went. Not on the H417 (448 KiB of RAM:
/// its margin is not at stake there, and the scan of it after each phase would be long).
#[no_mangle]
#[used]
static mut STACK_FREE: u32 = 0;
/// The first word above the painted area.
#[cfg(not(feature = "h417"))]
static mut PAINT_END: usize = 0;
#[cfg(not(feature = "h417"))]
const STACK_MARGIN: u32 = 512;
#[cfg(not(feature = "h417"))]
const STACK_PAINT: u32 = 0x5741_4b45;

#[cfg(not(feature = "h417"))]
extern "C" {
    static mut _stack_end: u32;
}

/// Paints the free RAM below the current stack, 256 bytes below it left alone.
#[cfg(not(feature = "h417"))]
fn stack_paint() {
    unsafe {
        let sp: usize;
        core::arch::asm!("mv {}, sp", out(reg) sp);
        let start = addr_of_mut!(_stack_end) as usize;
        let end = (sp - 256) & !3;
        let mut a = start;
        while a < end {
            write_volatile(a as *mut u32, STACK_PAINT);
            a += 4;
        }
        write_volatile(addr_of_mut!(PAINT_END), end);
        write_volatile(addr_of_mut!(STACK_FREE), end.saturating_sub(start) as u32);
    }
}

/// Updates `STACK_FREE`; whether the margin is kept.
#[cfg(not(feature = "h417"))]
fn stack_kept() -> bool {
    unsafe {
        let start = addr_of_mut!(_stack_end) as usize;
        let end = read_volatile(addr_of_mut!(PAINT_END));
        let mut a = start;
        while a < end && read_volatile(a as *const u32) == STACK_PAINT {
            a += 4;
        }
        let free = (a - start) as u32;
        if free < read_volatile(addr_of_mut!(STACK_FREE)) {
            write_volatile(addr_of_mut!(STACK_FREE), free);
        }
        free >= STACK_MARGIN
    }
}

#[cfg(feature = "h417")]
fn stack_paint() {}

#[cfg(feature = "h417")]
fn stack_kept() -> bool {
    true
}

/// Feature `watch` (diagnosis, on the V203, V307 or X035): after each poll of a phase, the state
/// of the interrupts, SysTick and two DMA channels, so that a run that hangs shows what stopped
/// (a minichlink session found the peripherals at their reset values: it is not the tool for this).
/// Polls so far, CSR 0x800 (MIE bit 3, MPIE bit 7), mcause, mepc, SysTick CTLR, SR, CNTL, CNTH,
/// CMPL, CMPH, PFIC ISR0 and ISR1 (enabled), IPR0 and IPR1 (pending), ITHRESDR, GISR, IACTR0 and
/// IACTR1 (active), DMA1 ISR, A's transmit channel's configuration and count, B's receive
/// channel's. No USART register: reading STATR starts a clear sequence.
#[cfg(feature = "watch")]
#[no_mangle]
#[used]
static mut WATCH: [u32; 23] = [0; 23];

#[cfg(all(feature = "watch", feature = "h417"))]
compile_error!("watch: V203, V307 or X035");

#[cfg(feature = "watch")]
fn watch() {
    let at = |a: usize| unsafe { read_volatile(a as *const u32) };
    let (gintenr, mcause, mepc): (u32, u32, u32);
    unsafe {
        core::arch::asm!("csrr {}, 0x800", out(reg) gintenr);
        core::arch::asm!("csrr {}, mcause", out(reg) mcause);
        core::arch::asm!("csrr {}, mepc", out(reg) mepc);
    }
    let w = [
        unsafe { read_volatile(addr_of_mut!(WATCH[0])) }.wrapping_add(1),
        gintenr,
        mcause,
        mepc,
        at(0xE000_F000),
        at(0xE000_F004),
        at(0xE000_F008),
        at(0xE000_F00C),
        at(0xE000_F010),
        at(0xE000_F014),
        at(0xE000_E000),
        at(0xE000_E004),
        at(0xE000_E020),
        at(0xE000_E024),
        at(0xE000_E040),
        at(0xE000_E04C),
        at(0xE000_E300),
        at(0xE000_E304),
        at(0x4002_0000),
        chip::dma::config(chip::A_TX_CH),
        u32::from(chip::dma::left(chip::A_TX_CH)),
        chip::dma::config(5),
        u32::from(chip::dma::left(5)),
    ];
    for (i, v) in w.into_iter().enumerate() {
        unsafe { write_volatile(addr_of_mut!(WATCH[i]), v) };
    }
}

/// `watch()` after each poll of the phase's future.
#[cfg(feature = "watch")]
struct Watched<F>(F);

#[cfg(feature = "watch")]
impl<F: Future> Future for Watched<F> {
    type Output = F::Output;
    fn poll(self: core::pin::Pin<&mut Self>, cx: &mut core::task::Context<'_>) -> core::task::Poll<F::Output> {
        // SAFETY: the inner future is never moved out of `self`.
        let r = unsafe { self.map_unchecked_mut(|s| &mut s.0) }.poll(cx);
        watch();
        r
    }
}

/// SysTick's count (its low word; the deltas taken here are short).
fn ticks() -> u32 {
    unsafe { read_volatile(0xE000_F008 as *const u32) }
}

#[cfg(feature = "restart")]
fn profile_add(slot: usize, v: u32) {
    if PROFILING.load(Relaxed) {
        unsafe { write_volatile(addr_of_mut!(PROFILE[slot]), read_volatile(addr_of_mut!(PROFILE[slot])).wrapping_add(v)) };
    }
}

/// A future that runs `.1` right after its first poll of `.0`.
struct AfterFirstPoll<F, G>(F, Option<G>);

impl<F: Future, G: FnOnce()> Future for AfterFirstPoll<F, G> {
    type Output = F::Output;
    fn poll(self: core::pin::Pin<&mut Self>, cx: &mut core::task::Context<'_>) -> core::task::Poll<F::Output> {
        // SAFETY: the inner future is never moved out of `self`, and `.1` is not pinned.
        let this = unsafe { self.get_unchecked_mut() };
        let r = unsafe { core::pin::Pin::new_unchecked(&mut this.0) }.poll(cx);
        if let Some(g) = this.1.take() {
            g();
        }
        r
    }
}

/// A future whose polls are timed into `PROFILE[1]` and counted in `PROFILE[2]`.
#[cfg(feature = "restart")]
struct Timed<F>(F);

#[cfg(feature = "restart")]
impl<F: Future> Future for Timed<F> {
    type Output = F::Output;
    fn poll(self: core::pin::Pin<&mut Self>, cx: &mut core::task::Context<'_>) -> core::task::Poll<F::Output> {
        let t0 = ticks();
        // SAFETY: the inner future is never moved out of `self`.
        let r = unsafe { self.map_unchecked_mut(|t| &mut t.0) }.poll(cx);
        profile_add(1, ticks().wrapping_sub(t0));
        profile_add(2, 1);
        r
    }
}
/// At startup (`startup_check`): DMA1 channels found enabled (bit n: channel n + 1), the system
/// clock not as a reset leaves it (bit 8), TIM2 counting (bit 9), DMA2 channels enabled
/// (bits 20-30: the V307's 11, the H417's 8), RCC's software-reset flag
/// (bit 17: a probe session sets it), and bit 18 when any of the others but 17 is set: the
/// program then stopped before any test.
#[no_mangle]
#[used]
static mut BOOT_DMA_ON: u32 = 0;

static DIAG_SET: AtomicBool = AtomicBool::new(false);

/// The first failure's details: DIAG[1] is the failure code. Only failures write it, and once
/// it is set the senders stop.
pub fn diag(words: &[u32]) {
    if DIAG_SET.load(Relaxed) {
        return;
    }
    DIAG_SET.store(true, Relaxed);
    unsafe {
        write_volatile(addr_of_mut!(DIAG[0]), read_volatile(addr_of_mut!(PHASE)));
        for (i, &w) in words.iter().take(7).enumerate() {
            write_volatile(addr_of_mut!(DIAG[i + 1]), w);
        }
    }
}

type Outcome = Result<[u32; 2], u32>;

async fn park() -> ! {
    unsafe {
        write_volatile(addr_of_mut!(LOAD), load::counts());
        write_volatile(addr_of_mut!(STATE), 3);
    }
    loop {
        Timer::after_millis(1000).await;
    }
}

/// Record a phase's outcome and stop on a failure.
async fn record(phase: usize, outcome: Outcome) {
    unsafe { write_volatile(addr_of_mut!(LOAD), load::counts()) };
    let outcome = if !stack_kept() && outcome.is_ok() { Err(F_STACK) } else { outcome };
    match outcome {
        Ok(detail) => unsafe {
            write_volatile(addr_of_mut!(DETAIL[phase]), detail);
            write_volatile(addr_of_mut!(RESULT[phase]), 1);
        },
        Err(code) => {
            diag(&[code]);
            // A check stopped because another failed first: that one is the phase's result.
            let code = if code & 0xff == F_ABORTED { unsafe { read_volatile(addr_of_mut!(DIAG[1])) } } else { code };
            unsafe { write_volatile(addr_of_mut!(RESULT[phase]), code) };
            park().await
        }
    }
}

async fn run(phase: usize, seconds: u64, work: impl Future<Output = Outcome>) {
    unsafe { write_volatile(addr_of_mut!(PHASE), phase as u32) };
    #[cfg(feature = "watch")]
    let work = Watched(work);
    let outcome = with_timeout(Duration::from_secs(seconds), work).await.unwrap_or(Err(F_PHASE_TIMEOUT));
    record(phase, outcome).await;
}

fn skip(phase: usize) {
    unsafe { write_volatile(addr_of_mut!(RESULT[phase]), F_SKIPPED) };
}

struct Board {
    usart_a: Peri<'static, A>,
    usart_b: Peri<'static, B>,
    tx_a: Peri<'static, chip::TxA>,
    rx_a: Peri<'static, chip::RxA>,
    tx_b: Peri<'static, chip::TxB>,
    rx_b: Peri<'static, chip::RxB>,
    tx_dma_a: Peri<'static, chip::TxDmaA>,
    rx_dma_a: Peri<'static, chip::RxDmaA>,
    ch6: Peri<'static, peripherals::DMA1_CH6>,
    ch7: Peri<'static, peripherals::DMA1_CH7>,
}

impl Board {
    /// Both USARTs, opened again for each phase (on this chip, dropping either half of one
    /// turns it off).
    fn open(&mut self, a: Config, b: Config) -> Result<(Uart<'_, A, Async>, Uart<'_, B, Async>), u32> {
        let ua = remap0!(Uart::new(
            self.usart_a.reborrow(),
            self.rx_a.reborrow(),
            self.tx_a.reborrow(),
            Irqs,
            self.tx_dma_a.reborrow(),
            self.rx_dma_a.reborrow(),
            a,
        ))
        .map_err(|_| F_CONFIG)?;
        let ub = remap0!(Uart::new(
            self.usart_b.reborrow(),
            self.rx_b.reborrow(),
            self.tx_b.reborrow(),
            Irqs,
            self.ch7.reborrow(),
            self.ch6.reborrow(),
            b,
        ))
        .map_err(|_| F_CONFIG)?;
        Ok((ua, ub))
    }
}

/// 8N1, or 8E1 (9 bits with the parity bit) with `parity`.
fn cfg(baud: u32, parity: bool) -> Config {
    let mut c = Config::default();
    c.baudrate = baud;
    if parity {
        c.data_bits = DataBits::DataBits9;
        c.parity = Parity::ParityEven;
    }
    c
}

static mut RING_A: [u8; 256] = [0; 256];
static mut RING_B: [u8; 256] = [0; 256];

/// One ring at a time per USART: the previous one is dropped before the next is made.
fn ring_a(len: usize) -> &'static mut [u8] {
    let ring = unsafe { &mut *addr_of_mut!(RING_A) };
    &mut ring[..len]
}

fn ring_b(len: usize) -> &'static mut [u8] {
    let ring = unsafe { &mut *addr_of_mut!(RING_B) };
    &mut ring[..len]
}

fn code(e: Error) -> u32 {
    match e {
        Error::Framing => 1,
        Error::Noise => 2,
        Error::Overrun => 3,
        Error::Parity => 4,
        _ => 5,
    }
}

fn ms(n: u64) -> Duration {
    Duration::from_millis(n)
}

/// `f`, unless the deadline comes first. The deadline is polled first: a future that only
/// completes when polled at the deadline (its waker never called) gives None, unlike
/// `with_timeout`, which polls it once more then.
async fn within<F: Future>(d: Duration, f: F) -> Option<F::Output> {
    match select(Timer::after(d), f).await {
        Either::First(()) => None,
        Either::Second(r) => Some(r),
    }
}

/// A break from `r`'s transmitter, after the frame it is sending.
fn sbk(r: pac::usart::Usart) {
    critical_section::with(|_| r.ctlr1().modify(|w| w.set_sbk(true)));
}

/// Whether B's DMA receive request is on (CTLR3 only: STATR is never read while receiving).
fn receiving_b() -> bool {
    pac::USART2.ctlr3().read().dmar()
}

// ---- Streams ----

/// What a sender and its checker share.
struct Shared {
    /// Bytes handed to the transmitter so far (counted before each write).
    sent: AtomicU32,
    /// Bytes the receiver has checked so far.
    received: AtomicU32,
    /// The sender is finished.
    done: AtomicBool,
    /// Asks an open-ended sender to finish.
    stop: AtomicBool,
    /// The receiver asks for 16 more bytes (`Send::tails`).
    tail: AtomicBool,
}

impl Shared {
    fn new() -> Self {
        Shared {
            sent: AtomicU32::new(0),
            received: AtomicU32::new(0),
            done: AtomicBool::new(false),
            stop: AtomicBool::new(false),
            tail: AtomicBool::new(false),
        }
    }
}

/// Returns once neither direction has sent nor received anything for a second: a stall.
async fn stalled(dirs: &[&Shared]) {
    let count = || dirs.iter().fold(0u32, |a, d| a.wrapping_add(d.sent.load(Relaxed)).wrapping_add(d.received.load(Relaxed)));
    let mut last = count();
    loop {
        Timer::after_millis(1000).await;
        let now = count();
        if now == last {
            return;
        }
        last = now;
    }
}

#[derive(Clone, Copy)]
struct Send {
    seed: u32,
    total: u32,
    max_chunk: u32,
    flush: bool,
    gaps: bool,
    /// After `total`, 16 more bytes each time the receiver asks (`Shared::tail`), until it is
    /// done: a receiver that allows errors asks when a restart has not been followed by 8
    /// bytes yet, so that it can show the reception resumed.
    tails: bool,
}

async fn send<T: Instance>(tx: &mut Tx<'_, T>, sh: &Shared, p: Send, mut rng: Rng) -> Result<(), u32> {
    // SAFETY: one sender per USART at a time (see `CHUNK`).
    let chunk = unsafe { &mut *addr_of_mut!(CHUNK[slot::<T>()]) };
    let mut offset = 0u32;
    let mut result = Ok(());
    // A failure anywhere (DIAG set) stops the traffic, so that the program stops with it.
    while offset < p.total && !sh.stop.load(Relaxed) && !DIAG_SET.load(Relaxed) {
        let len = (1 + rng.below(p.max_chunk)).min(p.total - offset);
        check::fill(p.seed, offset, &mut chunk[..len as usize]);
        offset += len;
        sh.sent.store(offset, Relaxed);
        if tx.write(&chunk[..len as usize]).await.is_err() {
            result = Err(F_WRITE);
            break;
        }
        if p.flush && rng.below(2) == 0 && tx.flush().await.is_err() {
            result = Err(F_WRITE);
            break;
        }
        if p.gaps && rng.below(4) == 0 {
            Timer::after_micros(u64::from(rng.below(200))).await;
        }
    }
    sh.done.store(true, Relaxed);
    if p.tails && result.is_ok() {
        // Until the receiver is done (it sets `stop`), or a failure: at most 64 tails.
        let mut tails = 0;
        while !sh.stop.load(Relaxed) && !DIAG_SET.load(Relaxed) && tails < 64 {
            // Both tasks run on one executor: no request can come between this load and store.
            if !sh.tail.load(Relaxed) {
                Timer::after_micros(500).await;
                continue;
            }
            sh.tail.store(false, Relaxed);
            tails += 1;
            check::fill(p.seed, offset, &mut chunk[..16]);
            offset += 16;
            sh.sent.store(offset, Relaxed);
            if tx.write(&chunk[..16]).await.is_err() {
                result = Err(F_WRITE);
                break;
            }
        }
    }
    result
}

struct Got {
    bytes: u32,
    overruns: u32,
}

/// Reads in random sizes of 1 to `max_buf` bytes (at most 512), waiting up to `delay_us`
/// between reads, until the sender is done and 40 ms bring nothing more. Overruns are allowed
/// with `overruns_ok`, no other error; after each one the reception restarts with the next
/// read, and the stream checker then wants new bytes only, all of those sent after it.
///
/// `wire`, when overruns are allowed: the DMA channel index of the transmitter feeding this
/// receiver, and the ring's capacity. Each overrun of the ring (not flagged ORE by the USART)
/// must then be real: the bytes that had left the transmitter (sent, less what its transfer
/// still holds) more than a ring ahead of the reader; otherwise the ring reported an overrun
/// that was not one.
async fn receive<T: Instance>(
    rx: &mut Ring<'_, T>,
    s: &mut Stream,
    sh: &Shared,
    overruns_ok: bool,
    mut rng: Rng,
    max_buf: u32,
    delay_us: u32,
    wire: Option<(usize, u32)>,
) -> Result<Got, u32> {
    // SAFETY: one receiver per USART at a time (see `READ`).
    let buf = unsafe { &mut *addr_of_mut!(READ[slot::<T>()]) };
    let mut got = Got { bytes: 0, overruns: 0 };
    let mut quiet = 0;
    // An error stopped the reception: the next read restarts it.
    let mut restart = false;
    let mut tails = 0;
    let lag_slot = slot::<T>();
    loop {
        // Another check failed: no more reads (a read would restart a stopped reception).
        if DIAG_SET.load(Relaxed) {
            return Err(F_ABORTED);
        }
        let len = (1 + rng.below(max_buf)) as usize;
        // An error stopped the reception: read() restarts it in its first poll. What had left
        // the transmitter's DMA is taken just before and just after that poll; no other task
        // runs in between (the sender cannot hand out more).
        let restarting = core::mem::take(&mut restart);
        let mut around = (None, None);
        let sent_then = sh.sent.load(Relaxed);
        if restarting {
            s.check_held()?;
            around.0 = wire.map(|(tx_ch, _)| sent_then.wrapping_sub(u32::from(chip::dma::left(tx_ch))));
        }
        let read = AfterFirstPoll(rx.read(&mut buf[..len]), Some(|| {
            if restarting {
                around.1 = wire.map(|(tx_ch, _)| sent_then.wrapping_sub(u32::from(chip::dma::left(tx_ch))));
            }
        }));
        let result = with_timeout(ms(20), read).await;
        if restarting {
            s.restarted(around.0, around.1, sent_then);
        }
        match result {
            Ok(Ok(n)) => {
                quiet = 0;
                got.bytes += n as u32;
                s.take(&buf[..n], sh.sent.load(Relaxed))?;
                sh.received.store(s.next, Relaxed);
            }
            Ok(Err(Error::Overrun)) if overruns_ok => {
                // The ring's overruns must be real. A USART overrun (ORE: the DMA did not take a
                // byte in time) cannot be told from them here: with the reader more than a ring
                // behind it passes as one, otherwise it stops the run as a false overrun.
                if let Some((tx_ch, cap)) = wire {
                    let left = u32::from(chip::dma::left(tx_ch));
                    let sent = sh.sent.load(Relaxed);
                    // The DMA has received no more than what left the transmitter's DMA, and the
                    // reader is at least at `position()`: a real overrun (more than a ring
                    // between them) makes this more than `cap`, and what arrives meanwhile only
                    // adds to it. Signed: below the reader is no overrun either.
                    let ahead = sent.wrapping_sub(left).wrapping_sub(s.position()) as i32;
                    if ahead <= cap as i32 {
                        diag(&[F_FALSE_OVERRUN, s.position(), sent, left, ahead as u32, cap]);
                        return Err(F_FALSE_OVERRUN);
                    }
                }
                quiet = 0;
                got.overruns += 1;
                restart = true;
            }
            Ok(Err(e)) => {
                diag(&[F_ERROR, code(e), s.next, sh.sent.load(Relaxed), lag_slot as u32]);
                return Err(F_ERROR | code(e) << 16);
            }
            Err(_) => {
                // Another check failed: this one stops too (the senders have stopped).
                if DIAG_SET.load(Relaxed) {
                    return Err(F_ABORTED);
                }
                if sh.done.load(Relaxed) {
                    quiet += 1;
                    if quiet == 2 && overruns_ok && s.resyncing() && tails < 64 {
                        // A restart that 8 bytes have not followed yet: 16 more from the sender.
                        tails += 1;
                        quiet = 0;
                        sh.tail.store(true, Relaxed);
                    } else if quiet == 2 {
                        s.finish(sh.sent.load(Relaxed))?;
                        // Releases a sender waiting for tail requests.
                        sh.stop.store(true, Relaxed);
                        return Ok(got);
                    }
                }
            }
        }
        if delay_us != 0 {
            Timer::after_micros(u64::from(rng.below(delay_us))).await;
        }
    }
}

/// One stream from `tx` to `rx`, checked.
async fn stream_once<T: Instance, U: Instance>(
    tx: &mut Tx<'_, T>,
    rx: &mut Ring<'_, U>,
    seed: u32,
    total: u32,
    max_buf: u32,
) -> Outcome {
    let sh = Shared::new();
    let mut s = Stream::new(seed);
    let p = Send { seed, total, max_chunk: 64, flush: false, gaps: false, tails: false };
    let work = join(send(tx, &sh, p, Rng::new(seed ^ 1)), receive(rx, &mut s, &sh, false, Rng::new(seed ^ 2), max_buf, 0, None));
    let (sent, got) = match select(work, stalled(&[&sh])).await {
        Either::First(r) => r,
        Either::Second(()) => return Err(F_TIMEOUT),
    };
    sent?;
    let got = got?;
    whole(&sh, total)?;
    Ok([got.bytes, 0])
}

/// The sender sent its whole stream (and maybe tails): nothing stopped it early.
fn whole(sh: &Shared, total: u32) -> Result<(), u32> {
    let sent = sh.sent.load(Relaxed);
    if sent < total {
        diag(&[F_SHORT, sent, total]);
        return Err(F_SHORT);
    }
    Ok(())
}

/// `data` sent by `tx` comes back whole through `rx` within 200 ms.
async fn echo<T: Instance, U: Instance>(tx: &mut Tx<'_, T>, rx: &mut Ring<'_, U>, data: &[u8]) -> Result<(), u32> {
    let mut buf = [0u8; 16];
    let n = data.len();
    let read = async {
        let mut got = 0;
        while got < n {
            got += rx.read(&mut buf[got..n]).await.map_err(|e| F_ERROR | code(e) << 16)?;
        }
        Ok::<(), u32>(())
    };
    match with_timeout(ms(200), join(read, tx.write(data))).await {
        Err(_) => Err(F_TIMEOUT),
        Ok((_, Err(_))) => Err(F_WRITE),
        Ok((Err(c), _)) => Err(c),
        Ok((Ok(()), Ok(()))) if buf[..n] != *data => Err(F_DATA),
        Ok(_) => Ok(()),
    }
}

/// Nothing arrives within 10 ms.
async fn quiet<T: Instance>(rx: &mut Ring<'_, T>) -> Result<(), u32> {
    let mut buf = [0u8; 16];
    match with_timeout(ms(10), rx.read(&mut buf)).await {
        Err(_) => Ok(()),
        Ok(Ok(n)) => Err(F_UNEXPECTED_DATA | (n as u32) << 16),
        Ok(Err(e)) => Err(F_ERROR | code(e) << 16),
    }
}

// ---- 0..=9: controls by registers ----

#[derive(Clone, Copy)]
enum Control {
    Byte,
    Break,
    Framing,
    Parity,
    Noise,
}

/// One frame from `tx` to `rx` by registers; `rx`'s STATR (PE FE NE ORE, RXNE) and DATAR must
/// show what was made.
async fn control(tx: pac::usart::Usart, rx: pac::usart::Usart, what: Control) -> Outcome {
    // Flags left from before: the RM's clear sequence (nothing else reads these registers yet).
    let _ = rx.statr().read();
    let _ = rx.datar().read();
    let inj = Inj::new(tx);
    let rx_ctlr1 = rx.ctlr1().read();
    // For DIAG if it fails: the transmitter's STATR before, its CTLR1 while sending, its STATR
    // just after the frame was written.
    let tx_before = tx.statr().read().0;
    let (mut tx_ctlr1, mut tx_written) = (tx.ctlr1().read().0, 0);
    let (statr, datar) = match what {
        Control::Byte => (0x20, 0x55),
        Control::Break => (0x22, 0x00),
        Control::Framing => (0x22, 0x55),
        Control::Parity => (0x21, 0x55),
        Control::Noise => (0x24, 0x4b),
    };
    let sent = match what {
        Control::Byte => inj.frame(0x55),
        Control::Break => inj.brk(),
        Control::Framing | Control::Parity => {
            let mut saved = 0;
            if let Control::Parity = what {
                // The receiver in 8E1 for this frame.
                rx.ctlr1().modify(|w| {
                    w.set_m(true);
                    w.set_pce(true);
                    w.set_ps(false);
                });
            }
            // 0x55 and a 9th bit: 0 is a stop bit 0 for the 8N1 receiver; 1 is a wrong parity
            // bit for the 8E1 one (0x55 has four bits set, its even parity bit is 0).
            let ninth = if let Control::Parity = what { 1u16 } else { 0 };
            let on = inj.nine_bits(true, &mut saved);
            tx_ctlr1 = tx.ctlr1().read().0;
            let written = on.and_then(|_| inj.frame(0x55 | ninth << 8));
            tx_written = tx.statr().read().0;
            written.and_then(|_| inj.nine_bits(false, &mut saved))
        }
        Control::Noise => inj.noise(0x4b),
    };
    Timer::after_millis(2).await;
    let rx_statr = rx.statr().read().0;
    let rx_datar = u32::from(rx.datar().read().dr());
    let (got_statr, got_datar) = (rx_statr & 0x2f, rx_datar & 0xff);
    let rx_ctlr1_after = rx.ctlr1().read().0;
    rx.ctlr1().write_value(rx_ctlr1);
    // The noise frame's bit 3 carries an edge in its middle: the receiver's samples read it
    // either way (1 on the V203, 0 on the X035). The noise flag is what is checked.
    let datar_ok = got_datar == datar || (matches!(what, Control::Noise) && got_datar == datar & !0x08);
    // A failed noise control only skips the noise phases: not a failure, so not in DIAG (which
    // would stop every sender after it). Its code keeps STATR and DATAR.
    if (got_statr != statr || !datar_ok || sent.is_err()) && !matches!(what, Control::Noise) {
        let tx_after = tx.statr().read().0;
        diag(&[
            F_CONTROL | sent.err().unwrap_or(0) << 8,
            tx_before,
            tx_ctlr1,
            tx_written,
            tx_after,
            rx_statr | rx_datar << 16,
            rx_ctlr1_after,
        ]);
    }
    sent?;
    if got_statr != statr || !datar_ok {
        return Err(F_CONTROL | got_statr << 8 | got_datar << 16);
    }
    Ok([got_statr, got_datar])
}

/// Returns whether the noise controls passed, A to B and B to A.
async fn controls(b: &mut Board) -> [bool; 2] {
    let (_ua, _ub) = match b.open(cfg(115_200, false), cfg(115_200, false)) {
        Ok(p) => p,
        Err(c) => {
            record(0, Err(c)).await;
            unreachable!()
        }
    };
    let mut noise = [false; 2];
    let directions = [(chip::A_REGS, pac::USART2), (pac::USART2, chip::A_REGS)];
    for (d, &(tx, rx)) in directions.iter().enumerate() {
        let base = 5 * d;
        for (i, what) in [Control::Byte, Control::Break, Control::Framing, Control::Parity].into_iter().enumerate() {
            run(base + i, 2, control(tx, rx, what)).await;
        }
        unsafe { write_volatile(addr_of_mut!(PHASE), (base + 4) as u32) };
        // The forged edge can miss the receiver's samples: up to 5 frames.
        let mut outcome = Err(F_CONTROL);
        for _ in 0..5 {
            outcome = control(tx, rx, Control::Noise).await;
            if outcome.is_ok() {
                break;
            }
        }
        match outcome {
            Ok(detail) => {
                noise[d] = true;
                record(base + 4, Ok(detail)).await;
            }
            // Not a stop: the noise phases in this direction are skipped.
            Err(c) => unsafe { write_volatile(addr_of_mut!(RESULT[base + 4]), c) },
        }
        // 14..=17, measured: a forged parity error from an injector that leaves 8E1 for 9 bits
        // just before the frame, sent at once (14, 16) or 20 µs after the switch (15, 17).
        for (i, settle) in [0u64, 20].into_iter().enumerate() {
            let phase = 14 + 2 * d + i;
            unsafe { write_volatile(addr_of_mut!(PHASE), phase as u32) };
            let misses = parity_switch(tx, rx, settle).await;
            record(phase, misses.map(|m| [m, 20])).await;
        }
    }
    noise
}

/// 20 frames 0x55 with a wrong parity bit, each from an injector switched from 8E1 to 9 bits
/// `settle_us` before it, into a receiver in 8E1 (registers only): how many show no PE.
async fn parity_switch(tx: pac::usart::Usart, rx: pac::usart::Usart, settle_us: u64) -> Result<u32, u32> {
    let inj = Inj::new(tx);
    let (tx_ctlr1, rx_ctlr1) = (tx.ctlr1().read(), rx.ctlr1().read());
    let to_8e1 = |w: &mut pac::usart::regs::Ctlr1| {
        w.set_m(true);
        w.set_pce(true);
        w.set_ps(false);
    };
    rx.ctlr1().modify(to_8e1);
    let mut misses = 0;
    let mut result = Ok(());
    for _ in 0..20 {
        result = inj.idle();
        if result.is_err() {
            break;
        }
        tx.ctlr1().modify(to_8e1);
        let _ = rx.statr().read();
        let _ = rx.datar().read();
        let mut saved = 0;
        result = inj.nine_bits(true, &mut saved).and_then(|_| {
            let start = embassy_time::Instant::now();
            while start.elapsed() < Duration::from_micros(settle_us) {}
            inj.frame(0x155)
        });
        result = result.and_then(|_| inj.nine_bits(false, &mut saved));
        if result.is_err() {
            break;
        }
        Timer::after_millis(1).await;
        let statr = rx.statr().read().0;
        let _ = rx.datar().read();
        if statr & 0x21 != 0x21 {
            misses += 1;
        }
    }
    tx.ctlr1().write_value(tx_ctlr1);
    rx.ctlr1().write_value(rx_ctlr1);
    result.map(|_| misses)
}

// ---- 10..=13: read_until_idle ----

async fn oneshot<T: Instance, U: Instance>(
    tx: &mut Tx<'_, T>,
    tx_regs: pac::usart::Usart,
    rx: &mut UartRx<'_, U, Async>,
    breaking: bool,
) -> Outcome {
    for round in 0..5u32 {
        let mut buf = [0u8; 32];
        let read = with_timeout(ms(50), rx.read_until_idle(&mut buf));
        let send = async {
            Timer::after_millis(1).await;
            if breaking {
                sbk(tx_regs);
                Ok(())
            } else {
                tx.write(&[0x55; 10]).await.map_err(|_| F_WRITE)
            }
        };
        let (got, sent) = join(read, send).await;
        sent?;
        let detail = match got {
            Ok(Ok(10)) if !breaking && buf[..10].iter().all(|&v| v == 0x55) => 0,
            Ok(Err(Error::Framing)) if breaking => 0,
            Ok(Ok(n)) => 1 | (n as u32) << 4,
            Ok(Err(e)) => 2 | code(e) << 4,
            Err(_) => 3,
        };
        if detail != 0 {
            // For DIAG: the first 4 bytes received, and the receive channel's configuration,
            // transfers left and memory address against the buffer's (B's channel, index 5).
            let first = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
            let ch = if core::any::TypeId::of::<U>() == core::any::TypeId::of::<B>() { 5 } else { chip::A_RX_CH };
            diag(&[F_ONESHOT | round << 8, detail, first, chip::dma::config(ch), u32::from(chip::dma::left(ch)), chip::dma::memory(ch), buf.as_ptr() as u32]);
            return Err(F_ONESHOT | round << 8 | detail << 16);
        }
        Timer::after_millis(5).await;
    }
    Ok([5, 0])
}

async fn oneshots(b: &mut Board) {
    let (ua, ub) = match b.open(cfg(115_200, false), cfg(115_200, false)) {
        Ok(p) => p,
        Err(c) => return record(10, Err(c)).await,
    };
    let (mut ta, mut ra) = ua.split();
    let (mut tb, mut rb) = ub.split();
    run(10, 5, oneshot(&mut ta, chip::A_REGS, &mut rb, false)).await;
    run(11, 5, oneshot(&mut ta, chip::A_REGS, &mut rb, true)).await;
    run(12, 5, oneshot(&mut tb, pac::USART2, &mut ra, false)).await;
    run(13, 5, oneshot(&mut tb, pac::USART2, &mut ra, true)).await;
}

// ---- 20..=30: the ring API on B ----

async fn api(b: &mut Board) {
    let (ua, ub) = match b.open(cfg(115_200, false), cfg(115_200, false)) {
        Ok(p) => p,
        Err(c) => return record(20, Err(c)).await,
    };
    let (mut ta, _ra) = ua.split();
    let (_tb, rb) = ub.split();
    let mut rx = rb.into_ring_buffered(ring_b(256));
    let mut buf = [0u8; 64];
    let data = [0x81u8, 0x55, 0x00, 0xff, 0xa5, 0x5a, 0x7e, 0x42];

    // 20: the first read starts reception; bytes that wait in the ring come out whole.
    run(20, 2, async {
        echo(&mut ta, &mut rx, &[1, 2, 3, 4]).await?;
        ta.write(&[5, 6, 7, 8]).await.map_err(|_| F_WRITE)?;
        Timer::after_millis(2).await;
        match with_timeout(ms(20), rx.read(&mut buf)).await {
            Ok(Ok(4)) if buf[..4] == [5, 6, 7, 8] => Ok([0, 0]),
            _ => Err(F_API | 3 << 16),
        }
    })
    .await;

    // 21: nothing arrives, nothing is read.
    run(21, 2, async {
        match with_timeout(ms(20), rx.read(&mut buf)).await {
            Err(_) => Ok([0, 0]),
            Ok(Ok(n)) => Err(F_UNEXPECTED_DATA | (n as u32) << 16),
            Ok(Err(e)) => Err(F_ERROR | code(e) << 16),
        }
    })
    .await;

    // 22: echoes.
    run(22, 2, async {
        for _ in 0..4 {
            echo(&mut ta, &mut rx, &data).await?;
        }
        Ok([4, 0])
    })
    .await;

    // 23: start_uart() drops the bytes not read yet.
    run(23, 2, async {
        ta.write(&[0x3c; 40]).await.map_err(|_| F_WRITE)?;
        Timer::after_millis(5).await;
        rx.start_uart();
        quiet(&mut rx).await?;
        echo(&mut ta, &mut rx, &data).await?;
        Ok([0, 0])
    })
    .await;

    // 24: 768 bytes into the 256-byte ring without reading: Overrun, reception stopped, then
    // reception again.
    run(24, 2, async {
        for _ in 0..8 {
            ta.write(&[0x5a; 96]).await.map_err(|_| F_WRITE)?;
        }
        Timer::after_millis(2).await;
        if !receiving_b() {
            return Err(F_PRECONDITION);
        }
        match with_timeout(ms(100), rx.read(&mut buf)).await {
            Ok(Err(Error::Overrun)) if !receiving_b() => {}
            Ok(Err(e)) => return Err(F_KIND | code(e) << 16),
            Ok(Ok(n)) => return Err(F_UNEXPECTED_DATA | (n as u32) << 16),
            Err(_) => return Err(F_MISSING),
        }
        echo(&mut ta, &mut rx, &data).await?;
        Ok([0, 0])
    })
    .await;

    // 25: a restart while the RX channel's transfer-complete flag is pending (its interrupt
    // masked, the ring filled to the end of its lap): the old flag must not count a lap of the
    // new reception.
    run(25, 2, async {
        chip::B_RX_DMA_IRQ.disable();
        let left = usize::from(chip::dma::left(5));
        let fill = [0x3cu8; 256];
        let wrote = ta.write(&fill[..left]).await;
        Timer::after_millis(2).await;
        let pending = chip::dma::complete(5);
        rx.start_uart();
        let restarted = receiving_b();
        unsafe { chip::B_RX_DMA_IRQ.enable() };
        wrote.map_err(|_| F_WRITE)?;
        if !pending || !restarted {
            return Err(F_PRECONDITION | u32::from(pending) << 16 | u32::from(restarted) << 17);
        }
        match with_timeout(ms(10), rx.read(&mut buf)).await {
            Err(_) => {}
            Ok(Ok(n)) => return Err(F_STALE | (n as u32) << 16),
            Ok(Err(e)) => return Err(F_ERROR | code(e) << 16),
        }
        echo(&mut ta, &mut rx, &[0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8]).await?;
        Ok([left as u32, 0])
    })
    .await;

    // 26: a break on an idle line while the task waits in read(): Framing, not the break's
    // 0x00 first, and nothing after; the error must wake the read (`within`).
    run(26, 4, async {
        for i in 0..10u32 {
            echo(&mut ta, &mut rx, &[0x55; 10]).await.map_err(|c| c | i << 8)?;
            Timer::after_millis(2).await;
            let (got, ()) = join(within(ms(20), rx.read(&mut buf)), async {
                Timer::after_millis(1).await;
                sbk(chip::A_REGS);
            })
            .await;
            match got {
                Some(Err(Error::Framing)) => {}
                Some(Ok(n)) => return Err(F_UNEXPECTED_DATA | i << 8 | (n as u32) << 16),
                Some(Err(e)) => return Err(F_KIND | i << 8 | code(e) << 16),
                None => return Err(F_WAKE | i << 8),
            }
            quiet(&mut rx).await.map_err(|c| c | i << 8)?;
        }
        Ok([10, 0])
    })
    .await;

    // 27: a break while the task is not in read(): the handler stops reception, the next read()
    // reports Framing, then nothing comes and reception runs again.
    run(27, 4, async {
        for i in 0..10u32 {
            echo(&mut ta, &mut rx, &[0x55; 10]).await.map_err(|c| c | i << 8)?;
            Timer::after_millis(2).await;
            if !receiving_b() {
                return Err(F_PRECONDITION | i << 8);
            }
            sbk(chip::A_REGS);
            Timer::after_millis(5).await;
            if receiving_b() {
                return Err(F_API | i << 8 | 1 << 16);
            }
            match with_timeout(ms(20), rx.read(&mut buf)).await {
                Ok(Err(Error::Framing)) => {}
                Ok(Ok(n)) => return Err(F_UNEXPECTED_DATA | i << 8 | (n as u32) << 16),
                Ok(Err(e)) => return Err(F_KIND | i << 8 | code(e) << 16),
                Err(_) => return Err(F_MISSING | i << 8),
            }
            quiet(&mut rx).await.map_err(|c| c | i << 8)?;
            if !receiving_b() {
                return Err(F_API | i << 8 | 2 << 16);
            }
        }
        Ok([10, 0])
    })
    .await;

    // 28: a break right after a burst: up to ten 0x55, never the break's 0x00, then Framing;
    // after it at most the last 0x55 (a break requested while it waited in DATAR goes first).
    run(28, 4, async {
        let mut short = 0;
        for i in 0..10u32 {
            ta.write(&[0x55; 10]).await.map_err(|_| F_WRITE | i << 8)?;
            sbk(chip::A_REGS);
            let (mut ones, mut zeros) = (0, 0);
            loop {
                match with_timeout(ms(20), rx.read(&mut buf)).await {
                    Ok(Ok(n)) => {
                        for &v in &buf[..n] {
                            match v {
                                0x55 if zeros == 0 => ones += 1,
                                0x00 => zeros += 1,
                                _ => return Err(F_DATA | i << 8 | u32::from(v) << 16),
                            }
                        }
                    }
                    Ok(Err(Error::Framing)) => break,
                    Ok(Err(e)) => return Err(F_KIND | i << 8 | code(e) << 16),
                    Err(_) => return Err(F_MISSING | i << 8),
                }
            }
            if ones > 10 || zeros > 0 {
                return Err(F_DATA | i << 8 | (ones | zeros << 4) << 16);
            }
            if ones < 10 {
                short += 1;
            }
            let mut after = 0;
            loop {
                match with_timeout(ms(10), rx.read(&mut buf)).await {
                    Err(_) => break,
                    Ok(Ok(n)) if buf[..n].iter().all(|&v| v == 0x55) && after + n <= 1 => after += n,
                    Ok(Ok(n)) => return Err(F_UNEXPECTED_DATA | i << 8 | (n as u32) << 16),
                    Ok(Err(e)) => return Err(F_ERROR | i << 8 | code(e) << 16),
                }
            }
        }
        Ok([10, short])
    })
    .await;

    // 29: one-byte reads. 30: reads larger than the ring.
    run(29, 5, stream_once(&mut ta, &mut rx, 0x2900_0000, 2_000, 1)).await;
    run(30, 5, stream_once(&mut ta, &mut rx, 0x3000_0000, 5_000, 512)).await;
}


// ---- 31: wakes, in a task of its own ----

/// 31's outcome, written by `wake_task`: 0 while it runs, 1 as expected, else the failure code.
static WAKE_RESULT: AtomicU32 = AtomicU32::new(0);
static mut WAKE_DETAIL: [u32; 2] = [0; 2];

impl Board {
    /// The same peripherals again, for `wake_task`: only while the main task leaves them alone.
    unsafe fn stolen() -> Board {
        Board {
            usart_a: A::steal(),
            usart_b: B::steal(),
            tx_a: chip::TxA::steal(),
            rx_a: chip::RxA::steal(),
            tx_b: chip::TxB::steal(),
            rx_b: chip::RxB::steal(),
            tx_dma_a: chip::TxDmaA::steal(),
            rx_dma_a: chip::RxDmaA::steal(),
            ch6: peripherals::DMA1_CH6::steal(),
            ch7: peripherals::DMA1_CH7::steal(),
        }
    }
}

/// 31: wakes. A task of its own, so that no timer of its own is pending and no earlier read
/// left its waker registered with the USART or the DMA; it sets no timer. A's bytes come from
/// the timer interrupt (`load::feed`), which wakes no task: each read() of B's ring completes
/// only if the driver wakes this task, and how long it took is checked after. 10 bytes then the
/// idle line (the idle-line interrupt), within 5 ms; 400 bytes at once at 115200 (the DMA's half
/// and full transfers every 128 bytes, 11.1 ms; the line idle only at the end), each read
/// within 15 ms; a break (the error interrupt), within 5 ms. A read never woken leaves this task
/// waiting: the main task gives up after 3 s. Detail: reads, the slowest in µs.
#[embassy_executor::task]
async fn wake_task() {
    let outcome = wake_checks().await;
    match outcome {
        Ok(detail) => {
            unsafe { write_volatile(addr_of_mut!(WAKE_DETAIL), detail) };
            WAKE_RESULT.store(1, Relaxed);
        }
        Err(c) => WAKE_RESULT.store(c, Relaxed),
    }
}

async fn wake_checks() -> Outcome {
    // SAFETY: the main task has dropped its UARTs and only waits for this task.
    let mut b = unsafe { Board::stolen() };
    let (ua, ub) = b.open(cfg(115_200, false), cfg(115_200, false))?;
    // A's transmitter enabled: the feeder writes its DATAR.
    let (_ta, _ra) = ua.split();
    let (_tb, rb) = ub.split();
    let mut rx = rb.into_ring_buffered(ring_b(256));
    rx.start_uart();
    load::feed_start();
    let r = wake_reads(&mut rx).await;
    load::stop();
    r
}

async fn wake_reads(rx: &mut Ring<'_, B>) -> Outcome {
    let mut buf = [0u8; 256];
    let (mut reads, mut slowest) = (0u32, 0u32);
    // 400 bytes: wherever the ring starts, a half and a full transfer each have more than 200
    // bytes (17 ms) after them before the next wake if theirs is missing.
    let cases = [(0x3100_0000u32, 10u32, 5_000u64), (0x3101_0000, 400, 15_000), (0, 0, 5_000)];
    for (case, &(seed, len, limit_us)) in cases.iter().enumerate() {
        let case = case as u32;
        load::feed(seed, len, case == 2);
        let mut got = 0u32;
        loop {
            let t0 = embassy_time::Instant::now();
            let r = rx.read(&mut buf).await;
            let us = t0.elapsed().as_micros();
            slowest = slowest.max(us as u32);
            if us > limit_us {
                diag(&[F_WAKE, case, got, us as u32]);
                return Err(F_WAKE | case << 8);
            }
            reads += 1;
            match r {
                Ok(n) if case < 2 => {
                    if got + n as u32 > len || (0..n).any(|i| buf[i] != check::byte(seed, got + i as u32)) {
                        diag(&[F_DATA, case, got, n as u32]);
                        return Err(F_DATA | case << 8);
                    }
                    got += n as u32;
                    if got == len {
                        break;
                    }
                }
                Err(Error::Framing) if case == 2 => break,
                Ok(n) => return Err(F_UNEXPECTED_DATA | (n as u32) << 16),
                Err(e) => return Err(F_KIND | case << 8 | code(e) << 16),
            }
        }
    }
    Ok([reads, slowest])
}

/// 31, from the main task: `wake_task` runs it, and this waits for its outcome, 3 s at most.
async fn wake_phase(spawner: &Spawner) {
    unsafe { write_volatile(addr_of_mut!(PHASE), 31) };
    WAKE_RESULT.store(0, Relaxed);
    match wake_task() {
        Ok(token) => spawner.spawn(token),
        Err(_) => return record(31, Err(F_PRECONDITION)).await,
    }
    let start = embassy_time::Instant::now();
    while WAKE_RESULT.load(Relaxed) == 0 && start.elapsed() < Duration::from_secs(3) {
        Timer::after_millis(5).await;
    }
    let outcome = match WAKE_RESULT.load(Relaxed) {
        // Still waiting in a read that was never woken.
        0 => Err(F_WAKE | 0xff << 8),
        1 => Ok(unsafe { read_volatile(addr_of_mut!(WAKE_DETAIL)) }),
        c => Err(c),
    };
    record(31, outcome).await;
}

// ---- 40..=48: full duplex ----

/// `overruns_ok`: overruns allowed, each checked to be real (with the WFI executor, whose sleeps
/// stop the V203's DMA, and in the long run, whose load can hold the readers back a lap).
async fn duplex(b: &mut Board, phase: usize, baud: u32, total: u32, loaded: bool, overruns_ok: bool, rings: (usize, usize)) -> Outcome {
    let (ua, ub) = b.open(cfg(baud, false), cfg(baud, false))?;
    let (mut ta, ra) = ua.split();
    let (mut tb, rb) = ub.split();
    let mut ra = ra.into_ring_buffered(ring_a(rings.0));
    let mut rb = rb.into_ring_buffered(ring_b(rings.1));
    // Receiving before anything is sent (the first read() would start it, maybe too late).
    ra.start_uart();
    rb.start_uart();
    let seed = 0x4000_0000 + phase as u32;
    let (ab, ba) = (Shared::new(), Shared::new());
    let (mut sab, mut sba) = (Stream::new(seed), Stream::new(!seed));
    let p = Send { seed, total, max_chunk: 256, flush: true, gaps: true, tails: overruns_ok };
    let before = load::counts();
    load::SEEN[0].store(0, Relaxed);
    load::SEEN[1].store(0, Relaxed);
    let work = join4(
        send(&mut ta, &ab, p, Rng::new(seed ^ 1)),
        receive(&mut rb, &mut sab, &ab, overruns_ok, Rng::new(seed ^ 2), 64, 0, Some((chip::A_TX_CH, rings.1 as u32))),
        send(&mut tb, &ba, Send { seed: !seed, ..p }, Rng::new(seed ^ 3)),
        receive(&mut ra, &mut sba, &ba, overruns_ok, Rng::new(seed ^ 4), 64, 0, Some((6, rings.0 as u32))),
    );
    // A phase fails on a stall, not on its speed, which is the chip's: how long it took is kept
    // apart (DUPLEX_MS); `run` caps the whole phase.
    let watched = async {
        match select(work, stalled(&[&ab, &ba])).await {
            Either::First(r) => Ok(r),
            Either::Second(()) => Err(F_TIMEOUT),
        }
    };
    let result = if loaded {
        load::start();
        let r = select(watched, load::critical_sections(Rng::new(seed ^ 5))).await;
        load::stop();
        match r {
            Either::First(r) => r,
            Either::Second(()) => Err(F_TIMEOUT),
        }
    } else {
        watched.await
    };
    let (s1, r1, s2, r2) = match result {
        Ok(r) => r,
        Err(c) => {
            diag(&[c, sab.next, ab.sent.load(Relaxed), sba.next, ba.sent.load(Relaxed)]);
            return Err(c);
        }
    };
    s1?;
    s2?;
    let (r1, r2) = (r1?, r2?);
    whole(&ab, total)?;
    whole(&ba, total)?;
    let after = load::counts();
    let preempted = after[1].wrapping_sub(before[1]);
    Ok([r1.bytes + r2.bytes, (r1.overruns + r2.overruns).min(0xffff) | preempted.min(0xffff) << 16])
}

async fn duplexes(b: &mut Board) {
    let before = load::counts();
    for (i, &(baud, total)) in chip::DUPLEX.iter().enumerate() {
        for (j, loaded) in [false, true].into_iter().enumerate() {
            let phase = 40 + 2 * i + j;
            let start = embassy_time::Instant::now();
            run(phase, 30, duplex(b, phase, baud, total, loaded, false, (256, 256))).await;
            unsafe { write_volatile(addr_of_mut!(DUPLEX_MS[2 * i + j]), start.elapsed().as_millis() as u32) };
        }
    }
    let after = load::counts();
    let (runs, preempted) = (after[0].wrapping_sub(before[0]), after[1].wrapping_sub(before[1]));
    unsafe { write_volatile(addr_of_mut!(PHASE), 48) };
    record(48, if preempted > 0 { Ok([runs, preempted]) } else { Err(F_NO_PREEMPTION) }).await;
    // 49: rings of 255 bytes (A) and 100 bytes (B), capacities that are not powers of two, at
    // 1 Mbaud under load: the ring's positions must wrap right at any capacity. A 100-byte ring
    // is 1 ms at 1 Mbaud, which the load can exceed (seen on the V307: the reader 164 bytes
    // behind): overruns are allowed, each checked real, as in 71 and 73.
    run(49, 30, duplex(b, 49, 1_000_000, 100_000, true, true, (255, 100))).await;
}

// ---- 50..=65: injected errors ----

/// Overrun rounds: the receiving channel's NDTR before (bits 16..) and 20 µs after (bits 0..16)
/// the restart that precedes the tail, the line idle. The restart leaves the channel running:
/// a change is a transfer with nothing received (a request the channel held, served).
#[no_mangle]
#[used]
static mut RESTART_NDTR: [u32; 28] = [0; 28];
static RESTARTS: AtomicU32 = AtomicU32::new(0);

/// The noise frames measured before each noise phase's rounds, B at 115200, B at 1 Mbaud, A at
/// 115200, A at 1 Mbaud, of 32 each: with NE and bit 3 read 0 | with NE and 1 << 8 | without NE
/// and 0 << 16 | without NE and 1 << 24.
#[no_mangle]
#[used]
static mut NOISE_CAL: [u32; 4] = [0; 4];

/// 32 noise frames from `tx` into `rx` by registers, at the rate both are set to, counted by
/// what the receiver saw (a measurement for the record: bit 3, read from three samples around
/// the forged edge, cannot tell whether the noise was made).
async fn noise_reading(tx: pac::usart::Usart, rx: pac::usart::Usart, slot: usize) -> Result<(), u32> {
    let inj = Inj::new(tx);
    let mut counts = [0u32; 4];
    for _ in 0..32 {
        // Flags left from before: the clear sequence (nothing else reads these registers now).
        let _ = rx.statr().read();
        let _ = rx.datar().read();
        inj.noise(0x4b)?;
        Timer::after_millis(2).await;
        let statr = rx.statr().read().0 & 0x2f;
        let b3 = usize::from(rx.datar().read().dr() & 0x08 != 0);
        match statr {
            0x24 => counts[b3] += 1,
            0x20 => counts[2 + b3] += 1,
            _ => {
                diag(&[F_INJECTOR, statr]);
                return Err(F_INJECTOR | statr << 8);
            }
        }
    }
    unsafe { write_volatile(addr_of_mut!(NOISE_CAL[slot]), counts[0] | counts[1] << 8 | counts[2] << 16 | counts[3] << 24) };
    Ok(())
}

const POSITIONS: [usize; 7] = [0, 1, 100, 127, 128, 254, 255];
const BURST: usize = 300;

#[derive(Default)]
struct Tally {
    rounds: u32,
    lost: u32,
    delivered: u32,
    /// Overrun rounds where bytes from the starvation's start on came before the error.
    late: u32,
    /// The first lost round: STATR bits the DUT's handler saw, and its entries above.
    first_lost: u32,
    /// Noise rounds without an error, run again: the noise not made (the receiver's three
    /// samples on one side of the forged edge) or its error lost, which bit 3 cannot tell.
    not_made: u32,
}

/// One injection into `rx`: ring restarted, the burst with its bad frame `k` bytes after the
/// restart, then, after another restart, a tail of 16 good frames that must arrive alone.
/// Returns false for a noise round without an error (to be run again).
async fn round<D: Instance>(
    rx: &mut Ring<'_, D>,
    rx_ch: usize,
    inj: &Inj,
    kind: Kind,
    k: usize,
    seed: u32,
    lost_ok: bool,
    t: &mut Tally,
) -> Result<bool, u32> {
    let mut r = [0u8; BURST + 1];
    let mut m = BURST;
    check::fill(seed, 0, &mut r[..m]);
    for v in r[..m].iter_mut() {
        // A 0x00 only for the break.
        if *v == 0 {
            *v = 1;
        }
    }
    match kind {
        Kind::Break => {
            r.copy_within(k..m, k + 1);
            r[k] = 0;
            m += 1;
        }
        Kind::Noise => r[k] = inject::noise_value(r[k]),
        _ => {}
    }
    rx.start_uart();
    // For DIAG if the error goes missing: the DUT's handler entries and the STATR bits it saw.
    let dut = if rx_ch == chip::A_RX_CH { 0 } else { 1 };
    let entries = load::ENTRIES[dut].load(Relaxed);
    load::SEEN[dut].store(0, Relaxed);
    let regs = if dut == 0 { chip::A_REGS } else { pac::USART2 };
    let ctlr1_start = regs.ctlr1().read().0;
    let done = AtomicBool::new(false);
    let expected = &r[..m];
    // The noise frame's bit 3 is read from three samples around the forged edge: either way.
    let noisy = if kind == Kind::Noise { k } else { usize::MAX };
    let same = |got: usize, b: &[u8]| {
        b.iter().zip(&expected[got..]).enumerate().all(|(i, (&v, &w))| if got + i == noisy { (v ^ w) & !0x08 == 0 } else { v == w })
    };
    let read = async {
        let mut buf = [0u8; 64];
        let mut got = 0usize;
        loop {
            match with_timeout(ms(20), rx.read(&mut buf)).await {
                Ok(Ok(n)) => {
                    if got + n > m || !same(got, &buf[..n]) {
                        let pack = |b: &[u8]| b.iter().take(4).enumerate().fold(0u32, |w, (i, &v)| w | u32::from(v) << (8 * i));
                        let end = (got + 4).min(m);
                        diag(&[
                            F_DATA,
                            k as u32 | (got as u32) << 16,
                            n as u32,
                            pack(&buf[..n]),
                            pack(&expected[got..end]),
                            u32::from(chip::dma::left(rx_ch)),
                            if got > 0 { u32::from(expected[got - 1]) } else { 0x100 },
                        ]);
                        return Err(F_DATA);
                    }
                    got += n;
                }
                Ok(Err(e)) => return Ok((Some(e), got)),
                Err(_) => {
                    if done.load(Relaxed) {
                        return Ok((None, got));
                    }
                }
            }
        }
    };
    let (burst, read) = with_timeout(ms(1000), join(inj.burst(Some(kind), expected, k, &done), read))
        .await
        .map_err(|_| F_TIMEOUT)?;
    burst?;
    let (error, got) = read?;
    // Another check failed meanwhile: nothing more (no restart, no tail).
    if DIAG_SET.load(Relaxed) {
        return Err(F_ABORTED);
    }
    // A noise round without an error: the noise not made, or its error lost; bit 3 cannot tell
    // (the error losses are what the framing and parity phases check). Counted, and the caller
    // runs the round again.
    let quiet_noise = kind == Kind::Noise && error.is_none() && got == m;
    if quiet_noise {
        t.not_made += 1;
    }
    let want = match kind {
        Kind::Break | Kind::Framing => Error::Framing,
        Kind::Parity => Error::Parity,
        Kind::Noise => Error::Noise,
        Kind::Overrun => Error::Overrun,
    };
    match error {
        Some(e) if e == want => {}
        Some(e) => {
            diag(&[F_KIND, k as u32, code(e), got as u32]);
            return Err(F_KIND | code(e) << 16);
        }
        None if quiet_noise => {}
        None if lost_ok && got == m => {
            if t.lost == 0 {
                t.first_lost = load::SEEN[dut].load(Relaxed) | load::ENTRIES[dut].load(Relaxed).wrapping_sub(entries) << 16;
            }
            t.lost += 1
        }
        None => {
            diag(&[
                F_MISSING,
                k as u32,
                got as u32 | inject::NINE_CTLR1.load(Relaxed) << 16,
                load::ENTRIES[dut].load(Relaxed).wrapping_sub(entries),
                load::SEEN[dut].load(Relaxed),
                ctlr1_start,
                regs.ctlr1().read().0 | regs.ctlr3().read().0 << 16,
            ]);
            return Err(F_MISSING);
        }
    }
    // Nothing from the bad frame on may come before the error (`read()` checks for a latched
    // error after each copy). An overrun is flagged at a frame after the starvation began.
    if error.is_some() && got > k {
        if kind != Kind::Overrun {
            diag(&[F_DATA, k as u32, got as u32, 0xbad]);
            return Err(F_DATA | 1 << 16);
        }
        t.late += 1;
    }
    t.rounds += 1;
    t.delivered += got as u32;
    if kind == Kind::Overrun {
        // The transfer that starved the channel must be over before the tail.
        let mut i = 0;
        while load::m2m_busy() && i < 100 {
            Timer::after_micros(100).await;
            i += 1;
        }
        let busy = load::m2m_busy();
        load::m2m_stop();
        if busy {
            return Err(F_PRECONDITION);
        }
    }

    let before = chip::dma::left(rx_ch);
    rx.start_uart();
    if kind == Kind::Overrun {
        let start = embassy_time::Instant::now();
        while start.elapsed() < Duration::from_micros(20) {}
        let after = chip::dma::left(rx_ch);
        let i = RESTARTS.load(Relaxed) as usize;
        if i < 28 {
            unsafe { write_volatile(addr_of_mut!(RESTART_NDTR[i]), u32::from(before) << 16 | u32::from(after)) };
            RESTARTS.store(i as u32 + 1, Relaxed);
        }
    }
    let mut tail = [0u8; 16];
    check::fill(!seed, 0, &mut tail);
    let tail_done = AtomicBool::new(false);
    inj.burst(None, &tail, usize::MAX, &tail_done).await?;
    let mut buf = [0u8; 16];
    let mut got = 0;
    while got < 16 {
        if DIAG_SET.load(Relaxed) {
            return Err(F_ABORTED);
        }
        match with_timeout(ms(20), rx.read(&mut buf[got..])).await {
            Ok(Ok(n)) => got += n,
            Ok(Err(e)) => return Err(F_TAIL | code(e) << 16),
            Err(_) => return Err(F_TAIL | (got as u32) << 24),
        }
    }
    if buf != tail {
        let n = chip::dma::left(rx_ch);
        diag(&[F_TAIL, k as u32, u32::from(buf[0]), u32::from(tail[0]), u32::from(before), u32::from(n)]);
        return Err(F_TAIL | 1 << 16);
    }
    quiet(rx).await.map(|()| !quiet_noise)
}

/// Injection rounds into `rx` (its USART also sending a stream, with `flush()` if `flush`),
/// while `back`, the injector's ring if given, checks that stream.
async fn inject_rounds<D: Instance, J: Instance>(
    tx: &mut Tx<'_, D>,
    rx: &mut Ring<'_, D>,
    rx_ch: usize,
    inj: &Inj,
    back: Option<&mut Ring<'_, J>>,
    kind: Kind,
    flush: bool,
    seed: u32,
    t: &mut Tally,
) -> Result<(), u32> {
    let repeats = if flush { 3 } else { 1 };
    inject_rounds_at(tx, rx, rx_ch, inj, back, kind, flush, flush, true, &POSITIONS, repeats, seed, t).await
}

/// The same with the positions, repeats, whether a lost error is allowed (`lost_ok`) and
/// whether the DUT transmits meanwhile (`stream`) given.
async fn inject_rounds_at<D: Instance, J: Instance>(
    tx: &mut Tx<'_, D>,
    rx: &mut Ring<'_, D>,
    rx_ch: usize,
    inj: &Inj,
    back: Option<&mut Ring<'_, J>>,
    kind: Kind,
    flush: bool,
    lost_ok: bool,
    stream: bool,
    positions: &[usize],
    repeats: usize,
    seed: u32,
    t: &mut Tally,
) -> Result<(), u32> {
    let sh = Shared::new();
    if !stream {
        sh.stop.store(true, Relaxed);
    }
    let mut back_stream = Stream::new(seed ^ 0x77);
    let p = Send { seed: seed ^ 0x77, total: u32::MAX, max_chunk: 64, flush, gaps: false, tails: false };
    let stream = send(tx, &sh, p, Rng::new(seed));
    let check = async {
        match back {
            Some(ring) => receive(ring, &mut back_stream, &sh, false, Rng::new(seed ^ 1), 64, 0, None).await.map(|_| ()),
            None => Ok(()),
        }
    };
    let rounds = async {
        let mut result = Ok(());
        'all: for rep in 0..repeats {
            for (i, &k) in positions.iter().enumerate() {
                // The checker of the other stream failed: no more rounds.
                if DIAG_SET.load(Relaxed) {
                    break 'all;
                }
                // A noise round without an error runs again, five times at most: the noise
                // error must be reported at every position.
                let tries = if kind == Kind::Noise { 5 } else { 1 };
                for attempt in 0..tries {
                    let round_seed = seed.wrapping_add(((rep * 16 + i) as u32) << 8).wrapping_add(attempt << 20);
                    match round(rx, rx_ch, inj, kind, k, round_seed, lost_ok, t).await {
                        Ok(true) => break,
                        Ok(false) if attempt + 1 < tries => {}
                        Ok(false) => {
                            diag(&[F_INJECTOR, k as u32, tries]);
                            result = Err(F_INJECTOR | ((rep * 16 + i) as u32) << 8);
                            break 'all;
                        }
                        Err(c) => {
                            result = Err(c | ((rep * 16 + i) as u32) << 8);
                            break 'all;
                        }
                    }
                }
            }
        }
        sh.stop.store(true, Relaxed);
        result
    };
    let (sent, checked, rounds) = join3(stream, check, rounds).await;
    rounds?;
    sent?;
    checked
}

async fn inject_into_b(b: &mut Board, kind: Kind, flush: bool) -> Outcome {
    let mut t = Tally::default();
    for (n, &baud) in [115_200u32, 1_000_000].iter().enumerate() {
        let parity = kind == Kind::Parity;
        let (ua, ub) = b.open(cfg(baud, parity), cfg(baud, parity))?;
        if kind == Kind::Noise {
            noise_reading(chip::A_REGS, pac::USART2, n).await?;
        }
        let (_ta, ra) = ua.split();
        let (mut tb, rb) = ub.split();
        let mut rb = rb.into_ring_buffered(ring_b(256));
        let inj = Inj::new(chip::A_REGS).paced(baud);
        let seed = 0x5000_0000 | (kind as u32) << 16 | (n as u32) << 12 | u32::from(flush) << 20;
        if kind == Kind::Break {
            let mut ra = ra.into_ring_buffered(ring_a(256));
            ra.start_uart();
            inject_rounds(&mut tb, &mut rb, 5, &inj, Some(&mut ra), kind, flush, seed, &mut t).await?;
        } else {
            let _ra = ra;
            inject_rounds::<B, A>(&mut tb, &mut rb, 5, &inj, None, kind, flush, seed, &mut t).await?;
        }
    }
    Ok([t.rounds | t.lost << 16, t.delivered | t.late << 20 | t.not_made << 26])
}

async fn inject_into_a(b: &mut Board, kind: Kind, flush: bool) -> Outcome {
    let mut t = Tally::default();
    for (n, &baud) in [115_200u32, 1_000_000].iter().enumerate() {
        let parity = kind == Kind::Parity;
        let (ua, ub) = b.open(cfg(baud, parity), cfg(baud, parity))?;
        if kind == Kind::Noise {
            noise_reading(pac::USART2, chip::A_REGS, 2 + n).await?;
        }
        let (mut ta, ra) = ua.split();
        let (_tb, rb) = ub.split();
        let mut ra = ra.into_ring_buffered(ring_a(256));
        let inj = Inj::new(pac::USART2).paced(baud);
        let seed = 0x6000_0000 | (kind as u32) << 16 | (n as u32) << 12 | u32::from(flush) << 20;
        if kind == Kind::Break {
            let mut rb = rb.into_ring_buffered(ring_b(256));
            rb.start_uart();
            inject_rounds(&mut ta, &mut ra, chip::A_RX_CH, &inj, Some(&mut rb), kind, flush, seed, &mut t).await?;
        } else {
            let _rb = rb;
            inject_rounds::<A, B>(&mut ta, &mut ra, chip::A_RX_CH, &inj, None, kind, flush, seed, &mut t).await?;
        }
    }
    Ok([t.rounds | t.lost << 16, t.delivered | t.late << 20 | t.not_made << 26])
}

async fn injections(b: &mut Board, noise: [bool; 2]) {
    let kinds = [Kind::Break, Kind::Framing, Kind::Parity, Kind::Noise, Kind::Overrun];
    for (i, &kind) in kinds.iter().enumerate() {
        if kind == Kind::Noise && !noise[0] {
            skip(50 + i);
            continue;
        }
        run(50 + i, 30, inject_into_b(b, kind, false)).await;
    }
    run(55, 30, inject_into_b(b, Kind::Framing, true)).await;
    for (i, &kind) in kinds.iter().enumerate() {
        if kind == Kind::Noise && !noise[1] {
            skip(60 + i);
            continue;
        }
        run(60 + i, 30, inject_into_a(b, kind, false)).await;
    }
    run(65, 30, inject_into_a(b, Kind::Framing, true)).await;
}

/// 90, measured by registers, no DMA: B in 8E1 receives a frame with a wrong parity bit from A,
/// then DATAR is read (as the DMA would) and STATR tells whether PE survived. Before the frame,
/// after B's flags are cleared: nothing (variant 0), a STATR read (1), a STATR write as
/// `UartTx::write()` does (2), a byte sent by B (3). Detail: PE survivals out of 10 per
/// variant, 4 bits each, variant 0 lowest.
async fn sequence_probe(b: &mut Board) -> Outcome {
    let (_ua, _ub) = b.open(cfg(115_200, true), cfg(115_200, true))?;
    let (a, bb) = (chip::A_REGS, pac::USART2);
    let inj = Inj::new(a);
    let mut survived = 0u32;
    for variant in 0..4u32 {
        let mut n = 0;
        for _ in 0..10 {
            // B's flags cleared: STATR read then DATAR read.
            let _ = bb.statr().read();
            let _ = bb.datar().read();
            match variant {
                1 => {
                    let _ = bb.statr().read();
                }
                2 => bb.statr().write(|w| {
                    w.0 = u32::from(u16::MAX);
                    w.set_tc(false);
                }),
                3 => {
                    bb.datar().write(|w| w.set_dr(0x33));
                    Timer::after_millis(2).await;
                }
                _ => {}
            }
            let mut saved = 0;
            inj.nine_bits(true, &mut saved)
                .and_then(|_| inj.frame(0x155))
                .and_then(|_| inj.nine_bits(false, &mut saved))?;
            Timer::after_millis(1).await;
            let _ = bb.datar().read();
            if bb.statr().read().pe() {
                n += 1;
            }
            let _ = bb.datar().read();
        }
        survived |= n << (4 * variant);
    }
    Ok([survived, 0])
}

/// 92, measured: does an overrun survive a DATAR read, as the DMA's, after B's flags were cleared
/// (STATR read then DATAR read: the sequence closed, variant 0), or after a STATR read (1)? Two
/// frames from A into B, with no read in between: the second one overruns. Detail: overruns
/// still flagged after the DATAR read, out of 10 per variant, 8 bits each; the byte that read
/// gave (the first frame, 0x55, if DATAR kept it).
async fn overrun_probe(b: &mut Board) -> Outcome {
    let (_ua, _ub) = b.open(cfg(115_200, false), cfg(115_200, false))?;
    let (a, bb) = (chip::A_REGS, pac::USART2);
    let inj = Inj::new(a);
    let (mut survived, mut kept) = (0u32, 0u32);
    for variant in 0..2u32 {
        let mut n = 0;
        for _ in 0..10 {
            let _ = bb.statr().read();
            let _ = bb.datar().read();
            if variant == 1 {
                let _ = bb.statr().read();
            }
            inj.frame(0x55).and_then(|_| inj.frame(0xaa)).and_then(|_| inj.idle())?;
            Timer::after_millis(1).await;
            kept = u32::from(bb.datar().read().dr());
            if bb.statr().read().ore() {
                n += 1;
            }
            let _ = bb.datar().read();
        }
        survived |= n << (8 * variant);
    }
    Ok([survived, kept])
}

/// 93, measured: how long `start_uart()` takes on B's ring, line idle: average and longest of
/// 32, in SysTick ticks (see `PROFILE` for their rate).
async fn restart_time_probe(b: &mut Board) -> Outcome {
    let (_ua, ub) = b.open(cfg(115_200, false), cfg(115_200, false))?;
    let (_tb, rb) = ub.split();
    let mut rx = rb.into_ring_buffered(ring_b(256));
    rx.start_uart();
    let (mut sum, mut max) = (0u32, 0u32);
    for _ in 0..32 {
        let t0 = ticks();
        rx.start_uart();
        let dt = ticks().wrapping_sub(t0);
        sum += dt;
        max = max.max(dt);
    }
    Ok([sum / 32, max])
}

/// Probe 94's first gap in its first pass (a measurement, not a failure: not in DIAG).
#[cfg(feature = "restart")]
#[no_mangle]
#[used]
static mut GAP94: [u32; 5] = [0; 5];

/// 94, measured: 200 restarts of B's ring at random times while A streams into it at 2 Mbaud,
/// with `Enter` reading STATR in front of the handler, then 200 without. After each restart, 8
/// bytes are read: new ones that follow each other in the stream from where the reception
/// resumed (contiguous); bytes that had been received before the restart (stale: the restart
/// handed out old data, older than the last `check::IN_FLIGHT` frames, which cannot be told
/// apart); bytes with a gap, or starting past what had left the transmitter just after the
/// restart (lost, no error); or an error. Detail per pass: contiguous | lost << 8 | errors << 16
/// | stale << 24.
#[cfg(feature = "restart")]
async fn restart_probe(b: &mut Board) -> Outcome {
    let (ua, ub) = b.open(cfg(2_000_000, false), cfg(2_000_000, false))?;
    let (mut ta, _ra) = ua.split();
    let (_tb, rb) = ub.split();
    let mut rx = rb.into_ring_buffered(ring_b(256));
    rx.start_uart();
    let seed = 0x9400_0000;
    let sh = Shared::new();
    let p = Send { seed, total: u32::MAX / 2, max_chunk: 256, flush: false, gaps: false, tails: false };
    let mut rng = Rng::new(seed ^ 7);
    let probe = async {
        let mut detail = [0u32; 2];
        for (pass, reads) in [true, false].into_iter().enumerate() {
        load::ENTER_READS_STATR.store(reads, Relaxed);
        let (mut contiguous, mut gaps, mut errors, mut stale, mut seen) = (0u32, 0u32, 0u32, 0u32, 0u32);
        for _ in 0..200 {
            Timer::after_micros(u64::from(50 + rng.below(300))).await;
            load::SEEN[1].store(0, Relaxed);
            let entries = load::ENTRIES[1].load(Relaxed);
            // What had left the transmitter, less the frames on their way: everything before
            // it was received before the restart.
            let sent = sh.sent.load(Relaxed);
            let floor = sent.wrapping_sub(u32::from(chip::dma::left(chip::A_TX_CH))).saturating_sub(check::IN_FLIGHT);
            rx.start_uart();
            // What had left it just after: the first byte received after the restart cannot be
            // further on (the rest goes out after the restart).
            let upper = sent.wrapping_sub(u32::from(chip::dma::left(chip::A_TX_CH)));
            let mut got = [0u8; 8];
            let mut k = 0;
            let mut error = false;
            while k < 8 {
                match with_timeout(ms(20), rx.read(&mut got[k..])).await {
                    Ok(Ok(n)) => k += n,
                    Ok(Err(_)) => {
                        error = true;
                        break;
                    }
                    Err(_) => return Err(F_TIMEOUT),
                }
            }
            let start = if error { None } else { follows(seed, &got, sh.sent.load(Relaxed)) };
            if error {
                errors += 1;
            } else if start.is_some_and(|at| at < floor) {
                stale += 1;
            } else if start.is_some_and(|at| at <= upper) {
                contiguous += 1;
            } else {
                gaps += 1;
                seen |= load::SEEN[1].load(Relaxed);
                if gaps == 1 && pass == 0 {
                    // The first gap: STATR bits and entries of B's handler since the restart,
                    // how far the sender had got, the 8 bytes.
                    unsafe {
                        write_volatile(
                            addr_of_mut!(GAP94),
                            [
                                load::SEEN[1].load(Relaxed),
                                load::ENTRIES[1].load(Relaxed).wrapping_sub(entries),
                                sh.sent.load(Relaxed),
                                u32::from_le_bytes([got[0], got[1], got[2], got[3]]),
                                u32::from_le_bytes([got[4], got[5], got[6], got[7]]),
                            ],
                        )
                    };
                }
            }
        }
        let _ = seen;
        detail[pass] = contiguous | gaps << 8 | errors << 16 | stale << 24;
        }
        load::ENTER_READS_STATR.store(false, Relaxed);
        Ok(detail)
    };
    let (sent, r) = join(send(&mut ta, &sh, p, Rng::new(seed ^ 1)), async {
        let r = probe.await;
        sh.stop.store(true, Relaxed);
        r
    })
    .await;
    sent?;
    r
}

/// 95, measured: 64 bytes written by A's TX DMA at 115200 (5.6 ms on the wire), the executor
/// sleeping or not (feature `wfi`): how long the write took in µs, up to 200 ms, and how many
/// bytes the channel still had to send then; 5 writes, the slowest kept.
#[cfg(feature = "restart")]
async fn dma_sleep_probe(b: &mut Board) -> Outcome {
    let (ua, _ub) = b.open(cfg(115_200, false), cfg(115_200, false))?;
    let (mut ta, _ra) = ua.split();
    let (mut slowest, mut left) = (0u32, 0u32);
    for _ in 0..5 {
        let start = embassy_time::Instant::now();
        let done = with_timeout(ms(200), ta.write(&[0x55; 64])).await;
        let us = start.elapsed().as_micros() as u32;
        if us > slowest {
            slowest = us;
            left = if done.is_ok() { 0 } else { u32::from(chip::dma::left(chip::A_TX_CH)) };
        }
        Timer::after_millis(10).await;
    }
    Ok([slowest, left])
}

/// 88, measured: the DMA ring alone, without `RingBufferedUartRx`: B, a blocking receiver at
/// 3 Mbaud with DMAR set by hand, feeds a `ReadableRingBuffer` of 256 on its RX channel while A
/// sends 100 000 bytes in chunks of up to 256; reads of 64 at most, each timed (`RING_PROFILE`),
/// the task yielding after an empty one; an overrun clears the ring. Nothing checked, so that it
/// also runs on main.
#[cfg(feature = "restart")]
async fn ring_cost_probe(b: &mut Board) -> Outcome {
    let ua = remap0!(Uart::new(
        b.usart_a.reborrow(),
        b.rx_a.reborrow(),
        b.tx_a.reborrow(),
        Irqs,
        b.tx_dma_a.reborrow(),
        b.rx_dma_a.reborrow(),
        cfg(3_000_000, false),
    ))
    .map_err(|_| F_CONFIG)?;
    let (mut ta, _ra) = ua.split();
    let _rb = remap0!(UartRx::new_blocking(b.usart_b.reborrow(), b.rx_b.reborrow(), cfg(3_000_000, false))).map_err(|_| F_CONFIG)?;
    // The options `into_ring_buffered` uses (the struct is non-exhaustive).
    let mut opts = hal::dma::TransferOptions::default();
    opts.half_transfer_ir = true;
    let datar = pac::USART2.datar().as_ptr() as *mut u8;
    let mut ring = unsafe { hal::dma::ReadableRingBuffer::new(b.ch6.reborrow(), chip::B_RX_REQUEST, datar, ring_b(256), opts) };
    ring.start();
    pac::USART2.ctlr3().modify(|w| w.set_dmar(true));
    let sh = Shared::new();
    let p = Send { seed: 0x8800_0000, total: 100_000, max_chunk: 256, flush: false, gaps: false, tails: false };
    let mut prof = [0u32; 8];
    let t0 = ticks();
    let reader = async {
        let mut buf = [0u8; 64];
        let mut idle = 0u32;
        while idle < 2000 {
            let t = ticks();
            let r = ring.read(&mut buf);
            let dt = ticks().wrapping_sub(t);
            match r {
                Ok((n, _)) if n > 0 => {
                    idle = 0;
                    prof[1] = prof[1].wrapping_add(dt);
                    prof[2] += 1;
                    prof[3] += n as u32;
                }
                Ok(_) => {
                    prof[4] = prof[4].wrapping_add(dt);
                    prof[5] += 1;
                    if sh.done.load(Relaxed) {
                        idle += 1;
                    }
                    embassy_futures::yield_now().await;
                }
                Err(_) => {
                    prof[6] += 1;
                    ring.clear();
                }
            }
        }
    };
    let (sent, ()) = join(send(&mut ta, &sh, p, Rng::new(1)), reader).await;
    pac::USART2.ctlr3().modify(|w| w.set_dmar(false));
    prof[0] = ticks().wrapping_sub(t0);
    prof[7] = unsafe { read_volatile(0xE000_F000 as *const u32) };
    unsafe { write_volatile(addr_of_mut!(RING_PROFILE), prof) };
    sent?;
    Ok([prof[3], prof[6]])
}

/// 89, measured: the cost of `read()` alone, B's ring at 3 Mbaud while A sends 100 000 bytes in
/// chunks of up to 256, read 64 bytes at most at a time, nothing checked (so that it also runs
/// on main, whose ring has the defects this branch fixes). In `PROFILE`: the phase, inside the
/// `read()` polls, those polls, the reads that returned bytes, those bytes (errors in DETAIL).
#[cfg(feature = "restart")]
async fn read_cost_probe(b: &mut Board) -> Outcome {
    let (ua, ub) = b.open(cfg(3_000_000, false), cfg(3_000_000, false))?;
    let (mut ta, _ra) = ua.split();
    let (_tb, rb) = ub.split();
    let mut rx = rb.into_ring_buffered(ring_b(256));
    rx.start_uart();
    let sh = Shared::new();
    let p = Send { seed: 0x9600_0000, total: 100_000, max_chunk: 256, flush: false, gaps: false, tails: false };
    let mut errors = 0u32;
    let t0 = ticks();
    PROFILING.store(true, Relaxed);
    let reader = async {
        let mut buf = [0u8; 64];
        let mut quiet = 0;
        loop {
            match with_timeout(ms(20), Timed(rx.read(&mut buf))).await {
                Ok(Ok(n)) => {
                    quiet = 0;
                    profile_add(3, 1);
                    profile_add(4, n as u32);
                }
                Ok(Err(_)) => errors += 1,
                Err(_) => {
                    quiet += 1;
                    if sh.done.load(Relaxed) && quiet >= 2 {
                        break;
                    }
                }
            }
        }
    };
    let (sent, ()) = join(send(&mut ta, &sh, p, Rng::new(1)), reader).await;
    PROFILING.store(false, Relaxed);
    unsafe {
        write_volatile(addr_of_mut!(PROFILE[0]), ticks().wrapping_sub(t0));
        write_volatile(addr_of_mut!(PROFILE[6]), read_volatile(0xE000_F000 as *const u32));
    }
    sent?;
    Ok([read_volatile_profile(4), errors])
}

#[cfg(feature = "restart")]
fn read_volatile_profile(slot: usize) -> u32 {
    unsafe { read_volatile(addr_of_mut!(PROFILE[slot])) }
}

/// Where `got` starts, if it is 8 consecutive bytes of stream `seed` starting within the last
/// 4096 sent (16 laps of the ring: older bytes cannot be in it).
#[cfg(feature = "restart")]
fn follows(seed: u32, got: &[u8; 8], sent: u32) -> Option<u32> {
    let low = sent.saturating_sub(4096);
    let mut start = sent.saturating_sub(8);
    loop {
        if (0..8u32).all(|i| check::byte(seed, start + i) == got[i as usize]) {
            return Some(start);
        }
        if start <= low {
            return None;
        }
        start -= 1;
    }
}

/// 91, measured: does a DMA request the receive channel could not serve survive the channel's
/// stop? B's ring runs; a memory-to-memory transfer at the highest priority holds the DMA while
/// A sends one byte to B, so B's channel keeps the request; then DMAR goes off and the channel
/// is stopped: paused (EN off, variant 0), fully set up again (EN off, CR written, NDTR
/// reloaded, 1), or paused with the byte then read by the CPU (2). Once the DMA is free, the
/// channel is enabled again with DMAR off and the line idle: a transfer in the next 20 µs is
/// the old request. Detail: such transfers out of 5 per variant, 4 bits each.
async fn held_request_probe(b: &mut Board) -> Outcome {
    let (ua, ub) = b.open(cfg(115_200, false), cfg(115_200, false))?;
    let (_ta, _ra) = ua.split();
    let (_tb, rb) = ub.split();
    let mut rx = rb.into_ring_buffered(ring_b(256));
    let inj = Inj::new(chip::A_REGS);
    let usart = pac::USART2;
    let mut fired = 0u32;
    for variant in 0..3u32 {
        let mut n = 0;
        for _ in 0..5 {
            rx.start_uart();
            load::m2m(true);
            inj.frame(0x5a)?;
            Timer::after_micros(200).await;
            let busy = load::m2m_busy();
            critical_section::with(|_| usart.ctlr3().modify(|w| w.set_dmar(false)));
            let config = chip::dma::config(5);
            chip::dma::set_enabled(5, false);
            match variant {
                1 => {
                    chip::dma::set_config(5, config & !1);
                    chip::dma::set_left(5, 256);
                }
                2 => {
                    let _ = usart.datar().read();
                }
                _ => {}
            }
            let mut i = 0;
            while load::m2m_busy() && i < 100 {
                Timer::after_micros(100).await;
                i += 1;
            }
            load::m2m_stop();
            if !busy || i == 100 {
                return Err(F_PRECONDITION | variant << 8);
            }
            let before = chip::dma::left(5);
            chip::dma::set_enabled(5, true);
            let start = embassy_time::Instant::now();
            while start.elapsed() < Duration::from_micros(20) {}
            if chip::dma::left(5) != before {
                n += 1;
            }
            // Flags of the byte, for the next round's start.
            let _ = usart.statr().read();
            let _ = usart.datar().read();
        }
        fired |= n << (4 * variant);
    }
    Ok([fired, 0])
}

/// 18, 19, measured: 20 parity rounds into B with the bad frame first (k 0) and 20 with it
/// second (k 1), at 115200, B transmitting meanwhile (18) or not (19). Lost errors are counted:
/// [lost at k 0 | lost at k 1 << 16, first lost round's handler view].
#[cfg(feature = "restart")]
async fn parity_probe(b: &mut Board, stream: bool) -> Outcome {
    let (ua, ub) = b.open(cfg(115_200, true), cfg(115_200, true))?;
    let (_ta, ra) = ua.split();
    let (mut tb, rb) = ub.split();
    let _ra = ra;
    let mut rb = rb.into_ring_buffered(ring_b(256));
    let inj = Inj::new(chip::A_REGS).paced(115_200);
    let seed = 0x1800_0000 | u32::from(stream) << 20;
    let (mut t0, mut t1) = (Tally::default(), Tally::default());
    inject_rounds_at::<B, A>(&mut tb, &mut rb, 5, &inj, None, Kind::Parity, false, true, stream, &[0], 20, seed, &mut t0).await?;
    inject_rounds_at::<B, A>(&mut tb, &mut rb, 5, &inj, None, Kind::Parity, false, true, stream, &[1], 20, seed ^ 1, &mut t1).await?;
    Ok([t0.lost | t1.lost << 16, if t0.lost != 0 { t0.first_lost } else { t1.first_lost }])
}

// ---- 70..=73: slow reader ----

async fn slow(b: &mut Board, phase: usize, ring: usize, baud: u32, total: u32, delay_us: u32, max_buf: u32, overruns: bool) -> Outcome {
    let (ua, ub) = b.open(cfg(baud, false), cfg(baud, false))?;
    let (mut ta, _ra) = ua.split();
    let (_tb, rb) = ub.split();
    let mut rb = rb.into_ring_buffered(ring_b(ring));
    rb.start_uart();
    let seed = 0x7000_0000 + phase as u32;
    let sh = Shared::new();
    let mut s = Stream::new(seed);
    let overruns_ok = overruns;
    let p = Send { seed, total, max_chunk: 64, flush: false, gaps: false, tails: overruns_ok };
    let work = join(
        send(&mut ta, &sh, p, Rng::new(seed ^ 1)),
        receive(&mut rb, &mut s, &sh, overruns_ok, Rng::new(seed ^ 2), max_buf, delay_us, Some((chip::A_TX_CH, ring as u32))),
    );
    let (sent, got) = match select(work, stalled(&[&sh])).await {
        Either::First(r) => r,
        Either::Second(()) => {
            diag(&[F_TIMEOUT, s.next, sh.sent.load(Relaxed)]);
            return Err(F_TIMEOUT);
        }
    };
    sent?;
    let got = got?;
    whole(&sh, total)?;
    if overruns && got.overruns == 0 {
        return Err(F_NO_OVERRUN);
    }
    Ok([got.bytes, got.overruns])
}

async fn slow_readers(b: &mut Board) {
    // A 32-byte lap is 2.8 ms at 115200; a 256-byte lap is 1.28 ms at 2 Mbaud.
    run(70, 30, slow(b, 70, 32, 115_200, 10_000, 900, 48, false)).await;
    run(71, 30, slow(b, 71, 32, 115_200, 10_000, 8_000, 48, true)).await;
    run(72, 30, slow(b, 72, 256, 2_000_000, 100_000, 400, 300, false)).await;
    run(73, 30, slow(b, 73, 256, 2_000_000, 100_000, 4_000, 300, true)).await;
}

// ---- 80: long run ----

#[no_mangle]
#[used]
static mut LONG_ROUNDS: u32 = 0;

/// Overruns in the long run so far, each one checked to be real.
#[no_mangle]
#[used]
static mut LONG_OVERRUNS: u32 = 0;

async fn long(b: &mut Board) -> Outcome {
    let rates = chip::LONG;
    let mut bytes = 0u32;
    let mut overruns = 0u32;
    for round in 0..64u32 {
        let d = duplex(b, 80, rates[round as usize % 4], chip::LONG_BYTES, true, true, (256, 256)).await.map_err(|c| c | (round & 0xff) << 8)?;
        overruns += d[1] & 0xffff;
        unsafe { write_volatile(addr_of_mut!(LONG_OVERRUNS), overruns) };
        bytes = bytes.wrapping_add(d[0]);
        unsafe { write_volatile(addr_of_mut!(LONG_ROUNDS), round + 1) };
    }
    Ok([bytes, overruns])
}

/// A probe may restart the core alone and leave the peripherals as the previous program had
/// them: DMA channels still enabled (they keep their addresses, and the HAL writes a channel's
/// addresses before it clears EN; DMA1's and, where there is one, DMA2's), TIM2 still counting,
/// the system clock still configured. No test runs on top of that: the program stops (STATE 3,
/// BOOT_DMA_ON bit 18). Returns BOOT_DMA_ON. A peripheral whose clock is off is not running, and
/// is not read (chip::clocked). Not covered: what runs before `main` (the runtime's RAM set-up,
/// an interrupt pending when it enables them), and a channel left enabled under a clock a
/// previous program switched off, which would run again once the HAL switches the clock on.
fn startup_check() -> u32 {
    let mut found = 0u32;
    if chip::clocked::dma1() {
        for ch in 0..chip::DMA1_CHANNELS {
            if chip::dma::enabled(ch) {
                found |= 1 << ch;
            }
        }
    }
    // The system clock as a reset leaves it: HSI (V203, V307, H417), HCLK = HSI / 6 (X035, whose
    // test clock is HSI too). Only the source: the HAL's clock set-up writes the prescalers again.
    #[cfg(not(feature = "x035"))]
    if pac::RCC.cfgr0().read().sws() != pac::rcc::vals::Sw::HSI {
        found |= 1 << 8;
    }
    #[cfg(feature = "x035")]
    if pac::RCC.cfgr0().read().hpre() != pac::rcc::vals::Hpre::DIV6 {
        found |= 1 << 8;
    }
    if chip::clocked::tim2() && pac::TIM2.ctlr1().read().cen() {
        found |= 1 << 9;
    }
    if chip::clocked::dma2() {
        found |= chip::dma::enabled2() << 20;
    }
    let soft = pac::RCC.rstsckr().read().sftrstf();
    pac::RCC.rstsckr().modify(|w| w.set_rmvf(true));
    found | u32::from(soft) << 17 | u32::from(found != 0) << 18
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(#[allow(unused_variables)] spawner: Spawner) {
    let boot = startup_check();
    if boot & 1 << 18 != 0 {
        // The leftover DMA channels and TIM2 off first, so that they overwrite nothing we write.
        chip::dma::stop_all();
        if chip::clocked::tim2() {
            pac::TIM2.ctlr1().modify(|w| w.set_cen(false));
        }
        unsafe { write_volatile(addr_of_mut!(BOOT_DMA_ON), boot) };
        unsafe { write_volatile(addr_of_mut!(STATE), 3) };
        loop {}
    }
    unsafe { write_volatile(addr_of_mut!(BOOT_DMA_ON), boot) };
    stack_paint();
    let mut config = hal::Config::default();
    config.rcc = chip::rcc();
    // Below the timer's P0, so that it can interrupt the DMA handlers too.
    config.dma_interrupt_priority = hal::interrupt::Priority::P8;
    let p = hal::init(config);
    // The V4F (V307): qingke-rt writes INTSYSCR = 0x3 on every V4, nesting on but no preemption
    // bit, and there the P0 timer never interrupts a P8 handler (phase 48 failed). One preemption
    // bit, as qingke-rt sets on the V3F, makes the load nest as it does on the V203 and X035.
    #[cfg(feature = "v307")]
    unsafe {
        core::arch::asm!("li t0, 0x7", "csrw 0x804, t0", out("t0") _);
    }
    load::set_priorities();
    unsafe { write_volatile(addr_of_mut!(STATE), 1) };

    let mut b = Board {
        #[cfg(any(feature = "v203", feature = "v307", feature = "h417"))]
        usart_a: p.USART1,
        #[cfg(any(feature = "v203", feature = "v307", feature = "h417"))]
        tx_a: p.PA9,
        #[cfg(any(feature = "v203", feature = "v307", feature = "h417"))]
        rx_a: p.PA10,
        #[cfg(any(feature = "v203", feature = "v307", feature = "h417"))]
        tx_dma_a: p.DMA1_CH4,
        #[cfg(any(feature = "v203", feature = "v307", feature = "h417"))]
        rx_dma_a: p.DMA1_CH5,
        #[cfg(feature = "x035")]
        usart_a: p.USART4,
        #[cfg(feature = "x035")]
        tx_a: p.PB0,
        #[cfg(feature = "x035")]
        rx_a: p.PB1,
        #[cfg(feature = "x035")]
        tx_dma_a: p.DMA1_CH1,
        #[cfg(feature = "x035")]
        rx_dma_a: p.DMA1_CH8,
        usart_b: p.USART2,
        tx_b: p.PA2,
        rx_b: p.PA3,
        ch6: p.DMA1_CH6,
        ch7: p.DMA1_CH7,
    };

    let noise = controls(&mut b).await;
    run(90, 10, sequence_probe(&mut b)).await;
    run(92, 10, overrun_probe(&mut b)).await;
    run(93, 10, restart_time_probe(&mut b)).await;
    #[cfg(feature = "restart")]
    run(95, 10, dma_sleep_probe(&mut b)).await;
    // The costs are measured with the executor spinning only.
    #[cfg(feature = "restart")]
    if !WFI {
        run(88, 20, ring_cost_probe(&mut b)).await;
        run(89, 20, read_cost_probe(&mut b)).await;
    }
    // Both need the DMA while the executor may sleep (91's memory-to-memory transfer).
    if FULL {
        run(91, 10, held_request_probe(&mut b)).await;
        // On ch32-hal's main, a restart while the ring receives writes NDTR with the channel
        // running: the DMA goes on from its address (measured on the V203), past the ring when
        // restarts come closer than a lap. No 94, 18 or 19 there (main-hal).
        #[cfg(all(feature = "restart", not(feature = "main-hal")))]
        run(94, 20, restart_probe(&mut b)).await;
    }
    // Measurements of lost parity errors right after a restart (lost ones counted, not
    // refused): with the probes, to keep the full image within the X035's Flash; 50..=52 and
    // 60..=62 check the same at offsets 0 and 1 and refuse any loss.
    #[cfg(all(feature = "restart", not(feature = "main-hal")))]
    if FULL {
        run(18, 30, parity_probe(&mut b, true)).await;
        run(19, 30, parity_probe(&mut b, false)).await;
    }
    // Feature `restart`: the probes only, a smaller image (the X035 has 62 KiB of Flash).
    #[cfg(not(feature = "restart"))]
    {
        if ONESHOTS {
            oneshots(&mut b).await;
        }
        if FULL {
            api(&mut b).await;
            wake_phase(&spawner).await;
            duplexes(&mut b).await;
            injections(&mut b, noise).await;
            slow_readers(&mut b).await;
            if cfg!(feature = "long") {
                run(80, 900, long(&mut b)).await;
            }
        }
    }
    #[cfg(feature = "restart")]
    let _ = noise;

    unsafe {
        write_volatile(addr_of_mut!(LOAD), load::counts());
        write_volatile(addr_of_mut!(STATE), 2);
    }
    loop {
        Timer::after_millis(1000).await;
    }
}
