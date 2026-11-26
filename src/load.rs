//! Load around the USARTs while they run: a timer interrupt that preempts their interrupt
//! handlers, critical sections in the application, and a memory-to-memory DMA transfer that
//! keeps the bus busy (or, at the highest priority, starves the other channels).
//!
//! Priorities: the USART and DMA interrupts are at P8, the timer at P0. On the QingKe V4B the
//! top priority bit is the preemption bit, so the timer can interrupt their handlers.
//! `TIMER_IN_USART` counts the times it did, inside a USART handler.

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering::Relaxed};

use ch32_hal::interrupt::typelevel::{Handler, Interrupt};
use ch32_hal::interrupt::InterruptExt;
use ch32_hal::pac;
use embassy_time::{Duration, Instant, Timer};

use crate::check::Rng;

pub static TIMER_RUNS: AtomicU32 = AtomicU32::new(0);
pub static TIMER_IN_USART: AtomicU32 = AtomicU32::new(0);
pub static CRITICAL_SECTIONS: AtomicU32 = AtomicU32::new(0);
pub static M2M_RUNS: AtomicU32 = AtomicU32::new(0);
static IN_USART: AtomicU32 = AtomicU32::new(0);
static SPIN: AtomicU32 = AtomicU32::new(1);

fn bump(counter: &AtomicU32) {
    // Each counter has a single writer: load and store are enough (no atomic RMW on rv32imc).
    counter.store(counter.load(Relaxed).wrapping_add(1), Relaxed);
}

/// Entries into USART A's (0) and USART B's (1) handlers, and every STATR bit they found set
/// (`SEEN` is reset by whoever looks at it).
pub static ENTRIES: [AtomicU32; 2] = [AtomicU32::new(0), AtomicU32::new(0)];
pub static SEEN: [AtomicU32; 2] = [AtomicU32::new(0), AtomicU32::new(0)];
/// Whether `Enter` reads STATR for `SEEN`: only in probe 94's first pass. That read starts the
/// clear sequence before the handler's own read: a DMA read in between ends it, and the handler
/// never sees the error (on a CH32X035 at 2 Mbaud, 43 restarts of 200 lost bytes without an
/// error that way, none without it).
pub static ENTER_READS_STATR: AtomicBool = AtomicBool::new(false);

/// Bound before the HAL's USART handler of A (`N` 0) or B (1); both share a
/// priority, so they never nest. Its STATR read comes just before the handler's own, which
/// starts the same clear sequence.
pub struct Enter<const N: usize>;
/// Bound after it.
pub struct Leave;

impl<I: Interrupt, const N: usize> Handler<I> for Enter<N> {
    unsafe fn on_interrupt() {
        IN_USART.store(1, Relaxed);
        bump(&ENTRIES[N]);
        if ENTER_READS_STATR.load(Relaxed) {
            let r = if N == 0 { crate::chip::A_REGS } else { pac::USART2 };
            SEEN[N].store(SEEN[N].load(Relaxed) | r.statr().read().0, Relaxed);
        }
    }
}

impl<I: Interrupt> Handler<I> for Leave {
    unsafe fn on_interrupt() {
        IN_USART.store(0, Relaxed);
    }
}

/// The feeder (`feed`): A's transmitter driven from the timer interrupt, which wakes no task,
/// so that a reader waiting for these bytes can only be woken by its driver. Bytes of stream
/// `FEED_SEED` from offset `FEED_POS` to `FEED_LEN`, then a break if `FEED_BREAK`.
static FEED_ON: AtomicBool = AtomicBool::new(false);
static FEED_SEED: AtomicU32 = AtomicU32::new(0);
static FEED_POS: AtomicU32 = AtomicU32::new(0);
static FEED_LEN: AtomicU32 = AtomicU32::new(0);
static FEED_BREAK: AtomicBool = AtomicBool::new(false);

/// Starts the timer, without the background DMA transfer, for the feeder alone.
pub fn feed_start() {
    FEED_ON.store(false, Relaxed);
    timer_start();
}

/// `len` bytes of stream `seed` from A's transmitter, from the timer interrupt (at most one
/// each 40 µs, as the transmitter takes them), then a break if `brk`.
pub fn feed(seed: u32, len: u32, brk: bool) {
    // With the timer interrupt held off: it never sees half of the new state.
    critical_section::with(|_| {
        FEED_SEED.store(seed, Relaxed);
        FEED_POS.store(0, Relaxed);
        FEED_LEN.store(len, Relaxed);
        FEED_BREAK.store(brk, Relaxed);
        FEED_ON.store(true, Relaxed);
    });
}

fn feed_tick() {
    if !FEED_ON.load(Relaxed) {
        return;
    }
    let r = crate::chip::A_REGS;
    if !r.statr().read().txe() {
        return;
    }
    let pos = FEED_POS.load(Relaxed);
    if pos < FEED_LEN.load(Relaxed) {
        r.datar().write(|w| w.set_dr(u16::from(crate::check::byte(FEED_SEED.load(Relaxed), pos))));
        FEED_POS.store(pos + 1, Relaxed);
    } else {
        if FEED_BREAK.load(Relaxed) {
            r.ctlr1().modify(|w| w.set_sbk(true));
        }
        FEED_ON.store(false, Relaxed);
    }
}

/// TIM2 update interrupt, every 40 µs while the load runs: the feeder, then up to 3 µs of work,
/// timed on TIM2's own 1 MHz count since the update, so that it lasts as long on every chip.
pub struct Disturb;

impl Handler<crate::chip::TimerIrq> for Disturb {
    unsafe fn on_interrupt() {
        pac::TIM2.intfr().write(|_| {});
        feed_tick();
        bump(&TIMER_RUNS);
        if IN_USART.load(Relaxed) != 0 {
            bump(&TIMER_IN_USART);
        }
        let mut x = SPIN.load(Relaxed);
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        SPIN.store(x, Relaxed);
        // CNT's width and register type differ between the chips' timers: its low 16 bits.
        let cnt = pac::TIM2.cnt().as_ptr() as *const u32;
        let end = x % 4;
        while core::ptr::read_volatile(cnt) & 0xffff < end {}
    }
}

pub fn set_priorities() {
    use ch32_hal::interrupt::Priority;
    crate::chip::A_IRQ.set_priority(Priority::P8);
    ch32_hal::interrupt::USART2.set_priority(Priority::P8);
    crate::chip::TIMER_IRQ.set_priority(Priority::P0);
}

pub fn start() {
    timer_start();
    m2m(false);
}

fn timer_start() {
    crate::chip::enable_timer();
    let t = pac::TIM2;
    t.ctlr1().write(|_| {});
    // To 1 MHz, 40 counts.
    t.psc().write_value(crate::chip::TIMER_PSC);
    t.atrlr().write_value(39);
    t.swevgr().write(|w| w.set_ug(true));
    t.intfr().write(|_| {});
    t.dmaintenr().write(|w| w.set_uie(true));
    t.ctlr1().write(|w| w.set_cen(true));
    unsafe { crate::chip::TIMER_IRQ.enable() };
}

pub fn stop() {
    FEED_ON.store(false, Relaxed);
    crate::chip::TIMER_IRQ.disable();
    pac::TIM2.ctlr1().write(|_| {});
    pac::TIM2.dmaintenr().write(|_| {});
    pac::TIM2.intfr().write(|_| {});
    m2m_stop();
}

/// Critical sections of up to 20 µs every 20 to 300 µs, as an application might take; also
/// restarts the background DMA transfer when it is done. Never returns: select it against the
/// work it loads.
pub async fn critical_sections(mut rng: Rng) {
    loop {
        Timer::after_micros(20 + u64::from(rng.below(280))).await;
        let length = Duration::from_micros(u64::from(rng.below(20)));
        critical_section::with(|_| {
            let start = Instant::now();
            while start.elapsed() < length {}
        });
        bump(&CRITICAL_SECTIONS);
        if !m2m_busy() {
            m2m(false);
        }
    }
}

static mut M2M_WORDS: [u32; 2] = [0x5a5a_a5a5, 0];

/// DMA1 channel `chip::M2M_CH`, memory to memory, 65535 words from one RAM word to another. At
/// the lowest priority it takes the bus whenever no other channel asks; at the highest (`high`)
/// it also wins against the receive channels (same level, lower channel number) until it is
/// done.
pub fn m2m(high: bool) {
    let words = core::ptr::addr_of_mut!(M2M_WORDS) as *mut u32;
    crate::chip::dma::m2m(crate::chip::M2M_CH, high, words as u32, unsafe { words.add(1) } as u32, 0xffff);
    bump(&M2M_RUNS);
}

pub fn m2m_busy() -> bool {
    use crate::chip::{dma, M2M_CH};
    dma::enabled(M2M_CH) && dma::left(M2M_CH) != 0
}

pub fn m2m_stop() {
    use crate::chip::{dma, M2M_CH};
    dma::set_config(M2M_CH, 0);
    dma::clear(M2M_CH);
}

pub fn counts() -> [u32; 4] {
    [
        TIMER_RUNS.load(Relaxed),
        TIMER_IN_USART.load(Relaxed),
        CRITICAL_SECTIONS.load(Relaxed),
        M2M_RUNS.load(Relaxed),
    ]
}
