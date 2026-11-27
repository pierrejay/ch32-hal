#![no_std]
#![no_main]

//! Bench check of the SPI slave ring with the DMA interrupt handler of the ring-buffer fix.
//!
//! As spi_slave.rs: SPI2 streams in halves over circular DMA, clocked by SPI1 as the master on
//! the same chip; the master sends a 4-word burst every millisecond carrying a running count and
//! checks the count the slave sends back, the slave checks every half it receives. Here every
//! word is checked from the first one, the run is bounded (60 s) and freezes at the first
//! anomaly, the slave restarts once on purpose while healthy (the master paused between two
//! bursts, both counts known afterwards), and TIM2, at a higher priority than the DMA
//! interrupts, reads the slave's RX position every 20 µs and checks that it never goes back.
//!
//! Results in `RESULTS`, read with a debugger: [0] stage (1 running, 2 passed, 3 failed,
//! 4 panicked), [1] the first failure (1 master break, 2 slave break, 3 missed half, 4 slave
//! error, 5 position went back, 6 restart refused, 7 too little done at the end (detail: what,
//! count), 8 no progress (detail: what, count), 9 master SPI error, 10 slave start refused,
//! 11 panic, 12 master did not pause), [2] bursts, [3] halves, [4] position reads, [5] positions
//! back, [6] restarts, [7] and [8] the failure's detail (expected, got), [9] most laps seen.
//!
//! Wiring: PA5 -> PB13 (SCK), PA7 -> PB15 (MOSI), PB14 -> PA6 (MISO), PA4 -> PB12 (NSS).

use core::cell::Cell;
use core::sync::atomic::Ordering::Relaxed;

use ch32_hal as hal;
use critical_section::Mutex;
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Ticker, Timer};
use hal::dma::RingPositionReader;
use hal::gpio::{Level, Output, Speed};
use hal::interrupt::{InterruptExt, Priority};
use hal::peripherals::{SPI1, SPI2};
use hal::spi::{self, SlaveConfig, Spi, SpiSlaveRing};
use hal::time::Hertz;
use hal::{bind_interrupts, pac};
use portable_atomic::{AtomicBool, AtomicU32};
use static_cell::ConstStaticCell;

/// Words per burst
const BURST: usize = 4;
/// Words per half: the slave wakes every 32 bursts, 32 ms
const HALF: usize = 32 * BURST;
const RUN: Duration = Duration::from_secs(60);
/// The planned restart, after this many halves (about 10 s)
const RESTART_AT: u32 = 300;
/// What a passing run must have done (60 s: about 60 000 bursts, 1 875 halves, 3 000 000 reads)
const MIN_BURSTS: u32 = 50_000;
const MIN_HALVES: u32 = 1_500;
const MIN_POS_READS: u32 = 1_000_000;

const STAGE: usize = 0;
const FAIL: usize = 1;
const BURSTS: usize = 2;
const HALVES: usize = 3;
const POS_READS: usize = 4;
const POS_BACK: usize = 5;
const RESTARTS: usize = 6;
const DETAIL_A: usize = 7;
const DETAIL_B: usize = 8;
const MAX_LAPS: usize = 9;

#[no_mangle]
static RESULTS: [AtomicU32; 12] = [const { AtomicU32::new(0) }; 12];

/// Set at the first failure, or at the end: the tasks freeze, keeping their drivers.
static STOP: AtomicBool = AtomicBool::new(false);
/// The restart: the master pauses by itself after the bursts of `RESTART_AT` halves (a half
/// boundary, NSS high) and says which count its next burst carries; the slave checks it has
/// every word up to there, restarts, says which count it sends next and releases the master.
static PAUSED: AtomicBool = AtomicBool::new(false);
static RESUME: AtomicBool = AtomicBool::new(false);
static MASTER_NEXT: AtomicU32 = AtomicU32::new(0);
static SLAVE_NEXT: AtomicU32 = AtomicU32::new(0);
/// The end, also at a half boundary: main asks, the master stops after a whole half's bursts,
/// checked, and gives its last count; the slave reports the count it expects next.
static FINISH: AtomicBool = AtomicBool::new(false);
static MASTER_DONE: AtomicBool = AtomicBool::new(false);
static MASTER_FINAL: AtomicU32 = AtomicU32::new(0);
static SLAVE_EXPECT: AtomicU32 = AtomicU32::new(0);
/// The position check runs only while set: cleared around the restart, which resets the laps.
static STABLE: AtomicBool = AtomicBool::new(false);
static READER: Mutex<Cell<Option<RingPositionReader<'static>>>> = Mutex::new(Cell::new(None));
static LAST: Mutex<Cell<Option<u64>>> = Mutex::new(Cell::new(None));

fn fail(code: u32, a: u32, b: u32) {
    if RESULTS[FAIL].compare_exchange(0, code, Relaxed, Relaxed).is_ok() {
        RESULTS[DETAIL_A].store(a, Relaxed);
        RESULTS[DETAIL_B].store(b, Relaxed);
        RESULTS[STAGE].store(3, Relaxed);
    }
    STOP.store(true, Relaxed);
}

/// Checks that `w` is the expected count, then expects the next one.
fn check(expect: &mut u16, w: u16, code: u32) {
    if w != *expect {
        fail(code, *expect as u32, w as u32);
    }
    *expect = w.wrapping_add(1);
}

/// Freeze the calling task, keeping what it owns (nothing is dropped, the hardware stays as it
/// was at the stop).
async fn freeze() -> ! {
    loop {
        Timer::after_secs(1).await;
    }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    let _ = RESULTS[FAIL].compare_exchange(0, 11, Relaxed, Relaxed);
    RESULTS[STAGE].store(4, Relaxed);
    STOP.store(true, Relaxed);
    loop {}
}

struct PosCheck;

impl hal::interrupt::typelevel::Handler<hal::interrupt::typelevel::TIM2> for PosCheck {
    unsafe fn on_interrupt() {
        pac::TIM2.intfr().write(|_| {});
        if !STABLE.load(Relaxed) || STOP.load(Relaxed) {
            return;
        }
        let Some(reader) = critical_section::with(|cs| READER.borrow(cs).get()) else {
            return;
        };
        let pos = reader.read();
        let p = pos.laps as u64 * (2 * HALF) as u64 + pos.index as u64;
        RESULTS[POS_READS].fetch_add(1, Relaxed);
        RESULTS[MAX_LAPS].fetch_max(pos.laps, Relaxed);
        critical_section::with(|cs| {
            let last = LAST.borrow(cs);
            if let Some(l) = last.get() {
                if p < l {
                    RESULTS[POS_BACK].fetch_add(1, Relaxed);
                    fail(5, l as u32, p as u32);
                }
            }
            last.set(Some(p));
        });
    }
}

bind_interrupts!(struct Irqs {
    TIM2 => PosCheck;
});

#[embassy_executor::task]
async fn master(mut spi: Spi<'static, SPI1, hal::mode::Async>, mut nss: Output<'static>) {
    let mut ticker = Ticker::every(Duration::from_millis(1));
    let mut count = 0u16;
    let mut bursts = 0u32;
    // the slave's TX buffer starts at 0, sent from the first burst
    let mut expect = 0u16;
    let per_half = (HALF / BURST) as u32;
    loop {
        ticker.next().await;
        if STOP.load(Relaxed) {
            freeze().await;
        }
        if bursts % per_half == 0 && FINISH.load(Relaxed) {
            // every burst sent so far is checked: stop at this half boundary
            MASTER_FINAL.store(count as u32, Relaxed);
            MASTER_DONE.store(true, Relaxed);
            freeze().await;
        }
        if bursts == RESTART_AT * per_half && !RESUME.load(Relaxed) {
            // a half boundary, NSS high: the slave restarts now
            MASTER_NEXT.store(count as u32, Relaxed);
            PAUSED.store(true, Relaxed);
            while !RESUME.load(Relaxed) {
                Timer::after_micros(100).await;
                if STOP.load(Relaxed) {
                    freeze().await;
                }
            }
            expect = SLAVE_NEXT.load(Relaxed) as u16;
            ticker.reset();
            continue;
        }
        let tx: [u16; BURST] = core::array::from_fn(|i| count.wrapping_add(i as u16));
        count = count.wrapping_add(BURST as u16);
        let mut rx = [0u16; BURST];
        nss.set_low();
        let r = spi.transfer(&mut rx, &tx).await;
        nss.set_high();
        if r.is_err() {
            fail(9, 0, 0);
        }
        if STOP.load(Relaxed) {
            freeze().await;
        }
        for w in rx {
            check(&mut expect, w, 1);
        }
        bursts += 1;
        RESULTS[BURSTS].fetch_add(1, Relaxed);
    }
}

fn fill(tx: &mut [u16], count: &mut u16) {
    for w in tx {
        *w = *count;
        *count = count.wrapping_add(1);
    }
}

#[embassy_executor::task]
async fn slave(mut ring: SpiSlaveRing<'static, SPI2, u16>, mut tx_count: u16) {
    // the master's count starts at 0
    let mut expect = 0u16;
    loop {
        let half = ring.next_half().await;
        if STOP.load(Relaxed) {
            freeze().await;
        }
        let half = match half {
            Ok(half) => half,
            Err(_) => {
                fail(4, 0, 0);
                freeze().await;
            }
        };
        if half.missed != 0 {
            fail(3, 0, half.missed);
            freeze().await;
        }
        let (rx, tx) = ring.half_mut(half.index);
        for &w in rx {
            check(&mut expect, w, 2);
        }
        if STOP.load(Relaxed) {
            freeze().await;
        }
        // goes out when the DMA comes back to this half, one pass later
        fill(tx, &mut tx_count);
        let n = RESULTS[HALVES].fetch_add(1, Relaxed) + 1;
        SLAVE_EXPECT.store(expect as u32, Relaxed);
        if n == RESTART_AT {
            let asked = Instant::now();
            while !PAUSED.load(Relaxed) {
                if asked.elapsed() > Duration::from_millis(100) {
                    fail(12, 0, 0);
                }
                Timer::after_micros(100).await;
                if STOP.load(Relaxed) {
                    freeze().await;
                }
            }
            // every word the master sent before its pause must have been checked
            if expect as u32 != MASTER_NEXT.load(Relaxed) {
                fail(2, MASTER_NEXT.load(Relaxed), expect as u32);
            }
            // no clock now: unless something failed meanwhile, stop the position check and
            // restart, in that order
            let go = critical_section::with(|_| {
                let go = !STOP.load(Relaxed);
                if go {
                    STABLE.store(false, Relaxed);
                }
                go
            });
            if !go {
                freeze().await;
            }
            critical_section::with(|cs| LAST.borrow(cs).set(None));
            // refill the whole TX buffer from a known count; the restart sends it from its start
            SLAVE_NEXT.store(tx_count as u32, Relaxed);
            fill(ring.tx_half_mut(0), &mut tx_count);
            fill(ring.tx_half_mut(1), &mut tx_count);
            if ring.restart().is_err() {
                fail(6, 0, 0);
                freeze().await;
            }
            expect = MASTER_NEXT.load(Relaxed) as u16;
            SLAVE_EXPECT.store(expect as u32, Relaxed);
            critical_section::with(|cs| READER.borrow(cs).set(Some(ring.rx_position())));
            RESULTS[RESTARTS].fetch_add(1, Relaxed);
            STABLE.store(true, Relaxed);
            RESUME.store(true, Relaxed);
        }
    }
}

#[embassy_executor::main(entry = "ch32_hal::entry")]
async fn main(spawner: Spawner) -> ! {
    let mut config = hal::Config::default();
    config.rcc = hal::rcc::Config::SYSCLK_FREQ_144MHZ_HSI;
    // Below TIM2's P0, so that the position check can interrupt the DMA handlers.
    config.dma_interrupt_priority = Priority::P8;
    let p = hal::init(config);
    // The V4F: qingke-rt leaves INTSYSCR without a preemption bit; one, so that P0 preempts P8.
    unsafe { core::arch::asm!("li t0, 0x7", "csrw 0x804, t0", out("t0") _) };
    RESULTS[STAGE].store(1, Relaxed);

    static RX: ConstStaticCell<[u16; 2 * HALF]> = ConstStaticCell::new([0; 2 * HALF]);
    static TX: ConstStaticCell<[u16; 2 * HALF]> = ConstStaticCell::new([0; 2 * HALF]);
    let mut ring = SpiSlaveRing::new(
        p.SPI2,
        p.PB13,
        p.PB15,
        p.PB14,
        p.PB12,
        p.DMA1_CH4,
        p.DMA1_CH5,
        RX.take(),
        TX.take(),
        SlaveConfig::default(),
    );
    // the first pass sends both TX halves as they are: start the count there
    let mut tx_count = 0u16;
    fill(ring.tx_half_mut(0), &mut tx_count);
    fill(ring.tx_half_mut(1), &mut tx_count);
    // NSS is high until the master runs: the slave is enabled before the first burst
    if ring.start().is_err() {
        fail(10, 0, 0);
        freeze().await;
    }
    critical_section::with(|cs| READER.borrow(cs).set(Some(ring.rx_position())));
    STABLE.store(true, Relaxed);
    spawner.spawn(slave(ring, tx_count).unwrap());

    let mut spi_config = spi::Config::default();
    spi_config.frequency = Hertz::mhz(6);
    let spi = Spi::new(p.SPI1, p.PA5, p.PA7, p.PA6, p.DMA1_CH3, p.DMA1_CH2, spi_config);
    let nss = Output::new(p.PA4, Level::High, Speed::High);
    spawner.spawn(master(spi, nss).unwrap());

    // TIM2 on its 144 MHz clock: an update every 20 µs.
    pac::RCC.apb1pcenr().modify(|w| w.set_tim2en(true));
    let t = pac::TIM2;
    t.ctlr1().write(|_| {});
    t.psc().write_value(0);
    t.atrlr().write_value(2879);
    t.swevgr().write(|w| w.set_ug(true));
    t.intfr().write(|_| {});
    t.dmaintenr().write(|w| w.set_uie(true));
    t.ctlr1().write(|w| w.set_cen(true));
    hal::interrupt::TIM2.set_priority(Priority::P0);
    unsafe { hal::interrupt::TIM2.enable() };

    // Progress: bursts, halves and position reads must keep coming (the restart pauses the
    // master for well under the 500 ms allowed).
    let start = Instant::now();
    let mut last = [0u32; 3];
    let mut since = [Instant::now(); 3];
    while !STOP.load(Relaxed) && start.elapsed() < RUN {
        Timer::after_millis(100).await;
        for (i, idx) in [BURSTS, HALVES, POS_READS].into_iter().enumerate() {
            let v = RESULTS[idx].load(Relaxed);
            if v != last[i] {
                last[i] = v;
                since[i] = Instant::now();
            } else if since[i].elapsed() > Duration::from_millis(500) {
                fail(8, i as u32 + 1, v);
            }
        }
    }
    if !STOP.load(Relaxed) {
        // stop at a half boundary, then wait for the slave to have checked every word
        FINISH.store(true, Relaxed);
        let asked = Instant::now();
        loop {
            if STOP.load(Relaxed) {
                break;
            }
            if MASTER_DONE.load(Relaxed) && SLAVE_EXPECT.load(Relaxed) == MASTER_FINAL.load(Relaxed) {
                break;
            }
            if asked.elapsed() > Duration::from_secs(1) {
                fail(8, 4, SLAVE_EXPECT.load(Relaxed));
                break;
            }
            Timer::after_millis(1).await;
        }
    }
    if !STOP.swap(true, Relaxed) {
        let done = [
            (RESULTS[BURSTS].load(Relaxed), MIN_BURSTS),
            (RESULTS[HALVES].load(Relaxed), MIN_HALVES),
            (RESULTS[POS_READS].load(Relaxed), MIN_POS_READS),
        ];
        let short = done.iter().position(|&(v, min)| v < min);
        let restarts = RESULTS[RESTARTS].load(Relaxed);
        if let Some(i) = short {
            fail(7, i as u32 + 1, done[i].0);
        } else if restarts != 1 {
            fail(7, 4, restarts);
        } else {
            RESULTS[STAGE].store(2, Relaxed);
        }
    }
    freeze().await
}
