// embassy-stm32's ring buffer read tests (embassy-rs/embassy@62622d7d2), adapted to ch32-hal's
// read-side API: `clear` for `reset`, `OverrunError` for `Error::Overrun`; no alignment, no
// read_latest. Plus `read_caught_up_wrap_between_register_reads`, the V203 case.
use core::cell::Cell;
use core::future::Future;
use core::pin::{pin, Pin};
use core::task::Context;
use std::boxed::Box;
use std::collections::VecDeque;
use std::vec;
use std::vec::Vec;

const CAP: usize = 16;

#[derive(Default)]
struct Dma {
    pos: Cell<usize>,
    total: Cell<usize>,
    laps: Cell<usize>,
    pending_laps: Cell<usize>,
    defer_irq: bool,
    lazy_reload: bool,
    race: Cell<usize>,
    race_after: Cell<usize>,
    polls: VecDeque<usize>,
}

impl Dma {
    fn advance(&self, n: usize) {
        for _ in 0..n {
            if self.pos.get() == CAP {
                self.wrap();
            }
            self.pos.set(self.pos.get() + 1);
            self.total.set(self.total.get() + 1);
            if self.pos.get() == CAP && !self.lazy_reload {
                self.wrap();
            }
        }
    }
    fn wrap(&self) {
        self.pos.set(0);
        if self.defer_irq {
            self.pending_laps.set(self.pending_laps.get() + 1);
        } else {
            self.laps.set(self.laps.get() + 1);
        }
    }
    fn register_access(&self) {
        match self.race_after.get() {
            0 => self.advance(self.race.take()),
            n => self.race_after.set(n - 1),
        }
    }
    fn irq(&self) {
        self.laps.set(self.laps.get() + self.pending_laps.take());
    }
}

impl DmaCtrl for Dma {
    fn get_remaining_transfers(&self) -> usize {
        let remaining = CAP - self.pos.get();
        self.register_access();
        remaining
    }
    // ch32-hal's trait also has this (the old read side uses it).
    fn get_complete_count(&self) -> usize {
        let laps = self.laps.get();
        self.register_access();
        laps
    }
    fn reset_complete_count(&mut self) -> usize {
        let laps = self.laps.take();
        self.register_access();
        laps
    }
    fn set_waker(&mut self, _waker: &Waker) {
        let n = self.polls.pop_front().unwrap_or(0);
        self.advance(n);
    }
    fn complete_pending(&self) -> bool {
        let pending = self.pending_laps.get() > 0;
        self.register_access();
        pending
    }
    fn clear_complete(&mut self) {
        self.pending_laps.set(0);
        self.register_access();
    }
}

fn seq(r: core::ops::Range<usize>) -> Vec<u32> {
    r.map(|i| i as u32).collect()
}
fn new_buf() -> &'static mut [u32] {
    Box::leak(Box::new([u32::MAX; CAP]))
}
fn poll<F: Future>(f: Pin<&mut F>) -> Poll<F::Output> {
    f.poll(&mut Context::from_waker(Waker::noop()))
}

struct Reader {
    rb: ReadableDmaRingBuffer<'static, u32>,
    dma: Dma,
}

impl Reader {
    fn new() -> Self {
        Self::with_dma(Dma::default())
    }
    fn with_dma(dma: Dma) -> Self {
        Self { rb: ReadableDmaRingBuffer::new(new_buf()), dma }
    }
    fn prepare(&mut self, n: usize) {
        for i in 0..n {
            self.rb.dma_buf[(self.dma.pos.get() + i) % CAP] = (self.dma.total.get() + i) as u32;
        }
    }
    fn dma(&mut self, n: usize) {
        self.prepare(n);
        self.dma.advance(n);
    }
    fn race(&mut self, n: usize) {
        self.prepare(n);
        self.dma.race.set(n);
    }
    fn race_during_copy(&mut self, n: usize) {
        self.race(n);
        self.dma.race_after.set(2);
    }
    /// The words read and the number still unread. `read()` returns the capacity less that
    /// number (its documented meaning); after an overrun the buffer is cleared, as
    /// RingBufferedUartRx does when it starts again.
    fn read(&mut self, len: usize) -> Result<(Vec<u32>, usize), OverrunError> {
        let mut buf = vec![0; len];
        let (n, remaining) = match self.rb.read(&mut self.dma, &mut buf) {
            Ok(r) => r,
            Err(e) => {
                self.rb.clear(&mut self.dma);
                return Err(e);
            }
        };
        buf.truncate(n);
        Ok((buf, CAP - remaining))
    }
}

#[test]
fn read_partial() {
    let mut t = Reader::new();
    assert_eq!(t.read(3), Ok((vec![], 0)));
    t.dma(5);
    assert_eq!(t.read(3), Ok((seq(0..3), 2)));
    assert_eq!(t.read(3), Ok((seq(3..5), 0)));
    assert_eq!(t.read(3), Ok((vec![], 0)));
}

#[test]
fn read_wraps() {
    let mut t = Reader::new();
    t.dma(10);
    assert_eq!(t.read(10), Ok((seq(0..10), 0)));
    t.dma(10);
    assert_eq!(t.read(CAP), Ok((seq(10..20), 0)));
}

#[test]
fn read_full_buffer() {
    for start in 0..CAP {
        let mut t = Reader::new();
        t.dma(start);
        t.read(CAP).unwrap();
        t.dma(CAP);
        assert_eq!(t.read(CAP + 1), Ok((seq(start..start + CAP), 0)), "start {start}");
    }
}

#[test]
fn read_overrun() {
    for start in 0..CAP {
        let mut t = Reader::new();
        t.dma(start);
        t.read(CAP).unwrap();
        t.dma(CAP + 1);
        assert_eq!(t.read(4), Err(OverrunError), "start {start}");
        assert_eq!(t.read(4), Ok((vec![], 0)), "start {start}");
        t.dma(3);
        let next = start + CAP + 1;
        assert_eq!(t.read(4), Ok((seq(next..next + 3), 0)), "start {start}");
    }
}

#[test]
fn read_overrun_partially_read() {
    let mut t = Reader::new();
    t.dma(10);
    assert_eq!(t.read(4), Ok((seq(0..4), 6)));
    t.dma(10);
    assert_eq!(t.read(2), Ok((seq(4..6), 14)));
    t.dma(3);
    assert_eq!(t.read(4), Err(OverrunError));
}

#[test]
fn read_overrun_many_laps() {
    let mut t = Reader::new();
    t.dma(5 * CAP + 3);
    assert_eq!(t.read(4), Err(OverrunError));
    t.dma(2);
    assert_eq!(t.read(4), Ok((seq(5 * CAP + 3..5 * CAP + 5), 0)));
}

#[test]
fn read_reset() {
    let mut t = Reader::new();
    t.dma(2 * CAP + 5);
    t.rb.clear(&mut t.dma);
    assert_eq!(t.read(CAP), Ok((vec![], 0)));
    t.dma(CAP);
    assert_eq!(t.read(CAP), Ok((seq(2 * CAP + 5..3 * CAP + 5), 0)));
}

#[test]
fn read_overwritten_while_copying() {
    let mut t = Reader::new();
    t.dma(CAP);
    t.race_during_copy(1);
    assert_eq!(t.read(CAP), Err(OverrunError));
    assert_eq!(t.read(CAP), Ok((vec![], 0)));
    t.dma(3);
    assert_eq!(t.read(CAP), Ok((seq(CAP + 1..CAP + 4), 0)));
}

#[test]
fn read_dma_continues_while_copying() {
    let mut t = Reader::new();
    t.dma(4);
    t.race_during_copy(3);
    assert_eq!(t.read(4), Ok((seq(0..4), 3)));
    assert_eq!(t.read(CAP), Ok((seq(4..7), 0)));
}

#[test]
fn read_wrap_between_register_reads() {
    let mut t = Reader::new();
    t.dma(11);
    assert_eq!(t.read(11), Ok((seq(0..11), 0)));
    t.race(11);
    assert_eq!(t.read(CAP), Ok((seq(11..22), 0)));
    assert_eq!(t.read(CAP), Ok((vec![], 0)));
    t.dma(2);
    assert_eq!(t.read(CAP), Ok((seq(22..24), 0)));
}

/// The V203 case: the reader has read everything up to the last word of the lap, and the DMA
/// writes that word, wrapping, between the two register reads of the next read.
#[test]
fn read_caught_up_wrap_between_register_reads() {
    for lap in 0..3 {
        let mut t = Reader::new();
        t.dma(lap * CAP + CAP - 1);
        let first = lap * CAP;
        if lap > 0 {
            // Catch up lap by lap, as a reader that keeps up would.
            t = Reader::new();
            for k in 0..lap {
                t.dma(CAP);
                assert_eq!(t.read(CAP), Ok((seq(k * CAP..k * CAP + CAP), 0)));
            }
            t.dma(CAP - 1);
        }
        let caught_up = first + CAP - 1;
        assert_eq!(t.read(CAP), Ok((seq(first..caught_up), 0)), "lap {lap}");
        t.race(1);
        assert_eq!(t.read(64.min(CAP)), Ok((seq(caught_up..caught_up + 1), 0)), "lap {lap}");
        t.dma(1);
        assert_eq!(t.read(CAP), Ok((seq(caught_up + 1..caught_up + 2), 0)), "lap {lap}");
    }
}

#[test]
fn read_wrap_with_irq_pending() {
    let mut t = Reader::with_dma(Dma { defer_irq: true, ..Default::default() });
    t.dma(11);
    assert_eq!(t.read(11), Ok((seq(0..11), 0)));
    t.dma(11);
    assert_eq!(t.read(CAP), Ok((seq(11..22), 0)));
    t.dma(2);
    assert_eq!(t.read(CAP), Ok((seq(22..24), 0)));
    t.dma.irq();
    assert_eq!(t.read(CAP), Ok((vec![], 0)));
    t.dma(2);
    assert_eq!(t.read(CAP), Ok((seq(24..26), 0)));
}

#[test]
fn read_ndtr_zero() {
    let mut t = Reader::with_dma(Dma { lazy_reload: true, ..Default::default() });
    t.dma(8);
    assert_eq!(t.read(8), Ok((seq(0..8), 0)));
    t.dma(8);
    assert_eq!(t.read(CAP), Ok((seq(8..16), 0)));
    assert_eq!(t.read(CAP), Ok((vec![], 0)));
    t.dma(1);
    assert_eq!(t.read(CAP), Ok((seq(16..17), 0)));
}

#[test]
fn read_two_wraps_with_irq_pending() {
    let mut t = Reader::with_dma(Dma { defer_irq: true, ..Default::default() });
    t.dma(10);
    assert_eq!(t.read(10), Ok((seq(0..10), 0)));
    t.dma(8);
    assert_eq!(t.read(CAP), Ok((seq(10..18), 0)));
    t.dma.irq();
    t.dma(15);
    assert_eq!(t.read(CAP), Ok((seq(18..33), 0)));
    t.dma.irq();
    t.dma(2);
    assert_eq!(t.read(CAP), Ok((seq(33..35), 0)));
}

fn read_stream(mut t: Reader, racy: bool) {
    let mut written = 0;
    let mut read = 0;
    for i in 0..1000 {
        t.dma.irq();
        let n = (i * 7) % (CAP - (written - read));
        if racy && i % 3 == 0 {
            t.race(n);
        } else {
            t.dma(n);
        }
        written += n;
        let len = (i * 5) % CAP + 1;
        let k = len.min(written - read);
        assert_eq!(
            t.read(len),
            Ok((seq(read..read + k), written - read - k)),
            "step {i}, defer_irq={}, lazy_reload={}, racy={racy}",
            t.dma.defer_irq,
            t.dma.lazy_reload
        );
        read += k;
    }
}

#[test]
fn read_streams() {
    for defer_irq in [false, true] {
        for lazy_reload in [false, true] {
            for racy in [false, true] {
                read_stream(Reader::with_dma(Dma { defer_irq, lazy_reload, ..Default::default() }), racy);
            }
        }
    }
}

#[test]
fn read_exact() {
    let mut t = Reader::new();
    t.dma(4);
    t.prepare(6);
    t.dma.polls = VecDeque::from([3, 3]);
    let mut buf = [0; 9];
    {
        let Reader { rb, dma } = &mut t;
        let mut fut = pin!(rb.read_exact(dma, &mut buf));
        assert_eq!(poll(fut.as_mut()), Poll::Pending);
        assert_eq!(poll(fut.as_mut()), Poll::Ready(Ok(1)));
    }
    assert_eq!(buf.to_vec(), seq(0..9));
}

#[test]
fn read_exact_overrun() {
    let mut t = Reader::new();
    t.dma(4);
    t.dma.polls = VecDeque::from([0, CAP + 1]);
    let mut buf = [0; 8];
    let Reader { rb, dma } = &mut t;
    let mut fut = pin!(rb.read_exact(dma, &mut buf));
    assert_eq!(poll(fut.as_mut()), Poll::Pending);
    assert_eq!(poll(fut.as_mut()), Poll::Ready(Err(OverrunError)));
}

// Data-only variants: the old read side reports `remaining` differently (capacity minus the read
// index), so these compare only the data and Ok/Err, to see the races on both implementations.
impl Reader {
    fn data(&mut self, len: usize) -> Result<Vec<u32>, OverrunError> {
        self.read(len).map(|(d, _)| d)
    }
}

#[test]
fn data_caught_up_wrap_between_register_reads() {
    let mut t = Reader::new();
    t.dma(CAP - 1);
    assert_eq!(t.data(CAP), Ok(seq(0..CAP - 1)));
    t.race(1);
    assert_eq!(t.data(CAP), Ok(seq(CAP - 1..CAP)));
    t.dma(1);
    assert_eq!(t.data(CAP), Ok(seq(CAP..CAP + 1)));
}

#[test]
fn data_wrap_between_register_reads() {
    let mut t = Reader::new();
    t.dma(11);
    assert_eq!(t.data(11), Ok(seq(0..11)));
    t.race(11);
    assert_eq!(t.data(CAP), Ok(seq(11..22)));
    assert_eq!(t.data(CAP), Ok(vec![]));
    t.dma(2);
    assert_eq!(t.data(CAP), Ok(seq(22..24)));
}

#[test]
fn data_wrap_with_irq_pending() {
    let mut t = Reader::with_dma(Dma { defer_irq: true, ..Default::default() });
    t.dma(11);
    assert_eq!(t.data(11), Ok(seq(0..11)));
    t.dma(11);
    assert_eq!(t.data(CAP), Ok(seq(11..22)));
    t.dma(2);
    assert_eq!(t.data(CAP), Ok(seq(22..24)));
    t.dma.irq();
    assert_eq!(t.data(CAP), Ok(vec![]));
    t.dma(2);
    assert_eq!(t.data(CAP), Ok(seq(24..26)));
}

#[test]
fn data_two_wraps_with_irq_pending() {
    let mut t = Reader::with_dma(Dma { defer_irq: true, ..Default::default() });
    t.dma(10);
    assert_eq!(t.data(10), Ok(seq(0..10)));
    t.dma(8);
    assert_eq!(t.data(CAP), Ok(seq(10..18)));
    t.dma.irq();
    t.dma(15);
    assert_eq!(t.data(CAP), Ok(seq(18..33)));
    t.dma.irq();
    t.dma(2);
    assert_eq!(t.data(CAP), Ok(seq(33..35)));
}

#[test]
fn data_ndtr_zero() {
    let mut t = Reader::with_dma(Dma { lazy_reload: true, ..Default::default() });
    t.dma(8);
    assert_eq!(t.data(8), Ok(seq(0..8)));
    t.dma(8);
    assert_eq!(t.data(CAP), Ok(seq(8..16)));
    assert_eq!(t.data(CAP), Ok(vec![]));
    t.dma(1);
    assert_eq!(t.data(CAP), Ok(seq(16..17)));
}

fn data_stream(mut t: Reader, racy: bool) {
    let (mut written, mut read) = (0, 0);
    for i in 0..1000 {
        t.dma.irq();
        let n = (i * 7) % (CAP - (written - read));
        if racy && i % 3 == 0 {
            t.race(n);
        } else {
            t.dma(n);
        }
        written += n;
        let len = (i * 5) % CAP + 1;
        let k = len.min(written - read);
        assert_eq!(t.data(len), Ok(seq(read..read + k)), "step {i}, defer_irq={}, lazy_reload={}, racy={racy}", t.dma.defer_irq, t.dma.lazy_reload);
        read += k;
    }
}

#[test]
fn data_stream_plain() {
    data_stream(Reader::new(), false);
}
#[test]
fn data_stream_racy() {
    data_stream(Reader::new(), true);
}
#[test]
fn data_stream_irq_pending() {
    data_stream(Reader::with_dma(Dma { defer_irq: true, ..Default::default() }), false);
}
#[test]
fn data_stream_irq_pending_racy() {
    data_stream(Reader::with_dma(Dma { defer_irq: true, ..Default::default() }), true);
}

/// A lap ends at each possible point of `clear()`, its interrupt running only afterwards (the
/// clear holds it off): what arrived before the position taken must not come back as new data,
/// and what follows must arrive.
#[test]
fn clear_with_a_lap_ending_inside() {
    // Register accesses 0..=6 cover clear() and the position sync of the next read; later,
    // the mock would write the prepared word at another position.
    for at in 0..7 {
        let mut t = Reader::with_dma(Dma { defer_irq: true, ..Default::default() });
        t.dma(CAP - 1);
        t.race(1);
        t.dma.race_after.set(at);
        t.rb.clear(&mut t.dma);
        t.dma.irq();
        let mut got = t.data(CAP).unwrap();
        t.dma(2);
        got.extend(t.data(CAP).unwrap());
        // The last word of the lap ended before or after the position taken: dropped or read,
        // never more than that, never anything older.
        assert!(got == seq(CAP..CAP + 2) || got == seq(CAP - 1..CAP + 2), "at {at}: {got:?}");
    }
}

/// clear() takes NDTR at 0 (the reload not done yet, its lap counted at the reload), then a full
/// lap arrives with no read in between: the buffer is exactly full, nothing overwritten.
#[test]
fn clear_at_ndtr_zero_then_a_full_lap() {
    let mut t = Reader::with_dma(Dma { lazy_reload: true, ..Default::default() });
    t.dma(CAP);
    t.rb.clear(&mut t.dma);
    t.dma.lazy_reload = false;
    t.dma(CAP);
    assert_eq!(t.read(CAP), Ok((seq(CAP..2 * CAP), 0)));
}
