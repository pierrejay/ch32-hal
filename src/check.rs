//! The test streams and their checker.
//!
//! Stream `seed` is a sequence of scrambled 32-bit words, little-endian: a byte from another
//! offset (an old lap, a duplicate, a byte from before a restart) does not match. Any 8
//! consecutive bytes cover a whole word, so they also tell where in the stream they come from.

use crate::{diag, F_DATA, F_EXCESS, F_RESYNC, F_SHORT, F_STALE};

pub fn word(seed: u32, index: u32) -> u32 {
    let mut x = seed ^ index.wrapping_mul(0x9e37_79b9);
    x ^= x >> 16;
    x = x.wrapping_mul(0x7feb_352d);
    x ^= x >> 15;
    x = x.wrapping_mul(0x846c_a68b);
    x ^ (x >> 16)
}

pub fn byte(seed: u32, offset: u32) -> u8 {
    word(seed, offset / 4).to_le_bytes()[(offset % 4) as usize]
}

/// A stream byte by byte, one word computed per four bytes (at 9 Mbaud both ways, the CPU
/// generates and checks 1.8 MB/s).
pub struct Bytes {
    seed: u32,
    index: u32,
    word: [u8; 4],
}

impl Bytes {
    pub fn new(seed: u32) -> Self {
        Bytes { seed, index: u32::MAX, word: [0; 4] }
    }
    pub fn at(&mut self, offset: u32) -> u8 {
        let index = offset / 4;
        if index != self.index {
            self.index = index;
            self.word = word(self.seed, index).to_le_bytes();
        }
        self.word[(offset % 4) as usize]
    }
}

pub fn fill(seed: u32, from: u32, out: &mut [u8]) {
    let mut bytes = Bytes::new(seed);
    for (i, b) in out.iter_mut().enumerate() {
        *b = bytes.at(from + i as u32);
    }
}

/// xorshift32: random sizes, delays and positions, the same on every run.
pub struct Rng(u32);

impl Rng {
    pub fn new(seed: u32) -> Self {
        Rng(seed | 1)
    }
    pub fn next(&mut self) -> u32 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.0 = x;
        x
    }
    /// 0..n (0 if n is 0).
    pub fn below(&mut self, n: u32) -> u32 {
        if n == 0 {
            0
        } else {
            self.next() % n
        }
    }
}

/// What a receiver has checked of one stream. After a reported error the reception restarts
/// and the bytes in between are gone: the next 8 bytes must then be found further on in the
/// stream, and everything after them must follow on. They must also be new: no byte that had
/// been received before the restart, and every byte handed to the transmitter after it must
/// arrive. Anything else fails, with `DIAG` set.
pub struct Stream {
    seed: u32,
    bytes: Bytes,
    /// Offset of the next byte expected.
    pub next: u32,
    resync: bool,
    held: [u8; 8],
    held_len: usize,
    /// Since the last restart: the lowest offset a byte received after it can have (what had
    /// left the transmitter's DMA, less the frames still on their way), the highest the first
    /// one can have (what had left the transmitter's DMA: what it still held goes out after the
    /// restart and must arrive), and how much had been handed to the transmitter.
    floor: u32,
    upper: u32,
    mark: u32,
}

/// Frames between the transmitter's DMA and the receiver's: its data and shift registers, and
/// the frame on the wire whose end the receiver has not seen yet.
pub const IN_FLIGHT: u32 = 3;

impl Stream {
    pub fn new(seed: u32) -> Self {
        Stream { seed, bytes: Bytes::new(seed), next: 0, resync: false, held: [0; 8], held_len: 0, floor: 0, upper: 0, mark: 0 }
    }

    /// Before a restart: bytes received since the previous one, fewer than 8, must follow each
    /// other in the stream, starting where that restart allowed (they are dropped next).
    pub fn check_held(&self) -> Result<(), u32> {
        if self.resync && self.held_len > 0 {
            let n = self.held_len as u32;
            let fits = (self.floor..=self.upper).any(|at| (0..n).all(|i| byte(self.seed, at + i) == self.held[i as usize]));
            if !fits {
                diag(&[F_RESYNC, self.floor, self.upper, n, u32::from(self.held[0])]);
                return Err(F_RESYNC);
            }
        }
        Ok(())
    }

    /// The reception restarted after a reported error, `sent` bytes having been handed to the
    /// transmitter. `before` and `after`, if known: the bytes that had left the transmitter's
    /// DMA (handed to it, less what its transfer still held) just before and just after the
    /// restart. Bytes from before `before`, less the frames on their way, were received before
    /// the restart and must not come after it; from `after` on, they leave the DMA after the
    /// restart and must arrive.
    pub fn restarted(&mut self, before: Option<u32>, after: Option<u32>, sent: u32) {
        self.resync = true;
        self.held_len = 0;
        self.floor = before.map_or(self.next, |l| l.saturating_sub(IN_FLIGHT).max(self.next));
        self.upper = after.unwrap_or(sent).max(self.floor);
        self.mark = sent;
    }

    /// Whether a restart is still waiting for 8 bytes to show where the reception resumed.
    pub fn resyncing(&self) -> bool {
        self.resync
    }

    /// The lowest offset the reader can be at: where it resumed after a restart, plus what it
    /// has received since.
    pub fn position(&self) -> u32 {
        if self.resync {
            self.floor + self.held_len as u32
        } else {
            self.next
        }
    }

    /// Bytes as received. `sent`: how far the sender has got (counted before each write, so no
    /// byte beyond it can have arrived).
    pub fn take(&mut self, mut data: &[u8], sent: u32) -> Result<(), u32> {
        if self.resync {
            let n = (8 - self.held_len).min(data.len());
            self.held[self.held_len..self.held_len + n].copy_from_slice(&data[..n]);
            self.held_len += n;
            data = &data[n..];
            if self.held_len < 8 {
                return Ok(());
            }
            let start = self.find(sent)?;
            if start < self.floor {
                // Bytes received before the restart, handed out after it.
                diag(&[F_STALE, start, self.floor, self.next, sent]);
                return Err(F_STALE);
            }
            if start > self.upper {
                // Bytes that left the transmitter after the restart went missing.
                diag(&[F_RESYNC, start, self.upper, self.floor, sent]);
                return Err(F_RESYNC);
            }
            self.next = start + 8;
            self.resync = false;
        }
        for &b in data {
            if self.next >= sent {
                diag(&[F_EXCESS, self.next, sent, u32::from(b)]);
                return Err(F_EXCESS);
            }
            let want = self.bytes.at(self.next);
            if b != want {
                diag(&[F_DATA, self.next, u32::from(b), u32::from(want), sent]);
                return Err(F_DATA);
            }
            self.next += 1;
        }
        Ok(())
    }

    /// Where the 8 held bytes start: the latest match between the last checked byte and what
    /// was sent (a restart drops bytes, it never goes back).
    fn find(&self, sent: u32) -> Result<u32, u32> {
        let mut start = sent.saturating_sub(8);
        while start >= self.next && start + 8 <= sent {
            if (0..8).all(|i| byte(self.seed, start + i as u32) == self.held[i]) {
                return Ok(start);
            }
            if start == 0 {
                break;
            }
            start -= 1;
        }
        let h = u32::from_le_bytes([self.held[0], self.held[1], self.held[2], self.held[3]]);
        diag(&[F_RESYNC, self.next, sent, h]);
        Err(F_RESYNC)
    }

    /// The sender is done and the line stayed quiet: the stream must be complete, apart from
    /// bytes an error dropped. A restart that 8 bytes have not followed yet fails here: the
    /// receivers that allow errors have the sender add bytes until they do (`Send::tails`), so
    /// that a receiver that stopped for good after an error cannot pass.
    pub fn finish(&mut self, sent: u32) -> Result<(), u32> {
        if self.resync {
            diag(&[F_RESYNC, self.next, sent, self.held_len as u32, self.mark, self.floor]);
            return Err(F_RESYNC);
        }
        if self.next != sent {
            diag(&[F_SHORT, self.next, sent]);
            return Err(F_SHORT);
        }
        Ok(())
    }
}
