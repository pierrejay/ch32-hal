# ch32-hal ring-buffered USART stress test

The program behind the tests of the ch32-hal pull request on `RingBufferedUartRx` and the DMA
ring buffer. It is test equipment: it drives registers directly to inject errors and to measure,
so it is not an example of how to use the HAL.

Two USARTs of one chip are wired across, A's TX to B's RX and B's TX to A's RX. The bytes
received through `RingBufferedUartRx` (and `UartRx::read_until_idle`) are checked against a
known stream (the cost measurements only count them); break, framing, parity and noise are
forged by the other USART through its registers, overruns caused by a competing DMA transfer.
The results stay in RAM, to be read with a debugger.

## Boards

| Feature | Chip | A | B = USART2 | Wires |
|---|---|---|---|---|
| `v203` | CH32V203C8T6 at 144 MHz | USART1: PA9 TX, PA10 RX | PA2 TX, PA3 RX | PA9 to PA3, PA2 to PA10 |
| `v307` | CH32V307VCT6 at 144 MHz | as the V203 | | as the V203 |
| `x035` | CH32X035F8U6 at 48 MHz | USART4: PB0 TX, PB1 RX | PA2 TX, PA3 RX | PB0 to PA3, PA2 to PB1 |
| `h417` | CH32H417, its V3F core alone at 100 MHz | as the V203 | | as the V203 |

On the H417 the code is copied at startup to the shared SRAM, which, unlike the flash, has no
wait states; the image takes the flash from 0, over the V5F's slot (save the flash first).

## Build

`rust-toolchain.toml` pins the nightly the tests were built with, and `.cargo/config.toml`
builds `core` from source. `h417/` holds ch32-hal's H417 V3F example files; this `memory.x`
gives the V3F image 192 KiB, over the V5F's slot.

    cargo build --release --locked --features "v203 spin"
    cargo build --release --locked --features "h417 spin" --target h417/riscv32imafc-unknown-none-elf.json

The ELF is `target/riscv32imc-unknown-none-elf/release/uart-stress`, or
`target/riscv32imafc-unknown-none-elf/release/uart-stress` for the H417. Features: the chip
(`v203`, `v307`, `x035` or `h417`); the executor (`spin`, or `wfi` to sleep in WFI between
polls); `long` adds a run of about a quarter of an hour; `restart` builds the controls and the
measurements only (below); `main-hal`, with `restart`, is for an image built against ch32-hal's
main (`rev = "0645921"` in `Cargo.toml`, built without `--locked`): 94, 18 and 19 left out
(below); `watch` (not on the H417), for diagnosis, copies the interrupts' and SysTick's state
after each poll of a phase (`WATCH`).

## Run

Write the image at the start of the flash and check it; zero the 4 KiB of RAM at the address
`python3 decode.py --base ELF` prints (else an earlier run's results stay there if the new image
does not start); reset; wait a minute and a half (three minutes on the X035 and the H417,
seventeen with `long`); halt the core without resetting it and read those 4 KiB. Then, with the
same ELF:

    python3 decode.py ELF ram.bin

With probe-rs 0.32 and a WCH-LinkE, for the V203 (`CH32X035F8U6`, `CH32V307VCT6`,
`CH32H417QEU6` for the others):

    ELF=target/riscv32imc-unknown-none-elf/release/uart-stress
    llvm-objcopy -O binary $ELF image.bin
    BASE=$(python3 decode.py --base $ELF)
    probe-rs download --chip CH32V203C8T6 --binary-format bin --base-address 0x08000000 --verify image.bin
    probe-rs write --chip CH32V203C8T6 b32 $BASE $(printf ' 0%.0s' $(seq 1024))
    probe-rs reset --chip CH32V203C8T6
    sleep 90
    probe-rs read --chip CH32V203C8T6 -o ram.bin -f binary b32 $BASE 1024
    python3 decode.py $ELF ram.bin

probe-rs's `read` halts the core and resumes it: right for a run that ended, not for looking at
one that stopped mid-phase, where the core is to stay halted (a probe tool that halts without a
reset).

`decode.py` needs `llvm-nm` (the `llvm-tools` component the toolchain file lists, or one on the
path). A debug probe may restart the core alone: if the program finds the chip as a previous
program left it (DMA channels or TIM2 enabled, the system clock configured), it stops before any
test and the decoder says why (`BOOT_DMA_ON`); reset the chip and run it again. The free RAM
below the stack is painted at startup, and each phase fails if fewer than 512 bytes of it were
never written (`STACK_FREE`, the worst seen; not on the H417): the V203 and the X035 have 20 KiB
of RAM. The image says what it was built for and ran; the decoder prints each phase with its
result and ends with `DONE: every phase as expected`, or with the phase where the program
stopped. The first unexpected result stops the senders and, at the end of its phase, the
program; its details stay in `DIAG`. A noise control (4, 9) that fails skips its noise phase
instead, and the decoder then reports the coverage incomplete.

## Phases

| Phases | What |
|---|---|
| 0–9, 14–17 | controls by registers: a byte, a break, framing, parity, noise each way; parity after a switch to 8E1 |
| 10–13 | `read_until_idle`: ten bytes, then a break, into each USART |
| 20–30 | the ring's API on B: first reads, nothing arriving, echo, `start_uart()` dropping unread bytes, a buffer overrun, a restart with the lap's flag pending, breaks while waiting in `read()` (26), while not in it (27), after a burst, one-byte reads, reads larger than the ring |
| 31 | wakes: each `read()` woken by the driver alone (idle line, DMA half and full transfer, error) |
| 40–47 | full duplex at four rates up to the fastest, without load (even) and under load (odd); 48: the load's timer did interrupt the USART handlers |
| 49 | rings of 255 and 100 bytes at 1 Mbaud under load: overruns allowed, each checked real |
| 50–55, 60–65 | break, framing, parity, noise, overrun injected into B (50–54) and into A (60–64), 7 offsets after a restart, 115200 and 1 Mbaud; 55, 65: framing errors while the receiving USART also transmits and calls `flush()` (losses counted) |
| 70–73 | slow readers: within a lap (70, 72: no overrun allowed), several laps behind (71, 73: overruns required, each real) |
| 80 | (`long`) full duplex under load, 64 rounds |
| 88, 89 | (`restart`) cost: the DMA ring alone, then `read()`, 100 000 bytes at 3 Mbaud |
| 90–93 | measurements by registers: how PE and ORE clear, whether a stopped channel keeps a DMA request, `start_uart()`'s duration |
| 94 | (`restart`) 200 restarts during a 2 Mbaud stream |
| 18, 19, 95 | (`restart`) parity errors around a restart; a DMA write while the executor sleeps |

## Host tests

`host/` tests the DMA ring buffer's read side on the host. `src/new_impl/ringbuffer.rs` is the
pull request's `src/dma/ringbuffer.rs` and `src/old_impl/ringbuffer.rs` main's (with two empty
default methods its trait lacks), each followed by the same tests (`tests_common.rs`:
embassy-stm32's read tests adapted to ch32-hal's API, and variants that compare only the data
returned); `new_impl` also by tests of the races found in review. From `host/`:

    T="--target $(rustc -vV | sed -n 's/^host: //p') --config unstable.build-std=[\"std\",\"test\",\"panic_unwind\"]"
    cargo test $T new_impl    # 36 passed
    cargo test $T old_impl    # 26 of 27 fail

(the `.cargo/config.toml` above builds `core` from source for the chips, hence `std` here). Among
main's failures are the 9 `data_*` tests, which compare only the data; the others also check the
counts `read()` and `read_exact()` return.

## What it checks

The phases are described at the top of `main.rs`. In short:

- controls by registers: each error the injector makes is seen by the other USART;
- `read_until_idle`, and the ring's API: restarts, a buffer overrun, a restart with a lap's
  transfer-complete flag pending, breaks while the task waits in `read()`, while it does not,
  right after a burst, one-byte reads, reads larger than the ring;
- wakes, in a task of its own (no timer of its own, no waker left registered by earlier reads):
  bytes sent from a timer interrupt, which wakes no task; each `read()` must complete, so be
  woken by the driver, within its time (the idle line, the DMA's half and full transfers, an
  error). Elsewhere the senders share the reader's task, so the other phases do not check the
  driver's wakes;
- full duplex with `flush()`, at four rates up to the USART's fastest, without and with load: a
  timer interrupt above the USART and DMA interrupts, critical sections, a memory-to-memory DMA
  transfer; then rings of 255 and 100 bytes, sizes that are not powers of two;
- break, framing, parity, noise and overrun errors injected in a stream, the bad frame 0 to
  255 bytes after a restart: the error must be reported, no byte from the bad frame on may come
  before it (for an overrun, from the start of the DMA's starvation: the USART flags a later
  frame, and the rounds where bytes from then on arrive before the error are counted), and the reception must start again clean;
  plus framing errors into a USART that also transmits and calls `flush()` (55, 65), where the
  missing reports are counted, not refused;
- noise: bit 3 of the noisy frame is read from three samples around the forged edge, and either
  way is accepted. A round without an error (the noise not made, or its error lost: they cannot
  be told apart) is counted and run again, five times at most: the noise error must come at
  every position. The noise phases check the error's kind and order; lost errors are what the
  framing and parity phases check, through the same flags, clear sequence and handler branch;
- slow readers: no error while they stay within a lap, and when they fall further behind,
  overruns checked as below, and never a wrong byte;
- the duplex and slow-reader streams are checked whole: each sender sends all of it, every byte arrives in order or
  after a reported error, and an error drops bytes only around it. After the reception restarts,
  the stream must resume between what had left the transmitter's DMA just before the restart,
  less the 3 frames on their way, and what had left it just after: no older byte (one of those
  last 3 frames could not be told apart), none skipped after (bytes lost while the restarting
  `read()` call runs cannot be seen in a stream; on an idle line, the API phase and the tails of
  the overrun injections check restarts exactly). The sender adds bytes as long as a restart has
  not been followed by 8, so that a receiver that stops after an error fails;
- in those streams, overruns are accepted in the slow readers that fall behind (71 and 73,
  which require one), in the run with a 100-byte ring (49, 1 ms at 1 Mbaud) and in the long run
  (80) only. Each one the ring reports must come with what had left the transmitter more than a
  ring ahead of the reader (the bytes on their way blur this by a few: an overrun declared with
  the ring just full would pass, one declared with the reader caught up cannot); a USART overrun
  (ORE) cannot be told from a ring overrun there;
- a few measurements of the hardware by registers (how flags clear, whether a DMA request the
  channel could not serve survives the channel's stop).

With `wfi`, on the V307 and the H417, whose DMA keeps going while the core sleeps, everything
runs but the cost measurements 88 and 89 (taken with the executor spinning); the injector waits
a whole frame after starting each one, so that the executor may be asleep when the frame ends and
raises its error. On the V203 and the X035 the DMA stops while the core sleeps (ch32-hal #201):
a ring cannot keep up there, and no phase streams through one (93 times ring restarts on an idle
line); the controls, 90, 92 and 93 run (and 95 with `restart`), and on the V203, which ch32-hal
keeps awake during one-shot DMA transfers, `read_until_idle`.

With `restart`, after the controls and the measurements by registers:

- 94: 200 restarts of a ring at random times during a 2 Mbaud stream, the 8 bytes read after
  each one counted as new and following on from where the reception resumed, stale (received
  before the restart, older than the last 3 frames), lost (a gap, or past what had left the
  transmitter just after the restart, no error), or with an error; a read timeout fails it.
  Two passes: the first with the test's interrupt hook reading `STATR` before the HAL's handler,
  so that a DMA read of `DATAR` in between can end the clear sequence and hide an error; the
  second without, the one to compare;
- 18, 19: parity errors into B with the bad frame first or second after a restart, B
  transmitting meanwhile or not; lost ones counted;
- 88: the DMA ring alone, `ReadableRingBuffer` on a blocking receiver, 100 000 bytes at
  3 Mbaud, reads of 64 bytes at most, each timed;
- 89: `read()` alone, the same stream into `RingBufferedUartRx`, each poll timed;
- 95: whether a DMA write progresses while the executor sleeps.

88 and 89 count bytes only. They also run on ch32-hal's main; 94 cannot. There, a restart while
the ring receives writes NDTR with the channel running, and on the V203 the DMA goes on from its
address: 94's restarts make it write past the ring, over the program, which crashes. 18 and 19
restart the ring the same way; the `main-hal` feature leaves these three out.
