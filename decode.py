#!/usr/bin/env python3
"""Decode a uart_stress RAM dump with the ELF that produced it: python3 decode.py ELF RAM. The
image says what it ran (`BUILD`: chip, wfi, long, restart, main-hal). `decode.py --base ELF` prints the
address of the 4 KiB to dump. Prints every phase that ran, the load counters and the first
failure's details, then DONE if every phase is as expected."""
import glob, os, shutil, struct, subprocess, sys

# llvm-nm from the path, else from a rustup toolchain's llvm-tools.
nm = shutil.which('llvm-nm') or (glob.glob(os.path.expanduser('~/.rustup/toolchains/*/lib/rustlib/*/bin/llvm-nm')) or [None])[0]
if not nm:
    sys.exit('STOP: llvm-nm not found (rustup component add llvm-tools)')
sym = {l.split()[2]: int(l.split()[0], 16) for l in subprocess.run([nm, sys.argv[2] if sys.argv[1] == '--base' else sys.argv[1]],
       capture_output=True, text=True).stdout.splitlines() if len(l.split()) == 3}
sizes = {'STATE': 4, 'PHASE': 4, 'RESULT': 96 * 4, 'DETAIL': 96 * 8, 'DIAG': 32, 'LOAD': 16, 'LONG_ROUNDS': 4, 'DUPLEX_MS': 32, 'BUILD': 4}
if any(k not in sym for k in sizes):
    sys.exit('STOP: result symbols missing from the ELF')
# The dump: 4 KiB from below the lowest result symbol (where the RAM is depends on the chip and
# on the image). `decode.py --base ELF` prints that address.
BASE = min(sym[k] for k in sizes) & ~0xff
if sys.argv[1] == '--base':
    print('%#x' % BASE)
    sys.exit(0)

# A RAM dump is 4 KiB: read no more, whatever the file is.
if len(sys.argv) != 3:
    sys.exit('usage: decode.py ELF RAM (or --base ELF)')
elf, ram = sys.argv[1], open(sys.argv[2], 'rb').read(4096)
if len(ram) != 4096:
    sys.exit('STOP: short dump, %d bytes' % len(ram))
for k, n in sizes.items():
    if sym[k] - BASE + n > len(ram):
        sys.exit('STOP: result symbol %s outside the dump' % k)
word = lambda name, i=0: struct.unpack_from('<I', ram, sym[name] - BASE + 4 * i)[0]
inside = lambda name, n: name in sym and 0 <= sym[name] - BASE <= len(ram) - n

build = word('BUILD')
wfi, long_run, restart, main_hal, chip_id = bool(build & 1), bool(build & 2), bool(build & 4), bool(build & 8), build >> 8
chip = {1: 'v203', 2: 'v307', 3: 'x035', 4: 'h417'}.get(chip_id)
if chip is None:
    sys.exit('STOP: BUILD %#x names no chip (a dump of another image?)' % build)
# The V307's and the H417's DMA keep going while the core sleeps: their wfi build runs it all.
# On the V203 and the X035 it runs no ring phase; one-shot transfers on the V203 only.
full = not wfi or chip in ('v307', 'h417')
oneshots = full or chip == 'v203'
print('BUILD %s%s%s%s%s' % (chip, ' wfi' if wfi else ' spin', ' long' if long_run else '', ' restart' if restart else '', ' main' if main_hal else ''))
RATES = {'v203': ('115200', '1M', '4.5M', '9M'), 'v307': ('115200', '1M', '4.5M', '9M'), 'x035': ('115200', '1M', '2M', '3M'), 'h417': ('115200', '1M', '3.125M', '6.25M')}[chip]

NAMES = {
    0: 'A->B byte', 1: 'A->B break', 2: 'A->B forged framing', 3: 'A->B forged parity', 4: 'A->B forged noise',
    5: 'B->A byte', 6: 'B->A break', 7: 'B->A forged framing', 8: 'B->A forged parity', 9: 'B->A forged noise',
    14: 'A->B parity, switch then frame at once', 15: 'A->B parity, switch, 20 us, frame',
    16: 'B->A parity, switch then frame at once', 17: 'B->A parity, switch, 20 us, frame',
    90: 'PE after a DATAR read, by what preceded', 91: 'held DMA request after the channel stops',
    92: 'ORE after a DATAR read, by what preceded', 93: 'start_uart() duration', 94: 'restarts while streaming at 2M', 95: 'DMA write, executor asleep or not', 88: 'ring alone at 3M, nothing checked', 89: 'read() cost at 3M, nothing checked',
    18: 'parity into B, k 0 / k 1, B sending', 19: 'parity into B, k 0 / k 1, B silent',
    10: 'read_until_idle B, ten bytes', 11: 'read_until_idle B, break', 12: 'read_until_idle A, ten bytes',
    13: 'read_until_idle A, break',
    20: 'first reads', 21: 'nothing arriving', 22: 'echo', 23: 'start_uart() drops unread bytes',
    24: 'ring overrun (768 into 256)', 25: 'restart with the lap flag pending', 26: 'break while waiting in read()',
    27: 'break while not in read()', 28: 'break right after a burst', 29: 'one-byte reads', 30: 'reads larger than the ring',
    31: 'wakes: idle line, DMA half/full, error',
    48: 'timer inside USART handlers', 49: 'duplex 1M + load, rings 255 and 100',
    50: 'into B: break (A checks B->A)', 51: 'into B: framing', 52: 'into B: parity', 53: 'into B: noise',
    54: 'into B: overrun', 55: 'into B: framing, B flushing (measure)',
    60: 'into A: break (B checks A->B)', 61: 'into A: framing', 62: 'into A: parity', 63: 'into A: noise',
    64: 'into A: overrun', 65: 'into A: framing, A flushing (measure)',
    70: 'slow reader, 32 B at 115200, within a lap', 71: 'slow reader, 32 B at 115200, several laps',
    72: 'slow reader, 256 B at 2M, within a lap', 73: 'slow reader, 256 B at 2M, several laps',
    80: 'long duplex under load',
}
FAIL = {2: 'timeout', 3: 'wrong data', 4: 'unexpected error', 5: 'error missing', 6: 'wrong error kind',
        7: 'more bytes than sent', 8: 'write failed', 9: 'no resync after an error', 10: 'bytes missing, no error',
        11: 'control: STATR/DATAR not as made', 12: 'precondition', 13: 'no overrun', 14: 'no preemption',
        15: 'injector stuck', 16: 'USART config', 17: 'API', 18: 'phase timeout', 19: 'unexpected data',
        20: 'skipped (noise control failed)', 21: 'tail after restart', 22: 'read_until_idle', 23: 'stale bytes',
        24: 'false overrun (less than a ring behind)', 25: 'stopped: another check failed first',
        26: 'read() not woken before its deadline', 27: 'stack margin under 512 bytes (STACK_FREE)'}
for i, r in enumerate(RATES):
    NAMES[40 + 2 * i] = 'duplex %s' % r
    NAMES[41 + 2 * i] = 'duplex %s + load' % r
ERR = {1: 'Framing', 2: 'Noise', 3: 'Overrun', 4: 'Parity', 5: 'other'}

# The phases main() runs for this build, in its order of conditions.
ran = list(range(10)) + [14, 15, 16, 17, 90, 92, 93]
if restart:
    ran += [95] + ([] if wfi else [88, 89]) + ([91] + ([] if main_hal else [94, 18, 19]) if full else [])
else:
    ran += [91] if full else []
    ran += [10, 11, 12, 13] if oneshots else []
    if full:
        ran += list(range(20, 32)) + list(range(40, 50)) + list(range(50, 56)) + list(range(60, 66))
        ran += [70, 71, 72, 73] + ([80] if long_run else [])
ran.sort()

def detail(p):
    d0, d1 = word('DETAIL', 2 * p), word('DETAIL', 2 * p + 1)
    if p < 10:
        return 'STATR %#04x DATAR %#04x' % (d0, d1)
    if 14 <= p <= 17:
        return '%d of %d frames without PE' % (d0, d1)
    if p == 90:
        return 'PE survives the DATAR read, of 10: after nothing %d, STATR read %d, STATR write %d, B sending %d' % tuple(
            d0 >> (4 * i) & 0xf for i in range(4))
    if p == 92:
        return 'ORE survives the DATAR read, of 10: sequence closed %d, after a STATR read %d; DATAR gave %#04x' % (
            d0 & 0xff, d0 >> 8 & 0xff, d1)
    if p == 94:
        f = lambda d: '%d new and contiguous, %d stale, %d with a gap and no error, %d error' % (
            d & 0xff, d >> 24 & 0xff, d >> 8 & 0xff, d >> 16 & 0xff)
        return 'of 200, handler preceded by a STATR read: %s; without: %s (stale: older than the last 3 frames)' % (f(d0), f(d1))
    if p == 88:
        return '%d bytes read, %d overruns (RING_PROFILE below)' % (d0, d1)
    if p == 89:
        return '%d bytes read, %d errors (PROFILE below)' % (d0, d1)
    if p == 95:
        return '64 bytes at 115200 (5.6 ms on the wire): slowest of 5 took %d us, %d bytes still to send then' % (d0, d1)
    if p == 93:
        return 'start_uart(): %d SysTick ticks on average, %d at most' % (d0, d1)
    if p == 91:
        return 'old request served when enabled again, of 5: after a pause %d, after a full set-up %d, after a pause and a CPU read %d' % tuple(
            d0 >> (4 * i) & 0xf for i in range(3))
    if p in (18, 19):
        return 'lost at k 0: %d/20, at k 1: %d/20; first lost: handler entries %d, STATR seen %#x' % (
            d0 & 0xffff, d0 >> 16, d1 >> 16, d1 & 0xffff)
    if p == 31:
        return '%d reads, each woken by the driver in time (idle line, DMA half/full, break); the slowest took %d us' % (d0, d1)
    if p == 49:
        return '%d bytes, %d overruns, every one real' % (d0, d1 & 0xffff)
    if 40 <= p <= 47:
        text = '%d bytes, %d overruns, timer inside USART handler %d' % (d0, d1 & 0xffff, d1 >> 16)
        t = word('DUPLEX_MS', p - 40)
        text += ', %d ms (%d kB/s each way)' % (t, d0 / 2 / t if t else 0)
        return text
    if p == 80:
        return '%d bytes, %d overruns, every one real' % (d0, d1)
    if p == 48:
        return 'timer runs %d, inside a USART handler %d' % (d0, d1)
    if 50 <= p <= 65:
        text = 'rounds %d, error lost %d, bytes delivered %d' % (d0 & 0xffff, d0 >> 16, d1 & 0xfffff)
        if p in (54, 64):
            text += ', rounds with bytes from the starvation on before the error %d' % (d1 >> 20 & 0x3f)
        if p in (53, 63):
            text += ' (14 positions, each until its error came, 5 tries at most; %d rounds without an error, run again)' % (d1 >> 26)
        return text
    if 70 <= p <= 73:
        return '%d bytes, %d overruns' % (d0, d1)
    return '%d %d' % (d0, d1)

state, phase = word('STATE'), word('PHASE')
print('STATE', state, ' PHASE', phase)
ok, skipped = True, 0
for p in ran:
    r = word('RESULT', p)
    name = NAMES.get(p, '?')
    if r == 1:
        print('%3d %-45s ok   %s' % (p, name, detail(p)))
        continue
    ok = False
    # The noise controls (4, 9) do not stop the run: their noise phases are skipped instead.
    if p in (4, 9) and r & 0xff == 11 or r == 20:
        skipped += 1
        print('%3d %-45s SKIPPED: noise not testable here (%#x), not a driver result' % (p, name, r))
        continue
    if r == 0:
        print('%3d %-45s not reached' % (p, name))
        continue
    low, rnd, extra = r & 0xff, r >> 8 & 0xff, r >> 16
    text = FAIL.get(low, 'code %d' % low)
    if low in (4, 6):
        text += ' (%s)' % ERR.get(extra & 0xff, extra)
    print('%3d %-45s FAIL %s, round %d, extra %#x (word %#010x)' % (p, name, text, rnd, extra, r))
print('LOAD timer runs %d, inside USART handler %d, critical sections %d, DMA transfers %d' % tuple(word('LOAD', i) for i in range(4)))
if long_run:
    print('LONG_ROUNDS', word('LONG_ROUNDS'), ' LONG_OVERRUNS (all real)', word('LONG_OVERRUNS') if inside('LONG_OVERRUNS', 4) else '?')
    # 80 succeeds only after its 64 rounds of LONG_BYTES each way (chip.rs): all of them received
    # when there was no overrun, fewer when there were (each checked to be real). Anything else
    # with an ok is not this run's result.
    sent = 128 * {'v203': 1000000, 'v307': 1000000, 'x035': 250000, 'h417': 500000}[chip]
    got, overruns = word('DETAIL', 160), word('DETAIL', 161)
    if word('RESULT', 80) == 1 and (word('LONG_ROUNDS') != 64 or not (got == sent if overruns == 0 else 0 < got < sent)):
        ok = False
        print('80 ok, but not after 64 rounds with %d bytes sent (%d received, %d overruns): not this run\'s result' % (sent, got, overruns))
if inside('PROFILE', 32):
    pr = [word('PROFILE', i) for i in range(8)]
    if pr[0]:
        hz = 'HCLK' if pr[6] & 4 else 'HCLK/8'
        print('PROFILE phase %s (ticks at %s): phase %d, in read() polls %d (%.0f%%) over %d polls, %d reads giving %d bytes, '
              'checks %d (%.0f%%); per byte: read %.0f, check %.0f ticks' % (
                  '89', hz, pr[0], pr[1], 100.0 * pr[1] / pr[0], pr[2], pr[3], pr[4], pr[5], 100.0 * pr[5] / pr[0],
                  pr[1] / max(pr[4], 1), pr[5] / max(pr[4], 1)))
if inside('RING_PROFILE', 32):
    rp = [word('RING_PROFILE', i) for i in range(8)]
    if rp[0]:
        hz = 'HCLK' if rp[7] & 4 else 'HCLK/8'
        print('RING_PROFILE 88 (ticks at %s): phase %d; %d reads with bytes, %d ticks (%.0f each), %d bytes (%.1f ticks each); '
              '%d empty reads, %d ticks (%.0f each); %d overruns' % (
                  hz, rp[0], rp[2], rp[1], rp[1] / max(rp[2], 1), rp[3], rp[1] / max(rp[3], 1), rp[5], rp[4],
                  rp[4] / max(rp[5], 1), rp[6]))
if inside('RESTART_NDTR', 28 * 4):
    rs = [word('RESTART_NDTR', i) for i in range(28)]
    if any(rs):
        print('RESTART_NDTR (before restart, 20 us after)', [(w >> 16, w & 0xffff) for w in rs if w])
if inside('STACK_FREE', 4):
    if chip == 'h417':
        print('STACK_FREE not measured on the H417')
    else:
        print('STACK_FREE %d bytes of stack never written, at worst (0: not measured or none left)' % word('STACK_FREE'))
if inside('WATCH', 23 * 4):
    names = ['polls', 'CSR 0x800', 'mcause', 'mepc', 'STK CTLR', 'SR', 'CNTL', 'CNTH', 'CMPL', 'CMPH', 'PFIC ISR0', 'ISR1',
             'IPR0', 'IPR1', 'ITHRESDR', 'GISR', 'IACTR0', 'IACTR1', 'DMA1 ISR', 'A TX CR', 'NDTR', 'B RX CR', 'NDTR']
    print('WATCH ' + ', '.join('%s %#x' % (n, word('WATCH', i)) for i, n in enumerate(names)))
boot_dirty = False
if 'BOOT_DMA_ON' in sym and inside('BOOT_DMA_ON', 4) and word('BOOT_DMA_ON'):
    on = word('BOOT_DMA_ON')
    if on >> 17 & 1:
        print('BOOT_DMA_ON: RCC\'s software-reset flag was set at startup (a probe session sets it)')
    boot_dirty = bool(on >> 18 & 1)
    if boot_dirty:
        print('BOOT_DMA_ON: the chip was left as a previous program had it: DMA1 channels enabled %s, DMA2 channels enabled %s, clock not as a reset leaves it %s, TIM2 counting %s' % (
            [n + 1 for n in range(8) if on >> n & 1], [n + 1 for n in range(11) if on >> (20 + n) & 1], bool(on >> 8 & 1), bool(on >> 9 & 1)))
if inside('NOISE_CAL', 16) and any(word('NOISE_CAL', i) for i in range(4)):
    for i, what in enumerate(('into B at 115200', 'into B at 1M', 'into A at 115200', 'into A at 1M')):
        c = word('NOISE_CAL', i)
        if c:
            print('NOISE_CAL %s, 32 frames by registers: NE with bit 3 = 0: %d, NE with 1: %d, no NE with 0: %d, no NE with 1: %d' % (
                (what,) + tuple(c >> (8 * j) & 0xff for j in range(4))))
if inside('GAP94', 20) and any(word('GAP94', i) for i in range(5)):
    g = [word('GAP94', i) for i in range(5)]
    print('GAP94 first gap (pass 1): handler STATR seen %#x, entries %d, sent %d, bytes %08x %08x' % tuple(g))
diag = [word('DIAG', i) for i in range(8)]
if any(diag):
    print('DIAG phase %d, code %s (word %#x), then %s' % (diag[0], FAIL.get(diag[1] & 0xff, diag[1]), diag[1],
                                                          ['%#x' % w for w in diag[2:]]))
if boot_dirty:
    # Its own exit status (3): no test ran, whatever image this was.
    print('STOP: the chip was not clean at startup; no test ran')
    sys.exit(3)
if state == 3:
    sys.exit('STOPPED at phase %d' % phase)
if state != 2:
    sys.exit('INCOMPLETE: STATE %d, phase %d' % (state, phase))
if any(diag):
    # Only a failure writes DIAG (main.rs, diag()).
    sys.exit('DONE, but DIAG records a failure')
if skipped:
    sys.exit('DONE, but %d phases skipped: coverage incomplete' % skipped)
if not ok:
    sys.exit('DONE, but not every phase as expected')
print('DONE: every phase as expected')
