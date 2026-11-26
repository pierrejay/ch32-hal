//! What differs between the boards: USART A and its pins, DMA channels and interrupt (USART B is
//! USART2 on PA2 and PA3, DMA1 channels 7 and 6, on all), the clock, the full-duplex rates (up
//! to the USART clock / 16), the timer of the load and the DMA channel of its memory-to-memory
//! transfer; and, in `dma`, the names of the DMA registers the tests reach directly.

use ch32_hal::{self as hal, peripherals};

#[cfg(any(feature = "v203", feature = "v307"))]
mod this {
    use super::*;
    /// CH32V203C8T6, or CH32V307VCT6 (feature `v307`, same clocks, pins and channels): A is
    /// USART1, TX PA9, RX PA10. Wires PA9 to PA3, PA2 to PA10.
    pub type A = peripherals::USART1;
    pub const A_REGS: hal::pac::usart::Usart = hal::pac::USART1;
    pub const A_IRQ: hal::pac::Interrupt = hal::pac::Interrupt::USART1;
    pub type TxA = peripherals::PA9;
    pub type RxA = peripherals::PA10;
    pub type TxDmaA = peripherals::DMA1_CH4;
    pub type RxDmaA = peripherals::DMA1_CH5;
    /// A's DMA1 channels, 0 based.
    pub const A_TX_CH: usize = 3;
    pub const A_RX_CH: usize = 4;
    /// The load's memory-to-memory channel, 0 based: below the receive channels, so that it wins
    /// against them at the same priority level.
    pub const M2M_CH: usize = 0;
    pub type TxB = peripherals::PA2;
    pub type RxB = peripherals::PA3;
    pub fn rcc() -> hal::rcc::Config {
        hal::rcc::Config::SYSCLK_FREQ_144MHZ_HSI
    }
    pub fn enable_timer() {
        hal::pac::RCC.apb1pcenr().modify(|w| w.set_tim2en(true));
    }
    /// Full duplex: baud rate, bytes each way. 9 Mbaud is the fastest on a 144 MHz USART clock.
    pub const DUPLEX: [(u32, u32); 4] = [(115_200, 20_000), (1_000_000, 100_000), (4_500_000, 200_000), (9_000_000, 200_000)];
    pub const LONG: [u32; 4] = [1_000_000, 2_000_000, 4_500_000, 9_000_000];
    /// Bytes each way in each round of the long run.
    pub const LONG_BYTES: u32 = 1_000_000;
    /// TIM2 on its 144 MHz clock, to 1 MHz.
    pub const TIMER_PSC: u16 = 143;
    pub type TimerIrq = hal::interrupt::typelevel::TIM2;
    pub const TIMER_IRQ: hal::pac::Interrupt = hal::pac::Interrupt::TIM2;
}

#[cfg(feature = "x035")]
mod this {
    use super::*;
    /// CH32X035F8U6 (QFN20, without the pins of a whole USART1): A is USART4, TX PB0, RX PB1.
    /// Wires PB0 to PA3, PA2 to PB1.
    pub type A = peripherals::USART4;
    pub const A_REGS: hal::pac::usart::Usart = hal::pac::USART4;
    pub const A_IRQ: hal::pac::Interrupt = hal::pac::Interrupt::USART4;
    pub type TxA = peripherals::PB0;
    pub type RxA = peripherals::PB1;
    pub type TxDmaA = peripherals::DMA1_CH1;
    pub type RxDmaA = peripherals::DMA1_CH8;
    pub const A_TX_CH: usize = 0;
    pub const A_RX_CH: usize = 7;
    pub const M2M_CH: usize = 1;
    pub type TxB = peripherals::PA2;
    pub type RxB = peripherals::PA3;
    pub fn rcc() -> hal::rcc::Config {
        hal::rcc::Config::SYSCLK_FREQ_48MHZ_HSI
    }
    pub fn enable_timer() {
        hal::pac::RCC.apb1pcenr().modify(|w| w.set_tim2en(true));
    }
    /// 3 Mbaud is the fastest on a 48 MHz USART clock.
    pub const DUPLEX: [(u32, u32); 4] = [(115_200, 20_000), (1_000_000, 100_000), (2_000_000, 200_000), (3_000_000, 200_000)];
    pub const LONG: [u32; 4] = [1_000_000, 2_000_000, 3_000_000, 3_000_000];
    /// Slower than the V203 under load: a quarter of its bytes per round keeps the long run
    /// within its quarter of an hour.
    pub const LONG_BYTES: u32 = 250_000;
    pub const TIMER_PSC: u16 = 47;
    pub type TimerIrq = hal::interrupt::typelevel::TIM2_UP;
    pub const TIMER_IRQ: hal::pac::Interrupt = hal::pac::Interrupt::TIM2_UP;
}

#[cfg(feature = "h417")]
mod this {
    use super::*;
    /// CH32H417 (the EVT board), on its V3F boot core alone: A is USART1, TX PA9, RX PA10, as on
    /// the V203; DMA through the DMAMUX. Wires PA9 to PA3, PA2 to PA10.
    pub type A = peripherals::USART1;
    pub const A_REGS: hal::pac::usart::Usart = hal::pac::USART1;
    pub const A_IRQ: hal::pac::Interrupt = hal::pac::Interrupt::USART1;
    pub type TxA = peripherals::PA9;
    pub type RxA = peripherals::PA10;
    pub type TxDmaA = peripherals::DMA1_CH4;
    pub type RxDmaA = peripherals::DMA1_CH5;
    pub const A_TX_CH: usize = 3;
    pub const A_RX_CH: usize = 4;
    pub const M2M_CH: usize = 0;
    pub type TxB = peripherals::PA2;
    pub type RxB = peripherals::PA3;
    /// The V3F and the peripheral buses at 100 MHz, from the HSI.
    pub fn rcc() -> hal::rcc::Config {
        hal::rcc::Config::with_sysclk_400m_v5f_400m_v3f_100m_hsi()
    }
    pub fn enable_timer() {
        hal::pac::RCC.hb1pcenr().modify(|w| w.set_tim2en(true));
    }
    /// Rates that divide the 100 MHz USART clock exactly, up to the fastest, 6.25 Mbaud.
    pub const DUPLEX: [(u32, u32); 4] = [(115_200, 20_000), (1_000_000, 100_000), (3_125_000, 200_000), (6_250_000, 200_000)];
    pub const LONG: [u32; 4] = [1_000_000, 2_500_000, 5_000_000, 6_250_000];
    pub const LONG_BYTES: u32 = 500_000;
    /// TIM2 on its 100 MHz clock, to 1 MHz.
    pub const TIMER_PSC: u16 = 99;
    pub type TimerIrq = hal::interrupt::typelevel::TIM2;
    pub const TIMER_IRQ: hal::pac::Interrupt = hal::pac::Interrupt::TIM2;
    /// B's receive channel (DMA1 channel 6) and its DMAMUX request.
    pub const B_RX_DMA_IRQ: hal::pac::Interrupt = hal::pac::Interrupt::DMA1_CH6;
    #[cfg(feature = "restart")]
    pub const B_RX_REQUEST: hal::dma::Request = 88;
}

/// B's receive channel (DMA1 channel 6) and its request (none without a DMAMUX).
#[cfg(not(feature = "h417"))]
pub const B_RX_DMA_IRQ: hal::pac::Interrupt = hal::pac::Interrupt::DMA1_CHANNEL6;
#[cfg(all(not(feature = "h417"), feature = "restart"))]
pub const B_RX_REQUEST: hal::dma::Request = ();

pub use this::*;

/// DMA1's channels: 7 on the V307, 8 on the V203, X035 and H417.
pub const DMA1_CHANNELS: usize = if cfg!(feature = "v307") { 7 } else { 8 };

/// Whether the DMA keeps going while the core sleeps in WFI (measured by 95): not on the V203
/// and the X035 (#201), yes on the V307 and the H417.
pub const DMA_IN_SLEEP: bool = cfg!(any(feature = "v307", feature = "h417"));

/// For decode.py, in `BUILD`: 1 V203, 2 V307, 3 X035, 4 H417.
pub const ID: u32 = if cfg!(feature = "v307") {
    2
} else if cfg!(feature = "x035") {
    3
} else if cfg!(feature = "h417") {
    4
} else {
    1
};

/// Whether RCC clocks DMA1, DMA2 and TIM2. A peripheral without its clock does not run, and
/// its registers are not to be read: the X035's TIM2 then reads 0xFFFF (a capture after a
/// reset: all its registers at 0xFFFF, APB1PCENR 0).
pub mod clocked {
    use super::hal::pac;

    pub fn dma1() -> bool {
        #[cfg(not(feature = "h417"))]
        return pac::RCC.ahbpcenr().read().dma1en();
        #[cfg(feature = "h417")]
        return pac::RCC.hbpcenr().read().dma1en();
    }

    /// The V307's and the H417's DMA2; false on the others, which have none.
    pub fn dma2() -> bool {
        #[cfg(feature = "v307")]
        return pac::RCC.ahbpcenr().read().dma2en();
        #[cfg(feature = "h417")]
        return pac::RCC.hbpcenr().read().dma2en();
        #[cfg(not(any(feature = "v307", feature = "h417")))]
        return false;
    }

    pub fn tim2() -> bool {
        #[cfg(not(feature = "h417"))]
        return pac::RCC.apb1pcenr().read().tim2en();
        #[cfg(feature = "h417")]
        return pac::RCC.hb1pcenr().read().tim2en();
    }
}

/// The DMA1 registers the tests reach directly, under one set of names, channels counted from
/// 0: the V1 DMA (V203, V307, X035) calls them CR, NDTR, PAR, MAR, ISR and IFCR, the H417's
/// CFGR, CNTR, PADDR, MADDR, INTFR and INTFCR. EN is bit 0 of the configuration on both.
pub mod dma {
    use super::hal::pac;

    /// Transfers the channel has still to make.
    pub fn left(ch: usize) -> u16 {
        #[cfg(not(feature = "h417"))]
        return pac::DMA1.ch(ch).ndtr().read().ndt();
        #[cfg(feature = "h417")]
        return pac::DMA1.ch(ch).cntr().read().ndt();
    }

    pub fn set_left(ch: usize, n: u16) {
        #[cfg(not(feature = "h417"))]
        pac::DMA1.ch(ch).ndtr().write(|w| w.set_ndt(n));
        #[cfg(feature = "h417")]
        pac::DMA1.ch(ch).cntr().write(|w| w.set_ndt(n));
    }

    /// The memory address the channel writes to or reads from.
    pub fn memory(ch: usize) -> u32 {
        #[cfg(not(feature = "h417"))]
        return pac::DMA1.ch(ch).mar().read();
        #[cfg(feature = "h417")]
        return pac::DMA1.ch(ch).maddr().read();
    }

    /// The channel's configuration register, raw.
    pub fn config(ch: usize) -> u32 {
        #[cfg(not(feature = "h417"))]
        return pac::DMA1.ch(ch).cr().read().0;
        #[cfg(feature = "h417")]
        return pac::DMA1.ch(ch).cfgr().read().0;
    }

    pub fn set_config(ch: usize, raw: u32) {
        #[cfg(not(feature = "h417"))]
        pac::DMA1.ch(ch).cr().write_value(pac::dma::regs::Cr(raw));
        #[cfg(feature = "h417")]
        pac::DMA1.ch(ch).cfgr().write_value(pac::dma::regs::Cfgr(raw));
    }

    pub fn enabled(ch: usize) -> bool {
        config(ch) & 1 != 0
    }

    /// The V307's DMA2 channels 8 to 11 sit 16 bytes apart from 0x4002_0490 (its reference
    /// manual, WCH's ch32v30x.h), not where the PAC's `ch()` would put them (20 bytes apart from
    /// channel 1): their configuration registers, enable in bit 0, reached by address.
    #[cfg(feature = "v307")]
    fn dma2_high(k: usize) -> *mut u32 {
        (0x4002_0490 + 16 * k) as *mut u32
    }

    /// Every DMA channel switched off: DMA1's, and DMA2's where there is one (11 on the V307,
    /// 8 on the H417); a DMA without its clock is left alone (super::clocked).
    pub fn stop_all() {
        if super::clocked::dma1() {
            for ch in 0..super::DMA1_CHANNELS {
                set_enabled(ch, false);
            }
        }
        if !super::clocked::dma2() {
            return;
        }
        #[cfg(feature = "v307")]
        {
            for ch in 0..7 {
                pac::DMA2.ch(ch).cr().modify(|w| w.set_en(false));
            }
            for k in 0..4 {
                unsafe { dma2_high(k).write_volatile(dma2_high(k).read_volatile() & !1) };
            }
        }
        #[cfg(feature = "h417")]
        for ch in 0..8 {
            pac::DMA2.ch(ch).cfgr().modify(|w| w.set_en(false));
        }
    }

    /// DMA2's channels found enabled (bit n: channel n + 1), on the chips that have one: 11 on
    /// the V307, 8 on the H417.
    pub fn enabled2() -> u32 {
        #[cfg(feature = "v307")]
        return (0..7).filter(|&ch| pac::DMA2.ch(ch).cr().read().en()).fold(0, |f, ch| f | 1 << ch)
            | (0..4).filter(|&k| unsafe { dma2_high(k).read_volatile() } & 1 != 0).fold(0, |f, k| f | 1 << (7 + k));
        #[cfg(feature = "h417")]
        return (0..8).filter(|&ch| pac::DMA2.ch(ch).cfgr().read().en()).fold(0, |f, ch| f | 1 << ch);
        #[cfg(not(any(feature = "v307", feature = "h417")))]
        return 0;
    }

    pub fn set_enabled(ch: usize, on: bool) {
        #[cfg(not(feature = "h417"))]
        pac::DMA1.ch(ch).cr().modify(|w| w.set_en(on));
        #[cfg(feature = "h417")]
        pac::DMA1.ch(ch).cfgr().modify(|w| w.set_en(on));
    }

    /// The transfer-complete flag.
    pub fn complete(ch: usize) -> bool {
        #[cfg(not(feature = "h417"))]
        return pac::DMA1.isr().read().tcif(ch);
        #[cfg(feature = "h417")]
        return pac::DMA1.intfr().read().tcif(ch);
    }

    /// Clears all of the channel's flags.
    pub fn clear(ch: usize) {
        #[cfg(not(feature = "h417"))]
        pac::DMA1.ifcr().write(|w| w.set_gif(ch, true));
        #[cfg(feature = "h417")]
        pac::DMA1.intfcr().write(|w| w.set_cgif(ch, true));
    }

    /// A memory-to-memory transfer of `words` 32-bit words from `from` to `to`, at the highest
    /// priority (`high`) or the lowest.
    pub fn m2m(ch: usize, high: bool, from: u32, to: u32, words: u16) {
        set_config(ch, 0);
        clear(ch);
        #[cfg(not(feature = "h417"))]
        {
            use pac::dma::vals::{Pl, Size};
            let c = pac::DMA1.ch(ch);
            c.par().write_value(from);
            c.mar().write_value(to);
            c.ndtr().write(|w| w.set_ndt(words));
            c.cr().write(|w| {
                w.set_mem2mem(true);
                w.set_pl(if high { Pl::VERYHIGH } else { Pl::LOW });
                w.set_psize(Size::BITS32);
                w.set_msize(Size::BITS32);
                w.set_en(true);
            });
        }
        #[cfg(feature = "h417")]
        {
            let c = pac::DMA1.ch(ch);
            c.paddr().write_value(from);
            c.maddr().write_value(to);
            c.cntr().write(|w| w.set_ndt(words));
            c.cfgr().write(|w| {
                w.set_mem2mem(true);
                w.set_pl(if high { 3 } else { 0 });
                w.set_psize(2);
                w.set_msize(2);
                w.set_en(true);
            });
        }
    }
}
