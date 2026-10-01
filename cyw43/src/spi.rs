use core::slice;

use aligned::{A4, Aligned};
use embassy_futures::yield_now;
use embassy_time::Timer;
use embedded_hal_1::digital::OutputPin;
use futures::FutureExt;

use crate::consts::*;
use crate::runner::{BusType, SealedBus};

/// Custom Spi Trait that _only_ supports the bus operation of the cyw43
/// Implementors are expected to hold the CS pin low during an operation.
///
/// The driver configures the device without `STATUS_ENABLE`, so it does not
/// append a status word to transfers and implementations must not clock one.
pub trait SpiBusCyw43 {
    /// Issues a write command on the bus
    /// First 32 bits of `word` are expected to be a cmd word
    async fn cmd_write(&mut self, write: &[u32]);

    /// Issues a read command on the bus
    /// `write` is expected to be a 32 bit cmd word
    /// `read` will contain the response of the device
    /// Backplane reads have a response delay that produces extra unspecified words at the beginning of `read`.
    /// Callers that want to read `n` words from the backplane provide a slice that is long enough for both.
    async fn cmd_read(&mut self, write: u32, read: &mut [u32]);

    /// Wait for events from the Device. A typical implementation would wait for the IRQ pin to be high.
    /// The default implementation always reports ready, resulting in active polling of the device.
    async fn wait_for_event(&mut self) {
        yield_now().await;
    }
}

const fn slice32_mut(x: &mut Aligned<A4, [u8]>) -> &mut [u32] {
    let len = size_of_val(x).div_ceil(4);
    unsafe { slice::from_raw_parts_mut(x as *mut Aligned<A4, [u8]> as *mut u32, len) }
}

const fn slice32_ref(x: &Aligned<A4, [u8]>) -> &[u32] {
    let len = size_of_val(x).div_ceil(4);
    unsafe { slice::from_raw_parts(x as *const Aligned<A4, [u8]> as *const u32, len) }
}

// DIAGNOSTIC: a RAM ring of the last `TRACE_LEN` bus operations, dumped once
// when a backplane read is seen to return impossible data, so the log shows
// what led up to it. Five words per entry: kind/func/len, address, first data
// word, backplane window, time in us. Atomics so no `unsafe`; only the runner
// touches it, so load + store is enough (no `fetch_add` on thumbv6m).
mod diag_trace {
    use core::sync::atomic::{AtomicBool, AtomicU32, Ordering::Relaxed};

    pub(crate) const READ: u32 = 1;
    pub(crate) const WRITE: u32 = 2;
    pub(crate) const BP_READ: u32 = 3;
    pub(crate) const BP_WRITE: u32 = 4;
    pub(crate) const WLAN_READ: u32 = 5;
    pub(crate) const WLAN_WRITE: u32 = 6;

    const TRACE_LEN: usize = 64;
    const WORDS: usize = 5;
    static TRACE: [AtomicU32; TRACE_LEN * WORDS] = [const { AtomicU32::new(0) }; TRACE_LEN * WORDS];
    static NEXT: AtomicU32 = AtomicU32::new(0);
    static FROZEN: AtomicBool = AtomicBool::new(false);

    pub(crate) fn record(kind: u32, func: u8, addr: u32, len: usize, val: u32, window: u32) {
        if FROZEN.load(Relaxed) {
            return;
        }
        let n = NEXT.load(Relaxed);
        NEXT.store(n.wrapping_add(1), Relaxed);
        let e = (n as usize % TRACE_LEN) * WORDS;
        TRACE[e].store(kind << 28 | (func as u32) << 24 | (len as u32 & 0xFFFF), Relaxed);
        TRACE[e + 1].store(addr, Relaxed);
        TRACE[e + 2].store(val, Relaxed);
        TRACE[e + 3].store(window, Relaxed);
        TRACE[e + 4].store(embassy_time::Instant::now().as_micros() as u32, Relaxed);
    }

    /// Log the trace, oldest first, and stop recording. Only the first call logs.
    #[allow(unused)]
    pub(crate) fn dump() {
        if FROZEN.load(Relaxed) {
            return;
        }
        FROZEN.store(true, Relaxed);
        let n = NEXT.load(Relaxed);
        let count = (n as usize).min(TRACE_LEN);
        for i in 0..count {
            let seq = n.wrapping_sub((count - i) as u32);
            let e = (seq as usize % TRACE_LEN) * WORDS;
            let meta = TRACE[e].load(Relaxed);
            let kind = match meta >> 28 {
                READ => "read",
                WRITE => "write",
                BP_READ => "bp_read",
                BP_WRITE => "bp_write",
                WLAN_READ => "wlan_read",
                WLAN_WRITE => "wlan_write",
                _ => "?",
            };
            warn!(
                "trace #{} t={}us {} f{} addr={:08x} len={} val={:08x} window={:08x}",
                seq,
                TRACE[e + 4].load(Relaxed),
                kind,
                (meta >> 24) & 0xF,
                TRACE[e + 1].load(Relaxed),
                meta & 0xFFFF,
                TRACE[e + 2].load(Relaxed),
                TRACE[e + 3].load(Relaxed)
            );
        }
    }
}

#[allow(unused_imports)]
pub(crate) use diag_trace::dump as diag_trace_dump;

/// Doc
pub struct SpiBus<PWR, SPI> {
    backplane_window: u32,
    pwr: PWR,
    spi: SPI,
}

impl<PWR, SPI> SpiBus<PWR, SPI>
where
    PWR: OutputPin,
    SPI: SpiBusCyw43,
{
    pub(crate) fn new(pwr: PWR, spi: SPI) -> Self {
        Self {
            backplane_window: 0xAAAA_AAAA,
            pwr,
            spi,
        }
    }

    async fn backplane_readn(&mut self, addr: u32, len: u32) -> u32 {
        trace!("backplane_readn addr = {:08x} len = {}", addr, len);

        self.backplane_set_window(addr).await;

        let mut bus_addr = addr & BACKPLANE_ADDRESS_MASK;
        if len == 4 {
            bus_addr |= BACKPLANE_ADDRESS_32BIT_FLAG;
        }

        let val = self.readn(FUNC_BACKPLANE, bus_addr, len).await;

        trace!("backplane_readn addr = {:08x} len = {} val = {:08x}", addr, len, val);

        val
    }

    async fn backplane_writen(&mut self, addr: u32, val: u32, len: u32) {
        trace!("backplane_writen addr = {:08x} len = {} val = {:08x}", addr, len, val);

        self.backplane_set_window(addr).await;

        let mut bus_addr = addr & BACKPLANE_ADDRESS_MASK;
        if len == 4 {
            bus_addr |= BACKPLANE_ADDRESS_32BIT_FLAG;
        }
        self.writen(FUNC_BACKPLANE, bus_addr, val, len).await;
    }

    async fn backplane_set_window(&mut self, addr: u32) {
        let new_window = addr & !BACKPLANE_ADDRESS_MASK;

        if (new_window >> 24) as u8 != (self.backplane_window >> 24) as u8 {
            self.write8(
                FUNC_BACKPLANE,
                REG_BACKPLANE_BACKPLANE_ADDRESS_HIGH,
                (new_window >> 24) as u8,
            )
            .await;
        }
        if (new_window >> 16) as u8 != (self.backplane_window >> 16) as u8 {
            self.write8(
                FUNC_BACKPLANE,
                REG_BACKPLANE_BACKPLANE_ADDRESS_MID,
                (new_window >> 16) as u8,
            )
            .await;
        }
        if (new_window >> 8) as u8 != (self.backplane_window >> 8) as u8 {
            self.write8(
                FUNC_BACKPLANE,
                REG_BACKPLANE_BACKPLANE_ADDRESS_LOW,
                (new_window >> 8) as u8,
            )
            .await;
        }
        self.backplane_window = new_window;
    }

    async fn readn(&mut self, func: u8, addr: u32, len: u32) -> u32 {
        let cmd = cmd_word(READ, INC_ADDR, func, addr, len);
        let mut buf = [0; SPI_BACKPLANE_READ_PAD_LEN_WORDS + 1];
        // if we are reading from the backplane, we need extra words for the response delay
        let pad = if func == FUNC_BACKPLANE {
            SPI_BACKPLANE_READ_PAD_LEN_WORDS
        } else {
            0
        };

        self.spi.cmd_read(cmd, &mut buf[..pad + 1]).await;
        diag_trace::record(diag_trace::READ, func, addr, len as usize, buf[pad], self.backplane_window);

        // the result follows the response delay
        buf[pad]
    }

    async fn writen(&mut self, func: u8, addr: u32, val: u32, len: u32) {
        let cmd = cmd_word(WRITE, INC_ADDR, func, addr, len);

        self.spi.cmd_write(&[cmd, val]).await;
        diag_trace::record(diag_trace::WRITE, func, addr, len as usize, val, self.backplane_window);
    }

    async fn read32_swapped(&mut self, func: u8, addr: u32) -> u32 {
        let cmd = cmd_word(READ, INC_ADDR, func, addr, 4);
        let cmd = swap16(cmd);
        let mut buf = [0; 1];

        self.spi.cmd_read(cmd, &mut buf).await;

        swap16(buf[0])
    }

    async fn write32_swapped(&mut self, func: u8, addr: u32, val: u32) {
        let cmd = cmd_word(WRITE, INC_ADDR, func, addr, 4);
        let buf = [swap16(cmd), swap16(val)];

        self.spi.cmd_write(&buf).await;
    }
}

impl<PWR, SPI> SealedBus for SpiBus<PWR, SPI>
where
    PWR: OutputPin,
    SPI: SpiBusCyw43,
{
    const TYPE: BusType = BusType::Spi;

    async fn init<'a>(&mut self, bluetooth_enabled: bool) -> crate::Result<()> {
        fn cmp<R: Eq>(left: R, right: R) -> Result<(), ()> {
            if left == right { Ok(()) } else { Err(()) }
        }

        // Reset
        trace!("WL_REG off/on");
        self.pwr.set_low().unwrap();
        Timer::after_millis(20).await;
        self.pwr.set_high().unwrap();
        Timer::after_millis(250).await;

        trace!("read REG_BUS_TEST_RO");
        while self
            .read32_swapped(FUNC_BUS, REG_BUS_TEST_RO)
            .inspect(|v| trace!("{:#x}", v))
            .await
            != FEEDBEAD
        {}

        trace!("write REG_BUS_TEST_RW");
        self.write32_swapped(FUNC_BUS, REG_BUS_TEST_RW, TEST_PATTERN).await;
        let val = self.read32_swapped(FUNC_BUS, REG_BUS_TEST_RW).await;
        trace!("{:#x}", val);
        cmp(val, TEST_PATTERN).map_err(|_| crate::Error)?;

        trace!("read REG_BUS_CTRL");
        let val = self.read32_swapped(FUNC_BUS, REG_BUS_CTRL).await;
        trace!("{:#010b}", (val & 0xff));

        // 32-bit word length, little endian (which is the default endianess).
        // TODO: C library is uint32_t val = WORD_LENGTH_32 | HIGH_SPEED_MODE| ENDIAN_BIG | INTERRUPT_POLARITY_HIGH | WAKE_UP | 0x4 << (8 * SPI_RESPONSE_DELAY) | INTR_WITH_STATUS << (8 * SPI_STATUS_ENABLE);
        trace!("write REG_BUS_CTRL");
        self.write32_swapped(
            FUNC_BUS,
            REG_BUS_CTRL,
            WORD_LENGTH_32
                | HIGH_SPEED
                | INTERRUPT_POLARITY_HIGH
                | WAKE_UP
                | 0x4 << (8 * REG_BUS_RESPONSE_DELAY)
                | INTR_WITH_STATUS << (8 * REG_BUS_STATUS_ENABLE),
        )
        .await;

        trace!("read REG_BUS_CTRL");
        let val = self.read8(FUNC_BUS, REG_BUS_CTRL).await;
        trace!("{:#b}", val);

        // TODO: C doesn't do this? i doubt it messes anything up
        trace!("read REG_BUS_TEST_RO");
        let val = self.read32(FUNC_BUS, REG_BUS_TEST_RO).await;
        trace!("{:#x}", val);
        cmp(val, FEEDBEAD).map_err(|_| crate::Error)?;

        // TODO: C doesn't do this? i doubt it messes anything up
        trace!("read REG_BUS_TEST_RW");
        let val = self.read32(FUNC_BUS, REG_BUS_TEST_RW).await;
        trace!("{:#x}", val);
        cmp(val, TEST_PATTERN).map_err(|_| crate::Error)?;

        trace!("write SPI_RESP_DELAY_F1 CYW43_BACKPLANE_READ_PAD_LEN_BYTES");
        self.write8(FUNC_BUS, SPI_RESP_DELAY_F1, SPI_BACKPLANE_READ_PAD_LEN_BYTES)
            .await;

        // TODO: Make sure error interrupt bits are clear?
        // cyw43_write_reg_u8(self, BUS_FUNCTION, SPI_INTERRUPT_REGISTER, DATA_UNAVAILABLE | COMMAND_ERROR | DATA_ERROR | F1_OVERFLOW) != 0)
        trace!("Make sure error interrupt bits are clear");
        self.write8(
            FUNC_BUS,
            REG_BUS_INTERRUPT,
            (IRQ_DATA_UNAVAILABLE | IRQ_COMMAND_ERROR | IRQ_DATA_ERROR | IRQ_F1_OVERFLOW) as u8,
        )
        .await;

        // Enable a selection of interrupts
        // TODO: why not all of these F2_F3_FIFO_RD_UNDERFLOW | F2_F3_FIFO_WR_OVERFLOW | COMMAND_ERROR | DATA_ERROR | F2_PACKET_AVAILABLE | F1_OVERFLOW | F1_INTR
        trace!("enable a selection of interrupts");
        let mut val = IRQ_F2_F3_FIFO_RD_UNDERFLOW
            | IRQ_F2_F3_FIFO_WR_OVERFLOW
            | IRQ_COMMAND_ERROR
            | IRQ_DATA_ERROR
            | IRQ_F2_PACKET_AVAILABLE
            | IRQ_F1_OVERFLOW;
        if bluetooth_enabled {
            val |= IRQ_F1_INTR;
        }
        self.write16(FUNC_BUS, REG_BUS_INTERRUPT_ENABLE, val).await;

        Ok(())
    }

    async fn wlan_read(&mut self, buf: &mut Aligned<A4, [u8]>) -> crate::Result<()> {
        let len_in_u8 = buf.len() as u32;
        let buf = slice32_mut(buf);

        let cmd = cmd_word(READ, INC_ADDR, FUNC_WLAN, 0, len_in_u8);
        let len_in_u32 = (len_in_u8 as usize).div_ceil(4);

        self.spi.cmd_read(cmd, &mut buf[..len_in_u32]).await;
        diag_trace::record(diag_trace::WLAN_READ, FUNC_WLAN, 0, len_in_u8 as usize, buf[0], self.backplane_window);

        Ok(())
    }

    async fn wlan_write(&mut self, buf: &mut Aligned<A4, [u8]>) -> crate::Result<()> {
        let len = buf.len() - 4;
        buf[..4].copy_from_slice(&cmd_word(WRITE, INC_ADDR, FUNC_WLAN, 0, len as u32).to_le_bytes());

        self.spi.cmd_write(slice32_ref(buf)).await;
        diag_trace::record(diag_trace::WLAN_WRITE, FUNC_WLAN, 0, len, slice32_ref(buf)[1], self.backplane_window);

        Ok(())
    }

    async fn bp_read(&mut self, mut addr: u32, mut data: &mut [u8], buf: &mut Aligned<A4, [u8]>) -> crate::Result<()> {
        trace!("bp_read addr = {:08x}", addr);

        // It seems the HW force-aligns the addr
        // to 2 if data.len() >= 2
        // to 4 if data.len() >= 4
        // To simplify, enforce 4-align for now.
        assert!(addr.is_multiple_of(4));

        while !data.is_empty() {
            // Ensure transfer doesn't cross a window boundary.
            let window_offs = addr & BACKPLANE_ADDRESS_MASK;
            let window_remaining = BACKPLANE_WINDOW_SIZE - window_offs as usize;

            let len = data.len().min(BACKPLANE_MAX_TRANSFER_SIZE).min(window_remaining);

            self.backplane_set_window(addr).await;

            let cmd = cmd_word(READ, INC_ADDR, FUNC_BACKPLANE, window_offs, len as u32);

            // round `buf` to word boundary, add the response delay words
            self.spi
                .cmd_read(
                    cmd,
                    &mut slice32_mut(buf)[..SPI_BACKPLANE_READ_PAD_LEN_WORDS + len.div_ceil(4)],
                )
                .await;

            // when writing out the data, we skip the response delay
            data[..len].copy_from_slice(&buf[SPI_BACKPLANE_READ_PAD_LEN_BYTES as usize..][..len]);
            diag_trace::record(
                diag_trace::BP_READ,
                FUNC_BACKPLANE,
                addr,
                len,
                slice32_mut(buf)[SPI_BACKPLANE_READ_PAD_LEN_WORDS],
                self.backplane_window,
            );

            // Advance ptr.
            addr += len as u32;
            data = &mut data[len..];
        }

        Ok(())
    }

    async fn bp_write(&mut self, mut addr: u32, mut data: &[u8], buf: &mut Aligned<A4, [u8]>) -> crate::Result<()> {
        trace!("bp_write addr = {:08x}", addr);

        // It seems the HW force-aligns the addr
        // to 2 if data.len() >= 2
        // to 4 if data.len() >= 4
        // To simplify, enforce 4-align for now.
        assert!(addr.is_multiple_of(4));

        while !data.is_empty() {
            // Ensure transfer doesn't cross a window boundary.
            let window_offs = addr & BACKPLANE_ADDRESS_MASK;
            let window_remaining = BACKPLANE_WINDOW_SIZE - window_offs as usize;

            let len = data.len().min(BACKPLANE_MAX_TRANSFER_SIZE).min(window_remaining);
            buf[4..][..len].copy_from_slice(&data[..len]);

            self.backplane_set_window(addr).await;

            let cmd = cmd_word(WRITE, INC_ADDR, FUNC_BACKPLANE, window_offs, len as u32);
            slice32_mut(buf)[0] = cmd;

            self.spi.cmd_write(&slice32_ref(buf)[..len.div_ceil(4) + 1]).await;
            diag_trace::record(
                diag_trace::BP_WRITE,
                FUNC_BACKPLANE,
                addr,
                len,
                slice32_ref(buf)[1],
                self.backplane_window,
            );

            // Advance ptr.
            addr += len as u32;
            data = &data[len..];
        }

        Ok(())
    }

    async fn bp_read8(&mut self, addr: u32) -> u8 {
        self.backplane_readn(addr, 1).await as u8
    }

    async fn bp_write8(&mut self, addr: u32, val: u8) {
        self.backplane_writen(addr, val as u32, 1).await
    }

    async fn bp_read16(&mut self, addr: u32) -> u16 {
        self.backplane_readn(addr, 2).await as u16
    }

    #[allow(unused)]
    async fn bp_write16(&mut self, addr: u32, val: u16) {
        self.backplane_writen(addr, val as u32, 2).await
    }

    #[allow(unused)]
    async fn bp_read32(&mut self, addr: u32) -> u32 {
        self.backplane_readn(addr, 4).await
    }

    async fn bp_write32(&mut self, addr: u32, val: u32) {
        self.backplane_writen(addr, val, 4).await
    }

    async fn read8(&mut self, func: u8, addr: u32) -> u8 {
        self.readn(func, addr, 1).await as u8
    }

    async fn write8(&mut self, func: u8, addr: u32, val: u8) {
        self.writen(func, addr, val as u32, 1).await
    }

    async fn read16(&mut self, func: u8, addr: u32) -> u16 {
        self.readn(func, addr, 2).await as u16
    }

    #[allow(unused)]
    async fn write16(&mut self, func: u8, addr: u32, val: u16) {
        self.writen(func, addr, val as u32, 2).await
    }

    async fn read32(&mut self, func: u8, addr: u32) -> u32 {
        self.readn(func, addr, 4).await
    }

    #[allow(unused)]
    async fn write32(&mut self, func: u8, addr: u32, val: u32) {
        self.writen(func, addr, val, 4).await
    }

    async fn wait_for_event(&mut self) {
        self.spi.wait_for_event().await;
    }
}

fn swap16(x: u32) -> u32 {
    x.rotate_left(16)
}

fn cmd_word(write: bool, incr: bool, func: u8, addr: u32, len: u32) -> u32 {
    (write as u32) << 31 | (incr as u32) << 30 | (func as u32 & 0b11) << 28 | (addr & 0x1FFFF) << 11 | (len & 0x7FF)
}
