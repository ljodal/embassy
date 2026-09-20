use core::slice;

use aligned::{A4, Aligned};
use embassy_futures::yield_now;
use embassy_time::Timer;
use embedded_hal_1::digital::OutputPin;
use futures::FutureExt;

use crate::consts::*;
use crate::runner::{BusType, SealedBus};
use crate::util::Throttle;

/// Custom Spi Trait that _only_ supports the bus operation of the cyw43
/// Implementors are expected to hold the CS pin low during an operation.
pub trait SpiBusCyw43 {
    /// Issues a write command on the bus
    /// First 32 bits of `word` are expected to be a cmd word
    async fn cmd_write(&mut self, write: &[u32]) -> u32;

    /// Issues a read command on the bus
    /// `write` is expected to be a 32 bit cmd word
    /// `read` will contain the response of the device
    /// Backplane reads have a response delay that produces one extra unspecified word at the beginning of `read`.
    /// Callers that want to read `n` word from the backplane, have to provide a slice that is `n+1` words long.
    async fn cmd_read(&mut self, write: u32, read: &mut [u32]) -> u32;

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

/// How many bus transactions to keep for post-mortem.
///
/// Kept small on purpose: the dump has twice now been cut off part way by the
/// USB serial buffer, losing exactly the entries worth having.
const OP_LOG_LEN: usize = 32;

/// How many times to re-issue a read the device answered with
/// `DATA_NOT_AVAILABLE` before giving up on it.
const READ_RETRIES: usize = 3;

/// How many times to ask a function whether it is ready again after a write
/// overflowed its FIFO, before carrying on regardless.
const WRITE_BACKOFF: usize = 8;

/// The last transactions before the bus first reported an error.
///
/// The wedge is only ever *observed* well after it starts: the backplane has
/// already been answering reads with the status word for a while before a ring
/// pointer looks wrong, and the diagnostics that run then would overwrite the
/// very history worth having. So recording stops the moment a transfer comes
/// back with an error bit set, and what is left is the run-up to it.
struct OpLog {
    ops: [(u32, u32); OP_LOG_LEN],
    next: usize,
    len: usize,
    frozen: bool,
}

impl OpLog {
    const fn new() -> Self {
        Self {
            ops: [(0, 0); OP_LOG_LEN],
            next: 0,
            len: 0,
            frozen: false,
        }
    }

    /// Start recording from a clean slate.
    ///
    /// The transfers `init` issues before the bus is configured come back with
    /// meaningless status words -- the device is not yet returning status at
    /// all -- so recording from power-on would freeze the log on the first
    /// probe and keep nothing worth having.
    fn arm(&mut self) {
        *self = Self::new();
    }

    /// Record one transaction and its status, unless already frozen.
    ///
    /// Freezes on the first status carrying an error bit, so the entry that
    /// tripped it is the last one in the log.
    fn record(&mut self, cmd: u32, status: u32) {
        if self.frozen {
            return;
        }

        self.ops[self.next] = (cmd, status);
        self.next = (self.next + 1) % OP_LOG_LEN;
        self.len = (self.len + 1).min(OP_LOG_LEN);

        const ERRORS: u32 = STATUS_DATA_NOT_AVAILABLE | STATUS_UNDERFLOW | STATUS_OVERFLOW | STATUS_HOST_CMD_DATA_ERR;
        if status & ERRORS != 0 {
            self.frozen = true;
        }
    }

    /// How many transactions are held.
    fn count(&self) -> usize {
        self.len
    }

    /// The `i`th transaction, oldest first.
    fn get(&self, i: usize) -> (u32, u32) {
        let start = (self.next + OP_LOG_LEN - self.len) % OP_LOG_LEN;
        self.ops[(start + i) % OP_LOG_LEN]
    }
}

/// Doc
pub struct SpiBus<PWR, SPI> {
    backplane_window: u32,
    pwr: PWR,
    spi: SPI,
    status: u32,
    ops: OpLog,
    /// Reads the device could not serve, however many times they were re-issued.
    starved: Throttle,
    /// Reads the device could not serve at first but answered on a re-issue.
    reissued: Throttle,
    /// Writes the device could not take.
    overflowed: Throttle,
    /// F2 read frames terminated because the device could not serve them.
    f2_aborted: Throttle,
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
            status: 0,
            ops: OpLog::new(),
            starved: Throttle::every(1024),
            reissued: Throttle::every(1024),
            overflowed: Throttle::every(1024),
            f2_aborted: Throttle::every(1024),
        }
    }

    /// Re-issue a read the device could not serve, up to `READ_RETRIES` times.
    ///
    /// gSPI answers a read it has no data for by setting `DATA_NOT_AVAILABLE`
    /// in the status word it returns with that very transfer, and clocking out
    /// padding in place of the data. The read is not consumed: the host is
    /// expected to notice and ask again. Nothing in this driver used to look,
    /// so the padding -- which is whatever the device last had in its shift
    /// register, typically a stale status word -- was handed to the caller as
    /// if it were a register value or a ring pointer.
    ///
    /// `DATA_NOT_AVAILABLE` also latches in `REG_BUS_INTERRUPT`, and the status
    /// word mirrors the latch, so it has to be cleared between attempts for the
    /// next status to describe only the next transfer.
    ///
    /// Returns whether the data is trustworthy.
    async fn reissue(&mut self, func: u8, attempt: usize) -> bool {
        // Only the backplane re-issues. F0 registers are always readable, so a
        // `DATA_NOT_AVAILABLE` seen there is a leftover from an F1 or F2 read;
        // and F2 carries packet data, where asking again would clock the rest
        // of a frame the device has already begun. F2 is handled in
        // `wlan_read` by terminating the frame instead.
        if func != FUNC_BACKPLANE || self.status & STATUS_DATA_NOT_AVAILABLE == 0 {
            // Count the reads that needed asking twice. Without this a quiet
            // run is ambiguous: it could mean the device answered everything
            // first time, or that it did not and the retry covered for it
            // every time. Those call for opposite conclusions.
            if attempt > 0
                && let Some(n) = self.reissued.admit()
            {
                warn!("gSPI func{} read answered on attempt {} (x{})", func, attempt + 1, n);
            }

            return true;
        }

        if attempt < READ_RETRIES {
            // Written out longhand rather than through `writen`, which reads
            // the function info register on an overflow and would make this
            // an async cycle.
            let cmd = cmd_word(WRITE, INC_ADDR, FUNC_BUS, REG_BUS_INTERRUPT, 2);
            self.status = self.spi.cmd_write(&[cmd, IRQ_DATA_UNAVAILABLE as u32]).await;
            self.ops.record(cmd, self.status);
            return false;
        }

        if let Some(n) = self.starved.admit() {
            warn!(
                "gSPI func{} read unanswered after {} attempts, discarding: status {:08x} (x{})",
                func, READ_RETRIES, self.status, n
            );
        }

        true
    }

    /// Terminate an F2 read frame and drop the packet it was carrying.
    ///
    /// The device answered a WLAN data read with `DATA_NOT_AVAILABLE`, so what
    /// came back is padding and the frame it had started is still open. Both
    /// reference drivers handle this by terminating the frame rather than
    /// re-reading it, because re-reading would clock out the remainder of a
    /// packet whose beginning is already lost.
    async fn abort_f2_read(&mut self) {
        self.writen(
            FUNC_BACKPLANE,
            REG_BACKPLANE_FRAME_CONTROL,
            FRAME_CONTROL_ABORT_F2_READ as u32,
            1,
        )
        .await;

        // "Wait whilst the FIFO is emptied of the packet; reading during this
        // period would cause all zeros to be read." -- WHD. This code used to
        // abort and carry straight on.
        Timer::after_millis(1).await;

        if let Some(n) = self.f2_aborted.admit() {
            warn!("gSPI F2 read unanswered, frame terminated and packet dropped (x{})", n);
        }
    }

    /// Report a write the device could not take, and wait for the function to
    /// report itself ready before the next one goes out.
    ///
    /// Status bit 2 is "FIFO overflow occurred due to current (F1, F2, F3)
    /// write command" -- the device could not accept what was just sent.
    /// Nothing in this driver looked at it, so a backplane write that
    /// overflowed was indistinguishable from one that landed, and the next
    /// write went out on top of it.
    ///
    /// Deliberately does not re-issue the write. How much of it landed is not
    /// knowable from here, and repeating a partial write would duplicate bytes
    /// in the Bluetooth ring. Backing off until the function is ready again is
    /// the part that is safe without that knowledge.
    async fn absorbed(&mut self, func: u8) {
        if func == FUNC_BUS || self.status & STATUS_OVERFLOW == 0 {
            return;
        }

        let info_reg = if func == FUNC_WLAN {
            SPI_FUNCTION2_INFO
        } else {
            SPI_FUNCTION1_INFO
        };

        let mut info = 0;
        for _ in 0..WRITE_BACKOFF {
            info = self.read16(FUNC_BUS, info_reg).await;
            if info & SPI_FUNCTIONX_READY != 0 {
                break;
            }
        }

        if let Some(n) = self.overflowed.admit() {
            warn!(
                "gSPI func{} write overflowed the FIFO: info {:04x} (ready {}) (x{})",
                func,
                info,
                info & SPI_FUNCTIONX_READY != 0,
                n
            );
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
        let mut buf = [0; 2];
        // if we are reading from the backplane, we need an extra word for the response delay
        let len = if func == FUNC_BACKPLANE { 2 } else { 1 };

        for attempt in 0..=READ_RETRIES {
            self.status = self.spi.cmd_read(cmd, &mut buf[..len]).await;
            self.ops.record(cmd, self.status);

            if self.reissue(func, attempt).await {
                break;
            }
        }

        // if we read from the backplane, the result is in the second word, after the response delay
        if func == FUNC_BACKPLANE { buf[1] } else { buf[0] }
    }

    async fn writen(&mut self, func: u8, addr: u32, val: u32, len: u32) {
        let cmd = cmd_word(WRITE, INC_ADDR, func, addr, len);

        self.status = self.spi.cmd_write(&[cmd, val]).await;
        self.ops.record(cmd, self.status);
        self.absorbed(func).await;
    }

    async fn read32_swapped(&mut self, func: u8, addr: u32) -> u32 {
        let cmd = cmd_word(READ, INC_ADDR, func, addr, 4);
        let cmd = swap16(cmd);
        let mut buf = [0; 1];

        self.status = self.spi.cmd_read(cmd, &mut buf).await;
        self.ops.record(cmd, self.status);

        swap16(buf[0])
    }

    async fn write32_swapped(&mut self, func: u8, addr: u32, val: u32) {
        let cmd = cmd_word(WRITE, INC_ADDR, func, addr, 4);
        let buf = [swap16(cmd), swap16(val)];

        self.status = self.spi.cmd_write(&buf).await;
        self.ops.record(cmd, self.status);
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
                | STATUS_ENABLE << (8 * REG_BUS_STATUS_ENABLE)
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
        self.write8(FUNC_BUS, SPI_RESP_DELAY_F1, WHD_BUS_SPI_BACKPLANE_READ_PADD_SIZE)
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

        // The bus is configured now, so from here on a status word means
        // something. Anything recorded before this point does not.
        self.ops.arm();

        Ok(())
    }

    async fn wlan_read(&mut self, buf: &mut Aligned<A4, [u8]>) -> crate::Result<()> {
        let len_in_u8 = buf.len() as u32;
        let buf = slice32_mut(buf);

        let cmd = cmd_word(READ, INC_ADDR, FUNC_WLAN, 0, len_in_u8);
        let len_in_u32 = (len_in_u8 as usize).div_ceil(4);

        self.status = self.spi.cmd_read(cmd, &mut buf[..len_in_u32]).await;
        self.ops.record(cmd, self.status);

        if self.status & STATUS_DATA_NOT_AVAILABLE != 0 {
            self.abort_f2_read().await;
            return Err(crate::Error);
        }

        Ok(())
    }

    async fn wlan_write(&mut self, buf: &mut Aligned<A4, [u8]>) -> crate::Result<()> {
        let len = buf.len() - 4;
        buf[..4].copy_from_slice(&cmd_word(WRITE, INC_ADDR, FUNC_WLAN, 0, len as u32).to_le_bytes());

        // `wlan_write` builds its command into the head of the buffer.
        let cmd = slice32_ref(buf)[0];
        self.status = self.spi.cmd_write(slice32_ref(buf)).await;
        self.ops.record(cmd, self.status);
        self.absorbed(FUNC_WLAN).await;

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

            for attempt in 0..=READ_RETRIES {
                // round `buf` to word boundary, add one extra word for the response delay
                self.status = self
                    .spi
                    .cmd_read(cmd, &mut slice32_mut(buf)[..len.div_ceil(4) + 1])
                    .await;
                self.ops.record(cmd, self.status);

                if self.reissue(FUNC_BACKPLANE, attempt).await {
                    break;
                }
            }

            // when writing out the data, we skip the response-delay byte
            data[..len].copy_from_slice(&buf[4..][..len]);

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

            self.status = self.spi.cmd_write(&slice32_ref(buf)[..len.div_ceil(4) + 1]).await;
            self.ops.record(cmd, self.status);
            self.absorbed(FUNC_BACKPLANE).await;

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
        if func == FUNC_BUS && addr == SPI_STATUS_REGISTER && self.status != 0 {
            let status = self.status;
            self.status = 0;

            status
        } else {
            self.readn(func, addr, 4).await
        }
    }

    #[allow(unused)]
    async fn write32(&mut self, func: u8, addr: u32, val: u32) {
        self.writen(func, addr, val, 4).await
    }

    async fn dump_bus_ops(&mut self) {
        let count = self.ops.count();
        if count == 0 {
            warn!("bus op log: empty");
            return;
        }

        warn!("bus op log: {} transactions, oldest first", count);
        for i in 0..count {
            let (cmd, status) = self.ops.get(i);
            warn!(
                "  {}: {} f{} {:05x} len {} -> {:08x}",
                i,
                if cmd >> 31 != 0 { "WR" } else { "RD" },
                (cmd >> 28) & 0b11,
                (cmd >> 11) & 0x1FFFF,
                cmd & 0x7FF,
                status
            );

            // Paced, because this dump has twice been truncated by the USB
            // serial buffer. It runs once per wedge, so the delay is free.
            if i % 4 == 3 {
                Timer::after_millis(2).await;
            }
        }
    }

    async fn bus_selftest(&mut self) -> (u32, u32) {
        let ro = self.read32(FUNC_BUS, REG_BUS_TEST_RO).await;
        let rw = self.read32(FUNC_BUS, REG_BUS_TEST_RW).await;
        (ro, rw)
    }

    async fn bus_reconfigure(&mut self) {
        // Deliberately not `init`: no WL_REG power cycle, no firmware reload,
        // no backplane access. Just the F0 registers, in the order `init` sets
        // them, so a bus whose configuration was lost gets it back.
        self.write32_swapped(
            FUNC_BUS,
            REG_BUS_CTRL,
            WORD_LENGTH_32
                | HIGH_SPEED
                | INTERRUPT_POLARITY_HIGH
                | WAKE_UP
                | 0x4 << (8 * REG_BUS_RESPONSE_DELAY)
                | STATUS_ENABLE << (8 * REG_BUS_STATUS_ENABLE)
                | INTR_WITH_STATUS << (8 * REG_BUS_STATUS_ENABLE),
        )
        .await;

        self.write8(FUNC_BUS, SPI_RESP_DELAY_F1, WHD_BUS_SPI_BACKPLANE_READ_PADD_SIZE)
            .await;

        self.write8(
            FUNC_BUS,
            REG_BUS_INTERRUPT,
            (IRQ_DATA_UNAVAILABLE | IRQ_COMMAND_ERROR | IRQ_DATA_ERROR | IRQ_F1_OVERFLOW) as u8,
        )
        .await;
    }

    fn take_cached_status(&mut self) -> u32 {
        core::mem::take(&mut self.status)
    }

    fn backplane_window_cached(&self) -> u32 {
        self.backplane_window
    }

    fn backplane_window_invalidate(&mut self) {
        // The same sentinel `new` uses: no real window matches it, so every
        // address byte gets written on the next `backplane_set_window`.
        self.backplane_window = 0xAAAA_AAAA;
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
