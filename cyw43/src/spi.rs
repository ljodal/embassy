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

/// Placeholder status for a transaction whose status was not read.
///
/// Most transfers no longer cost an extra bus transaction to ask how they went;
/// only the backplane and WLAN reads do. Zero is a status word with no bit set,
/// so it never freezes the op log and is obvious in a dump.
const UNINSTRUMENTED: u32 = 0;

/// The last transactions before the bus first reported an error.
///
/// The wedge is only ever *observed* well after it starts: the backplane has
/// already been answering reads with the status word for a while before a ring
/// pointer looks wrong, and the diagnostics that run then would overwrite the
/// very history worth having. So recording stops the moment a transfer comes
/// back with an error bit set, and what is left is the run-up to it.
///
/// Transactions whose status was not read carry `UNINSTRUMENTED`, which is the
/// great majority of writes and F0 register reads. The ones that matter -- the
/// backplane and WLAN reads -- carry the status that was read for them.
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
    /// F2 read frames terminated because the device could not serve them.
    f2_aborted: Throttle,
    /// Multi-byte backplane reads issued.
    bp_reads: u32,
    /// Of those, the ones an F2 packet arrived during.
    bp_races: u32,
    /// Of those races, the ones the device could not serve.
    bp_race_losses: u32,
    /// Paces the running tally of the above.
    race_report: Throttle,
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
            f2_aborted: Throttle::every(1024),
            bp_reads: 0,
            bp_races: 0,
            bp_race_losses: 0,
            race_report: Throttle::every(64),
        }
    }

    /// Read the bus status register.
    ///
    /// With `STATUS_ENABLE` clear the device no longer volunteers status, so
    /// anything that wants it asks. `SPI_STATUS_REGISTER` is F0: it needs
    /// neither the backplane nor the window, and has answered correctly in
    /// every wedge so far, which is exactly the property wanted from the thing
    /// used to detect one.
    ///
    /// Written longhand rather than through `readn`, which would call back into
    /// here for backplane reads and make this an async cycle.
    async fn read_status(&mut self) -> u32 {
        let cmd = cmd_word(READ, INC_ADDR, FUNC_BUS, SPI_STATUS_REGISTER, 4);
        let mut buf = [0u32; 1];
        self.spi.cmd_read(cmd, &mut buf).await;
        buf[0]
    }

    /// Account for one multi-byte backplane read against the arriving packet.
    ///
    /// Every wedge so far has been a long backplane read that the F2 packet
    /// available bit set during. What that cannot say, from a log that only
    /// ever gets captured at a failure, is how often the same collision happens
    /// and is survived -- which is the difference between a cause and a
    /// coincidence. So count both.
    fn account_race(&mut self, before: u32, after: u32) {
        self.bp_reads += 1;

        let arrived = before & STATUS_F2_PKT_AVAILABLE == 0 && after & STATUS_F2_PKT_AVAILABLE != 0;
        if !arrived {
            return;
        }

        self.bp_races += 1;
        if after & STATUS_DATA_NOT_AVAILABLE != 0 {
            self.bp_race_losses += 1;
        }

        if self.race_report.admit().is_some() {
            warn!(
                "backplane reads: {} issued, {} raced an arriving packet, {} of those unserved",
                self.bp_reads, self.bp_races, self.bp_race_losses
            );
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
            self.writen(FUNC_BUS, REG_BUS_INTERRUPT, IRQ_DATA_UNAVAILABLE as u32, 2)
                .await;
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
            self.spi.cmd_read(cmd, &mut buf[..len]).await;

            // Only the backplane spends a transaction on status. F0 registers
            // are always readable and F2 is handled where it is read, so asking
            // after every register access would double the traffic to learn
            // nothing.
            if func == FUNC_BACKPLANE {
                self.status = self.read_status().await;
            }
            self.ops.record(
                cmd,
                if func == FUNC_BACKPLANE {
                    self.status
                } else {
                    UNINSTRUMENTED
                },
            );

            if self.reissue(func, attempt).await {
                break;
            }
        }

        // if we read from the backplane, the result is in the second word, after the response delay
        if func == FUNC_BACKPLANE { buf[1] } else { buf[0] }
    }

    async fn writen(&mut self, func: u8, addr: u32, val: u32, len: u32) {
        let cmd = cmd_word(WRITE, INC_ADDR, func, addr, len);

        self.spi.cmd_write(&[cmd, val]).await;
        self.ops.record(cmd, UNINSTRUMENTED);
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
                // `STATUS_ENABLE` is deliberately absent. It makes the device
                // append a status word to every read and write, and neither
                // reference driver turns it on: `cyw43_ll.c:1561` passes
                // `INTR_WITH_STATUS` alone, and WHD writes
                // `(0 & STATUS_ENABLE)` next to it, which reads like someone
                // once tried the other way. `INTR_WITH_STATUS` only means
                // anything "if status is sent", so it is inert here; it is kept
                // because both references keep it.
                //
                // This driver used to set it and read the trailing word after
                // every transfer. That put the chip in a mode no reference
                // driver exercises, in the response path that wedges: on a
                // backplane underrun mid-data the device had to abandon the
                // data and still frame a trailing status. Status now comes from
                // `SPI_STATUS_REGISTER`, which is F0 and answers even when the
                // backplane does not.
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

        self.spi.cmd_read(cmd, &mut buf[..len_in_u32]).await;
        self.status = self.read_status().await;
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
        self.spi.cmd_write(slice32_ref(buf)).await;
        self.ops.record(cmd, UNINSTRUMENTED);

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

            // On gSPI the 32-bit access flag goes on every backplane transfer,
            // not just the 4-byte ones: it tells the bridge to move the data in
            // words rather than a byte at a time, so a 48-byte read costs the
            // backplane 12 transactions instead of 48. `cyw43_ll.c` has it
            // unconditional under `CYW43_USE_SPI` and applies it to bulk reads
            // too; only the SDIO path makes it conditional on the length.
            let cmd = cmd_word(
                READ,
                INC_ADDR,
                FUNC_BACKPLANE,
                window_offs | BACKPLANE_ADDRESS_32BIT_FLAG,
                len as u32,
            );

            for attempt in 0..=READ_RETRIES {
                // The status left by the previous transfer, so `account_race`
                // can tell a packet that arrived *during* this read from one
                // that was already waiting when it started -- the two look
                // identical afterwards, and only the first is the collision
                // under suspicion. Deliberately the stale value rather than a
                // fresh read: every backplane read already costs one extra
                // transaction for the status after it, and doubling that to
                // sharpen the "before" would perturb the timing this is
                // measuring. The fatal read is always preceded by another
                // backplane read, so this is at most one transaction old.
                let before = self.status;

                // round `buf` to word boundary, add one extra word for the response delay
                self.spi
                    .cmd_read(cmd, &mut slice32_mut(buf)[..len.div_ceil(4) + 1])
                    .await;

                self.status = self.read_status().await;
                self.ops.record(cmd, self.status);
                self.account_race(before, self.status);

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

            let cmd = cmd_word(
                WRITE,
                INC_ADDR,
                FUNC_BACKPLANE,
                window_offs | BACKPLANE_ADDRESS_32BIT_FLAG,
                len as u32,
            );
            slice32_mut(buf)[0] = cmd;

            self.spi.cmd_write(&slice32_ref(buf)[..len.div_ceil(4) + 1]).await;
            self.ops.record(cmd, UNINSTRUMENTED);

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
        // No special case for `SPI_STATUS_REGISTER` any more: there is no
        // cached status word to return in place of reading it, so every caller
        // that asks for status gets it from the wire.
        self.readn(func, addr, 4).await
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
            if status == UNINSTRUMENTED {
                warn!(
                    "  {}: {} f{} {:05x} len {}",
                    i,
                    if cmd >> 31 != 0 { "WR" } else { "RD" },
                    (cmd >> 28) & 0b11,
                    (cmd >> 11) & 0x1FFFF,
                    cmd & 0x7FF
                );
            } else {
                warn!(
                    "  {}: {} f{} {:05x} len {} -> {:08x}",
                    i,
                    if cmd >> 31 != 0 { "WR" } else { "RD" },
                    (cmd >> 28) & 0b11,
                    (cmd >> 11) & 0x1FFFF,
                    cmd & 0x7FF,
                    status
                );
            }

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
                // `STATUS_ENABLE` is deliberately absent. It makes the device
                // append a status word to every read and write, and neither
                // reference driver turns it on: `cyw43_ll.c:1561` passes
                // `INTR_WITH_STATUS` alone, and WHD writes
                // `(0 & STATUS_ENABLE)` next to it, which reads like someone
                // once tried the other way. `INTR_WITH_STATUS` only means
                // anything "if status is sent", so it is inert here; it is kept
                // because both references keep it.
                //
                // This driver used to set it and read the trailing word after
                // every transfer. That put the chip in a mode no reference
                // driver exercises, in the response path that wedges: on a
                // backplane underrun mid-data the device had to abandon the
                // data and still frame a trailing status. Status now comes from
                // `SPI_STATUS_REGISTER`, which is F0 and answers even when the
                // backplane does not.
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
        // Nothing to take. With `STATUS_ENABLE` clear the device does not
        // append status to a transfer, so there is no cached word that could
        // have been stale -- the failure mode this existed to measure cannot
        // arise. Kept so the caller's diagnostic compiles and stays inert.
        0
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
