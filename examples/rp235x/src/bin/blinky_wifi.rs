//! This example tests the RP Pico 2 W onboard LED.
//!
//! It does not work with the RP Pico 2 board. See `blinky.rs`.

#![no_std]
#![no_main]

use cyw43::{JoinOptions, aligned_bytes};
use cyw43_pio::{PioSpi, RM2_CLOCK_DIVIDER};
use defmt::unwrap;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_net::StackStorage;
use embassy_rp::clocks::RoscRng;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, DMA_CH1, PIO0, USB};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_rp::watchdog::{ResetReason, Watchdog};
use embassy_rp::{bind_interrupts, dma, usb};
use embassy_time::{Duration, Timer};
use static_cell::StaticCell;

// Program metadata for `picotool info`.
// This isn't needed, but it's recommended to have these minimal entries.
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"Blinky Example"),
    embassy_rp::binary_info::rp_program_description!(
        c"This example tests the RP Pico 2 W's onboard LED, connected to GPIO 0 of the cyw43 \
        (WiFi chip) via PIO 0 over the SPI bus."
    ),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>, dma::InterruptHandler<DMA_CH1>;
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
});

/// `log::info!` with seconds since boot in front, for the USB log.
macro_rules! tlog {
    ($($arg:tt)*) => {{
        let ms = embassy_time::Instant::now().as_millis();
        log::info!("[{}.{:03}] {}", ms / 1000, ms % 1000, format_args!($($arg)*));
    }};
}

// Set at build time: `WIFI_NETWORK=ssid WIFI_PASSWORD=pwd cargo build ...`
const WIFI_NETWORK: &str = match option_env!("WIFI_NETWORK") {
    Some(s) => s,
    None => "ssid",
};
const WIFI_PASSWORD: &str = match option_env!("WIFI_PASSWORD") {
    Some(s) => s,
    None => "pwd",
};

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static>) -> ! {
    runner.run().await
}

/// Logs without touching the chip: if this keeps going after the LED lines
/// stop, the executor is alive and the bus is wedged. A panic reboots (see
/// `panic`), and the next boot reports it here.
#[embassy_executor::task]
async fn heartbeat_task(last_reset: LastReset) {
    loop {
        Timer::after_secs(10).await;
        tlog!("alive, last reset: {}", last_reset);
    }
}

// Panic record, kept in watchdog scratch registers across the reset.
const PANIC_MAGIC: u32 = 0x7061_6e69; // "pani"
const SCRATCH_MAGIC: usize = 0;
const SCRATCH_LINE: usize = 1;
const SCRATCH_FILE_PTR: usize = 2;
const SCRATCH_FILE_LEN: usize = 3;
const SCRATCH_UPTIME: usize = 4;

/// Record where the panic happened and reset through the watchdog, so the
/// next boot can say. The message itself would need defmt, and a probe.
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let mut watchdog = Watchdog::new(unsafe { embassy_rp::peripherals::WATCHDOG::steal() });
    let (file, line) = info.location().map_or(("", 0), |l| (l.file(), l.line()));
    watchdog.set_scratch(SCRATCH_LINE, line);
    watchdog.set_scratch(SCRATCH_FILE_PTR, file.as_ptr() as u32);
    watchdog.set_scratch(SCRATCH_FILE_LEN, file.len() as u32);
    watchdog.set_scratch(SCRATCH_UPTIME, embassy_time::Instant::now().as_secs() as u32);
    watchdog.set_scratch(SCRATCH_MAGIC, PANIC_MAGIC);
    watchdog.trigger_reset();
    loop {
        core::hint::spin_loop();
    }
}

#[derive(Clone, Copy)]
enum LastReset {
    Panic { file: &'static str, line: u32, after: u32 },
    Watchdog(Option<ResetReason>),
}

impl LastReset {
    /// Read and clear the panic record, if the previous run left one.
    fn take(watchdog: &mut Watchdog) -> Self {
        let panicked = watchdog.scratch(SCRATCH_MAGIC) == PANIC_MAGIC;
        watchdog.set_scratch(SCRATCH_MAGIC, 0);
        if !panicked {
            return Self::Watchdog(watchdog.reset_reason());
        }

        // Only trust the file pointer if it points into flash.
        let ptr = watchdog.scratch(SCRATCH_FILE_PTR) as usize;
        let len = watchdog.scratch(SCRATCH_FILE_LEN) as usize;
        let file = if (0x1000_0000..0x1100_0000).contains(&ptr) && len <= 128 {
            let bytes = unsafe { core::slice::from_raw_parts(ptr as *const u8, len) };
            core::str::from_utf8(bytes).unwrap_or("?")
        } else {
            "?"
        };
        Self::Panic {
            file,
            line: watchdog.scratch(SCRATCH_LINE),
            after: watchdog.scratch(SCRATCH_UPTIME),
        }
    }
}

impl core::fmt::Display for LastReset {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Panic { file, line, after } => write!(f, "panicked at {}:{} after {}s", file, line, after),
            // A flash with `picotool -x` also reboots through the watchdog.
            Self::Watchdog(Some(reason)) => write!(f, "watchdog ({:?})", reason),
            Self::Watchdog(None) => write!(f, "power-on"),
        }
    }
}

#[embassy_executor::task]
async fn logger_task(driver: usb::Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

#[embassy_executor::task]
async fn ble_task(bt_device: cyw43::bluetooth::BtDriver<'static>) {
    use bt_hci::cmd::controller_baseband::{Reset, SetEventMask};
    use bt_hci::cmd::le::{LeSetEventMask, LeSetScanEnable, LeSetScanParams};
    use bt_hci::controller::{Controller, ControllerCmdSync, ExternalController};
    use bt_hci::param::{AddrKind, EventMask, LeEventMask, LeScanKind, ScanningFilterPolicy};

    let controller: ExternalController<_, 4> = ExternalController::new(bt_device);

    // Command responses arrive through `read`, so it runs alongside the setup.
    let read = async {
        let mut buf = controller.alloc_buf().unwrap();
        let mut events = 0u32;
        loop {
            let _ = controller.read(&mut buf).await;
            events += 1;
            if events % 100 == 0 {
                tlog!("{} ble events", events);
            }
        }
    };

    // Passive scan with duplicate filtering off, so advertising reports keep coming.
    let setup = async {
        controller.exec(&Reset::new()).await.unwrap();
        let mask = EventMask::new().enable_le_meta(true);
        controller.exec(&SetEventMask::new(mask)).await.unwrap();
        let le_mask = LeEventMask::new().enable_le_adv_report(true);
        controller.exec(&LeSetEventMask::new(le_mask)).await.unwrap();
        let interval = bt_hci::param::Duration::from_millis(100);
        controller
            .exec(&LeSetScanParams::new(
                LeScanKind::Passive,
                interval,
                interval,
                AddrKind::PUBLIC,
                ScanningFilterPolicy::BasicUnfiltered,
            ))
            .await
            .unwrap();
        controller.exec(&LeSetScanEnable::new(true, false)).await.unwrap();
        tlog!("ble scanning");
    };

    embassy_futures::join::join(read, setup).await;
}

#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, cyw43::SpiBus<Output<'static>, PioSpi<'static, PIO0, 0>>, cyw43::Cyw43439>,
) -> ! {
    runner.run().await
}

#[embassy_executor::main(executor = "embassy_rp::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    spawner.spawn(unwrap!(logger_task(usb::Driver::new(p.USB, Irqs))));
    let last_reset = LastReset::take(&mut Watchdog::new(p.WATCHDOG));
    spawner.spawn(unwrap!(heartbeat_task(last_reset)));
    let fw = aligned_bytes!("../../../../cyw43-firmware/43439A0.bin");
    let clm = aligned_bytes!("../../../../cyw43-firmware/43439A0_clm.bin");
    let nvram = aligned_bytes!("../../../../cyw43-firmware/nvram_rp2040.bin");
    let btfw = aligned_bytes!("../../../../cyw43-firmware/43439A0_btfw.bin");

    // To make flashing faster for development, you may want to flash the firmwares independently
    // at hardcoded addresses, instead of baking them into the program with `include_bytes!`:
    //     probe-rs download ../../cyw43-firmware/43439A0.bin --binary-format bin --chip RP235x --base-address 0x10100000
    //     probe-rs download ../../cyw43-firmware/43439A0_clm.bin --binary-format bin --chip RP235x --base-address 0x10140000
    //let fw = unsafe { core::slice::from_raw_parts(0x10100000 as *const u8, 230321) };
    //let clm = unsafe { core::slice::from_raw_parts(0x10140000 as *const u8, 4752) };

    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        // SPI communication won't work if the speed is too high, so we use a divider larger than `DEFAULT_CLOCK_DIVIDER`.
        // See: https://github.com/embassy-rs/embassy/issues/3960.
        RM2_CLOCK_DIVIDER,
        pio.irq0,
        cs,
        p.PIN_24,
        p.PIN_29,
        dma::Channel::new(p.DMA_CH0, Irqs),
        dma::Channel::new(p.DMA_CH1, Irqs),
    );

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (net_device, bt_device, mut control, runner) =
        cyw43::new_with_bluetooth(state, pwr, spi, fw, btfw, nvram).await;
    spawner.spawn(unwrap!(cyw43_task(runner)));

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    spawner.spawn(unwrap!(ble_task(bt_device)));

    static STACK: StaticCell<StackStorage> = StaticCell::new();
    let (stack, runner) = embassy_net::Stack::new(STACK.init(StackStorage::new()), RoscRng.next_u64());
    static DEVICE: StaticCell<cyw43::NetDriver<'static>> = StaticCell::new();
    let iface = unwrap!(stack.add_iface(DEVICE.init(net_device)));
    unwrap!(iface.set_dhcpv4(Some(Default::default())));
    spawner.spawn(unwrap!(net_task(runner)));

    while let Err(err) = control
        .join(WIFI_NETWORK, JoinOptions::new(WIFI_PASSWORD.as_bytes()))
        .await
    {
        tlog!("join failed: {:?}", err);
    }
    iface.wait_config_up().await;
    tlog!("wifi up: {:?}", iface.ip_addrs());

    let delay = Duration::from_millis(250);
    loop {
        tlog!("led on!");
        control.gpio_set(0, true).await;
        Timer::after(delay).await;

        tlog!("led off!");
        control.gpio_set(0, false).await;
        Timer::after(delay).await;
    }
}
