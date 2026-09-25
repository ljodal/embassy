//! Minimal reproducer for the cyw43 gSPI bus wedge on the Pico 2 W.
//!
//! No application logic: the chip is given the two kinds of traffic that
//! exercised the wedge in a larger program, and a monitor reports every
//! [`REPORT_EVERY`] whether each side is still moving.
//!
//! - `wifi`: join a network, get an address by DHCP, answer pings. Load it
//!   from another machine with `sudo ping -f <address>`.
//! - `ble`: load the Bluetooth firmware and keep the controller scanning with
//!   duplicate filtering off, so advertising reports stream in continuously.
//!
//! Both are on by default. For one side alone:
//!
//! ```sh
//! WIFI_NETWORK=ssid WIFI_PASSWORD=secret cargo run --release
//! WIFI_NETWORK=ssid WIFI_PASSWORD=secret cargo run --release --no-default-features --features wifi
//! cargo run --release --no-default-features --features ble
//! ```
//!
//! Logs go out over USB serial (CDC-ACM), so no probe is needed.
//!
//! Independently of both, the LED is toggled over the bus every
//! [`LED_EVERY`]. That is an ioctl, so it needs the bus in both directions:
//! when it stops completing, the bus has stopped, whichever side stopped it.

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};

use cyw43::aligned_bytes;
use cyw43_pio::PioSpi;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, DMA_CH1, PIO0, USB};
use embassy_rp::pio::{self, Pio};
use embassy_rp::{bind_interrupts, dma, usb};
use embassy_time::{Duration, Instant, Timer};
use fixed::FixedU32;
use fixed::types::extra::U8;
use log::{info, warn};
use panic_halt as _;
use static_cell::StaticCell;

#[cfg(not(any(feature = "wifi", feature = "ble")))]
compile_error!("enable `wifi`, `ble`, or both");

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>, dma::InterruptHandler<DMA_CH1>;
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
});

/// 150 MHz / 4 = 37.5 MHz, what the program this was found in ran at.
/// `RM2_CLOCK_DIVIDER` (3.0, 50 MHz) wedges too.
const CLOCK_DIVIDER: FixedU32<U8> = FixedU32::from_bits(0x0400);

const REPORT_EVERY: Duration = Duration::from_secs(10);
const LED_EVERY: Duration = Duration::from_millis(500);
/// How long without a completed LED ioctl before the bus is reported stalled.
const LED_STALL: Duration = Duration::from_secs(5);

/// LED ioctls completed.
static LED_TOGGLES: AtomicU32 = AtomicU32::new(0);
/// When the last one completed, in ms since boot (no 64-bit atomics on M33).
static LED_LAST_MS: AtomicU32 = AtomicU32::new(0);
/// HCI events received from the controller, and their total size.
#[cfg(feature = "ble")]
static BLE_EVENTS: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "ble")]
static BLE_BYTES: AtomicU32 = AtomicU32::new(0);

type Bus = cyw43::SpiBus<Output<'static>, PioSpi<'static, PIO0, 0>>;

#[embassy_executor::task]
async fn cyw43_task(runner: cyw43::Runner<'static, Bus, cyw43::Cyw43439>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn logger_task(driver: usb::Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

#[cfg(feature = "wifi")]
#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static>) -> ! {
    runner.run().await
}

#[embassy_executor::main(executor = "embassy_rp::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    spawner.spawn(logger_task(usb::Driver::new(p.USB, Irqs)).unwrap());
    // Time to attach a serial terminal before anything interesting is logged.
    Timer::after_secs(3).await;
    info!(
        "cyw43 wedge reproducer: wifi={} ble={}",
        cfg!(feature = "wifi"),
        cfg!(feature = "ble")
    );

    let fw = aligned_bytes!("../../../cyw43-firmware/43439A0.bin");
    let clm = aligned_bytes!("../../../cyw43-firmware/43439A0_clm.bin");
    let nvram = aligned_bytes!("../../../cyw43-firmware/nvram_rp2040.bin");

    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        CLOCK_DIVIDER,
        pio.irq0,
        cs,
        p.PIN_24,
        p.PIN_29,
        dma::Channel::new(p.DMA_CH0, Irqs),
        dma::Channel::new(p.DMA_CH1, Irqs),
    );

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());

    #[cfg(feature = "ble")]
    let (net_device, bt_device, mut control, runner) = {
        let btfw = aligned_bytes!("../../../cyw43-firmware/43439A0_btfw.bin");
        cyw43::new_with_bluetooth(state, pwr, spi, fw, btfw, nvram).await
    };
    #[cfg(not(feature = "ble"))]
    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw, nvram).await;

    spawner.spawn(cyw43_task(runner).unwrap());
    control.init(clm).await;
    info!("cyw43 up");

    #[cfg(feature = "ble")]
    spawner.spawn(ble_task(bt_device).unwrap());

    #[cfg(feature = "wifi")]
    wifi_up(spawner, net_device, &mut control).await;
    #[cfg(not(feature = "wifi"))]
    let _ = net_device;

    spawner.spawn(monitor_task().unwrap());

    let mut on = false;
    loop {
        on = !on;
        control.gpio_set(0, on).await;
        LED_TOGGLES.fetch_add(1, Ordering::Relaxed);
        LED_LAST_MS.store(Instant::now().as_millis() as u32, Ordering::Relaxed);
        Timer::after(LED_EVERY).await;
    }
}

#[cfg(feature = "wifi")]
async fn wifi_up(spawner: Spawner, net_device: cyw43::NetDriver<'static>, control: &mut cyw43::Control<'static>) {
    use cyw43::JoinOptions;
    use embassy_net::StackStorage;
    use embassy_rp::clocks::RoscRng;

    const WIFI_NETWORK: &str = env!("WIFI_NETWORK");
    const WIFI_PASSWORD: &str = env!("WIFI_PASSWORD");

    static STACK: StaticCell<StackStorage> = StaticCell::new();
    static DEVICE: StaticCell<cyw43::NetDriver<'static>> = StaticCell::new();

    let seed = RoscRng.next_u64();
    let (stack, runner) = embassy_net::Stack::new(STACK.init(StackStorage::new()), seed);
    let iface = stack.add_iface(DEVICE.init(net_device)).unwrap();
    iface.set_dhcpv4(Some(Default::default())).unwrap();
    spawner.spawn(net_task(runner).unwrap());

    while let Err(e) = control
        .join(WIFI_NETWORK, JoinOptions::new(WIFI_PASSWORD.as_bytes()))
        .await
    {
        warn!("join failed: {:?}", e);
    }
    iface.wait_config_up().await;
    info!("wifi up: {:?}", iface.ip_addrs());
}

#[cfg(feature = "ble")]
#[embassy_executor::task]
async fn ble_task(bt_device: cyw43::bluetooth::BtDriver<'static>) {
    use bt_hci::cmd::controller_baseband::{Reset, SetEventMask};
    use bt_hci::cmd::le::{LeSetEventMask, LeSetScanEnable, LeSetScanParams};
    use bt_hci::controller::{Controller, ControllerCmdSync, ExternalController};
    use bt_hci::param::{AddrKind, EventMask, LeEventMask, LeScanKind, ScanningFilterPolicy};
    use embassy_futures::join::join;

    let controller: ExternalController<_, 4> = ExternalController::new(bt_device);

    // Commands complete through `read`, so it has to be running while they
    // are issued.
    let read = async {
        let mut buf = controller.alloc_buf().unwrap();
        loop {
            match controller.read(&mut buf).await {
                Ok(pkt) => {
                    BLE_EVENTS.fetch_add(1, Ordering::Relaxed);
                    BLE_BYTES.fetch_add(packet_len(&pkt) as u32, Ordering::Relaxed);
                }
                Err(e) => warn!("hci read: {:?}", e),
            }
        }
    };

    let setup = async {
        controller.exec(&Reset::new()).await.unwrap();
        controller
            .exec(&SetEventMask::new(EventMask::new().enable_le_meta(true)))
            .await
            .unwrap();
        controller
            .exec(&LeSetEventMask::new(LeEventMask::new().enable_le_adv_report(true)))
            .await
            .unwrap();
        controller
            .exec(&LeSetScanParams::new(
                LeScanKind::Passive,
                bt_hci::param::Duration::from_millis(100),
                bt_hci::param::Duration::from_millis(100),
                AddrKind::PUBLIC,
                ScanningFilterPolicy::BasicUnfiltered,
            ))
            .await
            .unwrap();
        controller.exec(&LeSetScanEnable::new(true, false)).await.unwrap();
        info!("ble scanning");
    };

    join(read, setup).await;
}

#[cfg(feature = "ble")]
fn packet_len(pkt: &bt_hci::ControllerToHostPacket<'_>) -> usize {
    use bt_hci::ControllerToHostPacket::*;
    match pkt {
        Event(e) => e.data.len(),
        Acl(a) => a.data().len(),
        Sync(s) => s.data().len(),
        Iso(i) => i.data().len(),
    }
}

#[embassy_executor::task]
async fn monitor_task() {
    let start = Instant::now();
    let mut last_toggles = 0;
    #[cfg(feature = "ble")]
    let (mut last_events, mut last_bytes) = (0, 0);

    loop {
        Timer::after(REPORT_EVERY).await;
        let now = Instant::now();

        let toggles = LED_TOGGLES.load(Ordering::Relaxed);
        let since_led =
            Duration::from_millis((now.as_millis() as u32).wrapping_sub(LED_LAST_MS.load(Ordering::Relaxed)) as u64);

        #[cfg(feature = "ble")]
        let ble = {
            let (events, bytes) = (BLE_EVENTS.load(Ordering::Relaxed), BLE_BYTES.load(Ordering::Relaxed));
            let d = (events - last_events, bytes - last_bytes);
            (last_events, last_bytes) = (events, bytes);
            d
        };
        #[cfg(not(feature = "ble"))]
        let ble = (0, 0);

        info!(
            "[{}s] led ioctls +{} | ble events +{} ({} B)",
            (now - start).as_secs(),
            toggles - last_toggles,
            ble.0,
            ble.1,
        );
        if since_led > LED_STALL {
            warn!("bus stalled: no LED ioctl completed for {}s", since_led.as_secs());
        }
        #[cfg(feature = "ble")]
        if ble.0 == 0 {
            warn!("ble stalled: no HCI events in the last {}s", REPORT_EVERY.as_secs());
        }
        last_toggles = toggles;
    }
}
