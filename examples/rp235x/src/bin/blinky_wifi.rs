//! This example tests the RP Pico 2 W onboard LED.
//!
//! It does not work with the RP Pico 2 board. See `blinky.rs`.

#![no_std]
#![no_main]

use cyw43::{JoinOptions, aligned_bytes};
use cyw43_pio::{PioSpi, RM2_CLOCK_DIVIDER};
use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_net::StackStorage;
use embassy_rp::clocks::RoscRng;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, DMA_CH1, PIO0, USB};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_rp::{bind_interrupts, dma, usb};
use embassy_time::{Duration, Timer};
use panic_probe as _;
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
                log::info!("{} ble events", events);
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
        log::info!("ble scanning");
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
        log::info!("join failed: {:?}", err);
    }
    iface.wait_config_up().await;
    log::info!("wifi up: {:?}", iface.ip_addrs());

    let delay = Duration::from_millis(250);
    loop {
        log::info!("led on!");
        control.gpio_set(0, true).await;
        Timer::after(delay).await;

        log::info!("led off!");
        control.gpio_set(0, false).await;
        Timer::after(delay).await;
    }
}
