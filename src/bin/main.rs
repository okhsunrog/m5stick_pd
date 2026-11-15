#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use axp192_dd::{
    Axp192Async, AxpError, ChargeCurrentValue, Gpio0FunctionSelect, LdoId, PekShutdownTime,
};
use core::fmt::Write;
use defmt::{info, warn};
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Duration, Timer};
use embedded_graphics::{
    mono_font::{
        MonoTextStyle,
        ascii::{FONT_6X10, FONT_10X20},
    },
    pixelcolor::Rgb565,
    prelude::*,
    text::Text,
};
use embedded_hal::digital::OutputPin;
use esp_hal::{
    Async,
    clock::CpuClock,
    gpio::{Level, Output},
    i2c::master::{Config as I2cConfig, Error as I2cError, I2c},
    spi::{
        Mode,
        master::{Config, Spi},
    },
    time::Rate,
    timer::timg::TimerGroup,
};
use heapless::String;
use lcd_async::{
    Builder, Display, interface,
    models::Model,
    models::ST7789,
    options::{ColorInversion, Orientation, Rotation},
    raw_framebuf::RawFrameBuf,
};
use static_cell::{ConstStaticCell, StaticCell};

use {esp_backtrace as _, esp_println as _};

#[path = "../battery.rs"]
mod battery;
#[path = "../ble.rs"]
mod ble;

type Pmic = Axp192Async<axp192_dd::AxpInterface<I2c<'static, Async>>, I2cError>;

esp_bootloader_esp_idf::esp_app_desc!();

// Native ST7789 panel offsets; orientation rotates the 135x240 panel to landscape.
const X_OFFSET: u16 = 52;
const Y_OFFSET: u16 = 40;
const WIDTH: u16 = 240;
const HEIGHT: u16 = 135;

const PXL_SIZE: usize = 2;
const FRAME_BUFFER_SIZE: usize = (WIDTH * HEIGHT) as usize * PXL_SIZE;
static FRAME_BUFFER: ConstStaticCell<[u8; FRAME_BUFFER_SIZE]> =
    ConstStaticCell::new([0; FRAME_BUFFER_SIZE]);

#[esp_hal::main]
async fn main(_spawner: Spawner) {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let p = esp_hal::init(config);
    // Reuse bootloader RAM so the BLE heap does not crowd out the framebuffer and stack.
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 64 * 1024);
    esp_alloc::heap_allocator!(size: 8 * 1024);

    let timer0 = TimerGroup::new(p.TIMG1);
    esp_rtos::start(timer0.timer0);

    info!("Embassy initialized!");
    let config_i2c1: I2cConfig = I2cConfig::default().with_frequency(Rate::from_khz(400));
    let i2c1 = I2c::new(p.I2C1, config_i2c1)
        .unwrap()
        .with_sda(p.GPIO21)
        .with_scl(p.GPIO22)
        .into_async();

    let mut pmic = Axp192Async::new(i2c1);
    init_m5stickc_plus_pmic(&mut pmic).await.unwrap();

    // Creating SPI
    let sclk = p.GPIO13;
    let mosi = p.GPIO15;
    let res = p.GPIO18;
    let dc = p.GPIO23;
    let cs = p.GPIO5;
    let spi = Spi::new(
        p.SPI2,
        Config::default()
            .with_frequency(Rate::from_mhz(20))
            .with_mode(Mode::_0),
    )
    .unwrap()
    .with_sck(sclk)
    .with_mosi(mosi)
    .into_async();

    let res = Output::new(res, Level::Low, Default::default());
    let dc = Output::new(dc, Level::Low, Default::default());
    let cs = Output::new(cs, Level::High, Default::default());

    static SPI_BUS: StaticCell<Mutex<NoopRawMutex, Spi<'static, Async>>> = StaticCell::new();
    let spi_bus = Mutex::new(spi);
    let spi_bus = SPI_BUS.init(spi_bus);
    let spi_device = SpiDevice::new(spi_bus, cs);

    let di = interface::SpiInterface::new(spi_device, dc);
    let mut delay = embassy_time::Delay;

    info!("Initializing display over async SPI FIFO");
    let display = Builder::new(ST7789, di)
        .reset_pin(res)
        .display_size(HEIGHT, WIDTH)
        .orientation(Orientation {
            rotation: Rotation::Deg90,
            mirrored: false,
        })
        .display_offset(X_OFFSET, Y_OFFSET)
        .invert_colors(ColorInversion::Inverted)
        .init(&mut delay)
        .await
        .unwrap_or_else(|e| panic!("Display init failed: {:?}", defmt::Debug2Format(&e)));
    info!("Display initialized!");

    let connector = esp_radio::ble::controller::BleConnector::new(
        p.BT,
        esp_radio::ble::Config::default().with_modem_sleep(false),
    )
    .unwrap();
    info!(
        "BLE controller initialized; free heap: {} bytes",
        esp_alloc::HEAP.free()
    );
    let controller = trouble_host::prelude::ExternalController::<_, 1>::new(connector);
    embassy_futures::join::join(display_task(display, pmic), ble::run(controller)).await;
}

struct PowerReading {
    voltage_mv: f32,
    charge_ma: f32,
    discharge_ma: f32,
    usb_ma: f32,
    battery_present: bool,
    charging: bool,
    usb_present: bool,
}

async fn read_power(pmic: &mut Pmic) -> Result<PowerReading, AxpError<I2cError>> {
    let status = pmic.ll.charge_status().read_async().await?;
    let power = pmic.ll.power_status().read_async().await?;
    Ok(PowerReading {
        voltage_mv: pmic.get_battery_voltage_mv().await?,
        charge_ma: pmic.get_battery_charge_current_ma().await?,
        discharge_ma: pmic.get_battery_discharge_current_ma().await?,
        usb_ma: pmic.get_vbus_current_ma().await?,
        battery_present: status.battery_connected(),
        charging: status.charging_in_progress(),
        usb_present: power.vbus_present(),
    })
}

async fn display_task<DI, M, RST>(mut display: Display<DI, M, RST>, mut pmic: Pmic)
where
    DI: interface::Interface<Word = u8>,
    M: Model,
    RST: OutputPin,
{
    let frame_buffer = FRAME_BUFFER.take();
    let mut ticker = embassy_time::Ticker::every(Duration::from_secs(1));
    loop {
        let reading = read_power(&mut pmic).await;
        {
            let mut raw_fb = RawFrameBuf::<Rgb565, _>::new(
                frame_buffer.as_mut_slice(),
                WIDTH.into(),
                HEIGHT.into(),
            );
            raw_fb.clear(Rgb565::BLACK).unwrap();
            let header = MonoTextStyle::new(&FONT_6X10, Rgb565::CYAN);
            Text::new("OxiFOC Remote Stick", Point::new(5, 12), header)
                .draw(&mut raw_fb)
                .unwrap();
            let style = MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE);
            match reading {
                Ok(power) => {
                    let soc = battery::estimate_soc_percent(power.voltage_mv, power.charge_ma);
                    let mut line: String<32> = String::new();
                    if power.battery_present {
                        write!(
                            line,
                            "SoC ~{}%  {}",
                            soc,
                            if power.charging {
                                "CHG"
                            } else if power.usb_present {
                                "USB"
                            } else {
                                "BAT"
                            }
                        )
                        .unwrap();
                    } else {
                        line.push_str("No battery").unwrap();
                    }
                    Text::new(&line, Point::new(5, 36), style)
                        .draw(&mut raw_fb)
                        .unwrap();
                    for (label, value, unit, y) in [
                        ("Vbat", power.voltage_mv, "mV", 58),
                        ("Charge", power.charge_ma, "mA", 80),
                        ("Dischg", power.discharge_ma, "mA", 102),
                        ("USB in", power.usb_ma, "mA", 124),
                    ] {
                        line.clear();
                        write!(line, "{}: {:.1} {}", label, value, unit).unwrap();
                        Text::new(&line, Point::new(5, y), style)
                            .draw(&mut raw_fb)
                            .unwrap();
                    }
                    info!(
                        "Power: SoC~{}% Vbat={}mV charge={}mA discharge={}mA USB={}mA battery={} charging={} usb={}",
                        soc,
                        power.voltage_mv,
                        power.charge_ma,
                        power.discharge_ma,
                        power.usb_ma,
                        power.battery_present,
                        power.charging,
                        power.usb_present
                    );
                }
                Err(e) => {
                    warn!("Power read failed: {:?}", defmt::Debug2Format(&e));
                    Text::new("PMIC read error", Point::new(5, 58), style)
                        .draw(&mut raw_fb)
                        .unwrap();
                }
            }
        }
        display
            .show_raw_data(0, 0, WIDTH, HEIGHT, frame_buffer)
            .await
            .unwrap();
        info!("Power display updated");
        ticker.next().await;
    }
}

#[rustfmt::skip]
async fn init_m5stickc_plus_pmic(axp: &mut Pmic) -> Result<(), AxpError<I2cError>> {
    axp.ll.pek_key_parameters().modify_async(|r| {
        r.set_shutdown_time(PekShutdownTime::S4);
        r.set_auto_shutdown_if_pek_held_longer_than_shutdown_time(true);
    }).await?;
    let pek = axp.ll.pek_key_parameters().read_async().await?;
    assert!(matches!(pek.shutdown_time(), PekShutdownTime::S4));
    assert!(pek.auto_shutdown_if_pek_held_longer_than_shutdown_time());
    info!("Power key: hardware shutdown after 4 s (register readback verified)");

    axp.set_ldo_voltage_mv(LdoId::Ldo2, 3300).await?;
    axp.ll.charge_control_1().write_async(|r| r.set_charge_current(ChargeCurrentValue::Ma100)).await?;
    axp.set_gpio0_ldo_voltage_mv(3300).await?;
    axp.ll.gpio_0_control().write_async(|r| {
        r.set_function_select(Gpio0FunctionSelect::LowNoiseLdoOutput);
    }).await?;

    axp.ll.power_output_control().modify_async(|r| {
        r.set_dcdc_1_output_enable(true);
        r.set_dcdc_3_output_enable(false);
        r.set_ldo_2_output_enable(true);
        r.set_ldo_3_output_enable(true);
        r.set_dcdc_2_output_enable(false);
        r.set_exten_output_enable(false); // external 5V output
    }).await?;
    axp.set_battery_charge_high_temp_threshold_mv(3226).await?;
    axp.ll.backup_battery_charge_control().write_async(|r| {
        r.set_backup_charge_enable(true);
    }).await?;
        Timer::after(Duration::from_secs(1)).await;

    axp.ll.adc_enable_1().write_async(|r| {
        r.set_battery_voltage_adc_enable(true);
        r.set_battery_current_adc_enable(true);
        r.set_acin_voltage_adc_enable(true);
        r.set_acin_current_adc_enable(true);
        r.set_vbus_voltage_adc_enable(true);
        r.set_vbus_current_adc_enable(true);
        r.set_aps_voltage_adc_enable(true);
    }).await?;
    Timer::after(Duration::from_secs(1)).await;

    let bat_voltage = axp.get_battery_voltage_mv().await?;


    info!("Battery voltage: {} mV", bat_voltage);
    info!("Charge current: {} mA", axp.get_battery_charge_current_ma().await?);
    Ok(())
}
