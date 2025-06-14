#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use axp192_dd::{Axp192Async, AxpError, ChargeCurrentValue, Gpio0FunctionSelect, LdoId};
use core::fmt::Write;
use defmt::{error, info};
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Duration, Timer};
use embedded_graphics::{
    mono_font::{MonoTextStyle, ascii::FONT_10X20},
    pixelcolor::Rgb565,
    prelude::*,
    text::{Alignment, Text},
};
use esp_hal::{
    Async,
    clock::CpuClock,
    dma::{DmaRxBuf, DmaTxBuf},
    gpio::{Level, Output},
    i2c::master::{Config as I2cConfig, Error as I2cError, I2c},
    spi::{
        Mode,
        master::{Config, Spi, SpiDmaBus},
    },
    time::Rate,
    timer::timg::TimerGroup,
};
use fusb302b::Fusb302bAsync;
use heapless::String;
use lcd_async::{
    Builder, TestImage,
    interface::SpiInterface,
    models::ST7789,
    options::{ColorInversion, Orientation, Rotation},
    raw_framebuf::RawFrameBuf,
};
use static_cell::StaticCell;

use {esp_backtrace as _, esp_println as _};

// Uncomment for vertical orientation
// const X_OFFSET: u16 = 52;
// const Y_OFFSET: u16 = 40;
// const W_ACTIVE: u16 = 135;
// const H_ACTIVE: u16 = 240;
const X_OFFSET: u16 = 203;
const Y_OFFSET: u16 = 40;
const WIDTH: u16 = 240;
const HEIGHT: u16 = 135;

const PXL_SIZE: usize = 2;
const FRAME_BUFFER_SIZE: usize = (WIDTH * HEIGHT) as usize * PXL_SIZE;
static FRAME_BUFFER: StaticCell<[u8; FRAME_BUFFER_SIZE]> = StaticCell::new();

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let p = esp_hal::init(config);
    // esp_alloc::heap_allocator!(#[unsafe(link_section = ".dram2_uninit")] size: 64 * 1024);

    let timer0 = TimerGroup::new(p.TIMG1);
    esp_hal_embassy::init(timer0.timer0);

    info!("Embassy initialized!");
    let int_fusb_pin = p.GPIO0;
    let config_i2c0: I2cConfig = I2cConfig::default().with_frequency(Rate::from_khz(100));
    let config_i2c1: I2cConfig = I2cConfig::default().with_frequency(Rate::from_khz(400));
    let i2c0 = I2c::new(p.I2C0, config_i2c0)
        .unwrap()
        .with_sda(p.GPIO25)
        .with_scl(p.GPIO26)
        .into_async();

    let i2c1 = I2c::new(p.I2C1, config_i2c1)
        .unwrap()
        .with_sda(p.GPIO21)
        .with_scl(p.GPIO22)
        .into_async();

    let voltage: u32 = init_m5stickc_plus_pmic(i2c1).await.unwrap() as u32;

    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = esp_hal::dma_buffers!(4, 32_000);
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

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
    .with_dma(p.DMA_SPI2)
    .with_buffers(dma_rx_buf, dma_tx_buf)
    .into_async();

    let res = Output::new(res, Level::Low, Default::default());
    let dc = Output::new(dc, Level::Low, Default::default());
    let cs = Output::new(cs, Level::High, Default::default());

    static SPI_BUS: StaticCell<Mutex<NoopRawMutex, SpiDmaBus<'static, Async>>> = StaticCell::new();
    let spi_bus = Mutex::new(spi);
    let spi_bus = SPI_BUS.init(spi_bus);
    let spi_device = SpiDevice::new(spi_bus, cs);

    let di = SpiInterface::new(spi_device, dc);
    let mut delay = embassy_time::Delay;

    let mut display = match Builder::new(ST7789, di)
        .reset_pin(res)
        .display_size(WIDTH, HEIGHT)
        .orientation(Orientation {
            rotation: Rotation::Deg90,
            mirrored: false,
        })
        .display_offset(X_OFFSET, Y_OFFSET)
        .invert_colors(ColorInversion::Inverted)
        .init(&mut delay)
        .await
    {
        Ok(display) => display,
        Err(e) => {
            info!("Error! {}", defmt::Debug2Format(&e));
            return;
        }
    };
    info!("Display initialized!");

    let frame_buffer = FRAME_BUFFER.init([0; FRAME_BUFFER_SIZE]);
    {
        let mut raw_fb = RawFrameBuf::<Rgb565, _>::new(
            frame_buffer.as_mut_slice(),
            WIDTH as usize,
            HEIGHT as usize,
        );
        raw_fb.clear(Rgb565::BLACK).unwrap();
        let mut volt_str: String<30> = String::new();
        write!(&mut volt_str, "Voltage: {} mV", voltage).unwrap();
        Text::new(
            &volt_str,
            Point::new(0, 20),
            MonoTextStyle::new(&FONT_10X20, Rgb565::GREEN),
        )
        .draw(&mut raw_fb)
        .unwrap();
        //TestImage::new().draw(&mut raw_fb).unwrap();
    }

    display
        .show_raw_data(0, 0, WIDTH, HEIGHT, frame_buffer)
        .await
        .unwrap();

    spawner.must_spawn(pd_task(i2c0));

    loop {
        //info!("Hello world!");
        Timer::after(Duration::from_secs(1)).await;
    }
}

#[embassy_executor::task]
async fn pd_task(i2c: I2c<'static, Async>) {
    let mut fusb = Fusb302bAsync::new(i2c);
    match fusb.ll.device_id().read_async().await {
        Ok(device_id) => info!("FUSB302B device ID: {}", device_id),
        Err(e) => error!("Failed to read device ID: {:?}", e),
    };
}

#[rustfmt::skip]
async fn init_m5stickc_plus_pmic(i2c: I2c<'_, Async>) -> Result<f32, AxpError<I2cError>> {
    let mut axp = Axp192Async::new(i2c);
    axp.set_ldo_voltage_mv(LdoId::Ldo2, 3300).await?;
    axp.ll.adc_enable_1().write_async(|r| {
        r.set_battery_current_adc_enable(true);
        r.set_acin_voltage_adc_enable(true);
        r.set_acin_current_adc_enable(true);
        r.set_vbus_voltage_adc_enable(true);
        r.set_vbus_current_adc_enable(true);
        r.set_aps_voltage_adc_enable(true);
    }).await?;
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

    let bat_voltage = axp.get_battery_voltage_mv().await?;


    info!("Battery voltage: {} mV", bat_voltage);
    info!("Charge current: {} mA", axp.get_battery_charge_current_ma().await?);
    Ok(bat_voltage)
}
