#![no_std]
#![no_main]

// use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use fugit::RateExtU32;
use panic_probe as _;
use rp_pico::entry;

use rp_pico::hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio, pac,
    sio::Sio,
    spi,
    uart::{DataBits, StopBits, UartConfig},
    watchdog::Watchdog,
};

use display_interface_spi::SPIInterface;
use embedded_graphics::{pixelcolor::Rgb565, prelude::*};

use bme280_multibus::Bme280;
use ili9341::{DisplaySize240x320, Ili9341, Orientation};
use mh_z19::read_gas_concentration;
const SETTINGS: bme280_multibus::Settings = bme280_multibus::Settings {
    config: bme280_multibus::Config::reset()
        .set_standby_time(bme280_multibus::Standby::Millis125)
        .set_filter(bme280_multibus::Filter::X8),
    ctrl_meas: bme280_multibus::CtrlMeas::reset()
        .set_osrs_t(bme280_multibus::Oversampling::X8)
        .set_osrs_p(bme280_multibus::Oversampling::X8)
        .set_mode(bme280_multibus::Mode::Normal),
    ctrl_hum: bme280_multibus::Oversampling::X8,
};

mod dataset;
use dataset::Dataset;
mod display;
use display::Display;

const SECS_PER_UPDATE: i32 = 86400 / 180;
// const SECS_PER_UPDATE: i32 = 5; // for DEBUG

const TEMPERATURE_MIN: f32 = -40.0;
const TEMPERATURE_MAX: f32 = 85.0;
const HUMIDITY_MIN: f32 = 0.0;
const HUMIDITY_MAX: f32 = 100.0;
const PRESSURE_MIN: f32 = 900.0;
const PRESSURE_MAX: f32 = 1100.0;
const CO2_MIN: f32 = 400.0;
const CO2_MAX: f32 = 5000.0;

#[entry]
fn main() -> ! {
    // Initialize H/W
    // info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.led.into_push_pull_output();

    // SPI0 for BME280
    let _bme_clk = pins.gpio18.into_mode::<gpio::FunctionSpi>();
    let _bme_tx = pins.gpio19.into_mode::<gpio::FunctionSpi>();
    let _bme_rx = pins.gpio16.into_mode::<gpio::FunctionSpi>();
    let bme_cs = pins.gpio17.into_push_pull_output();
    let spi0 = spi::Spi::<_, _, 8>::new(pac.SPI0);
    let bme_spi = spi0.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        2.MHz(),
        &embedded_hal::spi::MODE_0,
    );
    let mut bme: Bme280<_> = Bme280::from_spi(bme_spi, bme_cs).unwrap();
    bme.reset().unwrap();
    delay.delay_ms(2);
    bme.settings(&SETTINGS).unwrap();
    delay.delay_ms(250);

    // SPI1 for LCD(ili9341)
    let _lcd_clk = pins.gpio10.into_mode::<gpio::FunctionSpi>();
    let _lcd_tx = pins.gpio11.into_mode::<gpio::FunctionSpi>();
    let _lcd_rx = pins.gpio12.into_mode::<gpio::FunctionSpi>();
    let lcd_cs = pins.gpio9.into_push_pull_output();
    let lcd_dc = pins.gpio8.into_push_pull_output();
    let lcd_reset = pins.gpio15.into_push_pull_output();
    let mut lcd_light = pins.gpio13.into_push_pull_output();
    let spi1 = spi::Spi::<_, _, 8>::new(pac.SPI1);
    let lcd_spi = spi1.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        20.MHz(),
        &embedded_hal::spi::MODE_0,
    );
    let lcd_iface = SPIInterface::new(lcd_spi, lcd_dc, lcd_cs);
    let mut lcd = Ili9341::new(
        lcd_iface,
        lcd_reset,
        &mut delay,
        Orientation::Landscape,
        DisplaySize240x320,
    )
    .unwrap();
    lcd_light.set_high().unwrap();
    lcd.clear(Rgb565::BLACK).unwrap();

    // UART0 for MH-Z19
    let uart_tx = pins.gpio0.into_mode::<gpio::FunctionUart>();
    let uart_rx = pins.gpio1.into_mode::<gpio::FunctionUart>();
    let uart_pins = (uart_tx, uart_rx);
    let uart0 = rp_pico::hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(9600.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    let mut temperature = Dataset::new(TEMPERATURE_MAX, TEMPERATURE_MIN);
    let mut humidity = Dataset::new(HUMIDITY_MAX, HUMIDITY_MIN);
    let mut pressure = Dataset::new(PRESSURE_MAX, PRESSURE_MIN);
    let mut co2 = Dataset::new(CO2_MAX, CO2_MIN);
    let mut display = Display::new();

    // Main Loop
    let mut counter = -1;
    loop {
        // wait 1sec
        // led_pin.set_high().unwrap();
        delay.delay_ms(500);
        led_pin.set_low().unwrap();
        delay.delay_ms(500);
        counter = counter + 1;

        // get samples
        let samples = bme.sample().unwrap();
        let mut ppm: f32 = 0.0;
        {
            let cmd = read_gas_concentration(1);
            uart0.write_full_blocking(&cmd);
            let mut buf = [0 as u8; 9];
            if uart0.read_full_blocking(&mut buf).is_ok() {
                // crate 'mh-z19' is wrong ? parse by self.
                let mut chksum = 0 as u8;
                for p in 1..8 {
                    chksum = (chksum + buf[p]) & 0xff;
                }
                chksum = 0xff - chksum + 1;
                if buf[8] == chksum {
                    let v = (buf[2] as u32) * 256 + (buf[3] as u32);
                    ppm = v as f32;
                }
            }
        }

        // append samples
        let p = samples.pressure / 100.0; // why pressure gets x100 ?
        temperature.append(samples.temperature);
        humidity.append(samples.humidity);
        pressure.append(p);
        co2.append(ppm);

        // update Data
        display.draw_data_temperature(&mut lcd, 0, &samples.temperature, &temperature.range());
        display.draw_data_humidity(&mut lcd, 60, &samples.humidity, &humidity.range());
        display.draw_data_pressure(&mut lcd, 120, &p, &pressure.range());
        display.draw_data_co2(&mut lcd, 180, &ppm, &co2.range());

        // update Graphs
        if counter % SECS_PER_UPDATE == 0 {
            temperature.next();
            display.temperature = temperature.export(40.0, -10.0, 60);
            display.draw_graph_temperature(&mut lcd, 0, Rgb565::RED);
        }
        if counter % SECS_PER_UPDATE == 1 {
            humidity.next();
            display.humidity = humidity.export(100.0, 0.0, 60);
            display.draw_graph_humidity(&mut lcd, 60, Rgb565::CYAN);
        }
        if counter % SECS_PER_UPDATE == 2 {
            pressure.next();
            display.pressure = pressure.export(1030.0, 970.0, 60);
            display.draw_graph_pressure(&mut lcd, 120, Rgb565::GREEN);
        }
        if counter % SECS_PER_UPDATE == 3 {
            co2.next();
            display.co2 = co2.export(2000.0, 400.0, 60);
            display.draw_graph_co2(&mut lcd, 180, Rgb565::MAGENTA);
        }
    }
}
