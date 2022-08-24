//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use bsp::entry;
use core::fmt::Write;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::prelude::*;
use embedded_time::duration::Microseconds;
use embedded_time::fixed_point::FixedPoint;
use embedded_time::rate::Extensions;
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    adc,
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
    I2C,
};

use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

const TEMP_SENSOR_RESOLUTION: u16 = 4096;

#[entry]
fn main() -> ! {
    info!("Program start");
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

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.led.into_push_pull_output();
    led_pin.set_low().unwrap();

    let i2c = I2C::i2c1(
        pac.I2C1,
        pins.gpio10.into_mode(), // sda
        pins.gpio11.into_mode(), // scl
        400.kHz(),
        &mut pac.RESETS,
        125_000_000.Hz(),
    );

    let interface = I2CDisplayInterface::new(i2c);
    let mut display =
        Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0).into_terminal_mode();
    display.init().unwrap();

    let mut adc = adc::Adc::new(pac.ADC, &mut pac.RESETS);
    let mut temp_sensor = adc.enable_temp_sensor();

    display.clear().unwrap();
    watchdog.start(Microseconds::new(1_000_000));

    loop {
        let reading: u16 = adc.read(&mut temp_sensor).unwrap();
        let voltage = reading as f32 * 3.3 / TEMP_SENSOR_RESOLUTION as f32;
        let temp = 27_f32 - (voltage - 0.706) / 0.001721;
        display.clear().unwrap();
        display
            .write_fmt(format_args!(
                "temp: {:.1}*C\nr:    {}\nv:    {:.4}V",
                temp, reading, voltage
            ))
            .unwrap();
        // led_pin.set_high().unwrap();
        // delay.delay_ms(50);
        // led_pin.set_low().unwrap();
        delay.delay_ms(500);
        watchdog.feed();
    }
}

// End of file
