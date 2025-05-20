use esp_idf_hal::gpio::AnyIOPin;
use esp_idf_hal::i2c;
use esp_idf_hal::ledc;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::spi;

use smart_leds::{SmartLedsWrite, RGB};
use tcs3472::AllChannelMeasurement;
use ws2812_spi::Ws2812;

use tcs3472::Tcs3472;

use esp_idf_hal::delay::Delay;

mod motors;
use motors::{MotorConfig, MotorState, Motors};

use std::rc::Rc;

fn measurement_to_float(m: AllChannelMeasurement) -> [f32; 3] {
    let mut max = m.red;

    if m.green > max {
        max = m.green;
    }

    if m.blue > max {
        max = m.blue;
    }

    [
        m.red as f32 / max as f32,
        m.green as f32 / max as f32,
        m.blue as f32 / max as f32,
    ]
}

enum Color {
    Blue,
    Red,
    Magenta,
    Green,
    Cyan,
    Yellow,
    Unknown,
}

impl Color {
    fn to_rgb(&self) -> RGB<u8> {
        match self {
            Color::Blue => { RGB::new(0, 0, 128) },
            Color::Red => { RGB::new(128, 0, 0) },
            Color::Magenta => { RGB::new(128, 0, 128) },
            Color::Green => { RGB::new(0, 128, 0) },
            Color::Cyan => { RGB::new(0, 128, 128) },
            Color::Yellow => { RGB::new(128, 128, 0) },
            Color::Unknown => { RGB::new(0, 0, 0) },
        }
    }
}

fn float_to_base_color(f: [f32; 3]) -> Color {
    if f[0] > 0.85 && f[1] < 0.85 && f[2] < 0.85 {
        Color::Red
    } else if f[0] < 0.85 && f[1] > 0.85 && f[2] < 0.85 {
        Color::Green
    } else if f[0] < 0.85 && f[1] < 0.85 && f[2] > 0.85 {
        Color::Blue
    } else if f[0] > 0.85 && f[1] < 0.85 && f[2] > 0.85 {
        Color::Magenta
    } else {
        Color::Unknown
    }
}

fn main() {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    let delay: Delay = Default::default();
    let peripherals = Peripherals::take().unwrap();
    let spi = peripherals.spi2;
    let mosi = peripherals.pins.gpio10;

    let driver = spi::SpiDriver::new_without_sclk::<spi::SPI2>(
        spi,
        mosi,
        Option::<AnyIOPin>::None,
        &spi::SpiDriverConfig::new(),
    )
    .unwrap();

    let config = spi::config::Config::new().baudrate(3800000.into());
    let spi = spi::SpiBusDriver::new(driver, &config).unwrap();

    let mut led = Ws2812::new(spi);

    let i2c = peripherals.i2c0;
    let scl = peripherals.pins.gpio8;
    let sda = peripherals.pins.gpio9;

    let i2c_config = i2c::I2cConfig::new().baudrate(100000.into());
    let i2c = i2c::I2cDriver::new(i2c, sda, scl, &i2c_config).unwrap();

    let mut sensor = Tcs3472::new(i2c);

    sensor.enable().unwrap();
    sensor.enable_rgbc().unwrap();
    sensor.set_integration_cycles(32).unwrap();

    // Right motor
    let mot1_1 = peripherals.pins.gpio0;
    let mot1_2 = peripherals.pins.gpio1;
    // Left motor
    let mot2_1 = peripherals.pins.gpio2;
    let mot2_2 = peripherals.pins.gpio3;

    let timer_config = ledc::config::TimerConfig::new().frequency(25000.into());
    let timer =
        Rc::new(ledc::LedcTimerDriver::new(peripherals.ledc.timer0, &timer_config).unwrap());

    let mot1_1 = ledc::LedcDriver::new(peripherals.ledc.channel0, timer.clone(), mot1_1).unwrap();
    let mot1_2 = ledc::LedcDriver::new(peripherals.ledc.channel1, timer.clone(), mot1_2).unwrap();
    let mot2_1 = ledc::LedcDriver::new(peripherals.ledc.channel2, timer.clone(), mot2_1).unwrap();
    let mot2_2 = ledc::LedcDriver::new(peripherals.ledc.channel3, timer, mot2_2).unwrap();

    let motor_config = MotorConfig::new(100, 94, false, true);
    let mut motors = Motors::new(motor_config).unwrap();

    let tx = motors.start(mot2_1, mot2_2, mot1_1, mot1_2).unwrap();

    log::info!("Starting main loop!");

    let mut state = MotorState::Stop;
    let mut new_state;
    loop {
        while !sensor.is_rgbc_status_valid().unwrap() {
            delay.delay_ms(10);
        }

        let m = sensor.read_all_channels().unwrap();
        let normalized = measurement_to_float(m);
        let color = float_to_base_color(normalized);

        new_state = match color {
            Color::Red => { MotorState::Stop },
            Color::Magenta => { MotorState::Forward },
            _ => { state },
        };

        if state != new_state {
            state = new_state;
            tx.send(state).unwrap();
        }

        log::info!("Measurement: {:?}, normalized: {:?}, rgb: {:?}", m, normalized, color.to_rgb());
        let rgb = [ color.to_rgb() ];
        led.write(rgb).unwrap();
        delay.delay_ms(10);
    }
}
