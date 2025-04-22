//! Touch 'n Drink Firmware
//!
//! Pinout ESP32-C3 Super Mini board
//!
//!                               | USB |
//! (Keypad Col1) A5/MISO/GPIO5 -  5 | 5V - 5V
//! (Keypad Col2)    MOSI/GPIO6 -  6 | G  - GND
//! (Keypad Col3)      SS/GPIO7 -  7 | 33 - 3V3
//!   (Board LED)     SDA/GPIO8 -  8 | 4  - GPIO4/A4/SCK (Buzzer)
//!     (I2C SDA)     SCL/GPIO9 -  9 | 3  - GPIO3/A3     (Keypad Row4)
//!     (I2C SCL)        GPIO10 - 10 | 2  - GPIO2/A2     (Keypad Row3)
//!     (NFC IRQ)     RX/GPIO20 - 20 | 1  - GPIO1/A1     (Keypad Row2)
//!                   TX/GPIO21 - 21 | 0  - GPIO0/A0     (Keypad Row1)
//!
//! Pinout OLED 2.42" Display
//!
//!            1   2   3   4
//!           GND VDD SCL SDA
//! GND - 1 |
//! VDD - 2 |
//! SCL - 3 |
//! SDA - 4 |
//!
//! Pinout 3x4 Matrix Keypad
//!
//!  1   2    3    4    5    6    7    8   9
//!  nc Col2 Row1 Col1 Row4 Col3 Row3 Row2 nc
//!
//! Pinout PN532 NFC Module
//!
//!             SCK MISO MOSI SS VCC GND IRQ RSTO
//!              1   2    3   4   5   6   7   8
//! GND - 1 |
//! VCC - 2 |
//! SDA - 3 |
//! SCL - 4 |
//!

#![no_std]
#![no_main]

mod config;
mod error;
mod json;
mod nfc;
mod pn532;

use embassy_executor::Spawner;
use embassy_time::{with_timeout, Duration};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::config::WatchdogConfig;
use esp_hal::gpio::{Input, Pull};
use esp_hal::i2c::master::{BusTimeout, Config as I2cConfig, I2c};
use esp_hal::peripherals::Peripherals;
use esp_hal::rng::Rng;
use esp_hal::rtc_cntl::{Rtc, RwdtStage};
use esp_hal::time::RateExtU32;
use esp_hal::timer::systimer::SystemTimer;
use esp_println::println;
use fugit::MicrosDurationU64;
use log::{error, info};

extern crate alloc;

static VERSION_STR: &str = env!("CARGO_PKG_VERSION");
static GIT_SHA_STR: &str = env!("GIT_SHORT_SHA");

// Delay in seconds after which to restart on panic
#[cfg(not(debug_assertions))]
const PANIC_RESTART_DELAY: Duration = Duration::from_secs(10);
#[cfg(debug_assertions)]
const PANIC_RESTART_DELAY: Duration = Duration::from_secs(600);

/// Custom halt function for esp-backtrace. Called after panic was handled and should halt
/// or restart the system.
#[export_name = "custom_halt"]
unsafe fn halt() -> ! {
    // System may be in any state at this time, thus everything is unsafe here. Stealing the
    // peripherals handle allows us to try to notify the user about this abnormal state and
    // restart the system. Any error should be ignored.
    let peripherals = Peripherals::steal();

    // TODO: Steal display driver and show a panic message to the user

    // Restart automatically after a delay
    println!("Restarting in {} seconds...", PANIC_RESTART_DELAY.as_secs());
    let mut rtc = Rtc::new(peripherals.LPWR);
    rtc.rwdt.set_timeout(
        RwdtStage::Stage0,
        MicrosDurationU64::secs(PANIC_RESTART_DELAY.as_secs()),
    );

    rtc.rwdt.unlisten();
    rtc.rwdt.enable();
    loop {
        esp_hal::riscv::asm::wfi();
    }
}

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    let esp_config = esp_hal::Config::default()
        .with_cpu_clock(CpuClock::max())
        // TODO: Enable watchdog
        .with_watchdog(WatchdogConfig::default());
    let peripherals = esp_hal::init(esp_config);
    let _rng = Rng::new(peripherals.RNG);
    //let _led = Output::new(peripherals.GPIO8, Level::High);

    // Initialize global allocator
    esp_alloc::heap_allocator!(150 * 1024);

    // Initialize async executor
    let systimer = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(systimer.alarm0);

    // Initialize logging
    esp_println::logger::init_logger_from_env();
    info!("Touch 'n Drink v{VERSION_STR} ({GIT_SHA_STR})");

    // Read system configuration
    let _config = config::Config::read().await;

    // Initialize article and user look up tables
    //let mut articles = article::Articles::new(config.vf_article_ids);
    //let mut users = user::Users::new();

    // Initialize I2C controller
    let _i2c_config = I2cConfig::default()
        // Standard-Mode: 100 kHz, Fast-Mode: 400 kHz
        .with_frequency(400.kHz())
        // Reset bus after 24 bus clock cycles (60 µs) of inactivity
        .with_timeout(BusTimeout::BusCycles(24));

    let i2c = I2c::new(
        peripherals.I2C0,
        I2cConfig::default()
            .with_frequency(400.kHz())
            .with_timeout(BusTimeout::BusCycles(24)),
    )
    .expect("I2C initialization failed")
    .with_sda(peripherals.GPIO9)
    .with_scl(peripherals.GPIO10)
    .into_async();

    // Share I2C bus. Since the mcu is single-core and I2C is not used in interrupts, I2C access
    // cannot be preempted and we can safely use a NoopMutex for shared access.

    // Initialize NFC reader
    let nfc_irq = Input::new(peripherals.GPIO20, Pull::Up);
    // let mut nfc = nfc::Nfc::new(I2cDevice::new(&i2c), nfc_irq)
    let mut nfc = nfc::Nfc::new(i2c, nfc_irq).await.expect("NFC init failed");
    //.await
    // Panic on failure since an initialization error indicates a serious error
    //.expect("NFC reader initialization failed");

    info!("UI: Waiting for NFC card...");

    loop {
        match with_timeout(Duration::from_secs(10), nfc.read()).await {
            Ok(Ok(uid)) => info!("NFC UID: {:?}", uid),
            Ok(Err(e)) => error!("Read error: {:?}", e),
            Err(_) => info!("Timeout"),
        }
    }
}
