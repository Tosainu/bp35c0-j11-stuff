#![no_std]
#![no_main]

use adafruit_feather_rp2040 as bsp;

#[cfg(not(feature = "defmt"))]
#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

#[rtic::app(
    device = bsp::hal::pac,
    peripherals = true,
    dispatchers = [I2C0_IRQ]
)]
mod app {
    use crate::bsp;

    use bsp::{hal, XOSC_CRYSTAL_FREQ};
    use hal::{
        fugit::{ExtU64, RateExtU32},
        gpio::{FunctionI2c, FunctionSio, FunctionSpi, FunctionUart},
        gpio::{PinState, PullDown, PullUp, SioOutput},
        i2c::I2C,
        spi::Spi,
        uart::{DataBits, StopBits, UartConfig, UartPeripheral},
        Clock,
    };

    use embedded_hal::digital::{OutputPin, StatefulOutputPin};
    use embedded_hal::spi::MODE_3;
    use embedded_hal_0_2_x::blocking::i2c::Write as I2cWrite;
    use embedded_hal_0_2_x::blocking::spi::{Transfer as SpiTransfer, Write as SpiWrite};
    use embedded_io::Write;

    use rtic_monotonics::rp2040::prelude::*;
    rp2040_timer_monotonic!(Mono);

    type Uart0TxPin = hal::gpio::Pin<hal::gpio::bank0::Gpio0, FunctionUart, PullDown>;
    type Uart0RxPin = hal::gpio::Pin<hal::gpio::bank0::Gpio1, FunctionUart, PullDown>;
    type Uart0Pins = (Uart0TxPin, Uart0RxPin);

    type Uart1TxPin = hal::gpio::Pin<hal::gpio::bank0::Gpio24, FunctionUart, PullDown>;
    type Uart1RxPin = hal::gpio::Pin<hal::gpio::bank0::Gpio25, FunctionUart, PullDown>;
    type Uart1Pins = (Uart1TxPin, Uart1RxPin);

    type I2c1SdaPin = hal::gpio::Pin<hal::gpio::bank0::Gpio2, FunctionI2c, PullUp>;
    type I2c1SclPin = hal::gpio::Pin<hal::gpio::bank0::Gpio3, FunctionI2c, PullUp>;
    type I2c1Pins = (I2c1SdaPin, I2c1SclPin);

    type Spi0TxPin = hal::gpio::Pin<hal::gpio::bank0::Gpio19, FunctionSpi, PullDown>;
    type Spi0RxPin = hal::gpio::Pin<hal::gpio::bank0::Gpio20, FunctionSpi, PullDown>;
    type Spi0SckPin = hal::gpio::Pin<hal::gpio::bank0::Gpio18, FunctionSpi, PullDown>;
    type Spi0Pins = (Spi0TxPin, Spi0RxPin, Spi0SckPin);
    type Spi0CsnAdt7310Pin =
        hal::gpio::Pin<hal::gpio::bank0::Gpio6, FunctionSio<SioOutput>, PullDown>;

    type Bp35c0J11ResetnPin =
        hal::gpio::Pin<hal::gpio::bank0::Gpio11, FunctionSio<SioOutput>, PullDown>;

    type Txs0108eOePin = hal::gpio::Pin<hal::gpio::bank0::Gpio10, FunctionSio<SioOutput>, PullDown>;

    type LcdResetnPin = hal::gpio::Pin<hal::gpio::bank0::Gpio7, FunctionSio<SioOutput>, PullDown>;

    type LedPin = hal::gpio::Pin<hal::gpio::bank0::Gpio13, FunctionSio<SioOutput>, PullDown>;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        uart0: UartPeripheral<hal::uart::Enabled, hal::pac::UART0, Uart0Pins>,
        uart1: UartPeripheral<hal::uart::Enabled, hal::pac::UART1, Uart1Pins>,
        i2c1: I2C<hal::pac::I2C1, I2c1Pins>,
        spi0: Spi<hal::spi::Enabled, hal::pac::SPI0, Spi0Pins>,
        spi0_csn_adt7310: Spi0CsnAdt7310Pin,
        bp35c0_j11_resetn: Bp35c0J11ResetnPin,
        txs0108e_oe: Txs0108eOePin,
        lcd_resetn: LcdResetnPin,
        led: LedPin,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let mut resets = ctx.device.RESETS;
        let mut watchdog = hal::watchdog::Watchdog::new(ctx.device.WATCHDOG);

        let clocks = hal::clocks::init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            ctx.device.XOSC,
            ctx.device.CLOCKS,
            ctx.device.PLL_SYS,
            ctx.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .unwrap();

        let sio = hal::Sio::new(ctx.device.SIO);
        let pins = bsp::Pins::new(
            ctx.device.IO_BANK0,
            ctx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        let pins_uart = (pins.tx.into_function(), pins.rx.into_function());
        let uart0 = UartPeripheral::new(ctx.device.UART0, pins_uart, &mut resets)
            .enable(
                UartConfig::new(115_200.Hz(), DataBits::Eight, None, StopBits::One),
                clocks.peripheral_clock.freq(),
            )
            .unwrap();

        let pins_uart = (pins.d24.into_function(), pins.d25.into_function());
        let uart1 = UartPeripheral::new(ctx.device.UART1, pins_uart, &mut resets)
            .enable(
                UartConfig::new(115_200.Hz(), DataBits::Eight, None, StopBits::One),
                clocks.peripheral_clock.freq(),
            )
            .unwrap();

        let i2c1 = I2C::i2c1(
            ctx.device.I2C1,
            pins.sda.reconfigure(),
            pins.scl.reconfigure(),
            400.kHz(),
            &mut resets,
            &clocks.system_clock,
        );

        let pins_spi = (
            pins.mosi.into_function(),
            pins.miso.into_function(),
            pins.sclk.into_function(),
        );
        let spi0 = Spi::<_, _, _, 8>::new(ctx.device.SPI0, pins_spi).init(
            &mut resets,
            clocks.peripheral_clock.freq(),
            5.MHz(),
            MODE_3,
        );

        let spi0_csn_adt7310 = pins.d4.into_push_pull_output_in_state(PinState::High);
        let bp35c0_j11_resetn = pins.d11.into_push_pull_output_in_state(PinState::Low);
        let txs0108e_oe = pins.d10.into_push_pull_output_in_state(PinState::Low);
        let lcd_resetn = pins.d5.into_push_pull_output_in_state(PinState::Low);
        let led = pins.d13.into_push_pull_output_in_state(PinState::High);

        Mono::start(ctx.device.TIMER, &resets);

        task1::spawn().unwrap();
        task2::spawn().unwrap();
        task_lcd::spawn().unwrap();

        (
            Shared {},
            Local {
                uart0,
                uart1,
                i2c1,
                spi0,
                spi0_csn_adt7310,
                bp35c0_j11_resetn,
                txs0108e_oe,
                lcd_resetn,
                led,
            },
        )
    }

    #[task(priority = 1, local = [uart0, x: u32 = 0])]
    async fn task1(ctx: task1::Context) {
        loop {
            *ctx.local.x += 1;

            write!(ctx.local.uart0, "(✗╹◡╹)ﾉ! {}\r\n", ctx.local.x).unwrap();

            Mono::delay(1000.millis()).await;
        }
    }

    #[task(priority = 1, local = [led])]
    async fn task2(ctx: task2::Context) {
        loop {
            ctx.local.led.toggle().unwrap();

            Mono::delay(100.millis()).await;
        }
    }

    #[task(
        priority = 1,
        local = [i2c1, lcd_resetn]
    )]
    async fn task_lcd(ctx: task_lcd::Context) {
        const LCD_ADDRESS: u8 = 0x3e;

        ctx.local.lcd_resetn.set_high().unwrap();
        Mono::delay(50.millis()).await;

        let i2c = ctx.local.i2c1;

        // Function Set (IS = 0)
        i2c.write(LCD_ADDRESS, &[0b00_000000, 0b0011_1000]).unwrap();
        Mono::delay(30.micros()).await;

        // Function Set (IS = 1)
        i2c.write(LCD_ADDRESS, &[0b00_000000, 0b0011_1001]).unwrap();
        Mono::delay(30.micros()).await;

        // Internal OSC frequency
        i2c.write(LCD_ADDRESS, &[0b00_000000, 0b0001_0100]).unwrap();
        Mono::delay(30.micros()).await;

        // Contrast set
        i2c.write(LCD_ADDRESS, &[0b00_000000, 0b0111_1100]).unwrap();
        Mono::delay(30.micros()).await;

        // Power/ICON control/Contrast set
        i2c.write(LCD_ADDRESS, &[0b00_000000, 0b0101_0101]).unwrap();
        Mono::delay(30.micros()).await;

        // Follower control
        i2c.write(LCD_ADDRESS, &[0b00_000000, 0b0110_1100]).unwrap();
        Mono::delay(200_000.micros()).await;

        // Function Set (IS = 0)
        i2c.write(LCD_ADDRESS, &[0b00_000000, 0b0011_1000]).unwrap();
        Mono::delay(30.micros()).await;

        // Display ON/OFF
        i2c.write(LCD_ADDRESS, &[0b00_000000, 0b0000_1100]).unwrap();
        Mono::delay(30.micros()).await;

        loop {
            // Clear Display
            i2c.write(LCD_ADDRESS, &[0b00_000000, 0b0000_0001]).unwrap();
            Mono::delay(2_000.micros()).await;

            i2c.write(LCD_ADDRESS, &[0b01_000000, 0b0100_1000]).unwrap(); // 'H'
            Mono::delay(125.millis()).await;
            i2c.write(LCD_ADDRESS, &[0b01_000000, 0b0110_0101]).unwrap(); // 'e'
            Mono::delay(125.millis()).await;
            i2c.write(LCD_ADDRESS, &[0b01_000000, 0b0110_1100]).unwrap(); // 'l'
            Mono::delay(125.millis()).await;
            i2c.write(LCD_ADDRESS, &[0b01_000000, 0b0110_1100]).unwrap(); // 'l'
            Mono::delay(125.millis()).await;
            i2c.write(LCD_ADDRESS, &[0b01_000000, 0b0110_1111]).unwrap(); // 'o'
            Mono::delay(125.millis()).await;

            // Set DDRAM address (
            i2c.write(LCD_ADDRESS, &[0b00_000000, 0b1100_0011]).unwrap();
            Mono::delay(30.micros()).await;

            i2c.write(LCD_ADDRESS, &[0b01_000000, 0b0101_0111]).unwrap(); // 'W'
            Mono::delay(125.millis()).await;
            i2c.write(LCD_ADDRESS, &[0b01_000000, 0b0110_1111]).unwrap(); // 'o'
            Mono::delay(125.millis()).await;
            i2c.write(LCD_ADDRESS, &[0b01_000000, 0b0111_0010]).unwrap(); // 'r'
            Mono::delay(125.millis()).await;
            i2c.write(LCD_ADDRESS, &[0b01_000000, 0b0110_1100]).unwrap(); // 'l'
            Mono::delay(125.millis()).await;
            i2c.write(LCD_ADDRESS, &[0b01_000000, 0b0110_0100]).unwrap(); // 'd'
            Mono::delay(750.millis()).await;
        }
    }
}
