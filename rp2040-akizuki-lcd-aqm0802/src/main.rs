#![no_std]
#![no_main]

use adafruit_feather_rp2040 as bsp;

use bsp::{entry, hal, XOSC_CRYSTAL_FREQ};
use hal::{fugit::RateExtU32, i2c::I2C};

use embedded_hal::{delay::DelayNs, digital::OutputPin, i2c::I2c};

#[cfg(not(feature = "defmt"))]
#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

#[entry]
fn main() -> ! {
    let mut pac = hal::pac::Peripherals::take().unwrap();
    let _core = hal::pac::CorePeripherals::take().unwrap();

    let mut watchdog = hal::watchdog::Watchdog::new(pac.WATCHDOG);

    let clocks = hal::clocks::init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .unwrap();

    let mut timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let sio = hal::Sio::new(pac.SIO);
    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led = pins.d13.into_push_pull_output();
    let mut lcd_resetn = pins.d5.into_push_pull_output();

    let mut i2c = I2C::i2c1(
        pac.I2C1,
        pins.sda.reconfigure(),
        pins.scl.reconfigure(),
        400.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    const LCD_ADDRESS: u8 = 0x3e;

    // めも: control byte, data byte の順で交互に書き込む
    // i2c.write(LCD_ADDRESS, &[
    //       ┌───── C0: 最終データのとき0、それ以外のとき1
    //       │┌──── Rs: Data register の読み書きのとき1
    //     0b11_000000, 0b0011_0000
    //     0b11_000000, 0b0011_0001
    //     0b01_000000, 0b0011_0010
    // ]).unwrap();

    // power on and external reset
    lcd_resetn.set_high().unwrap();
    timer.delay_us(50_000);

    // Function Set:
    //  0 0 0 0 1 DL N DH *0 IS
    //  where:   DL: interface data is 8/4 bits
    //            N: number of line is 2/1
    //           DH: double height font
    //           IS: instruction table select

    i2c.write(LCD_ADDRESS, &[0b00_000000, 0b0011_1000]).unwrap();
    timer.delay_us(30);

    i2c.write(LCD_ADDRESS, &[0b00_000000, 0b0011_1001]).unwrap();
    timer.delay_us(30);

    // Internal OSC frequency:
    //  0 0 0 0 0 1 BS F2 F1 F0
    // where:   BS: 1:1/4 bias
    //          BS: 0:1/5 bias
    //        F2~0: adjust internal OSC frequency for FR frequency.

    i2c.write(LCD_ADDRESS, &[0b00_000000, 0b0001_0100]).unwrap();
    timer.delay_us(30);

    // Contrast set:
    //  0 0 0 1 1 1 C3 C2 C1 C0

    i2c.write(LCD_ADDRESS, &[0b00_000000, 0b0111_0000]).unwrap();
    timer.delay_us(30);

    // Power/ICON control/Contrast set:
    //  0 0 0 1 0 1 Ion Bon C5 C4
    // where:  Ion: ICON display on/off
    //         Bon: set booster circuit on/off
    //       C5,C4: Contrast set for internal follower mode.

    i2c.write(LCD_ADDRESS, &[0b00_000000, 0b0101_0110]).unwrap();
    timer.delay_us(30);

    // Follower control:
    //  0 0 0 1 1 0 Fon Rab2 Rab1 Rab0
    // where:  Fon: set follower circuit on/off
    //      Rab2~0: select follower amplified ratio.

    i2c.write(LCD_ADDRESS, &[0b00_000000, 0b0110_1100]).unwrap();
    timer.delay_us(200_000);

    i2c.write(LCD_ADDRESS, &[0b00_000000, 0b0011_1000]).unwrap();
    timer.delay_us(30);

    // Display ON/OFF:
    //  0 0 0 0 0 0 1 D C B
    // where:  D=1: entire display on
    //         C=1: cursor on
    //         B=1: cursor position on

    i2c.write(LCD_ADDRESS, &[0b00_000000, 0b0000_1110]).unwrap();
    timer.delay_us(30);

    // Clear Display:
    //  0 0 0 0 0 0 0 0 0 1

    i2c.write(LCD_ADDRESS, &[0b00_000000, 0b0000_0001]).unwrap();
    timer.delay_us(2_000);

    #[rustfmt::skip]
    const PATTERNS: &[&[u8]] = &[
        &[
            0b11_000000, 0b0011_0000,
            0b11_000000, 0b0011_0001,
            0b11_000000, 0b0011_0010,
            0b11_000000, 0b0011_0011,
            0b11_000000, 0b0011_0100,
            0b11_000000, 0b0011_0101,
            0b11_000000, 0b0011_0110,
            0b01_000000, 0b0011_0111,
        ],
        &[
            0b11_000000, 0b0100_0001,
            0b11_000000, 0b0100_0010,
            0b11_000000, 0b0100_0011,
            0b11_000000, 0b0100_0100,
            0b11_000000, 0b0100_0101,
            0b11_000000, 0b0100_0110,
            0b11_000000, 0b0100_0111,
            0b01_000000, 0b0100_1000,
        ],
        &[
            0b11_000000, 0b0110_0001,
            0b11_000000, 0b0110_0010,
            0b11_000000, 0b0110_0011,
            0b11_000000, 0b0110_0100,
            0b11_000000, 0b0110_0101,
            0b11_000000, 0b0110_0110,
            0b11_000000, 0b0110_0111,
            0b01_000000, 0b0110_1000,
        ],
        &[
            0b11_000000, 0b1011_0001,
            0b11_000000, 0b1011_0010,
            0b11_000000, 0b1011_0011,
            0b11_000000, 0b1011_0100,
            0b11_000000, 0b1011_0101,
            0b11_000000, 0b1011_0110,
            0b11_000000, 0b1011_0111,
            0b01_000000, 0b1011_1000,
        ],
    ];

    loop {
        for (i, p) in PATTERNS.iter().enumerate() {
            // Set DDRAM address:
            //  0 0 1 AC6 AC5 AC4 AC3 AC2 AC1 AC0
            if i & 1 == 0 {
                led.set_high().unwrap();
                i2c.write(LCD_ADDRESS, &[0b00_000000, 0b1000_0000]).unwrap();
            } else {
                led.set_low().unwrap();
                i2c.write(LCD_ADDRESS, &[0b00_000000, 0b1100_0000]).unwrap();
            }
            timer.delay_us(30);

            i2c.write(LCD_ADDRESS, p).unwrap();

            timer.delay_us(500_000);
        }
    }
}
