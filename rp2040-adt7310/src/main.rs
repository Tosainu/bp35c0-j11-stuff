#![no_std]
#![no_main]

use core::panic::PanicInfo;

use adafruit_feather_rp2040 as bsp;

use bsp::{entry, hal, XOSC_CRYSTAL_FREQ};
use hal::{
    fugit::RateExtU32,
    gpio::PinState,
    spi::Spi,
    uart::{DataBits, StopBits, UartConfig, UartPeripheral},
    Clock,
};

use embedded_hal::{delay::DelayNs, digital::OutputPin, spi::MODE_3};
use embedded_hal_0_2_x::blocking::spi::{Transfer as SpiTransfer, Write as SpiWrite};
use embedded_io::Write;

#[panic_handler]
fn panic(_: &PanicInfo) -> ! {
    loop {}
}

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

    let pins_uart = (pins.tx.into_function(), pins.rx.into_function());
    let mut uart = UartPeripheral::new(pac.UART0, pins_uart, &mut pac.RESETS)
        .enable(
            UartConfig::new(115_200.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    let mut adt7310_csn = pins.d4.into_push_pull_output_in_state(PinState::High);

    let pins_spi = (
        pins.mosi.into_function(),
        pins.miso.into_function(),
        pins.sclk.into_function(),
    );
    let mut spi = Spi::<_, _, _, 8>::new(pac.SPI0, pins_spi).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        5.MHz(),
        MODE_3,
    );

    // serial interface reset
    {
        adt7310_csn.set_low().unwrap();
        spi.write(&[0xff_u8; 4]).unwrap();
        adt7310_csn.set_high().unwrap();
        timer.delay_us(1);
    }

    // レジスタ読み出し
    //                    ┌───────── 1: read, 0: write
    //                    │   ┌───── address
    //                    │   │ ┌─── 1: continuos read (temperature value register only)
    //    spi.write(&[0b0_0_001_0_00, ...]).unwrap();
    //
    // let mut buf = [0b0_1_010_0_00, 0xff, ...];
    // spi.transfer(buf.as_mut_slice()).unwrap();

    loop {
        adt7310_csn.set_low().unwrap();

        #[allow(clippy::unusual_byte_groupings)]
        //                  ┌───── configuration register
        //                  │         ┌───── Resolution (16-bit)
        //                  │         │  ┌───── Operation mode (One shot)
        spi.write(&[0b0_0_001_0_00, 0b1_01_0_0_0_00]).unwrap();

        timer.delay_ms(240);

        #[allow(clippy::unusual_byte_groupings)]
        //                     ┌───── temperature value register
        let mut buf = [0b0_1_010_0_00, 0, 0];
        spi.transfer(buf.as_mut_slice()).unwrap();

        let temp = i16::from_be_bytes([buf[1], buf[2]]);
        let temp_int = temp / 128;
        let temp_frac = 78125_u32 * (temp % 128).unsigned_abs() as u32;
        write!(
            uart,
            "\r\ntemp = {:3}.{:07} ({:#06x})",
            temp_int, temp_frac, temp
        )
        .unwrap();

        adt7310_csn.set_high().unwrap();

        timer.delay_ms(760);
    }
}
