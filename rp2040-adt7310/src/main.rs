#![no_std]
#![no_main]

use rp2040_hal as hal;

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

#[cfg(not(feature = "defmt"))]
#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

#[link_section = ".boot2"]
#[no_mangle]
#[used]
static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_GD25Q64CS;

const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;

#[hal::entry]
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
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let pins_uart = (pins.gpio0.into_function(), pins.gpio1.into_function());
    let mut uart = UartPeripheral::new(pac.UART0, pins_uart, &mut pac.RESETS)
        .enable(
            UartConfig::new(115_200.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    let mut adt7310_csn = pins.gpio6.into_push_pull_output_in_state(PinState::High);

    let pins_spi = (
        pins.gpio19.into_function(),
        pins.gpio20.into_function(),
        pins.gpio18.into_function(),
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
        write!(uart, "\r\ntemp = {temp_int:3}.{temp_frac:07} ({temp:#06x})").unwrap();

        adt7310_csn.set_high().unwrap();

        timer.delay_ms(760);
    }
}
