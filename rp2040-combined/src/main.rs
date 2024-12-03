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

use embedded_alloc::Heap;

#[global_allocator]
static ALLOCATOR: Heap = Heap::empty();

#[rtic::app(
    device = bsp::hal::pac,
    peripherals = true,
    dispatchers = [I2C0_IRQ]
)]
mod app {
    use crate::bsp;
    use crate::ALLOCATOR;

    use core::fmt::Write;
    use core::mem::MaybeUninit;
    use core::ptr::addr_of_mut;

    use bsp::{hal, XOSC_CRYSTAL_FREQ};
    use hal::{
        fugit::{ExtU64, RateExtU32},
        gpio::{FunctionI2c, FunctionPio0, FunctionSio, FunctionSpi, FunctionUart},
        gpio::{PinState, PullDown, PullUp, SioOutput},
        i2c::I2C,
        pio::PIOExt,
        spi::Spi,
        uart::{DataBits, StopBits, UartConfig, UartPeripheral},
        Clock,
    };

    use embedded_hal::digital::OutputPin;
    use embedded_hal::spi::MODE_3 as SPI_MODE_3;
    use embedded_hal_0_2_x::blocking::spi::{Transfer as SpiTransfer, Write as SpiWrite};

    use futures::FutureExt;

    use rtic_monotonics::{rp2040::prelude::*, TimerQueueBasedMonotonic};
    rp2040_timer_monotonic!(Mono);

    use route_b_secrets::{ROUTE_B_ID, ROUTE_B_PASSWORD};

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

    const UART1_RX_QUEUE_DEPTH: usize = 4;
    type Uart1RxQueue = rtic_sync::channel::Channel<bp35c0_j11::Response, UART1_RX_QUEUE_DEPTH>;
    type Uart1RxSender<'a> =
        rtic_sync::channel::Sender<'a, bp35c0_j11::Response, UART1_RX_QUEUE_DEPTH>;
    type Uart1RxReceiver<'a> =
        rtic_sync::channel::Receiver<'a, bp35c0_j11::Response, UART1_RX_QUEUE_DEPTH>;

    struct Bp35c0J11Writer<W: embedded_io::Write>(W);

    impl<W: embedded_io::Write> Bp35c0J11Writer<W> {
        fn send(&mut self, cmd: bp35c0_j11::Command) -> Result<(), W::Error> {
            #[cfg(feature = "defmt")]
            defmt::info!("Tx: {:?}", cmd);

            let mut buf = [0; 128];
            let len = bp35c0_j11::serialize_to_bytes(&cmd, &mut buf).unwrap();
            self.0.write_all(&buf[..len])
        }
    }

    struct NeoPixel<S: hal::pio::ValidStateMachine>(hal::pio::Tx<S>);

    impl<S: hal::pio::ValidStateMachine> NeoPixel<S> {
        fn set_rgb(&mut self, r: u8, g: u8, b: u8) -> bool {
            self.0.write(u32::from_be_bytes([g, r, b, 0]))
        }

        fn set_raw(&mut self, grbx: u32) -> bool {
            self.0.write(grbx)
        }
    }

    #[derive(Clone, Copy)]
    pub enum Bp35c0J11Status {
        Initializing,
        Scanning,
        Ready,
        Data {
            datetime: (u16, u8, u8, u8, u8),
            instant: u32,
            rssi: i8,
        },
    }

    #[shared]
    struct Shared {
        bp35c0_j11_status: Bp35c0J11Status,
        temperature_raw: i16,
    }

    #[local]
    struct Local {
        uart0: UartPeripheral<hal::uart::Enabled, hal::pac::UART0, Uart0Pins>,
        uart1_rx: hal::uart::Reader<hal::pac::UART1, Uart1Pins>,
        uart1_rx_receiver: Uart1RxReceiver<'static>,
        uart1_rx_sender: Uart1RxSender<'static>,
        uart1_tx: Bp35c0J11Writer<hal::uart::Writer<hal::pac::UART1, Uart1Pins>>,
        i2c1: I2C<hal::pac::I2C1, I2c1Pins>,
        spi0: Spi<hal::spi::Enabled, hal::pac::SPI0, Spi0Pins>,
        spi0_csn_adt7310: Spi0CsnAdt7310Pin,
        bp35c0_j11_resetn: Bp35c0J11ResetnPin,
        txs0108e_oe: Txs0108eOePin,
        lcd_resetn: LcdResetnPin,
        led: LedPin,
        neopixel: NeoPixel<(hal::pac::PIO0, hal::pio::SM0)>,
        bp35c0_j11_parser: bp35c0_j11::Parser,
    }

    #[init(
        local = [
            uart1_rx_queue: Uart1RxQueue = Uart1RxQueue::new(),
        ]
    )]
    fn init(ctx: init::Context) -> (Shared, Local) {
        {
            const HEAP_SIZE: usize = 8 * 1024;
            static mut HEAP: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
            unsafe { ALLOCATOR.init(addr_of_mut!(HEAP) as usize, HEAP_SIZE) }
        }

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
        let pins = hal::gpio::Pins::new(
            ctx.device.IO_BANK0,
            ctx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        let pins_uart = (pins.gpio0.into_function(), pins.gpio1.into_function());
        let uart0 = UartPeripheral::new(ctx.device.UART0, pins_uart, &mut resets)
            .enable(
                UartConfig::new(115_200.Hz(), DataBits::Eight, None, StopBits::One),
                clocks.peripheral_clock.freq(),
            )
            .unwrap();

        let pins_uart = (pins.gpio24.into_function(), pins.gpio25.into_function());
        let mut uart1 = UartPeripheral::new(ctx.device.UART1, pins_uart, &mut resets)
            .enable(
                UartConfig::new(115_200.Hz(), DataBits::Eight, None, StopBits::One),
                clocks.peripheral_clock.freq(),
            )
            .unwrap();
        uart1.enable_rx_interrupt();

        let i2c1 = I2C::i2c1(
            ctx.device.I2C1,
            pins.gpio2.reconfigure(),
            pins.gpio3.reconfigure(),
            400.kHz(),
            &mut resets,
            &clocks.system_clock,
        );

        let pins_spi = (
            pins.gpio19.into_function(),
            pins.gpio20.into_function(),
            pins.gpio18.into_function(),
        );
        let spi0 = Spi::<_, _, _, 8>::new(ctx.device.SPI0, pins_spi).init(
            &mut resets,
            clocks.peripheral_clock.freq(),
            5.MHz(),
            SPI_MODE_3,
        );

        // NeoPixel LED / WS2812 を PIO で制御する
        // https://cdn-shop.adafruit.com/datasheets/WS2812.pdf
        //
        // RP2040 Datasheet にサンプルコードがあるが、勉強のため見ないで書いた
        //
        // WS2812 は1本の線で、データを次のフォーマットでエンコードして送る
        //      ┌─────┐         ┌
        //   0: │ T0H │   T0L   │   T0H: 0.35 us, T0L: 0.80 us, +/- 0.15 us
        //      ┘     └─────────┘
        //      ┌─────────┐     ┌
        //   1: │   T1H   │ T1L │   T1H: 0.70 us, T1L: 0.60 us, +/- 0.15 us
        //      ┘         └─────┘
        //
        // PIO の1クロックを約 0.1 us (sys_clk: 125 MHz, divisor: 12 + 128/256) として
        // T0H: 0.3 us, T0L: 0.7 us, T1H: 0.7 us, T1L: 0.7 us 程度で制御する
        //
        // データは G, R, B の順で 8-bit ずつ、MSB-first で送る
        // シフト方向を左にし、[31:8] にデータを詰めるのを想定

        let neopixel: hal::gpio::Pin<_, FunctionPio0, _> = pins.gpio16.into_function();
        let neopixel_pin_id = neopixel.id().num;
        let program = pio_proc::pio_asm!(
            ".side_set 1",
            "loop:",
            "   out x, 1 side 0",        // (1) L:1
            "   jmp !x skip side 1 [2]", // (2) L:1, H:2
            "   nop side 1 [3]",         // (3) H:1, H:2
            "skip:",                     //
            "   jmp loop side 0 [5]",    // (4) H:1, L:5
        );
        let (mut pio, sm0, _, _, _) = ctx.device.PIO0.split(&mut resets);
        let installed = pio.install(&program.program).unwrap();
        let (mut sm, _, tx) = hal::pio::PIOBuilder::from_installed_program(installed)
            .clock_divisor_fixed_point(12, 128)
            .autopull(true)
            .pull_threshold(24)
            .out_shift_direction(hal::pio::ShiftDirection::Left)
            .side_set_pin_base(neopixel_pin_id)
            .build(sm0);
        sm.set_pindirs([(neopixel_pin_id, hal::pio::PinDir::Output)]);
        sm.start();

        let spi0_csn_adt7310 = pins.gpio6.into_push_pull_output_in_state(PinState::High);
        let bp35c0_j11_resetn = pins.gpio11.into_push_pull_output_in_state(PinState::Low);
        let txs0108e_oe = pins.gpio10.into_push_pull_output_in_state(PinState::Low);
        let lcd_resetn = pins.gpio7.into_push_pull_output_in_state(PinState::Low);
        let led = pins.gpio13.into_push_pull_output_in_state(PinState::Low);

        let (uart1_rx, uart1_tx) = uart1.split();
        let (uart1_rx_sender, uart1_rx_receiver) = ctx.local.uart1_rx_queue.split();

        Mono::start(ctx.device.TIMER, &resets);

        task_adt7310::spawn().unwrap();
        task_bp35c0_j11::spawn().unwrap();
        task_lcd::spawn().unwrap();
        task_led_blink::spawn().unwrap();
        task_neopixel::spawn().unwrap();

        (
            Shared {
                bp35c0_j11_status: Bp35c0J11Status::Initializing,
                temperature_raw: 0,
            },
            Local {
                uart0,
                uart1_rx,
                uart1_rx_receiver,
                uart1_rx_sender,
                uart1_tx: Bp35c0J11Writer(uart1_tx),
                i2c1,
                spi0,
                spi0_csn_adt7310,
                bp35c0_j11_resetn,
                txs0108e_oe,
                lcd_resetn,
                led,
                neopixel: NeoPixel(tx),
                bp35c0_j11_parser: bp35c0_j11::Parser::default(),
            },
        )
    }

    #[task(priority = 1, local = [led])]
    async fn task_led_blink(ctx: task_led_blink::Context) {
        loop {
            let now = Mono::now();

            ctx.local.led.set_high().unwrap();
            Mono::delay(100.millis()).await;
            ctx.local.led.set_low().unwrap();

            Mono::delay(200.millis()).await;

            ctx.local.led.set_high().unwrap();
            Mono::delay(100.millis()).await;
            ctx.local.led.set_low().unwrap();

            Mono::delay_until(now + 1_500.millis()).await;
        }
    }

    #[task(
        priority = 1,
        shared = [bp35c0_j11_status],
        local = [neopixel]
    )]
    async fn task_neopixel(ctx: task_neopixel::Context) {
        // misc/hsv2rgb.rs で生成
        static TABLE: [u32; 256] = [
            0x00_00_05_00,
            0x00_00_05_00,
            0x00_00_05_00,
            0x00_00_05_00,
            0x00_00_05_00,
            0x00_00_05_00,
            0x00_00_05_00,
            0x00_00_05_00,
            0x00_00_05_00,
            0x00_00_05_00,
            0x00_00_05_00,
            0x00_00_05_00,
            0x00_00_05_00,
            0x01_00_05_00,
            0x01_00_05_00,
            0x01_00_05_00,
            0x01_00_05_00,
            0x01_00_05_00,
            0x01_00_05_00,
            0x01_00_05_00,
            0x01_00_05_00,
            0x01_00_05_00,
            0x01_00_05_00,
            0x01_00_05_00,
            0x01_00_05_00,
            0x02_00_05_00,
            0x02_00_05_00,
            0x02_00_05_00,
            0x02_00_05_00,
            0x02_00_05_00,
            0x02_00_05_00,
            0x02_00_05_00,
            0x02_00_05_00,
            0x02_00_05_00,
            0x02_00_05_00,
            0x02_00_05_00,
            0x02_00_05_00,
            0x02_00_05_00,
            0x03_00_05_00,
            0x03_00_05_00,
            0x03_00_05_00,
            0x03_00_05_00,
            0x03_00_05_00,
            0x03_00_05_00,
            0x03_00_05_00,
            0x03_00_05_00,
            0x03_00_05_00,
            0x03_00_05_00,
            0x03_00_05_00,
            0x03_00_05_00,
            0x04_00_05_00,
            0x04_00_05_00,
            0x04_00_05_00,
            0x04_00_05_00,
            0x04_00_05_00,
            0x04_00_05_00,
            0x04_00_05_00,
            0x04_00_05_00,
            0x04_00_05_00,
            0x04_00_05_00,
            0x04_00_05_00,
            0x04_00_05_00,
            0x04_00_05_00,
            0x05_00_05_00,
            0x05_00_05_00,
            0x05_00_05_00,
            0x05_00_04_00,
            0x05_00_04_00,
            0x05_00_04_00,
            0x05_00_04_00,
            0x05_00_04_00,
            0x05_00_04_00,
            0x05_00_04_00,
            0x05_00_04_00,
            0x05_00_04_00,
            0x05_00_04_00,
            0x05_00_04_00,
            0x05_00_04_00,
            0x05_00_03_00,
            0x05_00_03_00,
            0x05_00_03_00,
            0x05_00_03_00,
            0x05_00_03_00,
            0x05_00_03_00,
            0x05_00_03_00,
            0x05_00_03_00,
            0x05_00_03_00,
            0x05_00_03_00,
            0x05_00_03_00,
            0x05_00_03_00,
            0x05_00_03_00,
            0x05_00_02_00,
            0x05_00_02_00,
            0x05_00_02_00,
            0x05_00_02_00,
            0x05_00_02_00,
            0x05_00_02_00,
            0x05_00_02_00,
            0x05_00_02_00,
            0x05_00_02_00,
            0x05_00_02_00,
            0x05_00_02_00,
            0x05_00_02_00,
            0x05_00_01_00,
            0x05_00_01_00,
            0x05_00_01_00,
            0x05_00_01_00,
            0x05_00_01_00,
            0x05_00_01_00,
            0x05_00_01_00,
            0x05_00_01_00,
            0x05_00_01_00,
            0x05_00_01_00,
            0x05_00_01_00,
            0x05_00_01_00,
            0x05_00_01_00,
            0x05_00_00_00,
            0x05_00_00_00,
            0x05_00_00_00,
            0x05_00_00_00,
            0x05_00_00_00,
            0x05_00_00_00,
            0x05_00_00_00,
            0x05_00_00_00,
            0x05_00_00_00,
            0x05_00_00_00,
            0x05_00_00_00,
            0x05_00_00_00,
            0x05_00_00_00,
            0x05_00_00_00,
            0x05_00_00_00,
            0x05_00_00_00,
            0x05_00_00_00,
            0x05_00_00_00,
            0x05_00_00_00,
            0x05_00_00_00,
            0x05_00_00_00,
            0x05_00_00_00,
            0x05_00_00_00,
            0x05_00_00_00,
            0x05_01_00_00,
            0x05_01_00_00,
            0x05_01_00_00,
            0x05_01_00_00,
            0x05_01_00_00,
            0x05_01_00_00,
            0x05_01_00_00,
            0x05_01_00_00,
            0x05_01_00_00,
            0x05_01_00_00,
            0x05_01_00_00,
            0x05_01_00_00,
            0x05_01_00_00,
            0x05_02_00_00,
            0x05_02_00_00,
            0x05_02_00_00,
            0x05_02_00_00,
            0x05_02_00_00,
            0x05_02_00_00,
            0x05_02_00_00,
            0x05_02_00_00,
            0x05_02_00_00,
            0x05_02_00_00,
            0x05_02_00_00,
            0x05_02_00_00,
            0x05_03_00_00,
            0x05_03_00_00,
            0x05_03_00_00,
            0x05_03_00_00,
            0x05_03_00_00,
            0x05_03_00_00,
            0x05_03_00_00,
            0x05_03_00_00,
            0x05_03_00_00,
            0x05_03_00_00,
            0x05_03_00_00,
            0x05_03_00_00,
            0x05_04_00_00,
            0x05_04_00_00,
            0x05_04_00_00,
            0x05_04_00_00,
            0x05_04_00_00,
            0x05_04_00_00,
            0x05_04_00_00,
            0x05_04_00_00,
            0x05_04_00_00,
            0x05_04_00_00,
            0x05_05_00_00,
            0x05_05_00_00,
            0x05_05_00_00,
            0x05_05_00_00,
            0x05_05_00_00,
            0x05_05_00_00,
            0x05_05_00_00,
            0x05_05_00_00,
            0x05_05_00_00,
            0x05_05_00_00,
            0x05_05_00_00,
            0x05_06_00_00,
            0x05_06_00_00,
            0x05_06_00_00,
            0x05_06_00_00,
            0x05_06_00_00,
            0x05_06_00_00,
            0x05_06_00_00,
            0x05_06_00_00,
            0x05_07_00_00,
            0x05_07_00_00,
            0x05_07_00_00,
            0x05_07_00_00,
            0x05_07_00_00,
            0x05_08_00_00,
            0x05_08_00_00,
            0x05_09_00_00,
            0x06_09_00_00,
            0x06_09_00_00,
            0x06_0a_00_00,
            0x06_0b_00_00,
            0x06_0b_00_00,
            0x07_0c_00_00,
            0x07_0d_00_00,
            0x07_0e_00_00,
            0x08_0f_00_00,
            0x08_10_00_00,
            0x08_12_00_00,
            0x09_13_00_00,
            0x09_15_00_00,
            0x0a_17_00_00,
            0x0b_1a_00_00,
            0x0b_1c_00_00,
            0x0c_20_00_00,
            0x0d_23_00_00,
            0x0e_27_00_00,
            0x0f_2c_00_00,
            0x10_32_00_00,
            0x11_38_00_00,
            0x13_3f_00_00,
            0x14_48_00_00,
            0x15_52_00_00,
            0x17_5e_00_00,
            0x19_6b_00_00,
            0x1b_7b_00_00,
            0x1d_8e_00_00,
            0x1e_a4_00_00,
            0x20_be_00_00,
            0x22_dc_00_00,
            0x23_ff_00_00,
            0x20_ff_00_00,
            0x1c_ff_00_00,
            0x18_ff_00_00,
            0x13_ff_00_00,
            0x10_ff_00_00,
            0x0c_ff_00_00,
            0x08_ff_00_00,
            0x03_ff_00_00,
            0x00_ff_00_00,
        ];

        let mut bp35c0_j11_status = ctx.shared.bp35c0_j11_status;

        loop {
            if let Bp35c0J11Status::Data { instant, .. } = bp35c0_j11_status.lock(|status| *status)
            {
                // 2500 W あたりで最高輝度にする
                let i = ((instant as usize * (TABLE.len() - 1)) / 2500).min(TABLE.len() - 1);
                ctx.local.neopixel.set_raw(TABLE[i]);
            } else {
                ctx.local.neopixel.set_rgb(0, 0, 0);
            }
            Mono::delay(1.secs()).await;
        }
    }

    #[task(
        priority = 1,
        shared = [bp35c0_j11_status],
        local = [bp35c0_j11_resetn, txs0108e_oe, uart1_rx_receiver, uart1_tx]
    )]
    async fn task_bp35c0_j11(ctx: task_bp35c0_j11::Context) {
        use bp35c0_j11::*;

        ctx.local.txs0108e_oe.set_high().unwrap();
        Mono::delay(500.millis()).await;

        ctx.local.bp35c0_j11_resetn.set_high().unwrap();

        Mono::delay(500.millis()).await;

        let mut status = ctx.shared.bp35c0_j11_status;
        let rx = ctx.local.uart1_rx_receiver;
        let tx = ctx.local.uart1_tx;

        loop {
            'retry: loop {
                if let Ok(Response::NotificationPoweredOn) = rx.recv().await {
                    break 'retry;
                }
            }

            'retry: loop {
                tx.send(Command::GetVersionInformation).unwrap();

                while let Ok(resp) = rx.recv().await {
                    if let Response::GetVersionInformation { result, .. } = resp {
                        if result == 0x01 {
                            break 'retry;
                        } else {
                            Mono::delay(1.secs()).await;
                            continue 'retry;
                        }
                    }
                }
            }

            'retry: loop {
                tx.send(Command::SetOperationMode {
                    mode: OperationMode::Dual,
                    han_sleep: false,
                    channel: Channel::Ch4F922p5MHz,
                    tx_power: TxPower::P20mW,
                })
                .unwrap();

                while let Ok(resp) = rx.recv().await {
                    if let Response::SetOperationMode { result, .. } = resp {
                        if result == 0x01 {
                            break 'retry;
                        } else {
                            Mono::delay(1.secs()).await;
                            continue 'retry;
                        }
                    }
                }
            }

            let channel = 'retry: loop {
                tx.send(Command::DoActiveScan {
                    duration: ScanDuration::T616p96ms,
                    mask_channels: 0x3fff0,
                    pairing_id: Some(u64::from_be_bytes(ROUTE_B_ID[24..32].try_into().unwrap())),
                })
                .unwrap();

                status.lock(|status| *status = Bp35c0J11Status::Scanning);

                let mut scan_result = None;
                while let Ok(resp) = rx.recv().await {
                    match resp {
                        Response::DoActiveScan { result } => match (result, scan_result.take()) {
                            (0x01, Some(channel)) => break 'retry channel,
                            _ => {
                                Mono::delay(1.secs()).await;
                                continue 'retry;
                            }
                        },
                        Response::NotificationActiveScan { channel, terminal } => {
                            if !terminal.is_empty() {
                                scan_result.replace(channel);
                            }
                        }
                        _ => {}
                    }
                }
            };

            #[cfg(feature = "defmt")]
            defmt::info!("channel = {}", channel);

            'retry: loop {
                tx.send(Command::SetOperationMode {
                    mode: OperationMode::Dual,
                    han_sleep: false,
                    channel,
                    tx_power: TxPower::P20mW,
                })
                .unwrap();

                while let Ok(resp) = rx.recv().await {
                    if let Response::SetOperationMode { result, .. } = resp {
                        if result == 0x01 {
                            break 'retry;
                        } else {
                            Mono::delay(1.secs()).await;
                            continue 'retry;
                        }
                    }
                }
            }

            'retry: loop {
                tx.send(Command::SetRouteBPanaAuthenticationInformation {
                    id: ROUTE_B_ID,
                    password: ROUTE_B_PASSWORD,
                })
                .unwrap();

                while let Ok(resp) = rx.recv().await {
                    if let Response::SetRouteBPanaAuthenticationInformation { result, .. } = resp {
                        if result == 0x01 {
                            break 'retry;
                        } else {
                            Mono::delay(1.secs()).await;
                            continue 'retry;
                        }
                    }
                }
            }

            'retry: loop {
                tx.send(Command::StartRouteBOperation).unwrap();

                while let Ok(resp) = rx.recv().await {
                    if let Response::StartRouteBOperation { result, .. } = resp {
                        if result == 0x01 {
                            break 'retry;
                        } else {
                            Mono::delay(1.secs()).await;
                            continue 'retry;
                        }
                    }
                }
            }

            'retry: loop {
                tx.send(Command::OpenUdpPort(0x0e1a)).unwrap();

                while let Ok(resp) = rx.recv().await {
                    if let Response::OpenUdpPort { result, .. } = resp {
                        if result == 0x01 {
                            break 'retry;
                        } else {
                            Mono::delay(1.secs()).await;
                            continue 'retry;
                        }
                    }
                }
            }

            let mac_address = 'retry: loop {
                tx.send(Command::StartRouteBPana).unwrap();

                'wait: while let Ok(resp) = rx.recv().await {
                    match resp {
                        Response::StartRouteBPana { result, .. } if result != 0x01 => break 'wait,
                        Response::NotificationPanaAuthentication {
                            result,
                            mac_address,
                        } => {
                            if result == 0x01 {
                                break 'retry mac_address;
                            } else {
                                break 'wait;
                            }
                        }
                        _ => {}
                    }
                }

                Mono::delay(1.secs()).await;
            };

            #[cfg(feature = "defmt")]
            defmt::info!("mac_address = {}", mac_address);

            status.lock(|status| *status = Bp35c0J11Status::Ready);

            let destination_address = 0xfe800000000000000000000000000000_u128
                | (mac_address ^ 0x02000000_00000000) as u128;

            'outer: for tid in 0.. {
                let tid_be = ((tid & 0xffff) as u16).to_be_bytes();

                'retry: loop {
                    tx.send(Command::TransmitData {
                        destination_address,
                        source_port: 0x0e1a,
                        destination_port: 0x0e1a,
                        data: &[
                            0x10, // EHD1
                            0x81, // EHD2
                            tid_be[0], tid_be[1], // TID
                            0x05, 0xff, 0x01, // SEOJ
                            0x02, 0x88, 0x01, // DEOJ
                            0x62, // ESV (Get)
                            0x04, // OPC
                            0x97, // EPC (現在時刻設定)
                            0x00, // PDC
                            0x98, // EPC (現在年月日設定)
                            0x00, // PDC
                            0xd3, // EPC (電力係数)
                            0x00, // PDC
                            0xe7, // EPC (瞬時電力計測値)
                            0x00, // PDC
                        ],
                    })
                    .unwrap();

                    while let Ok(resp) = rx.recv().await {
                        if let Response::TransmitData { result, .. } = resp {
                            if result == 0x01 {
                                break 'retry;
                            } else {
                                Mono::delay(1.secs()).await;
                                continue 'outer;
                            }
                        }
                    }
                }

                let now = Mono::now();

                let data_expected = [
                    0x10, // EHD1
                    0x81, // EHD2
                    tid_be[0], tid_be[1], // TID
                    0x02, 0x88, 0x01, // DEOJ
                    0x05, 0xff, 0x01, // SEOJ
                    0x72, // ESV (Get_Res)
                    0x04, // OPC
                ];

                // TODO: スマートメーターは ESV = 0x73 (プロパティ値通知, INT) で次のプロパティを送ってきていそうなので使う
                // - 定時積算電力量計測値 (正方向計測値) 0xEA
                // - 定時積算電力量計測値 (逆方向計測値) 0xEB
                //
                // INFO  Rx: NotificationUdpReceived { source_address: REDACTED, source_port: 3610, destination_port: 3610, source_pan_id: REDACTED, source_type: 0, encryption: 2, rssi: -69, data: "[16, 129, 0, 1, 2, 136, 1, 5, 255, 1, 115, 2, 234, 11, 7, 232, 9, 1, 10, 30, 0, 0, 0, 28, 210, 235, 11, 7, 232, 9, 1, 10, 30, 0, 0, 0, 0, 21]" }

                'wait: loop {
                    let resp = futures::select_biased! {
                        r = rx.recv().fuse() => r.ok(),
                        _ = Mono::delay_until(now + 10.secs()).fuse() => None,
                    };

                    match resp {
                        Some(Response::NotificationUdpReceived {
                            source_port: 0x0e1a,
                            destination_port: 0x0e1a,
                            source_type: 0x00,
                            data,
                            rssi,
                            ..
                        }) if data.starts_with(&data_expected) && data.len() == 34 => {
                            // TODO: ちゃんとデータサイズとかチェックする
                            status.lock(|status| {
                                *status = Bp35c0J11Status::Data {
                                    datetime: (
                                        u16::from_be_bytes([data[18], data[19]]), // year
                                        data[20],                                 // month
                                        data[21],                                 // day
                                        data[14],                                 // hour
                                        data[15],                                 // min
                                    ),
                                    instant: u32::from_be_bytes([
                                        data[30], data[31], data[32], data[33],
                                    ]),
                                    rssi,
                                }
                            });
                            break 'wait;
                        }

                        Some(_) => {}

                        None => {
                            #[cfg(feature = "defmt")]
                            defmt::warn!("timeout");
                            break 'wait;
                        }
                    }
                }

                Mono::delay_until(now + 10.secs()).await;
            }
        }
    }

    #[task(
        priority = 1,
        shared = [temperature_raw],
        local = [spi0, spi0_csn_adt7310]
    )]
    async fn task_adt7310(ctx: task_adt7310::Context) {
        let spi = ctx.local.spi0;
        let spi_csn = ctx.local.spi0_csn_adt7310;

        {
            spi_csn.set_low().unwrap();
            spi.write(&[0xff_u8; 4]).unwrap();
            spi_csn.set_high().unwrap();
            Mono::delay(1.millis()).await;
        }

        let mut temperature_raw = ctx.shared.temperature_raw;

        loop {
            let now = Mono::now();

            spi_csn.set_low().unwrap();

            #[allow(clippy::unusual_byte_groupings)]
            //                  ┌───── configuration register
            //                  │         ┌───── Resolution (16-bit)
            //                  │         │  ┌───── Operation mode (One shot)
            spi.write(&[0b0_0_001_0_00, 0b1_01_0_0_0_00]).unwrap();

            Mono::delay(240.millis()).await;

            #[allow(clippy::unusual_byte_groupings)]
            //                     ┌───── temperature value register
            let mut buf = [0b0_1_010_0_00, 0, 0];

            spi.transfer(buf.as_mut_slice()).unwrap();

            spi_csn.set_high().unwrap();

            temperature_raw.lock(|temp| *temp = i16::from_be_bytes([buf[1], buf[2]]));

            Mono::delay_until(now + 1.secs()).await;
        }
    }

    #[task(
        priority = 1,
        shared = [bp35c0_j11_status, temperature_raw],
        local = [i2c1, lcd_resetn, uart0]
    )]
    async fn task_lcd(ctx: task_lcd::Context) {
        const LCD_ADDRESS: u8 = 0x3e;

        ctx.local.lcd_resetn.set_high().unwrap();
        Mono::delay(50.millis()).await;

        type Duration = <Mono as TimerQueueBasedMonotonic>::Duration;

        let i2c = ctx.local.i2c1;
        let lcd = st7032i::St7032i::<_, _, _, Duration>::new(i2c, LCD_ADDRESS);

        let (lcd, delay) = lcd.set_instruction_set::<st7032i::Normal>().unwrap();
        Mono::delay(delay).await;

        let (mut lcd, delay) = lcd.set_instruction_set::<st7032i::Extention>().unwrap();
        Mono::delay(delay).await;

        let delay = lcd.cmd_internal_osc_frequency().unwrap();
        Mono::delay(delay).await;

        let delay = lcd.cmd_contrast_set(0b1100).unwrap();
        Mono::delay(delay).await;

        let delay = lcd.cmd_power_icon_contrast_set(false, true, 0b01).unwrap();
        Mono::delay(delay).await;

        let delay = lcd.cmd_follower_control(true, 0b100).unwrap();
        Mono::delay(delay).await;

        let (mut lcd, delay) = lcd.set_instruction_set::<st7032i::Normal>().unwrap();
        Mono::delay(delay).await;

        let delay = lcd.cmd_display_on_off(true, false, false).unwrap();
        Mono::delay(delay).await;

        let delay = lcd.cmd_set_cgram_address(0).unwrap();
        Mono::delay(delay).await;

        let delay = lcd
            .cmd_write_bytes([
                // CGRAM[0]
                0b000_00000,
                0b000_00100,
                0b000_00100,
                0b000_00100,
                0b000_00100,
                0b000_00100,
                0b000_00000,
                0b000_00000,
                // CGRAM[1]
                0b000_00000,
                0b000_00000,
                0b000_00000,
                0b000_00000,
                0b000_00000,
                0b000_10001,
                0b000_01110,
                0b000_00000,
            ])
            .unwrap();
        Mono::delay(delay).await;

        let delay = lcd.cmd_clear_display().unwrap();
        Mono::delay(delay).await;

        // 起動メッセージ
        {
            write!(lcd, " Hello!").unwrap();
            Mono::delay(st7032i::EXECUTION_TIME_SHORT.into()).await;

            let delay = lcd
                .cmd_set_ddram_address(st7032i::DDRAM_ADDRESS_LINE2)
                .unwrap();
            Mono::delay(delay).await;

            let _ = lcd
                .cmd_write_chars([
                    '('.into(),
                    '*'.into(),
                    st7032i::Character::Cgram(0),
                    st7032i::Character::Cgram(1),
                    st7032i::Character::Cgram(0),
                    ')'.into(),
                    'ﾉ'.into(),
                ])
                .unwrap();
            Mono::delay(3.secs()).await;
        }

        let mut bp35c0_j11_status = ctx.shared.bp35c0_j11_status;
        let mut temperature_raw = ctx.shared.temperature_raw;

        #[derive(Clone, Copy)]
        enum DisplayItemBp35c0J11 {
            Datetime,
            Instant,
            Rssi,
        }

        #[derive(Clone, Copy)]
        enum DisplayItem {
            Bp35c0J11(DisplayItemBp35c0J11),
            Temperature,
        }

        let mut next = DisplayItem::Bp35c0J11(DisplayItemBp35c0J11::Datetime);

        loop {
            let delay = lcd.cmd_clear_display().unwrap();
            Mono::delay(delay).await;

            let now = Mono::now();

            let linebreak = |lcd: &mut st7032i::St7032i<_, _, _, Duration>| {
                let delay = lcd
                    .cmd_set_ddram_address(st7032i::DDRAM_ADDRESS_LINE2)
                    .unwrap();
                Mono::delay(delay)
            };

            next = match next {
                DisplayItem::Bp35c0J11(i) => match (bp35c0_j11_status.lock(|s| *s), i) {
                    (Bp35c0J11Status::Initializing, _) => {
                        write!(lcd, "BP35C0:").unwrap();
                        Mono::delay(st7032i::EXECUTION_TIME_SHORT.into()).await;

                        linebreak(&mut lcd).await;

                        write!(lcd, " Init...").unwrap();
                        Mono::delay(st7032i::EXECUTION_TIME_SHORT.into()).await;

                        DisplayItem::Temperature
                    }

                    (Bp35c0J11Status::Scanning, _) => {
                        write!(lcd, "BP35C0:").unwrap();
                        Mono::delay(st7032i::EXECUTION_TIME_SHORT.into()).await;

                        linebreak(&mut lcd).await;

                        write!(lcd, " Scan...").unwrap();
                        Mono::delay(st7032i::EXECUTION_TIME_SHORT.into()).await;

                        DisplayItem::Temperature
                    }

                    (Bp35c0J11Status::Ready, _) => {
                        write!(lcd, "BP35C0:").unwrap();
                        Mono::delay(st7032i::EXECUTION_TIME_SHORT.into()).await;

                        linebreak(&mut lcd).await;

                        write!(lcd, "  Ready!").unwrap();
                        Mono::delay(st7032i::EXECUTION_TIME_SHORT.into()).await;

                        DisplayItem::Temperature
                    }

                    (Bp35c0J11Status::Data { datetime, .. }, DisplayItemBp35c0J11::Datetime) => {
                        let (year, month, day, hour, min) = datetime;

                        write!(lcd, "{:02}-{:02}-{:02}", year % 100, month, day).unwrap();
                        Mono::delay(st7032i::EXECUTION_TIME_SHORT.into()).await;

                        linebreak(&mut lcd).await;

                        write!(lcd, "   {:02}:{:02}", hour, min).unwrap();
                        Mono::delay(st7032i::EXECUTION_TIME_SHORT.into()).await;

                        DisplayItem::Bp35c0J11(DisplayItemBp35c0J11::Instant)
                    }

                    (Bp35c0J11Status::Data { instant, .. }, DisplayItemBp35c0J11::Instant) => {
                        write!(lcd, "Instant:").unwrap();
                        Mono::delay(st7032i::EXECUTION_TIME_SHORT.into()).await;

                        linebreak(&mut lcd).await;

                        write!(lcd, "{:>6} W", instant).unwrap();
                        Mono::delay(st7032i::EXECUTION_TIME_SHORT.into()).await;

                        DisplayItem::Bp35c0J11(DisplayItemBp35c0J11::Rssi)
                    }

                    (Bp35c0J11Status::Data { rssi, .. }, DisplayItemBp35c0J11::Rssi) => {
                        write!(lcd, "RSSI:").unwrap();
                        Mono::delay(st7032i::EXECUTION_TIME_SHORT.into()).await;

                        linebreak(&mut lcd).await;

                        write!(lcd, " {:>3} dBm", rssi).unwrap();
                        Mono::delay(st7032i::EXECUTION_TIME_SHORT.into()).await;

                        DisplayItem::Temperature
                    }
                },

                DisplayItem::Temperature => {
                    let temp_raw = temperature_raw.lock(|temp| *temp);
                    let temp_int = temp_raw / 128;
                    let temp_frac = 78125_u32 * (temp_raw % 128).unsigned_abs() as u32;

                    write!(lcd, "Temp:").unwrap();
                    Mono::delay(st7032i::EXECUTION_TIME_SHORT.into()).await;

                    linebreak(&mut lcd).await;

                    write!(lcd, "{:3}.{:04}", temp_int, temp_frac / 1000).unwrap();
                    Mono::delay(st7032i::EXECUTION_TIME_SHORT.into()).await;

                    DisplayItem::Bp35c0J11(DisplayItemBp35c0J11::Datetime)
                }
            };

            Mono::delay_until(now + 3.secs()).await;
        }
    }

    #[task(
        priority = 1,
        binds = UART1_IRQ,
        local = [uart1_rx, uart1_rx_sender, bp35c0_j11_parser]
    )]
    fn uart1_irq(ctx: uart1_irq::Context) {
        let uart = ctx.local.uart1_rx;
        let parser = ctx.local.bp35c0_j11_parser;
        let sender = ctx.local.uart1_rx_sender;
        let mut rx_buf = [0; 32];
        if let Ok(len) = uart.read_raw(&mut rx_buf) {
            for resp in rx_buf[0..len].iter().flat_map(|&c| parser.parse(c)) {
                #[cfg(feature = "defmt")]
                defmt::info!("Rx: {:?}", resp);
                sender.try_send(resp).unwrap();
            }
        }
    }
}
