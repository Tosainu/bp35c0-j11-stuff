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

    use core::fmt::Write;
    use core::mem::MaybeUninit;

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
    use embedded_hal::spi::MODE_3 as SPI_MODE_3;
    use embedded_hal_0_2_x::blocking::spi::{Transfer as SpiTransfer, Write as SpiWrite};

    use rtic_monotonics::rp2040::prelude::*;
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

    #[derive(Clone, Copy)]
    pub enum Bp35c0J11Status {
        Initializing,
        Scanning,
        Ready,
        Data {
            year: u16,
            month: u8,
            day: u8,
            hour: u8,
            min: u8,
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
            unsafe { crate::ALLOCATOR.init(HEAP.as_ptr() as usize, HEAP_SIZE) }
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
        let mut uart1 = UartPeripheral::new(ctx.device.UART1, pins_uart, &mut resets)
            .enable(
                UartConfig::new(115_200.Hz(), DataBits::Eight, None, StopBits::One),
                clocks.peripheral_clock.freq(),
            )
            .unwrap();
        uart1.enable_rx_interrupt();

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
            SPI_MODE_3,
        );

        let spi0_csn_adt7310 = pins.d4.into_push_pull_output_in_state(PinState::High);
        let bp35c0_j11_resetn = pins.d11.into_push_pull_output_in_state(PinState::Low);
        let txs0108e_oe = pins.d10.into_push_pull_output_in_state(PinState::Low);
        let lcd_resetn = pins.d5.into_push_pull_output_in_state(PinState::Low);
        let led = pins.d13.into_push_pull_output_in_state(PinState::High);

        let (uart1_rx, uart1_tx) = uart1.split();
        let (uart1_rx_sender, uart1_rx_receiver) = ctx.local.uart1_rx_queue.split();

        Mono::start(ctx.device.TIMER, &resets);

        task_adt7310::spawn().unwrap();
        task_bp35c0_j11::spawn().unwrap();
        task_lcd::spawn().unwrap();
        task_led_blink::spawn().unwrap();

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
                bp35c0_j11_parser: bp35c0_j11::Parser::default(),
            },
        )
    }

    #[task(priority = 1, local = [led])]
    async fn task_led_blink(ctx: task_led_blink::Context) {
        loop {
            ctx.local.led.toggle().unwrap();

            Mono::delay(100.millis()).await;
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

            for tid in 0.. {
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
                            0x62, // ESV
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
                                continue 'retry;
                            }
                        }
                    }
                }

                let data_expected = [
                    0x10, // EHD1
                    0x81, // EHD2
                    tid_be[0], tid_be[1], // TID
                    0x02, 0x88, 0x01, // DEOJ
                    0x05, 0xff, 0x01, // SEOJ
                ];
                'wait: while let Ok(resp) = rx.recv().await {
                    match resp {
                        Response::NotificationUdpReceived {
                            source_port: 0x0e1a,
                            destination_port: 0x0e1a,
                            data,
                            rssi,
                            ..
                        } if data.starts_with(&data_expected) && data.len() == 34 => {
                            // TODO: ちゃんとデータサイズとかチェックする
                            status.lock(|status| {
                                *status = Bp35c0J11Status::Data {
                                    year: u16::from_be_bytes([data[18], data[19]]),
                                    month: data[20],
                                    day: data[21],
                                    hour: data[14],
                                    min: data[15],
                                    instant: u32::from_be_bytes([
                                        data[30], data[31], data[32], data[33],
                                    ]),
                                    rssi,
                                }
                            });
                            break 'wait;
                        }
                        _ => (),
                    };
                }

                Mono::delay(10.secs()).await;
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

        let i2c = ctx.local.i2c1;
        let lcd = st7032i::St7032i::<_, _, _>::new(i2c, LCD_ADDRESS);

        let (lcd, delay) = lcd.set_instruction_set::<st7032i::Normal>().unwrap();
        Mono::delay(delay.convert().into()).await;

        let (mut lcd, delay) = lcd.set_instruction_set::<st7032i::Extention>().unwrap();
        Mono::delay(delay.convert().into()).await;

        let delay = lcd.cmd_internal_osc_frequency().unwrap();
        Mono::delay(delay.convert().into()).await;

        let delay = lcd.cmd_contrast_set(0b1100).unwrap();
        Mono::delay(delay.convert().into()).await;

        let delay = lcd.cmd_power_icon_contrast_set(false, true, 0b01).unwrap();
        Mono::delay(delay.convert().into()).await;

        let delay = lcd.cmd_follower_control(true, 0b100).unwrap();
        Mono::delay(delay.convert().into()).await;

        let (mut lcd, delay) = lcd.set_instruction_set::<st7032i::Normal>().unwrap();
        Mono::delay(delay.convert().into()).await;

        let delay = lcd.cmd_display_on_off(true, false, false).unwrap();
        Mono::delay(delay.convert().into()).await;

        let delay = lcd.cmd_set_cgram_address(0).unwrap();
        Mono::delay(delay.convert().into()).await;

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
        Mono::delay(delay.convert().into()).await;

        let delay = lcd.cmd_clear_display().unwrap();
        Mono::delay(delay.convert().into()).await;

        // 起動メッセージ
        {
            write!(lcd, " Hello!").unwrap();
            Mono::delay(st7032i::EXECUTION_TIME_SHORT.convert().into()).await;

            let delay = lcd
                .cmd_set_ddram_address(st7032i::DDRAM_ADDRESS_LINE2)
                .unwrap();
            Mono::delay(delay.convert().into()).await;

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

        loop {
            match bp35c0_j11_status.lock(|status| *status) {
                Bp35c0J11Status::Initializing => {
                    let delay = lcd.cmd_clear_display().unwrap();
                    Mono::delay(delay.convert().into()).await;

                    let now = Mono::now();

                    write!(lcd, "BP35C0:").unwrap();
                    Mono::delay(st7032i::EXECUTION_TIME_SHORT.convert().into()).await;

                    let delay = lcd
                        .cmd_set_ddram_address(st7032i::DDRAM_ADDRESS_LINE2)
                        .unwrap();
                    Mono::delay(delay.convert().into()).await;

                    write!(lcd, " Init...").unwrap();
                    Mono::delay(st7032i::EXECUTION_TIME_SHORT.convert().into()).await;

                    Mono::delay_until(now + 3.secs()).await;
                }

                Bp35c0J11Status::Scanning => {
                    let delay = lcd.cmd_clear_display().unwrap();
                    Mono::delay(delay.convert().into()).await;

                    let now = Mono::now();

                    write!(lcd, "BP35C0:").unwrap();
                    Mono::delay(st7032i::EXECUTION_TIME_SHORT.convert().into()).await;

                    let delay = lcd
                        .cmd_set_ddram_address(st7032i::DDRAM_ADDRESS_LINE2)
                        .unwrap();
                    Mono::delay(delay.convert().into()).await;

                    write!(lcd, " Scan...").unwrap();
                    Mono::delay(st7032i::EXECUTION_TIME_SHORT.convert().into()).await;

                    Mono::delay_until(now + 3.secs()).await;
                }

                Bp35c0J11Status::Ready => {
                    let delay = lcd.cmd_clear_display().unwrap();
                    Mono::delay(delay.convert().into()).await;

                    let now = Mono::now();

                    write!(lcd, "BP35C0:").unwrap();
                    Mono::delay(st7032i::EXECUTION_TIME_SHORT.convert().into()).await;

                    let delay = lcd
                        .cmd_set_ddram_address(st7032i::DDRAM_ADDRESS_LINE2)
                        .unwrap();
                    Mono::delay(delay.convert().into()).await;

                    write!(lcd, "  Ready!").unwrap();
                    Mono::delay(st7032i::EXECUTION_TIME_SHORT.convert().into()).await;

                    Mono::delay_until(now + 3.secs()).await;
                }

                Bp35c0J11Status::Data {
                    year,
                    month,
                    day,
                    hour,
                    min,
                    instant,
                    rssi,
                } => {
                    {
                        let delay = lcd.cmd_clear_display().unwrap();
                        Mono::delay(delay.convert().into()).await;

                        let now = Mono::now();

                        write!(lcd, "Date:").unwrap();
                        Mono::delay(st7032i::EXECUTION_TIME_SHORT.convert().into()).await;

                        let delay = lcd
                            .cmd_set_ddram_address(st7032i::DDRAM_ADDRESS_LINE2)
                            .unwrap();
                        Mono::delay(delay.convert().into()).await;

                        write!(lcd, "{:02}-{:02}-{:02}", year % 100, month, day).unwrap();
                        Mono::delay(st7032i::EXECUTION_TIME_SHORT.convert().into()).await;

                        Mono::delay_until(now + 3.secs()).await;
                    }

                    {
                        let delay = lcd.cmd_clear_display().unwrap();
                        Mono::delay(delay.convert().into()).await;

                        let now = Mono::now();

                        write!(lcd, "Time:").unwrap();
                        Mono::delay(st7032i::EXECUTION_TIME_SHORT.convert().into()).await;

                        let delay = lcd
                            .cmd_set_ddram_address(st7032i::DDRAM_ADDRESS_LINE2)
                            .unwrap();
                        Mono::delay(delay.convert().into()).await;

                        write!(lcd, "   {:02}:{:02}", hour, min).unwrap();
                        Mono::delay(st7032i::EXECUTION_TIME_SHORT.convert().into()).await;

                        Mono::delay_until(now + 3.secs()).await;
                    }

                    {
                        let delay = lcd.cmd_clear_display().unwrap();
                        Mono::delay(delay.convert().into()).await;

                        let now = Mono::now();

                        write!(lcd, "Instant:").unwrap();
                        Mono::delay(st7032i::EXECUTION_TIME_SHORT.convert().into()).await;

                        let delay = lcd
                            .cmd_set_ddram_address(st7032i::DDRAM_ADDRESS_LINE2)
                            .unwrap();
                        Mono::delay(delay.convert().into()).await;

                        write!(lcd, "{:>6} W", instant).unwrap();
                        Mono::delay(st7032i::EXECUTION_TIME_SHORT.convert().into()).await;

                        Mono::delay_until(now + 3.secs()).await;
                    }

                    {
                        let delay = lcd.cmd_clear_display().unwrap();
                        Mono::delay(delay.convert().into()).await;

                        let now = Mono::now();

                        write!(lcd, "RSSI:").unwrap();
                        Mono::delay(st7032i::EXECUTION_TIME_SHORT.convert().into()).await;

                        let delay = lcd
                            .cmd_set_ddram_address(st7032i::DDRAM_ADDRESS_LINE2)
                            .unwrap();
                        Mono::delay(delay.convert().into()).await;

                        write!(lcd, " {:>3} dBm", rssi).unwrap();
                        Mono::delay(st7032i::EXECUTION_TIME_SHORT.convert().into()).await;

                        Mono::delay_until(now + 3.secs()).await;
                    }
                }
            }

            {
                let delay = lcd.cmd_clear_display().unwrap();
                Mono::delay(delay.convert().into()).await;

                let temp_raw = temperature_raw.lock(|temp| *temp);
                let temp_int = temp_raw / 128;
                let temp_frac = 78125_u32 * (temp_raw % 128).unsigned_abs() as u32;

                let now = Mono::now();

                write!(lcd, "Temp:").unwrap();
                Mono::delay(st7032i::EXECUTION_TIME_SHORT.convert().into()).await;

                let delay = lcd
                    .cmd_set_ddram_address(st7032i::DDRAM_ADDRESS_LINE2)
                    .unwrap();
                Mono::delay(delay.convert().into()).await;

                write!(lcd, "{:3}.{:04}", temp_int, temp_frac / 1000).unwrap();
                Mono::delay(st7032i::EXECUTION_TIME_SHORT.convert().into()).await;

                Mono::delay_until(now + 3.secs()).await;
            }
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
