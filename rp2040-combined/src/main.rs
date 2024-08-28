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
    use embedded_hal_0_2_x::blocking::i2c::Write as I2cWrite;
    use embedded_hal_0_2_x::blocking::spi::{Transfer as SpiTransfer, Write as SpiWrite};
    use embedded_io::Write;

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

    const UART1_TX_QUEUE_DEPTH: usize = 256;
    type Uart1TxQueue = heapless::spsc::Queue<u8, UART1_TX_QUEUE_DEPTH>;
    type Uart1TxSender<'a> = heapless::spsc::Producer<'a, u8, UART1_TX_QUEUE_DEPTH>;
    type Uart1TxReceiver<'a> = heapless::spsc::Consumer<'a, u8, UART1_TX_QUEUE_DEPTH>;

    const UART1_RX_QUEUE_DEPTH: usize = 4;
    type Uart1RxQueue = rtic_sync::channel::Channel<bp35c0_j11::Response, UART1_RX_QUEUE_DEPTH>;
    type Uart1RxSender<'a> =
        rtic_sync::channel::Sender<'a, bp35c0_j11::Response, UART1_RX_QUEUE_DEPTH>;
    type Uart1RxReceiver<'a> =
        rtic_sync::channel::Receiver<'a, bp35c0_j11::Response, UART1_RX_QUEUE_DEPTH>;

    #[shared]
    struct Shared {
        uart1_tx: hal::uart::Writer<hal::pac::UART1, Uart1Pins>,
        uart1_tx_receiver: Uart1TxReceiver<'static>,
    }

    #[local]
    struct Local {
        uart0: UartPeripheral<hal::uart::Enabled, hal::pac::UART0, Uart0Pins>,
        uart1_tx_sender: Uart1TxSender<'static>,
        uart1_rx: hal::uart::Reader<hal::pac::UART1, Uart1Pins>,
        uart1_rx_sender: Uart1RxSender<'static>,
        uart1_rx_receiver: Uart1RxReceiver<'static>,
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
            uart1_tx_queue: Uart1TxQueue = Uart1TxQueue::new(),
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
        uart1.enable_tx_interrupt();
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
        let (uart1_tx_sender, uart1_tx_receiver) = ctx.local.uart1_tx_queue.split();
        let (uart1_rx_sender, uart1_rx_receiver) = ctx.local.uart1_rx_queue.split();

        Mono::start(ctx.device.TIMER, &resets);

        task_adt7310::spawn().unwrap();
        task_bp35c0_j11::spawn().unwrap();
        task_lcd::spawn().unwrap();
        task_led_blink::spawn().unwrap();

        (
            Shared {
                uart1_tx,
                uart1_tx_receiver,
            },
            Local {
                uart0,
                uart1_tx_sender,
                uart1_rx,
                uart1_rx_sender,
                uart1_rx_receiver,
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

    #[idle(
        shared = [uart1_tx, uart1_tx_receiver]
    )]
    fn idle(mut ctx: idle::Context) -> ! {
        loop {
            (&mut ctx.shared.uart1_tx, &mut ctx.shared.uart1_tx_receiver).lock(|uart, tx_queue| {
                while let Some(&c) = tx_queue.peek() {
                    match uart.write(&[c; 1]) {
                        Ok(1) => tx_queue.dequeue().unwrap(),
                        _ => break,
                    };
                }
            });
        }
    }

    #[task(priority = 2, local = [led])]
    async fn task_led_blink(ctx: task_led_blink::Context) {
        loop {
            ctx.local.led.toggle().unwrap();

            Mono::delay(100.millis()).await;
        }
    }

    #[task(
        priority = 2,
        local = [bp35c0_j11_resetn, txs0108e_oe, uart1_tx_sender, uart1_rx_receiver]
    )]
    async fn task_bp35c0_j11(ctx: task_bp35c0_j11::Context) {
        use bp35c0_j11::*;

        ctx.local.txs0108e_oe.set_high().unwrap();
        Mono::delay(500.millis()).await;

        ctx.local.bp35c0_j11_resetn.set_high().unwrap();

        Mono::delay(500.millis()).await;

        fn send(queue: &mut Uart1TxSender, cmd: bp35c0_j11::Command) {
            #[cfg(feature = "defmt")]
            defmt::info!("Tx: {:?}", cmd);

            let mut buf = [0; 128];
            let len = bp35c0_j11::serialize_to_bytes(&cmd, &mut buf).unwrap();

            if len > queue.capacity() - queue.len() {
                #[cfg(feature = "defmt")]
                defmt::error!("can't send: {}/{}", len, queue.capacity() - queue.len());
                return;
            }

            for c in buf {
                unsafe { queue.enqueue_unchecked(c) }
            }
        }

        let sender = ctx.local.uart1_tx_sender;
        let receiver = ctx.local.uart1_rx_receiver;

        loop {
            'retry: loop {
                if let Ok(Response::NotificationPoweredOn) = receiver.recv().await {
                    break 'retry;
                }
            }

            'retry: loop {
                send(sender, Command::GetVersionInformation);
                match receiver.recv().await {
                    Ok(Response::GetVersionInformation { result: 0x01, .. }) => break 'retry,
                    _ => Mono::delay(1.secs()).await,
                }
            }

            'retry: loop {
                send(
                    sender,
                    Command::SetOperationMode {
                        mode: OperationMode::Dual,
                        han_sleep: false,
                        channel: Channel::Ch4F922p5MHz,
                        tx_power: TxPower::P20mW,
                    },
                );
                match receiver.recv().await {
                    Ok(Response::SetOperationMode { result: 0x01 }) => break 'retry,
                    _ => Mono::delay(1.secs()).await,
                }
            }

            let scan_result = 'retry: loop {
                send(
                    sender,
                    Command::DoActiveScan {
                        duration: ScanDuration::T616p96ms,
                        mask_channels: 0x3fff0,
                        pairing_id: Some(u64::from_be_bytes(
                            ROUTE_B_ID[24..32].try_into().unwrap(),
                        )),
                    },
                );
                let mut scan_result = None;
                while let Ok(resp) = receiver.recv().await {
                    match resp {
                        Response::DoActiveScan { result: 0x01 } => match scan_result {
                            Some(channel) => break 'retry channel,
                            _ => continue 'retry,
                        },
                        Response::NotificationActiveScan { channel, terminal } => {
                            if !terminal.is_empty() {
                                scan_result.replace(channel);
                            }
                        }
                        _ => continue 'retry,
                    }
                }
            };

            #[cfg(feature = "defmt")]
            defmt::info!("scan_result = {}", scan_result);

            'retry: loop {
                send(
                    sender,
                    Command::SetOperationMode {
                        mode: OperationMode::Dual,
                        han_sleep: false,
                        channel: scan_result,
                        tx_power: TxPower::P20mW,
                    },
                );
                match receiver.recv().await {
                    Ok(Response::SetOperationMode { result: 0x01 }) => break 'retry,
                    _ => Mono::delay(1.secs()).await,
                }
            }

            'retry: loop {
                send(
                    sender,
                    Command::SetRouteBPanaAuthenticationInformation {
                        id: ROUTE_B_ID,
                        password: ROUTE_B_PASSWORD,
                    },
                );
                match receiver.recv().await {
                    Ok(Response::SetRouteBPanaAuthenticationInformation { result: 0x01 }) => {
                        break 'retry
                    }
                    _ => Mono::delay(1.secs()).await,
                }
            }

            'retry: loop {
                send(sender, Command::StartRouteBOperation);
                match receiver.recv().await {
                    Ok(Response::StartRouteBOperation { result: 0x01, .. }) => break 'retry,
                    _ => Mono::delay(1.secs()).await,
                }
            }

            'retry: loop {
                send(sender, Command::OpenUdpPort(0x0e1a));
                match receiver.recv().await {
                    Ok(Response::OpenUdpPort { result: 0x01, .. }) => break 'retry,
                    _ => Mono::delay(1.secs()).await,
                }
            }

            let mac_address = 'retry: loop {
                send(sender, Command::StartRouteBPana);
                while let Ok(resp) = receiver.recv().await {
                    match resp {
                        Response::StartRouteBPana { result: 0x01, .. } => (),
                        Response::NotificationPanaAuthentication {
                            result: 0x01,
                            mac_address,
                        } => break 'retry mac_address,
                        _ => continue 'retry,
                    };
                }
            };

            #[cfg(feature = "defmt")]
            defmt::info!("mac_address = {}", mac_address);

            let destination_address = 0xfe800000000000000000000000000000_u128
                | (mac_address ^ 0x02000000_00000000) as u128;

            for tid in 0.. {
                let tid_be = ((tid & 0xffff) as u16).to_be_bytes();
                'retry: loop {
                    send(
                        sender,
                        Command::TransmitData {
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
                        },
                    );
                    match receiver.recv().await {
                        Ok(Response::TransmitData { result: 0x01, .. }) => break 'retry,
                        _ => Mono::delay(1.secs()).await,
                    }
                }

                let data_expected = [
                    0x10, // EHD1
                    0x81, // EHD2
                    tid_be[0], tid_be[1], // TID
                    0x02, 0x88, 0x01, // DEOJ
                    0x05, 0xff, 0x01, // SEOJ
                ];
                while let Ok(resp) = receiver.recv().await {
                    match resp {
                        Response::NotificationUdpReceived {
                            source_port: 0x0e1a,
                            destination_port: 0x0e1a,
                            data,
                            ..
                        } if data.starts_with(&data_expected) => break,
                        _ => (),
                    };
                }

                Mono::delay(10.secs()).await;
            }
        }
    }

    #[task(
        priority = 2,
        local = [spi0, spi0_csn_adt7310, uart0]
    )]
    async fn task_adt7310(ctx: task_adt7310::Context) {
        let spi = ctx.local.spi0;
        let spi_csn = ctx.local.spi0_csn_adt7310;
        let uart = ctx.local.uart0;

        {
            spi_csn.set_low().unwrap();
            spi.write(&[0xff_u8; 4]).unwrap();
            spi_csn.set_high().unwrap();
            Mono::delay(1.millis()).await;
        }

        loop {
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

            let temp = i16::from_be_bytes([buf[1], buf[2]]);
            let temp_int = temp / 128;
            let temp_frac = 78125_u32 * (temp % 128).unsigned_abs() as u32;
            write!(
                uart,
                "\r\ntemp = {:3}.{:07} ({:#06x})",
                temp_int, temp_frac, temp
            )
            .unwrap();

            spi_csn.set_high().unwrap();

            Mono::delay(760.millis()).await;
        }
    }

    #[task(
        priority = 2,
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

    #[task(
        priority = 1,
        binds = UART1_IRQ,
        shared = [uart1_tx, uart1_tx_receiver],
        local = [uart1_rx, uart1_rx_sender, bp35c0_j11_parser]
    )]
    fn uart1_irq(ctx: uart1_irq::Context) {
        (ctx.shared.uart1_tx, ctx.shared.uart1_tx_receiver).lock(|uart, tx_queue| {
            while let Some(&c) = tx_queue.peek() {
                match uart.write(&[c; 1]) {
                    Ok(1) => tx_queue.dequeue().unwrap(),
                    _ => break,
                };
            }
        });

        {
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
}
