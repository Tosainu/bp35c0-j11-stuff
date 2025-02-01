#![no_std]
#![no_main]

use core::cell::RefCell;
use core::ops::DerefMut;

extern crate alloc;
use alloc::boxed::Box;
use alloc::{vec, vec::Vec};

use rp2040_hal as hal;

use hal::{
    fugit::{ExtU64, RateExtU32},
    timer::Instant,
    uart::{DataBits, StopBits, UartConfig, UartPeripheral},
    usb::UsbBus,
    Clock,
};

use embedded_alloc::Heap;
use embedded_hal::digital::OutputPin;
use embedded_io::Write;

use usb_device::class_prelude::*;
use usb_device::device::{StringDescriptors, UsbDeviceBuilder, UsbDeviceState, UsbVidPid};

use bp35c0_j11::*;
use route_b_secrets::*;

#[global_allocator]
static ALLOCATOR: Heap = Heap::empty();

#[link_section = ".boot2"]
#[no_mangle]
#[used]
static BOOT2_FIRMWARE: [u8; 256] = {
    #[cfg(feature = "adafruit-feather-rp2040")]
    {
        rp2040_boot2::BOOT_LOADER_GD25Q64CS
    }
    #[cfg(feature = "rp-pico")]
    {
        rp2040_boot2::BOOT_LOADER_W25Q080
    }
};

const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;

#[cfg(not(feature = "defmt"))]
#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

enum State<'a> {
    Start,
    ReleaseReset,
    GetVersionInformation,
    SetOperationModeForActiveScan,
    ActiveScan,
    SetOperationModeForRouteB(Channel),
    SetRouteBPanaAuthenticationInformation,
    StartRouteBOperation,
    OpenUdpPort,
    StartRouteBPana,
    Loop(u64),

    Wait(Instant, Box<dyn FnOnce() -> State<'a> + 'a>),
    WaitForResponse(Box<dyn FnMut(Response) -> Option<State<'a>> + 'a>),
    SendCommand {
        buf: Vec<u8>,
        sent: usize,
        next_state: Box<State<'a>>,
    },
}

fn configure_pins(
    pins: hal::gpio::Pins,
) -> (
    impl hal::uart::ValidUartPinout<hal::pac::UART1>,
    impl OutputPin,
    impl OutputPin,
) {
    #[cfg(feature = "adafruit-feather-rp2040")]
    return (
        (pins.gpio24.into_function(), pins.gpio25.into_function()),
        pins.gpio11.into_push_pull_output(),
        pins.gpio10.into_push_pull_output(),
    );

    #[cfg(feature = "rp-pico")]
    return (
        (pins.gpio4.into_function(), pins.gpio5.into_function()),
        pins.gpio10.into_push_pull_output(),
        pins.gpio11.into_push_pull_output(),
    );
}

#[hal::entry]
fn main() -> ! {
    {
        use core::mem::MaybeUninit;
        use core::ptr::addr_of_mut;
        const HEAP_SIZE: usize = 8 * 1024;
        static mut HEAP: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { ALLOCATOR.init(addr_of_mut!(HEAP) as usize, HEAP_SIZE) }
    }

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

    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let (pins_uart, mut pin_reset_n, mut pin_txs0108e_oe) = configure_pins(pins);

    let mut uart = UartPeripheral::new(pac.UART1, pins_uart, &mut pac.RESETS)
        .enable(
            UartConfig::new(115_200.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    let usb_bus = UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    let usb_cdc = RefCell::new(usbd_serial::SerialPort::new(&usb_bus));
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default().product("Serial port")])
        .unwrap()
        .device_class(usbd_serial::USB_CLASS_CDC)
        .build();

    let mut state = State::Start;

    let mut parser = Parser::default();

    loop {
        let _ = usb_dev.poll(&mut [usb_cdc.borrow_mut().deref_mut()]);

        if usb_dev.state() != UsbDeviceState::Configured {
            continue;
        }

        let mut rx_buf = [0; 32];
        if let Ok(len) = uart.read_raw(&mut rx_buf) {
            for resp in rx_buf[0..len].iter().flat_map(|&c| parser.parse(c)) {
                write!(usb_cdc.borrow_mut(), "[+] Rx: {:#x?}\r\n", resp).unwrap();
                if let State::WaitForResponse(ref mut f) = state {
                    if let Some(new_state) = f(resp) {
                        state = new_state;
                    }
                }
            }
        }

        let send = |cmd, f| {
            write!(usb_cdc.borrow_mut(), "[+] Tx: {:#x?}\r\n", cmd).unwrap();
            let mut buf = vec![0; 128];
            let len = serialize_to_bytes(&cmd, &mut buf).unwrap();
            buf.resize(len, 0);
            State::SendCommand {
                buf,
                sent: 0,
                next_state: Box::new(State::WaitForResponse(f)),
            }
        };

        let tick = timer.get_counter();
        match state {
            State::Start => {
                write!(
                    usb_cdc.borrow_mut(),
                    "\r\n\
                    [+] ================================\r\n\
                    [+] (✗╹◡╹)ﾉ {}\r\n\
                    [+] ================================\r\n\
                    \r\n",
                    env!("CARGO_BIN_NAME")
                )
                .unwrap();

                pin_txs0108e_oe.set_high().unwrap();
                state = State::Wait(tick + 500.millis(), Box::new(|| State::ReleaseReset));
            }

            State::ReleaseReset => {
                pin_reset_n.set_high().unwrap();
                state = State::WaitForResponse(Box::new(|resp| match resp {
                    Response::NotificationPoweredOn => {
                        write!(usb_cdc.borrow_mut(), "[+] Ready\r\n").unwrap();
                        Some(State::GetVersionInformation)
                    }
                    _ => None,
                }));
            }

            State::GetVersionInformation => {
                state = send(
                    Command::GetVersionInformation,
                    Box::new(|resp| match resp {
                        Response::GetVersionInformation { .. } => {
                            Some(State::SetOperationModeForActiveScan)
                        }
                        _ => None,
                    }),
                );
            }

            State::SetOperationModeForActiveScan => {
                state = send(
                    Command::SetOperationMode {
                        mode: OperationMode::Dual,
                        han_sleep: false,
                        channel: Channel::Ch4F922p5MHz,
                        tx_power: TxPower::P20mW,
                    },
                    Box::new(|resp| match resp {
                        Response::SetOperationMode { .. } => Some(State::ActiveScan),
                        _ => None,
                    }),
                );
            }

            State::ActiveScan => {
                let mut scan_result = None;
                state = send(
                    Command::DoActiveScan {
                        duration: ScanDuration::T616p96ms,
                        mask_channels: 0x3fff0,
                        pairing_id: Some(u64::from_be_bytes(
                            ROUTE_B_ID[24..32].try_into().unwrap(),
                        )),
                    },
                    Box::new(move |resp| match resp {
                        Response::DoActiveScan { .. } => match scan_result.take() {
                            Some(channel) => Some(State::SetOperationModeForRouteB(channel)),
                            _ => Some(State::ActiveScan),
                        },
                        Response::NotificationActiveScan { channel, terminal } => {
                            if !terminal.is_empty() {
                                scan_result.replace(channel);
                            }
                            None
                        }
                        _ => None,
                    }),
                );
            }

            State::SetOperationModeForRouteB(channel) => {
                state = send(
                    Command::SetOperationMode {
                        mode: OperationMode::Dual,
                        han_sleep: false,
                        channel,
                        tx_power: TxPower::P20mW,
                    },
                    Box::new(|resp| match resp {
                        Response::SetOperationMode { .. } => {
                            Some(State::SetRouteBPanaAuthenticationInformation)
                        }
                        _ => None,
                    }),
                );
            }

            State::SetRouteBPanaAuthenticationInformation => {
                state = send(
                    Command::SetRouteBPanaAuthenticationInformation {
                        id: ROUTE_B_ID,
                        password: ROUTE_B_PASSWORD,
                    },
                    Box::new(|resp| match resp {
                        Response::SetRouteBPanaAuthenticationInformation { .. } => {
                            Some(State::StartRouteBOperation)
                        }
                        _ => None,
                    }),
                );
            }

            State::StartRouteBOperation => {
                state = send(
                    Command::StartRouteBOperation,
                    Box::new(|resp| match resp {
                        Response::StartRouteBOperation { result, .. } => {
                            if result == 0x01 {
                                Some(State::OpenUdpPort)
                            } else {
                                Some(State::StartRouteBOperation)
                            }
                        }
                        _ => None,
                    }),
                );
            }

            State::OpenUdpPort => {
                state = send(
                    Command::OpenUdpPort(0x0e1a),
                    Box::new(|resp| match resp {
                        Response::OpenUdpPort { .. } => Some(State::StartRouteBPana),
                        _ => None,
                    }),
                );
            }

            State::StartRouteBPana => {
                state = send(
                    Command::StartRouteBPana,
                    Box::new(|resp| match resp {
                        Response::StartRouteBPana { .. } => None,
                        Response::NotificationPanaAuthentication {
                            result: _,
                            mac_address,
                        } => Some(State::Loop(mac_address)),
                        _ => None,
                    }),
                );
            }

            State::Loop(mac_address) => {
                let destination_address = 0xfe800000000000000000000000000000_u128
                    | (mac_address ^ 0x02000000_00000000) as u128;
                state = State::Wait(
                    tick + 10.secs(),
                    Box::new(move || {
                        send(
                            Command::TransmitData {
                                destination_address,
                                source_port: 0x0e1a,
                                destination_port: 0x0e1a,
                                data: &[
                                    0x10, // EHD1
                                    0x81, // EHD2
                                    0x00, 0x06, // TID
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
                            Box::new(move |resp| match resp {
                                Response::TransmitData { .. } => Some(State::Loop(mac_address)),
                                _ => None,
                            }),
                        )
                    }),
                );
            }

            State::Wait(t, next_state) if tick >= t => {
                state = next_state();
            }

            State::SendCommand {
                buf,
                sent,
                next_state,
            } => match uart.write(&buf[sent..]) {
                Ok(len) => {
                    if buf.len() >= sent + len {
                        state = *next_state;
                    } else {
                        state = State::SendCommand {
                            buf,
                            sent: sent + len,
                            next_state,
                        };
                    }
                }
                _ => todo!(),
            },

            _ => (),
        }
    }
}
