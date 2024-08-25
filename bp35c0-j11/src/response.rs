use crate::*;

use alloc::vec::Vec;

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TerminalInformation {
    mac_address: u64,
    pan_id: u16,
    rssi: i8,
}

#[derive(Clone, Eq, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Response {
    GetStatus {
        result: u8,
        global_block: GlobalBlockStatus,
        route_b_block: RouteBBlockStatus,
        han_block: HanBlockStatus,
    },
    GetOperationMode {
        result: u8,
        mode: OperationMode,
        han_sleep: bool,
        channel: Channel,
        tx_power: TxPower,
    },

    SetOperationMode {
        result: u8,
    },
    SetNeighborDiscoverySettings {
        result: u8,
    },
    SetUartSettings {
        result: u8,
    },

    OpenUdpPort {
        result: u8,
    },
    CloseUdpPort {
        result: u8,
    },
    TransmitData {
        result: u8,
        #[cfg_attr(feature = "defmt", defmt(Debug2Format))]
        result_udp: Option<(u8, Vec<u8>)>,
    },
    DoActiveScan {
        result: u8,
    },

    NotificationActiveScan {
        channel: Channel,
        #[cfg_attr(feature = "defmt", defmt(Debug2Format))]
        terminal: Vec<TerminalInformation>,
    },

    GetVersionInformation {
        result: u8,
        firmware_id: u16,
        major: u8,
        minor: u8,
        revision: u32,
    },

    NotificationUdpReceived {
        source_address: u128,
        source_port: u16,
        destination_port: u16,
        source_pan_id: u16,
        source_type: u8,
        encryption: u8,
        rssi: i8,
        #[cfg_attr(feature = "defmt", defmt(Debug2Format))]
        data: Vec<u8>,
    },
    NotificationPoweredOn,
    NotificationConnectionChanged {
        state: u8,
        mac_address: u64,
        rssi: i8,
    },
    NotificationPanaAuthentication {
        result: u8,
        mac_address: u64,
    },

    GetRouteBEncryptionKey {
        result: u8,
        encryption_key: [u8; 16],
    },
    GetRouteBPanId {
        result: u8,
        pan_id: u16,
    },

    SetRouteBPanaAuthenticationInformation {
        result: u8,
    },

    StartRouteBOperation {
        result: u8,
        terminal: Option<(Channel, TerminalInformation)>,
    },
    StartRouteBPana {
        result: u8,
    },
    TerminateRouteBPana {
        result: u8,
    },
    TerminateRouteBOperation {
        result: u8,
    },
    RedoRouteBPanaAuthentication {
        result: u8,
    },

    Unknown {
        code: u16,
        #[cfg_attr(feature = "defmt", defmt(Debug2Format))]
        data: Vec<u8>,
    },
}

fn make_response(code: u16, data: &[u8]) -> Response {
    match (code, data) {
        (0x2001, _) if data.len() == 4 => Response::GetStatus {
            result: data[0],
            global_block: data[1].into(),
            route_b_block: data[2].into(),
            han_block: data[3].into(),
        },

        (0x2107, _) if data.len() == 5 => Response::GetOperationMode {
            result: data[0],
            mode: data[1].into(),
            han_sleep: data[2] != 0,
            channel: data[3].into(),
            tx_power: data[4].into(),
        },

        (0x205f, [d0]) => Response::SetOperationMode { result: *d0 },

        (0x2101, [d0]) => Response::SetNeighborDiscoverySettings { result: *d0 },

        (0x210a, [d0]) => Response::SetUartSettings { result: *d0 },

        (0x2005, [d0]) => Response::OpenUdpPort { result: *d0 },

        (0x2006, [d0]) => Response::CloseUdpPort { result: *d0 },

        (0x2008, [result @ 0x01, result_udp, data @ ..]) if !data.is_empty() => {
            Response::TransmitData {
                result: *result,
                result_udp: Some((*result_udp, data.into())),
            }
        }

        (0x2008, [result]) => Response::TransmitData {
            result: *result,
            result_udp: None,
        },

        (0x2051, [d0]) => Response::DoActiveScan { result: *d0 },

        (0x4051, [0x00, channel, n, data @ ..]) if data.len() == 11_usize * *n as usize => {
            Response::NotificationActiveScan {
                channel: (*channel).into(),
                terminal: data
                    .chunks_exact(11)
                    .map(|b| TerminalInformation {
                        mac_address: u64::from_be_bytes(b[0..8].try_into().unwrap()),
                        pan_id: u16::from_be_bytes(b[8..10].try_into().unwrap()),
                        rssi: b[10] as _,
                    })
                    .collect(),
            }
        }

        (0x4051, [0x01, channel]) => Response::NotificationActiveScan {
            channel: (*channel).into(),
            terminal: Vec::new(),
        },

        (0x206b, _) if data.len() == 9 => {
            let result = data[0];
            let firmware_id = u16::from_be_bytes(data[1..3].try_into().unwrap());
            let major = data[3];
            let minor = data[4];
            let revision = u32::from_be_bytes(data[5..9].try_into().unwrap());
            Response::GetVersionInformation {
                result,
                firmware_id,
                major,
                minor,
                revision,
            }
        }

        (0x6018, _)
            if data
                .get(25..27)
                .and_then(|s| s.try_into().ok())
                .map(|s| u16::from_be_bytes(s) as usize + 27)
                == Some(data.len()) =>
        {
            Response::NotificationUdpReceived {
                source_address: u128::from_be_bytes(data[0..16].try_into().unwrap()),
                source_port: u16::from_be_bytes(data[16..18].try_into().unwrap()),
                destination_port: u16::from_be_bytes(data[18..20].try_into().unwrap()),
                source_pan_id: u16::from_be_bytes(data[20..22].try_into().unwrap()),
                source_type: data[22],
                encryption: data[23],
                rssi: data[24] as _,
                data: data[27..].into(),
            }
        }

        (0x6019, []) => Response::NotificationPoweredOn,

        (0x601a, _) if data.len() == 10 => Response::NotificationConnectionChanged {
            state: data[0],
            mac_address: u64::from_be_bytes(data[1..9].try_into().unwrap()),
            rssi: data[9] as _,
        },

        (0x6028, _) if data.len() == 9 => Response::NotificationPanaAuthentication {
            result: data[0],
            mac_address: u64::from_be_bytes(data[1..9].try_into().unwrap()),
        },

        (0x2059, _) if data.len() == 17 => Response::GetRouteBEncryptionKey {
            result: data[0],
            encryption_key: {
                let mut arr = [0; 16];
                arr.copy_from_slice(&data[1..17]);
                arr
            },
        },

        (0x205e, _) if data.len() == 3 => {
            let result = data[0];
            let pan_id = u16::from_be_bytes(data[1..3].try_into().unwrap());
            Response::GetRouteBPanId { result, pan_id }
        }

        (0x2054, [d0]) if data.len() == 1 => {
            Response::SetRouteBPanaAuthenticationInformation { result: *d0 }
        }

        (0x2053, [result @ 0x01, data @ ..]) if data.len() == 12 => {
            Response::StartRouteBOperation {
                result: *result,
                terminal: Some((
                    data[0].into(),
                    TerminalInformation {
                        pan_id: u16::from_be_bytes(data[1..3].try_into().unwrap()),
                        mac_address: u64::from_be_bytes(data[3..11].try_into().unwrap()),
                        rssi: data[11] as _,
                    },
                )),
            }
        }

        (0x2053, [d0]) => Response::StartRouteBOperation {
            result: *d0,
            terminal: None,
        },

        (0x2056, [d0]) => Response::StartRouteBPana { result: *d0 },

        (0x2057, [d0]) => Response::TerminateRouteBPana { result: *d0 },

        (0x2058, [d0]) => Response::TerminateRouteBOperation { result: *d0 },

        (0x20d2, [d0]) => Response::RedoRouteBPanaAuthentication { result: *d0 },

        _ => Response::Unknown {
            code,
            data: Vec::from(data),
        },
    }
}

#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum State {
    UniqueCode(u8),
    Header {
        ptr: usize,
    },
    Data {
        ptr: usize,
        code: u16,
        sum: u16,
        sum_current: u16,
        len: usize,
    },
    Ready {
        len: usize,
        code: u16,
    },
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Parser {
    buffer: [u8; 2048],
    state: State,
}

const UNIQUE_CODE0: u8 = 0xd0;
const UNIQUE_CODE1: u8 = 0xf9;
const UNIQUE_CODE2: u8 = 0xee;
const UNIQUE_CODE3: u8 = 0x5d;

const HEADER_SUM_INIT: u16 =
    UNIQUE_CODE0 as u16 + UNIQUE_CODE1 as u16 + UNIQUE_CODE2 as u16 + UNIQUE_CODE3 as u16;

impl Default for State {
    fn default() -> Self {
        State::UniqueCode(UNIQUE_CODE0)
    }
}

impl State {
    fn next(self, buffer: &mut [u8], c: u8) -> State {
        match self {
            State::Ready { .. } => {
                if c == UNIQUE_CODE0 {
                    State::UniqueCode(UNIQUE_CODE1)
                } else {
                    State::default()
                }
            }

            State::UniqueCode(expected) => match (c, expected) {
                (UNIQUE_CODE0, UNIQUE_CODE0) => State::UniqueCode(UNIQUE_CODE1),
                (UNIQUE_CODE1, UNIQUE_CODE1) => State::UniqueCode(UNIQUE_CODE2),
                (UNIQUE_CODE2, UNIQUE_CODE2) => State::UniqueCode(UNIQUE_CODE3),
                (UNIQUE_CODE3, UNIQUE_CODE3) => State::Header { ptr: 0 },
                _ => State::default(),
            },

            State::Header { ptr } if ptr + 1 < 8 => {
                buffer[ptr] = c;
                State::Header { ptr: ptr + 1 }
            }

            State::Header { ptr } => {
                buffer[ptr] = c;

                let sum_header = u16::from_be_bytes([buffer[4], buffer[5]]);
                if buffer[..4]
                    .iter()
                    .fold(HEADER_SUM_INIT, |acc, &x| acc + x as u16)
                    != sum_header
                {
                    return State::default();
                }

                let len = u16::from_be_bytes([buffer[2], buffer[3]]);
                if len < 4 || len as usize > buffer.len() {
                    return State::default();
                }

                let code = u16::from_be_bytes([buffer[0], buffer[1]]);
                let sum_data = u16::from_be_bytes([buffer[6], buffer[7]]);
                if len == 4 && sum_data == 0 {
                    State::Ready { len: 0, code }
                } else {
                    State::Data {
                        ptr: 0,
                        code,
                        sum: sum_data,
                        sum_current: 0,
                        len: (len - 4).into(),
                    }
                }
            }

            State::Data {
                ptr,
                code,
                sum,
                sum_current,
                len,
            } if ptr + 1 < len => {
                buffer[ptr] = c;
                State::Data {
                    ptr: ptr + 1,
                    code,
                    sum,
                    sum_current: sum_current + c as u16,
                    len,
                }
            }

            State::Data {
                ptr,
                code,
                sum,
                sum_current,
                len,
            } => {
                buffer[ptr] = c;
                if sum_current + c as u16 != sum {
                    return State::default();
                }
                State::Ready { len, code }
            }
        }
    }
}

impl Default for Parser {
    fn default() -> Self {
        Parser {
            buffer: [0; 2048],
            state: State::default(),
        }
    }
}

impl Parser {
    pub fn parse(&mut self, c: u8) -> Option<Response> {
        self.state = self.state.next(&mut self.buffer, c);
        match self.state {
            State::Ready { len, code } => Some(make_response(code, &self.buffer[0..len])),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use crate::response::*;
    use alloc::vec;

    #[test]
    fn test_parser() {
        let mut p = Parser::default();

        assert_eq!(p.parse(0xd0), None);
        assert_eq!(p.parse(0xf9), None);
        assert_eq!(p.parse(0xee), None);
        assert_eq!(p.parse(0x5d), None);
        assert_eq!(p.parse(0x20), None);
        assert_eq!(p.parse(0x01), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0x08), None);
        assert_eq!(p.parse(0x03), None);
        assert_eq!(p.parse(0x3d), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0x07), None);
        assert_eq!(p.parse(0x01), None);
        assert_eq!(p.parse(0x02), None);
        assert_eq!(p.parse(0x01), None);
        assert_eq!(
            p.parse(0x03),
            Some(Response::GetStatus {
                result: 0x01,
                global_block: GlobalBlockStatus::Inactive,
                route_b_block: RouteBBlockStatus::Inactive,
                han_block: HanBlockStatus::Authentication,
            })
        );

        assert_eq!(p.parse(0xd0), None);
        assert_eq!(p.parse(0xf9), None);
        assert_eq!(p.parse(0xee), None);
        assert_eq!(p.parse(0x5d), None);
        assert_eq!(p.parse(0x20), None);
        assert_eq!(p.parse(0x5f), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0x05), None);
        assert_eq!(p.parse(0x03), None);
        assert_eq!(p.parse(0x98), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0x01), None);
        assert_eq!(
            p.parse(0x01),
            Some(Response::SetOperationMode { result: 0x01 })
        );

        assert_eq!(p.parse(0xd0), None);
        assert_eq!(p.parse(0xf9), None);
        assert_eq!(p.parse(0xee), None);
        assert_eq!(p.parse(0x5d), None);
        assert_eq!(p.parse(0x21), None);
        assert_eq!(p.parse(0x01), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0x05), None);
        assert_eq!(p.parse(0x03), None);
        assert_eq!(p.parse(0x3b), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0x01), None);
        assert_eq!(
            p.parse(0x01),
            Some(Response::SetNeighborDiscoverySettings { result: 0x01 })
        );

        assert_eq!(p.parse(0xd0), None);
        assert_eq!(p.parse(0xf9), None);
        assert_eq!(p.parse(0xee), None);
        assert_eq!(p.parse(0x5d), None);
        assert_eq!(p.parse(0x21), None);
        assert_eq!(p.parse(0x0a), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0x05), None);
        assert_eq!(p.parse(0x03), None);
        assert_eq!(p.parse(0x44), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0x01), None);
        assert_eq!(
            p.parse(0x01),
            Some(Response::SetUartSettings { result: 0x01 })
        );

        assert_eq!(p.parse(0xd0), None);
        assert_eq!(p.parse(0xf9), None);
        assert_eq!(p.parse(0xee), None);
        assert_eq!(p.parse(0x5d), None);
        assert_eq!(p.parse(0x20), None);
        assert_eq!(p.parse(0x05), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0x05), None);
        assert_eq!(p.parse(0x03), None);
        assert_eq!(p.parse(0x3e), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0x01), None);
        assert_eq!(p.parse(0x01), Some(Response::OpenUdpPort { result: 0x01 }));

        assert_eq!(p.parse(0xd0), None);
        assert_eq!(p.parse(0xf9), None);
        assert_eq!(p.parse(0xee), None);
        assert_eq!(p.parse(0x5d), None);
        assert_eq!(p.parse(0x20), None);
        assert_eq!(p.parse(0x06), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0x05), None);
        assert_eq!(p.parse(0x03), None);
        assert_eq!(p.parse(0x3f), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0x01), None);
        assert_eq!(p.parse(0x01), Some(Response::CloseUdpPort { result: 0x01 }));

        assert_eq!(p.parse(0xd0), None);
        assert_eq!(p.parse(0xf9), None);
        assert_eq!(p.parse(0xee), None);
        assert_eq!(p.parse(0x5d), None);
        assert_eq!(p.parse(0x20), None);
        assert_eq!(p.parse(0x08), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0x05), None);
        assert_eq!(p.parse(0x03), None);
        assert_eq!(p.parse(0x41), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0x02), None);
        assert_eq!(
            p.parse(0x02),
            Some(Response::TransmitData {
                result: 0x02,
                result_udp: None,
            })
        );

        assert_eq!(p.parse(0xd0), None);
        assert_eq!(p.parse(0xf9), None);
        assert_eq!(p.parse(0xee), None);
        assert_eq!(p.parse(0x5d), None);
        assert_eq!(p.parse(0x20), None);
        assert_eq!(p.parse(0x08), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0x09), None);
        assert_eq!(p.parse(0x03), None);
        assert_eq!(p.parse(0x45), None);
        assert_eq!(p.parse(0x02), None);
        assert_eq!(p.parse(0x7a), None);
        assert_eq!(p.parse(0x01), None);
        assert_eq!(p.parse(0x12), None);
        assert_eq!(p.parse(0xab), None);
        assert_eq!(p.parse(0xcd), None);
        assert_eq!(
            p.parse(0xef),
            Some(Response::TransmitData {
                result: 0x01,
                result_udp: Some((0x12, vec![0xab, 0xcd, 0xef])),
            })
        );

        assert_eq!(p.parse(0xd0), None);
        assert_eq!(p.parse(0xf9), None);
        assert_eq!(p.parse(0xee), None);
        assert_eq!(p.parse(0x5d), None);
        assert_eq!(p.parse(0x20), None);
        assert_eq!(p.parse(0x51), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0x05), None);
        assert_eq!(p.parse(0x03), None);
        assert_eq!(p.parse(0x8a), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0x01), None);
        assert_eq!(p.parse(0x01), Some(Response::DoActiveScan { result: 0x01 }));

        assert_eq!(p.parse(0xd0), None);
        assert_eq!(p.parse(0xf9), None);
        assert_eq!(p.parse(0xee), None);
        assert_eq!(p.parse(0x5d), None);
        assert_eq!(p.parse(0x40), None);
        assert_eq!(p.parse(0x51), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0x06), None);
        assert_eq!(p.parse(0x03), None);
        assert_eq!(p.parse(0xab), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0x08), None);
        assert_eq!(p.parse(0x01), None);
        assert_eq!(
            p.parse(0x07),
            Some(Response::NotificationActiveScan {
                channel: Channel::Ch7F923p7MHz,
                terminal: Vec::new()
            })
        );

        assert_eq!(p.parse(0xd0), None);
        assert_eq!(p.parse(0xf9), None);
        assert_eq!(p.parse(0xee), None);
        assert_eq!(p.parse(0x5d), None);
        assert_eq!(p.parse(0x40), None);
        assert_eq!(p.parse(0x51), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0x1d), None);
        assert_eq!(p.parse(0x03), None);
        assert_eq!(p.parse(0xc2), None);
        assert_eq!(p.parse(0x05), None);
        assert_eq!(p.parse(0x1e), None);
        assert_eq!(p.parse(0x00), None); // result
        assert_eq!(p.parse(0x0c), None); // channel
        assert_eq!(p.parse(0x02), None); // # of scans
        assert_eq!(p.parse(0x00), None); // terminal[0].mac_address
        assert_eq!(p.parse(0x01), None);
        assert_eq!(p.parse(0x02), None);
        assert_eq!(p.parse(0x03), None);
        assert_eq!(p.parse(0x04), None);
        assert_eq!(p.parse(0x05), None);
        assert_eq!(p.parse(0x06), None);
        assert_eq!(p.parse(0x07), None);
        assert_eq!(p.parse(0xab), None); // terminal[0].pan_id
        assert_eq!(p.parse(0xcd), None);
        assert_eq!(p.parse(0xde), None); // terminal[0].rssi
        assert_eq!(p.parse(0x00), None); // terminal[1].mac_address
        assert_eq!(p.parse(0x10), None);
        assert_eq!(p.parse(0x20), None);
        assert_eq!(p.parse(0x30), None);
        assert_eq!(p.parse(0x40), None);
        assert_eq!(p.parse(0x50), None);
        assert_eq!(p.parse(0x60), None);
        assert_eq!(p.parse(0x70), None);
        assert_eq!(p.parse(0x12), None); // terminal[1].pan_id
        assert_eq!(p.parse(0x34), None);
        assert_eq!(
            p.parse(0x98), // terminal[1].rssi
            Some(Response::NotificationActiveScan {
                channel: Channel::Ch12F925p7MHz,
                terminal: vec![
                    TerminalInformation {
                        mac_address: 0x0001020304050607,
                        pan_id: 0xabcd,
                        rssi: 0xde_u8 as i8,
                    },
                    TerminalInformation {
                        mac_address: 0x0010203040506070,
                        pan_id: 0x1234,
                        rssi: 0x98_u8 as i8,
                    },
                ],
            })
        );

        assert_eq!(p.parse(0xd0), None);
        assert_eq!(p.parse(0xf9), None);
        assert_eq!(p.parse(0xee), None);
        assert_eq!(p.parse(0x5d), None);
        assert_eq!(p.parse(0x20), None);
        assert_eq!(p.parse(0x6b), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0x0d), None);
        assert_eq!(p.parse(0x03), None);
        assert_eq!(p.parse(0xac), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0xa8), None);
        assert_eq!(p.parse(0x01), None);
        assert_eq!(p.parse(0x04), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0x01), None);
        assert_eq!(p.parse(0x03), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0x15), None);
        assert_eq!(
            p.parse(0x8a),
            Some(Response::GetVersionInformation {
                result: 0x01,
                firmware_id: 0x400,
                major: 0x01,
                minor: 0x03,
                revision: 0x0000158a,
            })
        );

        assert_eq!(p.parse(0xd0), None);
        assert_eq!(p.parse(0xf9), None);
        assert_eq!(p.parse(0xee), None);
        assert_eq!(p.parse(0x5d), None);
        assert_eq!(p.parse(0x60), None);
        assert_eq!(p.parse(0x18), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0x23), None);
        assert_eq!(p.parse(0x03), None);
        assert_eq!(p.parse(0xaf), None);
        assert_eq!(p.parse(0x06), None);
        assert_eq!(p.parse(0xde), None);
        assert_eq!(p.parse(0xfe), None); // source_address
        assert_eq!(p.parse(0x80), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0x01), None);
        assert_eq!(p.parse(0x02), None);
        assert_eq!(p.parse(0x03), None);
        assert_eq!(p.parse(0x04), None);
        assert_eq!(p.parse(0x05), None);
        assert_eq!(p.parse(0x06), None);
        assert_eq!(p.parse(0x07), None);
        assert_eq!(p.parse(0x08), None);
        assert_eq!(p.parse(0x09), None);
        assert_eq!(p.parse(0x0a), None);
        assert_eq!(p.parse(0x0b), None);
        assert_eq!(p.parse(0x0c), None);
        assert_eq!(p.parse(0x0d), None);
        assert_eq!(p.parse(0xab), None); // source_port
        assert_eq!(p.parse(0xcd), None);
        assert_eq!(p.parse(0x12), None); // destination_port
        assert_eq!(p.parse(0x34), None);
        assert_eq!(p.parse(0x56), None); // source_pan_id
        assert_eq!(p.parse(0x78), None);
        assert_eq!(p.parse(0x01), None); // source_type
        assert_eq!(p.parse(0x02), None); // encryption
        assert_eq!(p.parse(0xde), None); // rssi
        assert_eq!(p.parse(0x00), None); // len
        assert_eq!(p.parse(0x04), None);
        assert_eq!(p.parse(0x98), None); // data
        assert_eq!(p.parse(0x76), None);
        assert_eq!(p.parse(0x54), None);
        assert_eq!(
            p.parse(0x32),
            Some(Response::NotificationUdpReceived {
                source_address: 0xfe80000102030405060708090a0b0c0d,
                source_port: 0xabcd,
                destination_port: 0x1234,
                source_pan_id: 0x5678,
                source_type: 0x01,
                encryption: 0x02,
                rssi: 0xde_u8 as i8,
                data: vec![0x98, 0x76, 0x54, 0x32],
            })
        );

        assert_eq!(p.parse(0xd0), None);
        assert_eq!(p.parse(0xf9), None);
        assert_eq!(p.parse(0xee), None);
        assert_eq!(p.parse(0x5d), None);
        assert_eq!(p.parse(0x60), None);
        assert_eq!(p.parse(0x19), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0x04), None);
        assert_eq!(p.parse(0x03), None);
        assert_eq!(p.parse(0x91), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0x00), Some(Response::NotificationPoweredOn));

        assert_eq!(p.parse(0xd0), None);
        assert_eq!(p.parse(0xf9), None);
        assert_eq!(p.parse(0xee), None);
        assert_eq!(p.parse(0x5d), None);
        assert_eq!(p.parse(0x60), None);
        assert_eq!(p.parse(0x1a), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0x0e), None);
        assert_eq!(p.parse(0x03), None);
        assert_eq!(p.parse(0x9c), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0xfb), None);
        assert_eq!(p.parse(0x01), None); // state
        assert_eq!(p.parse(0x00), None); // mac_address
        assert_eq!(p.parse(0x01), None);
        assert_eq!(p.parse(0x02), None);
        assert_eq!(p.parse(0x03), None);
        assert_eq!(p.parse(0x04), None);
        assert_eq!(p.parse(0x05), None);
        assert_eq!(p.parse(0x06), None);
        assert_eq!(p.parse(0x07), None);
        assert_eq!(
            p.parse(0xde), // terminal[0].rssi
            Some(Response::NotificationConnectionChanged {
                state: 0x01,
                mac_address: 0x0001020304050607,
                rssi: 0xde_u8 as i8,
            })
        );

        assert_eq!(p.parse(0xd0), None);
        assert_eq!(p.parse(0xf9), None);
        assert_eq!(p.parse(0xee), None);
        assert_eq!(p.parse(0x5d), None);
        assert_eq!(p.parse(0x60), None);
        assert_eq!(p.parse(0x28), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0x0d), None);
        assert_eq!(p.parse(0x03), None);
        assert_eq!(p.parse(0xa9), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0x1d), None);
        assert_eq!(p.parse(0x01), None); // result
        assert_eq!(p.parse(0x00), None); // mac_address
        assert_eq!(p.parse(0x01), None);
        assert_eq!(p.parse(0x02), None);
        assert_eq!(p.parse(0x03), None);
        assert_eq!(p.parse(0x04), None);
        assert_eq!(p.parse(0x05), None);
        assert_eq!(p.parse(0x06), None);
        assert_eq!(
            p.parse(0x07),
            Some(Response::NotificationPanaAuthentication {
                result: 0x01,
                mac_address: 0x0001020304050607,
            })
        );

        assert_eq!(p.parse(0xd0), None);
        assert_eq!(p.parse(0xf9), None);
        assert_eq!(p.parse(0xee), None);
        assert_eq!(p.parse(0x5d), None);
        assert_eq!(p.parse(0x20), None);
        assert_eq!(p.parse(0x59), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0x15), None);
        assert_eq!(p.parse(0x03), None);
        assert_eq!(p.parse(0xa2), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0x79), None);
        assert_eq!(p.parse(0x01), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0x01), None);
        assert_eq!(p.parse(0x02), None);
        assert_eq!(p.parse(0x03), None);
        assert_eq!(p.parse(0x04), None);
        assert_eq!(p.parse(0x05), None);
        assert_eq!(p.parse(0x06), None);
        assert_eq!(p.parse(0x07), None);
        assert_eq!(p.parse(0x08), None);
        assert_eq!(p.parse(0x09), None);
        assert_eq!(p.parse(0x0a), None);
        assert_eq!(p.parse(0x0b), None);
        assert_eq!(p.parse(0x0c), None);
        assert_eq!(p.parse(0x0d), None);
        assert_eq!(p.parse(0x0e), None);
        assert_eq!(
            p.parse(0x0f),
            Some(Response::GetRouteBEncryptionKey {
                result: 0x01,
                encryption_key: [
                    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c,
                    0x0d, 0x0e, 0x0f
                ]
            })
        );

        assert_eq!(p.parse(0xd0), None);
        assert_eq!(p.parse(0xf9), None);
        assert_eq!(p.parse(0xee), None);
        assert_eq!(p.parse(0x5d), None);
        assert_eq!(p.parse(0x20), None);
        assert_eq!(p.parse(0x5e), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0x07), None);
        assert_eq!(p.parse(0x03), None);
        assert_eq!(p.parse(0x99), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0x06), None);
        assert_eq!(p.parse(0x01), None);
        assert_eq!(p.parse(0x02), None);
        assert_eq!(
            p.parse(0x03),
            Some(Response::GetRouteBPanId {
                result: 0x01,
                pan_id: 0x0203,
            })
        );

        assert_eq!(p.parse(0xd0), None);
        assert_eq!(p.parse(0xf9), None);
        assert_eq!(p.parse(0xee), None);
        assert_eq!(p.parse(0x5d), None);
        assert_eq!(p.parse(0x20), None);
        assert_eq!(p.parse(0x54), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0x05), None);
        assert_eq!(p.parse(0x03), None);
        assert_eq!(p.parse(0x8d), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0x01), None);
        assert_eq!(
            p.parse(0x01),
            Some(Response::SetRouteBPanaAuthenticationInformation { result: 0x01 })
        );

        assert_eq!(p.parse(0xd0), None);
        assert_eq!(p.parse(0xf9), None);
        assert_eq!(p.parse(0xee), None);
        assert_eq!(p.parse(0x5d), None);
        assert_eq!(p.parse(0x20), None);
        assert_eq!(p.parse(0x53), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0x05), None);
        assert_eq!(p.parse(0x03), None);
        assert_eq!(p.parse(0x8c), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0x02), None);
        assert_eq!(
            p.parse(0x02),
            Some(Response::StartRouteBOperation {
                result: 0x02,
                terminal: None,
            })
        );

        assert_eq!(p.parse(0xd0), None);
        assert_eq!(p.parse(0xf9), None);
        assert_eq!(p.parse(0xee), None);
        assert_eq!(p.parse(0x5d), None);
        assert_eq!(p.parse(0x20), None);
        assert_eq!(p.parse(0x53), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0x11), None);
        assert_eq!(p.parse(0x03), None);
        assert_eq!(p.parse(0x98), None);
        assert_eq!(p.parse(0x02), None);
        assert_eq!(p.parse(0x7b), None);
        assert_eq!(p.parse(0x01), None);
        assert_eq!(p.parse(0x08), None); // channel
        assert_eq!(p.parse(0xab), None); // pan_id
        assert_eq!(p.parse(0xcd), None);
        assert_eq!(p.parse(0x00), None); // mac_address
        assert_eq!(p.parse(0x01), None);
        assert_eq!(p.parse(0x02), None);
        assert_eq!(p.parse(0x03), None);
        assert_eq!(p.parse(0x04), None);
        assert_eq!(p.parse(0x05), None);
        assert_eq!(p.parse(0x06), None);
        assert_eq!(p.parse(0x07), None);
        assert_eq!(
            p.parse(0xde), // terminal[0].rssi
            Some(Response::StartRouteBOperation {
                result: 0x01,
                terminal: Some((
                    Channel::Ch8F924p1MHz,
                    TerminalInformation {
                        mac_address: 0x0001020304050607,
                        pan_id: 0xabcd,
                        rssi: 0xde_u8 as i8,
                    }
                )),
            })
        );

        assert_eq!(p.parse(0xd0), None);
        assert_eq!(p.parse(0xf9), None);
        assert_eq!(p.parse(0xee), None);
        assert_eq!(p.parse(0x5d), None);
        assert_eq!(p.parse(0x20), None);
        assert_eq!(p.parse(0x56), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0x05), None);
        assert_eq!(p.parse(0x03), None);
        assert_eq!(p.parse(0x8f), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0x01), None);
        assert_eq!(
            p.parse(0x01),
            Some(Response::StartRouteBPana { result: 0x01 })
        );

        assert_eq!(p.parse(0xd0), None);
        assert_eq!(p.parse(0xf9), None);
        assert_eq!(p.parse(0xee), None);
        assert_eq!(p.parse(0x5d), None);
        assert_eq!(p.parse(0x20), None);
        assert_eq!(p.parse(0x57), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0x05), None);
        assert_eq!(p.parse(0x03), None);
        assert_eq!(p.parse(0x90), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0x01), None);
        assert_eq!(
            p.parse(0x01),
            Some(Response::TerminateRouteBPana { result: 0x01 })
        );

        assert_eq!(p.parse(0xd0), None);
        assert_eq!(p.parse(0xf9), None);
        assert_eq!(p.parse(0xee), None);
        assert_eq!(p.parse(0x5d), None);
        assert_eq!(p.parse(0x20), None);
        assert_eq!(p.parse(0x58), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0x05), None);
        assert_eq!(p.parse(0x03), None);
        assert_eq!(p.parse(0x91), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0x01), None);
        assert_eq!(
            p.parse(0x01),
            Some(Response::TerminateRouteBOperation { result: 0x01 })
        );

        assert_eq!(p.parse(0xd0), None);
        assert_eq!(p.parse(0xf9), None);
        assert_eq!(p.parse(0xee), None);
        assert_eq!(p.parse(0x5d), None);
        assert_eq!(p.parse(0x20), None);
        assert_eq!(p.parse(0xd2), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0x05), None);
        assert_eq!(p.parse(0x04), None);
        assert_eq!(p.parse(0x0b), None);
        assert_eq!(p.parse(0x00), None);
        assert_eq!(p.parse(0x01), None);
        assert_eq!(
            p.parse(0x01),
            Some(Response::RedoRouteBPanaAuthentication { result: 0x01 })
        );
    }
}
