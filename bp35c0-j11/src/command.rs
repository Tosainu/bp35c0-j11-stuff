use crate::*;

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Command<'a> {
    GetStatus,
    GetUdpPorts,
    GetIpAddress,
    GetMacAddress,
    GetClientMacAddress,
    GetClientIpAddress,
    GetNeighborDiscoverySettings,
    GetOperationMode,
    GetUartSettings,

    SetOperationMode {
        mode: OperationMode,
        han_sleep: bool,
        channel: Channel,
        tx_power: TxPower,
    },
    SetNeighborDiscoverySettings {
        enable: bool,
    },
    SetUartSettings {
        flow_control: bool,
    },

    OpenUdpPort(u16),
    CloseUdpPort(u16),
    TransmitData {
        destination_address: u128,
        source_port: u16,
        destination_port: u16,
        data: &'a [u8],
    },
    DoActiveScan {
        duration: ScanDuration,
        mask_channels: u32,
        pairing_id: Option<u64>,
    },

    GetVersionInformation,

    SetRouteBPanaAuthenticationInformation {
        id: &'a [u8],
        password: &'a [u8],
    },

    StartRouteBOperation,
    StartRouteBPana,
    TerminateRouteBPana,
    TerminateRouteBOperation,
    RedoRouteBPanaAuthentication,
}

#[allow(clippy::result_unit_err)] // TODO:
pub fn serialize_to_bytes<'a>(cmd: &'a Command<'a>, buf: &mut [u8]) -> Result<usize, ()> {
    let mut total_len = COMMAND_RANGE_DATA.start;

    if buf.len() < total_len {
        return Err(());
    }

    buf[COMMAND_RANGE_UNIQUE_CODE].copy_from_slice(&0xd0ea83fc_u32.to_be_bytes());

    match cmd {
        Command::GetStatus => {
            buf[COMMAND_RANGE_COMMAND_CODE].copy_from_slice(&0x0001_u16.to_be_bytes())
        }

        Command::GetUdpPorts => {
            buf[COMMAND_RANGE_COMMAND_CODE].copy_from_slice(&0x0007_u16.to_be_bytes())
        }

        Command::GetIpAddress => {
            buf[COMMAND_RANGE_COMMAND_CODE].copy_from_slice(&0x0009_u16.to_be_bytes())
        }

        Command::GetMacAddress => {
            buf[COMMAND_RANGE_COMMAND_CODE].copy_from_slice(&0x000e_u16.to_be_bytes())
        }

        Command::GetClientMacAddress => {
            buf[COMMAND_RANGE_COMMAND_CODE].copy_from_slice(&0x0011_u16.to_be_bytes())
        }

        Command::GetClientIpAddress => {
            buf[COMMAND_RANGE_COMMAND_CODE].copy_from_slice(&0x0100_u16.to_be_bytes())
        }

        Command::GetNeighborDiscoverySettings => {
            buf[COMMAND_RANGE_COMMAND_CODE].copy_from_slice(&0x0102_u16.to_be_bytes())
        }

        Command::GetOperationMode => {
            buf[COMMAND_RANGE_COMMAND_CODE].copy_from_slice(&0x0107_u16.to_be_bytes())
        }

        Command::GetUartSettings => {
            buf[COMMAND_RANGE_COMMAND_CODE].copy_from_slice(&0x010b_u16.to_be_bytes())
        }

        Command::SetOperationMode {
            mode,
            han_sleep,
            channel,
            tx_power,
        } => {
            let data = [
                u8::from(*mode),
                if *han_sleep { 0x01 } else { 0x00 },
                u8::from(*channel),
                u8::from(*tx_power),
            ];
            total_len += data.len();
            if buf.len() < total_len {
                return Err(());
            }
            buf[COMMAND_RANGE_COMMAND_CODE].copy_from_slice(&0x005f_u16.to_be_bytes());
            buf[COMMAND_RANGE_DATA.start..COMMAND_RANGE_DATA.start + data.len()]
                .copy_from_slice(&data);
        }

        Command::SetNeighborDiscoverySettings { enable } => {
            let data = [*enable as _];
            total_len += data.len();
            if buf.len() < total_len {
                return Err(());
            }
            buf[COMMAND_RANGE_COMMAND_CODE].copy_from_slice(&0x0101_u16.to_be_bytes());
            buf[COMMAND_RANGE_DATA.start..COMMAND_RANGE_DATA.start + data.len()]
                .copy_from_slice(&data);
        }

        Command::SetUartSettings { flow_control } => {
            let data = [*flow_control as _];
            total_len += data.len();
            if buf.len() < total_len {
                return Err(());
            }
            buf[COMMAND_RANGE_COMMAND_CODE].copy_from_slice(&0x010a_u16.to_be_bytes());
            buf[COMMAND_RANGE_DATA.start..COMMAND_RANGE_DATA.start + data.len()]
                .copy_from_slice(&data);
        }

        Command::OpenUdpPort(port) => {
            let data = port.to_be_bytes();
            total_len += data.len();
            if buf.len() < total_len {
                return Err(());
            }
            buf[COMMAND_RANGE_COMMAND_CODE].copy_from_slice(&0x0005_u16.to_be_bytes());
            buf[COMMAND_RANGE_DATA.start..COMMAND_RANGE_DATA.start + data.len()]
                .copy_from_slice(&data);
        }

        Command::CloseUdpPort(port) => {
            let data = port.to_be_bytes();
            total_len += data.len();
            if buf.len() < total_len {
                return Err(());
            }
            buf[COMMAND_RANGE_COMMAND_CODE].copy_from_slice(&0x0006_u16.to_be_bytes());
            buf[COMMAND_RANGE_DATA.start..COMMAND_RANGE_DATA.start + data.len()]
                .copy_from_slice(&data);
        }

        Command::TransmitData {
            destination_address,
            source_port,
            destination_port,
            data,
        } => {
            if data.len() > 0x4d0 {
                return Err(());
            }
            buf[COMMAND_RANGE_COMMAND_CODE].copy_from_slice(&0x0008_u16.to_be_bytes());

            let mut data_len = 0;
            for s in [
                destination_address.to_be_bytes().as_slice(),
                source_port.to_be_bytes().as_slice(),
                destination_port.to_be_bytes().as_slice(),
                (data.len() as u16).to_be_bytes().as_slice(),
                data,
            ] {
                let new_data_len = data_len + s.len();
                if buf.len() < total_len + new_data_len {
                    return Err(());
                }
                buf[total_len + data_len..total_len + new_data_len].copy_from_slice(s);
                data_len = new_data_len;
            }
            total_len += data_len;
        }

        Command::DoActiveScan {
            duration,
            mask_channels,
            pairing_id: None,
        } => {
            buf[COMMAND_RANGE_COMMAND_CODE].copy_from_slice(&0x0051_u16.to_be_bytes());

            let mut data_len = 0;
            for s in [
                u8::from(*duration).to_be_bytes().as_slice(),
                mask_channels.to_be_bytes().as_slice(),
                0x00_u8.to_be_bytes().as_slice(),
            ] {
                let new_data_len = data_len + s.len();
                if buf.len() < total_len + new_data_len {
                    return Err(());
                }
                buf[total_len + data_len..total_len + new_data_len].copy_from_slice(s);
                data_len = new_data_len;
            }
            total_len += data_len;
        }

        Command::DoActiveScan {
            duration,
            mask_channels,
            pairing_id: Some(pairing_id),
        } => {
            buf[COMMAND_RANGE_COMMAND_CODE].copy_from_slice(&0x0051_u16.to_be_bytes());

            let mut data_len = 0;
            for s in [
                u8::from(*duration).to_be_bytes().as_slice(),
                mask_channels.to_be_bytes().as_slice(),
                0x01_u8.to_be_bytes().as_slice(),
                pairing_id.to_be_bytes().as_slice(),
            ] {
                let new_data_len = data_len + s.len();
                if buf.len() < total_len + new_data_len {
                    return Err(());
                }
                buf[total_len + data_len..total_len + new_data_len].copy_from_slice(s);
                data_len = new_data_len;
            }
            total_len += data_len;
        }

        Command::GetVersionInformation => {
            buf[COMMAND_RANGE_COMMAND_CODE].copy_from_slice(&0x006b_u16.to_be_bytes())
        }

        Command::SetRouteBPanaAuthenticationInformation { id, password } => {
            if id.len() != 32 || password.len() != 12 {
                return Err(());
            }

            buf[COMMAND_RANGE_COMMAND_CODE].copy_from_slice(&0x0054_u16.to_be_bytes());

            let mut data_len = 0;
            for s in [id, password] {
                let new_data_len = data_len + s.len();
                if buf.len() < total_len + new_data_len {
                    return Err(());
                }
                buf[total_len + data_len..total_len + new_data_len].copy_from_slice(s);
                data_len = new_data_len;
            }
            total_len += data_len;
        }

        Command::StartRouteBOperation => {
            buf[COMMAND_RANGE_COMMAND_CODE].copy_from_slice(&0x0053_u16.to_be_bytes())
        }

        Command::StartRouteBPana => {
            buf[COMMAND_RANGE_COMMAND_CODE].copy_from_slice(&0x0056_u16.to_be_bytes())
        }

        Command::TerminateRouteBPana => {
            buf[COMMAND_RANGE_COMMAND_CODE].copy_from_slice(&0x0057_u16.to_be_bytes())
        }

        Command::TerminateRouteBOperation => {
            buf[COMMAND_RANGE_COMMAND_CODE].copy_from_slice(&0x0058_u16.to_be_bytes())
        }

        Command::RedoRouteBPanaAuthentication => {
            buf[COMMAND_RANGE_COMMAND_CODE].copy_from_slice(&0x00d2_u16.to_be_bytes())
        }
    }

    let message_len = (total_len - 4 - 2 - 2) as u16;
    buf[COMMAND_RANGE_MESSAGE_LEN].copy_from_slice(&message_len.to_be_bytes());

    let header_sum = buf[COMMAND_RANGE_UNIQUE_CODE.start..COMMAND_RANGE_HEADER_SUM.start]
        .iter()
        .fold(0_u16, |acc, &x| acc + x as u16);
    buf[COMMAND_RANGE_HEADER_SUM].copy_from_slice(&header_sum.to_be_bytes());

    let data_sum = buf[COMMAND_RANGE_DATA.start..total_len]
        .iter()
        .fold(0_u16, |acc, &x| acc + x as u16);
    buf[COMMAND_RANGE_DATA_SUM].copy_from_slice(&data_sum.to_be_bytes());

    Ok(total_len)
}

#[cfg(test)]
mod tests {
    extern crate std;
    use crate::command::*;

    #[test]
    fn test_serialize_to_bytes() {
        let mut buf = [0; 256];

        assert_eq!(
            serialize_to_bytes(&Command::GetStatus, &mut buf).map(|len| &buf[..len]),
            Ok([
                0xd0_u8, 0xea_u8, 0x83_u8, 0xfc_u8, // unique code
                0x00_u8, 0x01_u8, // command
                0x00_u8, 0x04_u8, // length
                0x03_u8, 0x3e_u8, // header sum
                0x00_u8, 0x00_u8, // data sum
            ]
            .as_slice())
        );

        assert_eq!(
            serialize_to_bytes(&Command::GetUdpPorts, &mut buf).map(|len| &buf[..len]),
            Ok([
                0xd0_u8, 0xea_u8, 0x83_u8, 0xfc_u8, // unique code
                0x00_u8, 0x07_u8, // command
                0x00_u8, 0x04_u8, // length
                0x03_u8, 0x44_u8, // header sum
                0x00_u8, 0x00_u8, // data sum
            ]
            .as_slice())
        );

        assert_eq!(
            serialize_to_bytes(&Command::GetIpAddress, &mut buf).map(|len| &buf[..len]),
            Ok([
                0xd0_u8, 0xea_u8, 0x83_u8, 0xfc_u8, // unique code
                0x00_u8, 0x09_u8, // command
                0x00_u8, 0x04_u8, // length
                0x03_u8, 0x46_u8, // header sum
                0x00_u8, 0x00_u8, // data sum
            ]
            .as_slice())
        );

        assert_eq!(
            serialize_to_bytes(&Command::GetMacAddress, &mut buf).map(|len| &buf[..len]),
            Ok([
                0xd0_u8, 0xea_u8, 0x83_u8, 0xfc_u8, // unique code
                0x00_u8, 0x0e_u8, // command
                0x00_u8, 0x04_u8, // length
                0x03_u8, 0x4b_u8, // header sum
                0x00_u8, 0x00_u8, // data sum
            ]
            .as_slice())
        );

        assert_eq!(
            serialize_to_bytes(&Command::GetClientMacAddress, &mut buf).map(|len| &buf[..len]),
            Ok([
                0xd0_u8, 0xea_u8, 0x83_u8, 0xfc_u8, // unique code
                0x00_u8, 0x11_u8, // command
                0x00_u8, 0x04_u8, // length
                0x03_u8, 0x4e_u8, // header sum
                0x00_u8, 0x00_u8, // data sum
            ]
            .as_slice())
        );

        assert_eq!(
            serialize_to_bytes(&Command::GetClientIpAddress, &mut buf).map(|len| &buf[..len]),
            Ok([
                0xd0_u8, 0xea_u8, 0x83_u8, 0xfc_u8, // unique code
                0x01_u8, 0x00_u8, // command
                0x00_u8, 0x04_u8, // length
                0x03_u8, 0x3e_u8, // header sum
                0x00_u8, 0x00_u8, // data sum
            ]
            .as_slice())
        );

        assert_eq!(
            serialize_to_bytes(&Command::GetNeighborDiscoverySettings, &mut buf)
                .map(|len| &buf[..len]),
            Ok([
                0xd0_u8, 0xea_u8, 0x83_u8, 0xfc_u8, // unique code
                0x01_u8, 0x02_u8, // command
                0x00_u8, 0x04_u8, // length
                0x03_u8, 0x40_u8, // header sum
                0x00_u8, 0x00_u8, // data sum
            ]
            .as_slice())
        );

        assert_eq!(
            serialize_to_bytes(&Command::GetOperationMode, &mut buf).map(|len| &buf[..len]),
            Ok([
                0xd0_u8, 0xea_u8, 0x83_u8, 0xfc_u8, // unique code
                0x01_u8, 0x07_u8, // command
                0x00_u8, 0x04_u8, // length
                0x03_u8, 0x45_u8, // header sum
                0x00_u8, 0x00_u8, // data sum
            ]
            .as_slice())
        );

        assert_eq!(
            serialize_to_bytes(&Command::GetUartSettings, &mut buf).map(|len| &buf[..len]),
            Ok([
                0xd0_u8, 0xea_u8, 0x83_u8, 0xfc_u8, // unique code
                0x01_u8, 0x0b_u8, // command
                0x00_u8, 0x04_u8, // length
                0x03_u8, 0x49_u8, // header sum
                0x00_u8, 0x00_u8, // data sum
            ]
            .as_slice())
        );

        assert_eq!(
            serialize_to_bytes(
                &Command::SetOperationMode {
                    mode: OperationMode::Dual,
                    han_sleep: false,
                    channel: Channel::Ch5F922p9MHz,
                    tx_power: TxPower::P20mW,
                },
                &mut buf,
            )
            .map(|len| &buf[..len]),
            Ok([
                0xd0_u8, 0xea_u8, 0x83_u8, 0xfc_u8, // unique code
                0x00_u8, 0x5f_u8, // command
                0x00_u8, 0x08_u8, // length
                0x03_u8, 0xa0_u8, // header sum
                0x00_u8, 0x0a_u8, // data sum
                0x05_u8, // mode
                0x00_u8, // han_sleep
                0x05_u8, // channel
                0x00_u8, // tx_power
            ]
            .as_slice())
        );

        assert_eq!(
            serialize_to_bytes(
                &Command::SetNeighborDiscoverySettings { enable: true },
                &mut buf
            )
            .map(|len| &buf[..len]),
            Ok([
                0xd0_u8, 0xea_u8, 0x83_u8, 0xfc_u8, // unique code
                0x01_u8, 0x01_u8, // command
                0x00_u8, 0x05_u8, // length
                0x03_u8, 0x40_u8, // header sum
                0x00_u8, 0x01_u8, // data sum
                0x01_u8, // enable
            ]
            .as_slice())
        );

        assert_eq!(
            serialize_to_bytes(&Command::SetUartSettings { flow_control: true }, &mut buf)
                .map(|len| &buf[..len]),
            Ok([
                0xd0_u8, 0xea_u8, 0x83_u8, 0xfc_u8, // unique code
                0x01_u8, 0x0A_u8, // command
                0x00_u8, 0x05_u8, // length
                0x03_u8, 0x49_u8, // header sum
                0x00_u8, 0x01_u8, // data sum
                0x01_u8, // flow control
            ]
            .as_slice())
        );

        assert_eq!(
            serialize_to_bytes(&Command::OpenUdpPort(0x1234), &mut buf).map(|len| &buf[..len]),
            Ok([
                0xd0_u8, 0xea_u8, 0x83_u8, 0xfc_u8, // unique code
                0x00_u8, 0x05_u8, // command
                0x00_u8, 0x06_u8, // length
                0x03_u8, 0x44_u8, // header sum
                0x00_u8, 0x46_u8, // data sum
                0x12_u8, 0x34_u8, // port
            ]
            .as_slice())
        );

        assert_eq!(
            serialize_to_bytes(&Command::CloseUdpPort(0x1234), &mut buf).map(|len| &buf[..len]),
            Ok([
                0xd0_u8, 0xea_u8, 0x83_u8, 0xfc_u8, // unique code
                0x00_u8, 0x06_u8, // command
                0x00_u8, 0x06_u8, // length
                0x03_u8, 0x45_u8, // header sum
                0x00_u8, 0x46_u8, // data sum
                0x12_u8, 0x34_u8, // port
            ]
            .as_slice())
        );

        assert_eq!(
            serialize_to_bytes(
                &Command::TransmitData {
                    destination_address: 0x123456789abcdef0123456789abcdef0,
                    source_port: 0xabcd,
                    destination_port: 0x1234,
                    data: &[0xde, 0xad, 0xbe, 0xef],
                },
                &mut buf
            )
            .map(|len| &buf[..len]),
            Ok([
                0xd0_u8, 0xea_u8, 0x83_u8, 0xfc_u8, // unique code
                0x00_u8, 0x08_u8, // command
                0x00_u8, 0x1e_u8, // length
                0x03_u8, 0x5f_u8, // header sum
                0x0d_u8, 0x6a_u8, // data sum
                0x12_u8, 0x34_u8, 0x56_u8, 0x78_u8, // address
                0x9a_u8, 0xbc_u8, 0xde_u8, 0xf0_u8, //
                0x12_u8, 0x34_u8, 0x56_u8, 0x78_u8, //
                0x9a_u8, 0xbc_u8, 0xde_u8, 0xf0_u8, //
                0xab_u8, 0xcd_u8, // source port
                0x12_u8, 0x34_u8, // destination port
                0x00_u8, 0x04_u8, // length
                0xde_u8, 0xad_u8, 0xbe_u8, 0xef_u8, // data
            ]
            .as_slice())
        );

        assert_eq!(
            serialize_to_bytes(
                &Command::DoActiveScan {
                    duration: ScanDuration::T38p56ms,
                    mask_channels: 0x12345678,
                    pairing_id: None,
                },
                &mut buf
            )
            .map(|len| &buf[..len]),
            Ok([
                0xd0_u8, 0xea_u8, 0x83_u8, 0xfc_u8, // unique code
                0x00_u8, 0x51_u8, // command
                0x00_u8, 0x0a_u8, // length
                0x03_u8, 0x94_u8, // header sum
                0x01_u8, 0x16_u8, // data sum
                0x02_u8, // duration
                0x12_u8, 0x34_u8, 0x56_u8, 0x78_u8, // channel mask
                0x00_u8, // has pairing id
            ]
            .as_slice())
        );

        assert_eq!(
            serialize_to_bytes(
                &Command::DoActiveScan {
                    duration: ScanDuration::T38p56ms,
                    mask_channels: 0x12345678,
                    pairing_id: Some(0x12345678_9abcdef0),
                },
                &mut buf
            )
            .map(|len| &buf[..len]),
            Ok([
                0xd0_u8, 0xea_u8, 0x83_u8, 0xfc_u8, // unique code
                0x00_u8, 0x51_u8, // command
                0x00_u8, 0x12_u8, // length
                0x03_u8, 0x9c_u8, // header sum
                0x05_u8, 0x4f_u8, // data sum
                0x02_u8, // duration
                0x12_u8, 0x34_u8, 0x56_u8, 0x78_u8, // channel mask
                0x01_u8, // has pairing id
                0x12_u8, 0x34_u8, 0x56_u8, 0x78_u8, // pairing id
                0x9a_u8, 0xbc_u8, 0xde_u8, 0xf0_u8, //
            ]
            .as_slice())
        );

        assert_eq!(
            serialize_to_bytes(&Command::GetVersionInformation, &mut buf).map(|len| &buf[..len]),
            Ok([
                0xd0_u8, 0xea_u8, 0x83_u8, 0xfc_u8, // unique code
                0x00_u8, 0x6b_u8, // command
                0x00_u8, 0x04_u8, // length
                0x03_u8, 0xa8_u8, // header sum
                0x00_u8, 0x00_u8, // data sum
            ]
            .as_slice())
        );

        assert_eq!(
            serialize_to_bytes(
                &Command::SetRouteBPanaAuthenticationInformation {
                    id: b"ABCDEFGHIJKLMNOPQRSTUVWXYZ012345",
                    password: b"0123456789ab",
                },
                &mut buf,
            )
            .map(|len| &buf[..len]),
            Ok([
                0xd0_u8, 0xea_u8, 0x83_u8, 0xfc_u8, // unique code
                0x00_u8, 0x54_u8, // command
                0x00_u8, 0x30_u8, // length
                0x03_u8, 0xbd_u8, // header sum
                0x0b_u8, 0xde_u8, // data sum
                // id
                b'A', b'B', b'C', b'D', b'E', b'F', b'G', b'H', //
                b'I', b'J', b'K', b'L', b'M', b'N', b'O', b'P', //
                b'Q', b'R', b'S', b'T', b'U', b'V', b'W', b'X', //
                b'Y', b'Z', b'0', b'1', b'2', b'3', b'4', b'5', //
                // password
                b'0', b'1', b'2', b'3', b'4', b'5', b'6', b'7', b'8', b'9', b'a', b'b',
            ]
            .as_slice())
        );

        assert_eq!(
            serialize_to_bytes(&Command::StartRouteBOperation, &mut buf).map(|len| &buf[..len]),
            Ok([
                0xd0_u8, 0xea_u8, 0x83_u8, 0xfc_u8, // unique code
                0x00_u8, 0x53_u8, // command
                0x00_u8, 0x04_u8, // length
                0x03_u8, 0x90_u8, // header sum
                0x00_u8, 0x00_u8, // data sum
            ]
            .as_slice())
        );

        assert_eq!(
            serialize_to_bytes(&Command::StartRouteBPana, &mut buf).map(|len| &buf[..len]),
            Ok([
                0xd0_u8, 0xea_u8, 0x83_u8, 0xfc_u8, // unique code
                0x00_u8, 0x56_u8, // command
                0x00_u8, 0x04_u8, // length
                0x03_u8, 0x93_u8, // header sum
                0x00_u8, 0x00_u8, // data sum
            ]
            .as_slice())
        );

        assert_eq!(
            serialize_to_bytes(&Command::TerminateRouteBPana, &mut buf).map(|len| &buf[..len]),
            Ok([
                0xd0_u8, 0xea_u8, 0x83_u8, 0xfc_u8, // unique code
                0x00_u8, 0x57_u8, // command
                0x00_u8, 0x04_u8, // length
                0x03_u8, 0x94_u8, // header sum
                0x00_u8, 0x00_u8, // data sum
            ]
            .as_slice())
        );

        assert_eq!(
            serialize_to_bytes(&Command::TerminateRouteBOperation, &mut buf).map(|len| &buf[..len]),
            Ok([
                0xd0_u8, 0xea_u8, 0x83_u8, 0xfc_u8, // unique code
                0x00_u8, 0x58_u8, // command
                0x00_u8, 0x04_u8, // length
                0x03_u8, 0x95_u8, // header sum
                0x00_u8, 0x00_u8, // data sum
            ]
            .as_slice())
        );

        assert_eq!(
            serialize_to_bytes(&Command::RedoRouteBPanaAuthentication, &mut buf)
                .map(|len| &buf[..len]),
            Ok([
                0xd0_u8, 0xea_u8, 0x83_u8, 0xfc_u8, // unique code
                0x00_u8, 0xd2_u8, // command
                0x00_u8, 0x04_u8, // length
                0x04_u8, 0x0f_u8, // header sum
                0x00_u8, 0x00_u8, // data sum
            ]
            .as_slice())
        );
    }
}
