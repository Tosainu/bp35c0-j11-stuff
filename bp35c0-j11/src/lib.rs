#![no_std]

extern crate alloc;

use core::convert::From;
use core::ops::{Range, RangeFrom};

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum GlobalBlockStatus {
    Inactive,
    Active,
    Unknown(u8),
}

impl From<u8> for GlobalBlockStatus {
    fn from(v: u8) -> Self {
        match v {
            0x02 => GlobalBlockStatus::Inactive,
            0x03 => GlobalBlockStatus::Active,
            _ => GlobalBlockStatus::Unknown(v),
        }
    }
}

impl From<GlobalBlockStatus> for u8 {
    fn from(v: GlobalBlockStatus) -> Self {
        match v {
            GlobalBlockStatus::Inactive => 0x02,
            GlobalBlockStatus::Active => 0x03,
            GlobalBlockStatus::Unknown(v) => v,
        }
    }
}

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RouteBBlockStatus {
    Inactive,
    Active,
    Authentication,
    Unknown(u8),
}

impl From<u8> for RouteBBlockStatus {
    fn from(v: u8) -> Self {
        match v {
            0x01 => RouteBBlockStatus::Inactive,
            0x02 => RouteBBlockStatus::Active,
            0x03 => RouteBBlockStatus::Authentication,
            _ => RouteBBlockStatus::Unknown(v),
        }
    }
}

impl From<RouteBBlockStatus> for u8 {
    fn from(v: RouteBBlockStatus) -> Self {
        match v {
            RouteBBlockStatus::Inactive => 0x01,
            RouteBBlockStatus::Active => 0x02,
            RouteBBlockStatus::Authentication => 0x03,
            RouteBBlockStatus::Unknown(v) => v,
        }
    }
}

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum HanBlockStatus {
    Inactive,
    Active,
    Authentication,
    Unknown(u8),
}

impl From<u8> for HanBlockStatus {
    fn from(v: u8) -> Self {
        match v {
            0x01 => HanBlockStatus::Inactive,
            0x02 => HanBlockStatus::Active,
            0x03 => HanBlockStatus::Authentication,
            _ => HanBlockStatus::Unknown(v),
        }
    }
}

impl From<HanBlockStatus> for u8 {
    fn from(v: HanBlockStatus) -> Self {
        match v {
            HanBlockStatus::Inactive => 0x01,
            HanBlockStatus::Active => 0x02,
            HanBlockStatus::Authentication => 0x03,
            HanBlockStatus::Unknown(v) => v,
        }
    }
}

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum OperationMode {
    HanPanCoordinator,
    HanCoordinator,
    HanEndDevice,
    Dual,
    Unknown(u8),
}

impl From<u8> for OperationMode {
    fn from(v: u8) -> Self {
        match v {
            0x01 => OperationMode::HanPanCoordinator,
            0x02 => OperationMode::HanCoordinator,
            0x03 => OperationMode::HanEndDevice,
            0x05 => OperationMode::Dual,
            _ => OperationMode::Unknown(v),
        }
    }
}

impl From<OperationMode> for u8 {
    fn from(v: OperationMode) -> Self {
        match v {
            OperationMode::HanPanCoordinator => 0x01,
            OperationMode::HanCoordinator => 0x02,
            OperationMode::HanEndDevice => 0x03,
            OperationMode::Dual => 0x05,
            OperationMode::Unknown(v) => v,
        }
    }
}

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Channel {
    Ch4F922p5MHz,
    Ch5F922p9MHz,
    Ch6F923p3MHz,
    Ch7F923p7MHz,
    Ch8F924p1MHz,
    Ch9F924p5MHz,
    Ch10F924p9MHz,
    Ch11F925p3MHz,
    Ch12F925p7MHz,
    Ch13F926p1MHz,
    Ch14F926p5MHz,
    Ch15F926p9MHz,
    Ch16F927p3MHz,
    Ch17F927p7MHz,
    Unknown(u8),
}

impl From<u8> for Channel {
    fn from(v: u8) -> Self {
        match v {
            0x04 => Channel::Ch4F922p5MHz,
            0x05 => Channel::Ch5F922p9MHz,
            0x06 => Channel::Ch6F923p3MHz,
            0x07 => Channel::Ch7F923p7MHz,
            0x08 => Channel::Ch8F924p1MHz,
            0x09 => Channel::Ch9F924p5MHz,
            0x0a => Channel::Ch10F924p9MHz,
            0x0b => Channel::Ch11F925p3MHz,
            0x0c => Channel::Ch12F925p7MHz,
            0x0d => Channel::Ch13F926p1MHz,
            0x0e => Channel::Ch14F926p5MHz,
            0x0f => Channel::Ch15F926p9MHz,
            0x10 => Channel::Ch16F927p3MHz,
            0x11 => Channel::Ch17F927p7MHz,
            _ => Channel::Unknown(v),
        }
    }
}

impl From<Channel> for u8 {
    fn from(v: Channel) -> Self {
        match v {
            Channel::Ch4F922p5MHz => 0x04,
            Channel::Ch5F922p9MHz => 0x05,
            Channel::Ch6F923p3MHz => 0x06,
            Channel::Ch7F923p7MHz => 0x07,
            Channel::Ch8F924p1MHz => 0x08,
            Channel::Ch9F924p5MHz => 0x09,
            Channel::Ch10F924p9MHz => 0x0a,
            Channel::Ch11F925p3MHz => 0x0b,
            Channel::Ch12F925p7MHz => 0x0c,
            Channel::Ch13F926p1MHz => 0x0d,
            Channel::Ch14F926p5MHz => 0x0e,
            Channel::Ch15F926p9MHz => 0x0f,
            Channel::Ch16F927p3MHz => 0x10,
            Channel::Ch17F927p7MHz => 0x11,
            Channel::Unknown(v) => v,
        }
    }
}

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TxPower {
    P20mW,
    P10mW,
    P1mW,
    Unknown(u8),
}

impl From<u8> for TxPower {
    fn from(v: u8) -> Self {
        match v {
            0x00 => TxPower::P20mW,
            0x01 => TxPower::P10mW,
            0x02 => TxPower::P1mW,
            _ => TxPower::Unknown(v),
        }
    }
}

impl From<TxPower> for u8 {
    fn from(v: TxPower) -> Self {
        match v {
            TxPower::P20mW => 0x00,
            TxPower::P10mW => 0x01,
            TxPower::P1mW => 0x02,
            TxPower::Unknown(v) => v,
        }
    }
}

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ScanDuration {
    T19p28ms,
    T38p56ms,
    T77p12ms,
    T154p24ms,
    T308p48ms,
    T616p96ms,
    T1233p92ms,
    T2467p84ms,
    T4935p68ms,
    T9871p36ms,
    T19742p72ms,
    T39485p44ms,
    T78970p88ms,
    T157941p76ms,
    Unknown(u8),
}

impl From<u8> for ScanDuration {
    fn from(v: u8) -> Self {
        match v {
            0x01 => ScanDuration::T19p28ms,
            0x02 => ScanDuration::T38p56ms,
            0x03 => ScanDuration::T77p12ms,
            0x04 => ScanDuration::T154p24ms,
            0x05 => ScanDuration::T308p48ms,
            0x06 => ScanDuration::T616p96ms,
            0x07 => ScanDuration::T1233p92ms,
            0x08 => ScanDuration::T2467p84ms,
            0x09 => ScanDuration::T4935p68ms,
            0x0a => ScanDuration::T9871p36ms,
            0x0b => ScanDuration::T19742p72ms,
            0x0c => ScanDuration::T39485p44ms,
            0x0d => ScanDuration::T78970p88ms,
            0x0e => ScanDuration::T157941p76ms,
            _ => ScanDuration::Unknown(v),
        }
    }
}

impl From<ScanDuration> for u8 {
    fn from(v: ScanDuration) -> Self {
        match v {
            ScanDuration::T19p28ms => 0x01,
            ScanDuration::T38p56ms => 0x02,
            ScanDuration::T77p12ms => 0x03,
            ScanDuration::T154p24ms => 0x04,
            ScanDuration::T308p48ms => 0x05,
            ScanDuration::T616p96ms => 0x06,
            ScanDuration::T1233p92ms => 0x07,
            ScanDuration::T2467p84ms => 0x08,
            ScanDuration::T4935p68ms => 0x09,
            ScanDuration::T9871p36ms => 0x0a,
            ScanDuration::T19742p72ms => 0x0b,
            ScanDuration::T39485p44ms => 0x0c,
            ScanDuration::T78970p88ms => 0x0d,
            ScanDuration::T157941p76ms => 0x0e,
            ScanDuration::Unknown(v) => v,
        }
    }
}

const COMMAND_RANGE_UNIQUE_CODE: Range<usize> = 0..4;
const COMMAND_RANGE_COMMAND_CODE: Range<usize> = 4..6;
const COMMAND_RANGE_MESSAGE_LEN: Range<usize> = 6..8;
const COMMAND_RANGE_HEADER_SUM: Range<usize> = 8..10;
const COMMAND_RANGE_DATA_SUM: Range<usize> = 10..12;
const COMMAND_RANGE_DATA: RangeFrom<usize> = 12..;

mod command;
pub use command::*;

mod response;
pub use response::*;

#[cfg(test)]
mod tests {
    extern crate std;
    use crate::*;

    #[test]
    fn test_global_block_satatus() {
        assert_eq!(GlobalBlockStatus::from(0x02u8), GlobalBlockStatus::Inactive);
        assert_eq!(GlobalBlockStatus::from(0x03u8), GlobalBlockStatus::Active);
        assert_eq!(
            GlobalBlockStatus::from(0x7fu8),
            GlobalBlockStatus::Unknown(0x7f)
        );

        assert_eq!(u8::from(GlobalBlockStatus::Inactive), 0x02);
        assert_eq!(u8::from(GlobalBlockStatus::Active), 0x03);
        assert_eq!(u8::from(GlobalBlockStatus::Unknown(0x7f)), 0x7f);
    }

    #[test]
    fn test_route_b_block_satatus() {
        assert_eq!(RouteBBlockStatus::from(0x01u8), RouteBBlockStatus::Inactive);
        assert_eq!(RouteBBlockStatus::from(0x02u8), RouteBBlockStatus::Active);
        assert_eq!(
            RouteBBlockStatus::from(0x03u8),
            RouteBBlockStatus::Authentication
        );
        assert_eq!(
            RouteBBlockStatus::from(0x7fu8),
            RouteBBlockStatus::Unknown(0x7f)
        );

        assert_eq!(u8::from(RouteBBlockStatus::Inactive), 0x01);
        assert_eq!(u8::from(RouteBBlockStatus::Active), 0x02);
        assert_eq!(u8::from(RouteBBlockStatus::Authentication), 0x03);
        assert_eq!(u8::from(RouteBBlockStatus::Unknown(0x7f)), 0x7f);
    }

    #[test]
    fn test_han_block_satatus() {
        assert_eq!(HanBlockStatus::from(0x01u8), HanBlockStatus::Inactive);
        assert_eq!(HanBlockStatus::from(0x02u8), HanBlockStatus::Active);
        assert_eq!(HanBlockStatus::from(0x03u8), HanBlockStatus::Authentication);
        assert_eq!(HanBlockStatus::from(0x7fu8), HanBlockStatus::Unknown(0x7f));

        assert_eq!(u8::from(HanBlockStatus::Inactive), 0x01);
        assert_eq!(u8::from(HanBlockStatus::Active), 0x02);
        assert_eq!(u8::from(HanBlockStatus::Authentication), 0x03);
        assert_eq!(u8::from(HanBlockStatus::Unknown(0x7f)), 0x7f);
    }

    #[test]
    fn test_operation_mode() {
        assert_eq!(
            OperationMode::from(0x01u8),
            OperationMode::HanPanCoordinator
        );
        assert_eq!(OperationMode::from(0x02u8), OperationMode::HanCoordinator);
        assert_eq!(OperationMode::from(0x03u8), OperationMode::HanEndDevice);
        assert_eq!(OperationMode::from(0x05u8), OperationMode::Dual);
        assert_eq!(OperationMode::from(0x7fu8), OperationMode::Unknown(0x7f));

        assert_eq!(u8::from(OperationMode::HanPanCoordinator), 0x01);
        assert_eq!(u8::from(OperationMode::HanCoordinator), 0x02);
        assert_eq!(u8::from(OperationMode::HanEndDevice), 0x03);
        assert_eq!(u8::from(OperationMode::Dual), 0x05);
        assert_eq!(u8::from(OperationMode::Unknown(0x7f)), 0x7f);
    }

    #[test]
    fn test_channel() {
        assert_eq!(Channel::from(0x04u8), Channel::Ch4F922p5MHz);
        assert_eq!(Channel::from(0x05u8), Channel::Ch5F922p9MHz);
        assert_eq!(Channel::from(0x06u8), Channel::Ch6F923p3MHz);
        assert_eq!(Channel::from(0x07u8), Channel::Ch7F923p7MHz);
        assert_eq!(Channel::from(0x08u8), Channel::Ch8F924p1MHz);
        assert_eq!(Channel::from(0x09u8), Channel::Ch9F924p5MHz);
        assert_eq!(Channel::from(0x0au8), Channel::Ch10F924p9MHz);
        assert_eq!(Channel::from(0x0bu8), Channel::Ch11F925p3MHz);
        assert_eq!(Channel::from(0x0cu8), Channel::Ch12F925p7MHz);
        assert_eq!(Channel::from(0x0du8), Channel::Ch13F926p1MHz);
        assert_eq!(Channel::from(0x0eu8), Channel::Ch14F926p5MHz);
        assert_eq!(Channel::from(0x0fu8), Channel::Ch15F926p9MHz);
        assert_eq!(Channel::from(0x10u8), Channel::Ch16F927p3MHz);
        assert_eq!(Channel::from(0x11u8), Channel::Ch17F927p7MHz);
        assert_eq!(Channel::from(0x7fu8), Channel::Unknown(0x7f));

        assert_eq!(u8::from(Channel::Ch4F922p5MHz), 0x04);
        assert_eq!(u8::from(Channel::Ch5F922p9MHz), 0x05);
        assert_eq!(u8::from(Channel::Ch6F923p3MHz), 0x06);
        assert_eq!(u8::from(Channel::Ch7F923p7MHz), 0x07);
        assert_eq!(u8::from(Channel::Ch8F924p1MHz), 0x08);
        assert_eq!(u8::from(Channel::Ch9F924p5MHz), 0x09);
        assert_eq!(u8::from(Channel::Ch10F924p9MHz), 0x0a);
        assert_eq!(u8::from(Channel::Ch11F925p3MHz), 0x0b);
        assert_eq!(u8::from(Channel::Ch12F925p7MHz), 0x0c);
        assert_eq!(u8::from(Channel::Ch13F926p1MHz), 0x0d);
        assert_eq!(u8::from(Channel::Ch14F926p5MHz), 0x0e);
        assert_eq!(u8::from(Channel::Ch15F926p9MHz), 0x0f);
        assert_eq!(u8::from(Channel::Ch16F927p3MHz), 0x10);
        assert_eq!(u8::from(Channel::Ch17F927p7MHz), 0x11);
        assert_eq!(u8::from(Channel::Unknown(0x7f)), 0x7f);
    }

    #[test]
    fn test_tx_power() {
        assert_eq!(TxPower::from(0x00u8), TxPower::P20mW);
        assert_eq!(TxPower::from(0x01u8), TxPower::P10mW);
        assert_eq!(TxPower::from(0x02u8), TxPower::P1mW);
        assert_eq!(TxPower::from(0x7fu8), TxPower::Unknown(0x7f));

        assert_eq!(u8::from(TxPower::P20mW), 0x00);
        assert_eq!(u8::from(TxPower::P10mW), 0x01);
        assert_eq!(u8::from(TxPower::P1mW), 0x02);
        assert_eq!(u8::from(TxPower::Unknown(0x7f)), 0x7f);
    }
}
