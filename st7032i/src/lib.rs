#![no_std]

use embedded_hal_0_2_x::blocking::i2c::{AddressMode, Write, WriteIter};
use fugit::NanosDurationU32;

pub struct One {}
pub struct Two {}

pub trait DisplayRows {
    fn value() -> u8;
}

impl DisplayRows for One {
    fn value() -> u8 {
        0b0000
    }
}

impl DisplayRows for Two {
    fn value() -> u8 {
        0b1000
    }
}

pub struct Single {}
pub struct Double {}

pub trait CharacterHeight {
    fn value() -> u8;
}

impl CharacterHeight for Single {
    fn value() -> u8 {
        0b0000
    }
}

impl CharacterHeight for Double {
    fn value() -> u8 {
        0b0100
    }
}

pub struct Normal {}
pub struct Extention {}
pub struct Unknown {}

pub trait InstructionSet {
    fn value() -> u8;
}

impl InstructionSet for Normal {
    fn value() -> u8 {
        0b0000_0000
    }
}

impl InstructionSet for Extention {
    fn value() -> u8 {
        0b0000_0001
    }
}

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum Character {
    Char(char),
    Cgram(u8),
}

pub const DDRAM_ADDRESS_LINE1: u8 = 0x00;
pub const DDRAM_ADDRESS_LINE2: u8 = 0x40;

pub const EXECUTION_TIME_SHORT: NanosDurationU32 = NanosDurationU32::nanos(26_300);
pub const EXECUTION_TIME_LONG: NanosDurationU32 = NanosDurationU32::micros(1_080);

pub struct St7032i<'a, I, W, A, N = Two, DH = Single>
where
    W: Write<A> + WriteIter<A>,
    A: AddressMode + Copy,
    N: DisplayRows,
    DH: CharacterHeight,
{
    dev: &'a mut W,
    addr: A,
    phantom: core::marker::PhantomData<(I, N, DH)>,
}

impl<'a, W, A, N, DH> St7032i<'a, Unknown, W, A, N, DH>
where
    W: Write<A> + WriteIter<A>,
    A: AddressMode + Copy,
    N: DisplayRows,
    DH: CharacterHeight,
{
    pub fn new(dev: &'a mut W, addr: A) -> Self {
        Self {
            dev,
            addr,
            phantom: core::marker::PhantomData,
        }
    }
}

impl<'a, I, W, A, N, DH> St7032i<'a, I, W, A, N, DH>
where
    W: Write<A> + WriteIter<A>,
    A: AddressMode + Copy,
    N: DisplayRows,
    DH: CharacterHeight,
{
    #[allow(clippy::type_complexity)]
    pub fn set_instruction_set<ITarget: InstructionSet>(
        mut self,
    ) -> Result<(St7032i<'a, ITarget, W, A, N, DH>, NanosDurationU32), <W as Write<A>>::Error> {
        let value = 0b0011_0000 | N::value() | DH::value() | ITarget::value();
        self.write_raw(&[0b00_000000, value]).map(|_| {
            (
                St7032i::<ITarget, W, A, N, DH> {
                    dev: self.dev,
                    addr: self.addr,
                    phantom: core::marker::PhantomData,
                },
                EXECUTION_TIME_SHORT,
            )
        })
    }

    pub fn write_raw(&mut self, data: &[u8]) -> Result<(), <W as Write<A>>::Error> {
        Write::write(self.dev, self.addr, data)
    }

    pub fn write_iter_raw<B>(&mut self, data: B) -> Result<(), <W as WriteIter<A>>::Error>
    where
        B: IntoIterator<Item = u8>,
    {
        WriteIter::write(self.dev, self.addr, data)
    }
}

impl<'a, I, W, A, N, DH> St7032i<'a, I, W, A, N, DH>
where
    I: InstructionSet,
    W: Write<A> + WriteIter<A>,
    A: AddressMode + Copy,
    N: DisplayRows,
    DH: CharacterHeight,
{
    pub fn cmd_clear_display(&mut self) -> Result<NanosDurationU32, <W as Write<A>>::Error> {
        self.write_raw(&[0b00_000000, 0b0000_0001])
            .map(|_| EXECUTION_TIME_LONG)
    }

    pub fn cmd_display_on_off(
        &mut self,
        display: bool,
        cursor: bool,
        cursor_blink: bool,
    ) -> Result<NanosDurationU32, <W as Write<A>>::Error> {
        let mut value = 0b0000_1000;
        if display {
            value |= 0b100;
        }
        if cursor {
            value |= 0b010;
        }
        if cursor_blink {
            value |= 0b001;
        }
        self.write_raw(&[0b00_000000, value])
            .map(|_| EXECUTION_TIME_SHORT)
    }

    pub fn cmd_set_ddram_address(
        &mut self,
        addr: u8,
    ) -> Result<NanosDurationU32, <W as Write<A>>::Error> {
        let value = 0b1000_0000 | (addr & 0b0111_1111);
        self.write_raw(&[0b00_000000, value])
            .map(|_| EXECUTION_TIME_SHORT)
    }

    pub fn cmd_write_chars<C>(
        &mut self,
        chars: C,
    ) -> Result<NanosDurationU32, <W as WriteIter<A>>::Error>
    where
        C: IntoIterator<Item = Character>,
        C::IntoIter: Iterator<Item = C::Item> + Clone,
    {
        self.write_iter_raw(stuff_control_bytes(
            chars
                .into_iter()
                .map(|c| to_character_code(c).unwrap_or(0b0010_0000)),
        ))
        .map(|_| EXECUTION_TIME_SHORT)
    }

    pub fn cmd_write_bytes<C>(
        &mut self,
        bytes: C,
    ) -> Result<NanosDurationU32, <W as WriteIter<A>>::Error>
    where
        C: IntoIterator<Item = u8>,
        C::IntoIter: Iterator<Item = C::Item> + Clone,
    {
        self.write_iter_raw(stuff_control_bytes(bytes))
            .map(|_| EXECUTION_TIME_SHORT)
    }
}

impl<'a, W, A, N, DH> St7032i<'a, Normal, W, A, N, DH>
where
    W: Write<A> + WriteIter<A>,
    A: AddressMode + Copy,
    N: DisplayRows,
    DH: CharacterHeight,
{
    pub fn cmd_set_cgram_address(
        &mut self,
        addr: u8,
    ) -> Result<NanosDurationU32, <W as Write<A>>::Error> {
        let value = 0b0100_0000 | (addr & 0b011_1111);
        self.write_raw(&[0b00_000000, value])
            .map(|_| EXECUTION_TIME_SHORT)
    }
}

impl<'a, W, A, N, DH> St7032i<'a, Extention, W, A, N, DH>
where
    W: Write<A> + WriteIter<A>,
    A: AddressMode + Copy,
    N: DisplayRows,
    DH: CharacterHeight,
{
    pub fn cmd_internal_osc_frequency(
        &mut self,
    ) -> Result<NanosDurationU32, <W as Write<A>>::Error> {
        self.write_raw(&[0b00_000000, 0b0001_0100])
            .map(|_| EXECUTION_TIME_SHORT)
    }

    pub fn cmd_power_icon_contrast_set(
        &mut self,
        icon: bool,
        booster: bool,
        contrast_hi2: u8,
    ) -> Result<NanosDurationU32, <W as Write<A>>::Error> {
        let mut value = 0b0101_0000 | (contrast_hi2 & 0b0011);
        if icon {
            value |= 0b1000;
        }
        if booster {
            value |= 0b0100;
        }
        self.write_raw(&[0b00_000000, value])
            .map(|_| EXECUTION_TIME_SHORT)
    }

    pub fn cmd_follower_control(
        &mut self,
        enable: bool,
        ratio: u8,
    ) -> Result<NanosDurationU32, <W as Write<A>>::Error> {
        let mut value = 0b0110_0000 | (ratio & 0b0111);
        if enable {
            value |= 0b1000;
        }
        self.write_raw(&[0b00_000000, value])
            .map(|_| EXECUTION_TIME_SHORT)
    }

    pub fn cmd_contrast_set(
        &mut self,
        contrast_low4: u8,
    ) -> Result<NanosDurationU32, <W as Write<A>>::Error> {
        let value = 0b0111_0000 | (contrast_low4 & 0b0000_1111);
        self.write_raw(&[0b00_000000, value])
            .map(|_| EXECUTION_TIME_SHORT)
    }
}

impl<'a, W, A, N, DH> core::fmt::Write for St7032i<'a, Normal, W, A, N, DH>
where
    W: Write<A> + WriteIter<A>,
    A: AddressMode + Copy,
    N: DisplayRows,
    DH: CharacterHeight,
{
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        match self.cmd_write_chars(s.chars().map(Character::from)) {
            Ok(_) => Ok(()),
            Err(_) => Err(core::fmt::Error),
        }
    }
}

impl From<char> for Character {
    fn from(c: char) -> Self {
        Character::Char(c)
    }
}

fn to_character_code(c: Character) -> Option<u8> {
    match c {
        // 0b0010_0000 ~ 0b0111_1101
        Character::Char(c @ ' '..='}') => Some(c as u8),

        // 0b1010_0001 ~ 0b1101_1111
        Character::Char(c @ '｡'..='ﾟ') => Some((c as u16 - 0xfec0) as u8),

        Character::Cgram(c @ 0..=7) => Some(c),

        _ => None,
    }
}

fn stuff_control_bytes<C>(chars: C) -> impl Iterator<Item = u8>
where
    C: IntoIterator<Item = u8>,
    C::IntoIter: Iterator<Item = C::Item> + Clone,
{
    let chars = chars.into_iter();
    chars
        // chars と要素数が同じで 0b11_000000 だけの Iterator を作る
        //                          │└──── C0: 最終データのとき0、それ以外のとき1
        //                          └───── Rs: Data register の読み書きのとき1
        .clone()
        .map(|_| 0b11_000000)
        // 最後だけ 0b01_000000 にする
        .skip(1)
        .chain(core::iter::once(0b01_000000))
        // ↑ で作った control byte の Iterator と chars を zip
        // chars は ST7032 のコードに変換 未対応の文字はスペースに置換
        .zip(chars)
        // control byte, data byte の順で交互に出力
        .flat_map(|c| [c.0, c.1])
}

#[cfg(test)]
mod tests {
    extern crate std;
    use crate::*;

    #[test]
    fn test_to_character_code() {
        assert_eq!(to_character_code(' '.into()), Some(0b0010_0000));
        assert_eq!(to_character_code('!'.into()), Some(0b0010_0001));
        assert_eq!(to_character_code('0'.into()), Some(0b0011_0000));
        assert_eq!(to_character_code('@'.into()), Some(0b0100_0000));
        assert_eq!(to_character_code('A'.into()), Some(0b0100_0001));
        assert_eq!(to_character_code('`'.into()), Some(0b0110_0000));
        assert_eq!(to_character_code('a'.into()), Some(0b0110_0001));
        assert_eq!(to_character_code('}'.into()), Some(0b0111_1101));
        assert_eq!(to_character_code('~'.into()), None);
        assert_eq!(to_character_code('｡'.into()), Some(0b1010_0001));
        assert_eq!(to_character_code('ｱ'.into()), Some(0b1011_0001));
        assert_eq!(to_character_code('ﾟ'.into()), Some(0b1101_1111));

        assert_eq!(to_character_code(Character::Cgram(0)), Some(0b0000_0000));
        assert_eq!(to_character_code(Character::Cgram(7)), Some(0b0000_0111));
        assert_eq!(to_character_code(Character::Cgram(8)), None);
    }

    #[test]
    fn test_to_write_command() {
        {
            let mut iter = stuff_control_bytes([] as [u8; 0]);
            assert_eq!(iter.next(), None);
        }

        {
            let mut iter = stuff_control_bytes([0xa5]);
            assert_eq!(iter.next(), Some(0b01_000000));
            assert_eq!(iter.next(), Some(0xa5));
            assert_eq!(iter.next(), None);
        }

        {
            let mut iter = stuff_control_bytes([0xa5, 0x5a, 0xff]);
            assert_eq!(iter.next(), Some(0b11_000000));
            assert_eq!(iter.next(), Some(0xa5));
            assert_eq!(iter.next(), Some(0b11_000000));
            assert_eq!(iter.next(), Some(0x5a));
            assert_eq!(iter.next(), Some(0b01_000000));
            assert_eq!(iter.next(), Some(0xff));
            assert_eq!(iter.next(), None);
        }
    }
}
