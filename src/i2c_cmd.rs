use core::iter::once;

use either::Either::{Left, Right};
use pio::{Instruction, SideSet};

const SIDESET: SideSet = SideSet::new(true, 1, true);
const SC0SD0: Instruction = Instruction {
    operands: pio::InstructionOperands::SET {
        destination: pio::SetDestination::PINDIRS,
        data: 0,
    },
    delay: 7,
    side_set: Some(0),
};
const SC0SD1: Instruction = Instruction {
    operands: pio::InstructionOperands::SET {
        destination: pio::SetDestination::PINDIRS,
        data: 1,
    },
    delay: 7,
    side_set: Some(0),
};
const SC1SD0: Instruction = Instruction {
    operands: pio::InstructionOperands::SET {
        destination: pio::SetDestination::PINDIRS,
        data: 0,
    },
    delay: 7,
    side_set: Some(1),
};
const SC1SD1: Instruction = Instruction {
    operands: pio::InstructionOperands::SET {
        destination: pio::SetDestination::PINDIRS,
        data: 1,
    },
    delay: 7,
    side_set: Some(1),
};

const START: [Instruction; 2] = [SC1SD0, SC0SD0];
const STOP: [Instruction; 3] = [SC0SD0, SC1SD0, SC1SD1];
const RESTART: [Instruction; 4] = [SC0SD1, SC1SD1, SC1SD0, SC0SD0];

const NAK_BIT: u16 = 0b0000_0000_0000_0001;
const FINAL_BIT: u16 = 0b0000_0010_0000_0000;
const INSTR_OFFSET: usize = 10;
const DATA_OFFSET: usize = 1;

#[derive(Debug)]
pub struct Data<'b> {
    pub byte: either::Either<u8, &'b mut u8>,
    pub is_address: bool,
}
impl<'b> Data<'b> {
    pub fn encode(&self, last: bool) -> u16 {
        match self.byte.as_ref().left() {
            // if write: send data & let the target handle the nak/ack
            Some(b) => (u16::from(*b) << DATA_OFFSET) | NAK_BIT,
            // if read: clock out and idle state & ack only if not last
            None => (0xFF << DATA_OFFSET) | if last { NAK_BIT | FINAL_BIT } else { 0 },
        }
    }
    pub fn address(v: u8) -> Self {
        Self {
            byte: Left(v),
            is_address: true,
        }
    }
    pub fn write(v: u8) -> Self {
        Self {
            byte: Left(v),
            is_address: false,
        }
    }
    pub fn read(v: &'b mut u8) -> Self {
        Self {
            byte: Right(v),
            is_address: false,
        }
    }
}
#[cfg(feature = "defmt")]
impl defmt::Format for Data<'_> {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "Data {{  byte:");
        match self.byte {
            Left(b) => defmt::write!(fmt, "{:x}", b),
            Right(_) => defmt::write!(fmt, "…"),
        }
        defmt::write!(fmt, " }}");
    }
}
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CmdWord<'b> {
    Raw(u16),
    Data(Data<'b>),
}
impl From<Instruction> for CmdWord<'_> {
    fn from(value: Instruction) -> Self {
        CmdWord::Raw(value.encode(SIDESET))
    }
}
impl<'b> CmdWord<'b> {
    pub fn encode(&self, last: bool) -> u16 {
        match self {
            CmdWord::Raw(r) => *r,
            CmdWord::Data(d) => d.encode(last),
        }
    }
    pub fn address(v: u8) -> Self {
        CmdWord::Data(Data::address(v))
    }
    pub fn write(v: u8) -> Self {
        CmdWord::Data(Data::write(v))
    }
    pub fn read(v: &'b mut u8) -> Self {
        CmdWord::Data(Data::read(v))
    }
}
fn cmd_len(cmd: &[Instruction]) -> impl Iterator<Item = u16> {
    let encoded_len = (cmd.len() - 1) << INSTR_OFFSET;
    once(encoded_len as u16)
}
pub fn stop() -> impl Iterator<Item = u16> {
    cmd_len(&STOP).chain(STOP.into_iter().map(|i| i.encode(SIDESET)))
}

pub fn start<'b>() -> impl Iterator<Item = CmdWord<'b>> {
    cmd_len(&START)
        .map(CmdWord::Raw)
        .chain(START.into_iter().map(CmdWord::from))
}

pub fn restart<'b>() -> impl Iterator<Item = CmdWord<'b>> {
    cmd_len(&RESTART)
        .map(CmdWord::Raw)
        .chain(RESTART.into_iter().map(CmdWord::from))
}
