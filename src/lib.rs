#![no_std]
//! Implements an I2C Controller using the RP2040 PIO block.
//!
//! This implementation is based on the c-sdk's example with the following variation:
//!
//! - It uses WAIT on GPIO to handle clock stretching without requiring SCL to be SDA+1.
//! - It keeps autopush enabled a flushes the RX FIFO during write operations.
//!
//! # General command word description
//!
//! TX Encoding:
//! | 15:10 | 9     | 8:1  | 0   |
//! | Instr | Final | Data | NAK |
//!
//! If Instr has a value n > 0, then this FIFO word has no
//! data payload, and the next n + 1 words will be executed as instructions.
//! Otherwise, shift out the 8 data bits, followed by the ACK bit.
//!
//! The Instr mechanism allows stop/start/repstart sequences to be programmed
//! by the processor, and then carried out by the state machine at defined points
//! in the datastream.
//!
//! The "Final" field should be set for the final byte in a transfer.
//! This tells the state machine to ignore a NAK: if this field is not
//! set, then any NAK will cause the state machine to halt and interrupt.
//!
//! Autopull should be enabled, with a threshold of 16.
//! Autopush should be enabled, with a threshold of 8.
//! The TX FIFO should be accessed with halfword writes, to ensure
//! the data is immediately available in the OSR.
//!
//! Pin mapping:
//! - Input pin 0 is SDA
//! - Jump pin is SDA
//! - Side-set pin 0 is SCL
//! - Set pin 0 is SDA
//! - OUT pin 0 is SDA
//!
//! The OE outputs should be inverted in the system IO controls!
//! (It's possible for the inversion to be done in this program,
//! but costs 2 instructions: 1 for inversion, and one to cope
//! with the side effect of the MOV on TX shift counter.)

use embedded_hal::blocking::i2c::{self, AddressMode, Operation, SevenBitAddress, TenBitAddress};
use fugit::HertzU32;
use pio::{Instruction, InstructionOperands, SideSet};
use rp2040_hal::{
    gpio::{Disabled, DisabledConfig, Function, FunctionConfig, Pin, PinId, ValidPinMode},
    pio::{
        PIOExt, PinDir, PinState, Rx, ShiftDirection, StateMachine, StateMachineIndex, Tx,
        UninitStateMachine, PIO,
    },
};

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

const SIDESET: SideSet = SideSet::new(true, 1, true);

const NAK_BIT: u16 = 0b0000_0000_0000_0001;
const FINAL_BIT: u16 = 0b0000_0010_0000_0000;
const INSTR_OFFSET: usize = 10;
const DATA_OFFSET: usize = 1;

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    NoAcknowledgeAddress,
    NoAcknowledgeData,
}

/// Instance of I2C Controller.
pub struct I2C<'pio, P, SMI, SDA, SCL>
where
    P: PIOExt + FunctionConfig,
    SMI: StateMachineIndex,
    SDA: PinId,
    SCL: PinId,
    Function<P>: ValidPinMode<SDA> + ValidPinMode<SCL>,
{
    pio: &'pio mut PIO<P>,
    sm: StateMachine<(P, SMI), rp2040_hal::pio::Running>,
    tx: Tx<(P, SMI)>,
    rx: Rx<(P, SMI)>,
    _sda: Pin<SDA, Function<P>>,
    _scl: Pin<SCL, Function<P>>,
}

impl<'pio, P, SMI, SDA, SCL> I2C<'pio, P, SMI, SDA, SCL>
where
    P: PIOExt + FunctionConfig,
    SMI: StateMachineIndex,
    SDA: PinId,
    SCL: PinId,
    Function<P>: ValidPinMode<SDA> + ValidPinMode<SCL>,
{
    /// Creates a new instance of this driver.
    ///
    /// Note: the PIO must have been reset before using this driver.
    pub fn new<SdaDisabledConfig: DisabledConfig, SclDisabledConfig: DisabledConfig>(
        pio: &'pio mut PIO<P>,
        sda: rp2040_hal::gpio::Pin<SDA, Disabled<SdaDisabledConfig>>,
        scl: rp2040_hal::gpio::Pin<SCL, Disabled<SclDisabledConfig>>,
        sm: UninitStateMachine<(P, SMI)>,
        bus_freq: HertzU32,
        clock_freq: HertzU32,
    ) -> Self
    where
        Function<P>: ValidPinMode<SDA> + ValidPinMode<SCL>,
    {
        let mut program = pio_proc::pio_asm!(
            ".side_set 1 opt pindirs"

            "byte_nack:"
            "  jmp  y--     byte_end  ; continue if NAK was expected"
            "  irq  wait    0    rel  ; otherwise stop, ask for help (raises the irq line (0+SMI::id())%4)"
            "  jmp          byte_end  ; resumed, finalize the current byte"

            "byte_send:"
            "  out  y       1         ; Unpack FINAL"
            "  set  x       7         ; loop 8 times"

            "bitloop:"
            "  out  pindirs 1                [7] ; Serialize write data (all-ones is reading)"
            "  nop                    side 1 [2] ; SCL rising edge"
            //      polarity
            "  wait 1       gpio 0           [4] ; Allow clock to be stretched"
            "  in   pins 1                   [7] ; Sample read data in middle of SCL pulse"
            "  jmp  x--     bitloop   side 0 [7] ; SCL falling edge"

            // Handle ACK pulse
            "  out  pindirs 1                [7] ; On reads, we provide the ACK"
            "  nop                    side 1 [7] ; SCL rising edge"
            //      polarity
            "  wait 1       gpio 0           [7] ; Allow clock to be stretched"
            "  jmp  pin     byte_nack side 0 [2] ; Test SDA for ACK/NACK, fall through if ACK"

            "byte_end:"
            "  push block             ; flush the current byte in isr to the FIFO"

            ".wrap_target"
            "  out  x       6         ; Unpack Instr count"
            "  jmp  !x      byte_send ; Instr == 0, this is a data record"
            "  out  null    10        ; Instr > 0, remainder of this OSR is invalid"

            "do_exec:"
            "  out  exec    16        ; Execute one instruction per FIFO word"
            "  jmp  x--     do_exec"
            ".wrap"
        )
        .program;
        // patch the program to allow scl to be any pin
        program.code[7] |= u16::from(SCL::DYN.num);
        program.code[12] |= u16::from(SCL::DYN.num);

        // Install the program into PIO instruction memory.
        let installed = pio.install(&program).unwrap();
        let wrap_target = installed.wrap_target();

        // Configure the PIO state machine.
        let div = clock_freq.to_Hz() as f32 / ((32 * bus_freq).to_Hz() as f32);

        // init
        let (mut sm, rx, tx) = rp2040_hal::pio::PIOBuilder::from_program(installed)
            // use both RX & TX FIFO
            .buffers(rp2040_hal::pio::Buffers::RxTx)
            // Pin configuration
            .set_pins(SDA::DYN.num, 1)
            .out_pins(SDA::DYN.num, 1)
            .in_pin_base(SDA::DYN.num)
            .side_set_pin_base(SCL::DYN.num)
            .jmp_pin(SDA::DYN.num)
            // OSR config
            .out_shift_direction(rp2040_hal::pio::ShiftDirection::Left)
            .autopull(true)
            .pull_threshold(16)
            // ISR config
            .in_shift_direction(ShiftDirection::Left)
            .push_threshold(8)
            // clock config
            .clock_divisor(div)
            .build(sm);

        // enable pull up on SDA & SCL: idle bus
        let sda = sda.into_pull_up_input();
        let scl = scl.into_pull_up_input();

        // This will pull the bus high for a little bit of time
        sm.set_pins([
            (SCL::DYN.num, PinState::High),
            (SDA::DYN.num, PinState::High),
        ]);
        sm.set_pindirs([
            (SCL::DYN.num, PinDir::Output),
            (SDA::DYN.num, PinDir::Output),
        ]);

        // attach SDA pin to pio
        let mut sda: Pin<SDA, Function<P>> = sda.into_mode();
        // configure SDA pin as inverted
        sda.set_output_enable_override(rp2040_hal::gpio::OutputEnableOverride::Invert);

        // attach SCL pin to pio
        let mut scl: Pin<SCL, Function<P>> = scl.into_mode();
        // configure SCL pin as inverted
        scl.set_output_enable_override(rp2040_hal::gpio::OutputEnableOverride::Invert);

        // the PIO now keeps the pin as Input, we can set the pin state to Low.
        sm.set_pins([(SDA::DYN.num, PinState::Low), (SCL::DYN.num, PinState::Low)]);

        // Set the state machine on the entry point.
        sm.exec_instruction(
            InstructionOperands::JMP {
                condition: pio::JmpCondition::Always,
                address: wrap_target,
            }
            .encode(),
        );

        // enable
        let sm = sm.start();

        Self {
            pio,
            sm,
            tx,
            rx,
            _sda: sda,
            _scl: scl,
        }
    }

    fn has_errored(&mut self) -> bool {
        let mask = 1 << SMI::id();
        self.pio.get_irq_raw() & mask != 0
    }

    fn resume_after_error(&mut self) {
        self.tx.drain_fifo();
        self.pio.clear_irq(1 << SMI::id());
        while !self.sm.stalled() {
            let _ = self.rx.read();
        }
    }

    fn put(&mut self, data: u16) {
        while !self.tx.write_u16_replicated(data) {}
    }

    fn put_data(&mut self, data: u8, read_ack: bool, last: bool) {
        let final_field = if last { FINAL_BIT } else { 0 };
        let nak_field = if read_ack { NAK_BIT } else { 0 };
        let data_field = u16::from(data) << DATA_OFFSET;

        let word = final_field | data_field | nak_field;
        self.put(word);
    }

    fn put_instr_sequence<T, U>(&mut self, seq: T)
    where
        T: IntoIterator<IntoIter = U>,
        U: Iterator<Item = Instruction> + ExactSizeIterator,
    {
        let seq = seq.into_iter();
        assert!(seq.len() < 64);
        let n = seq.len() as u16;

        self.put((n - 1) << INSTR_OFFSET);
        for instr in seq {
            self.put(instr.encode(SIDESET));
        }
    }

    fn start(&mut self) {
        self.put_instr_sequence([SC1SD0, SC0SD0])
    }

    fn stop(&mut self) {
        if self.has_errored() {
            self.resume_after_error();
        }
        self.put_instr_sequence([SC0SD0, SC1SD0, SC1SD1])
    }

    fn restart(&mut self) {
        self.put_instr_sequence([SC0SD1, SC1SD1, SC1SD0, SC0SD0])
    }

    fn setup<A>(&mut self, address: A, read: bool, do_restart: bool) -> Result<(), Error>
    where
        A: Into<u16> + 'static,
    {
        // TODO: validate addr
        let address: u16 = address.into();

        // flush read fifo
        assert!(self.rx.read().is_none(), "rx FIFO shall be empty");

        // send start condition
        if !do_restart {
            self.start();
        } else {
            self.restart();
        }

        let read_flag = if read { 1 } else { 0 };

        // send address
        use core::any::TypeId;
        let a_tid = TypeId::of::<A>();
        let mut address_len: u32 = if TypeId::of::<SevenBitAddress>() == a_tid {
            let addr = (address << 1) | read_flag;
            self.put_data(addr as u8, true, false);
            1
        } else if TypeId::of::<TenBitAddress>() == a_tid {
            let addr_hi = 0xF0 | ((address >> 7) & 0x6) | read_flag;
            let addr_lo = address & 0xFF;
            self.put_data(addr_hi as u8, true, false);
            self.put_data(addr_lo as u8, true, false);
            2
        } else {
            panic!("Unsupported address type.");
        };

        while !(self.has_errored() || address_len == 0) {
            while address_len > 0 && self.rx.read().is_some() {
                address_len -= 1;
            }
        }

        if self.has_errored() {
            Err(Error::NoAcknowledgeAddress)
        } else {
            Ok(())
        }
    }

    fn read(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        assert!(
            !self.has_errored() && self.rx.is_empty(),
            "Invalid state in entering read: has_errored:{} rx.is_empty:{}",
            self.has_errored(),
            self.rx.is_empty()
        );

        let mut queued = 0;
        let mut iter = buffer.iter_mut();

        // while there are still bytes to queue
        while iter.len() != 0 && !self.has_errored() {
            if queued < iter.len() && !self.tx.is_full() {
                queued += 1;
                let last = queued == iter.len();
                self.put_data(0xFF, last, last);
            }

            if let Some(byte) = self.rx.read() {
                queued -= 1;
                if let Some(data) = iter.next() {
                    *data = (byte & 0xFF) as u8;
                }
            }
        }

        if self.has_errored() {
            Err(Error::NoAcknowledgeData)
        } else {
            Ok(())
        }
    }

    fn write<B>(&mut self, buffer: B) -> Result<(), Error>
    where
        B: IntoIterator<Item = u8>,
    {
        assert!(
            !self.has_errored() && self.rx.is_empty(),
            "Invalid state in entering write: has_errored:{} rx.is_empty:{}",
            self.has_errored(),
            self.rx.is_empty()
        );

        let mut queued = 0;
        let mut iter = buffer.into_iter().peekable();
        while let (Some(byte), false) = (iter.next(), self.has_errored()) {
            // ignore any received bytes
            if self.rx.read().is_some() {
                queued -= 1;
            }
            self.put_data(byte, true, iter.peek().is_none());
            queued += 1;
        }

        while !(queued == 0 || self.has_errored()) {
            if self.rx.read().is_some() {
                queued -= 1;
            }
        }

        if self.has_errored() {
            Err(Error::NoAcknowledgeData)
        } else {
            Ok(())
        }
    }
}

impl<A, P, SMI, SDA, SCL> i2c::Read<A> for I2C<'_, P, SMI, SDA, SCL>
where
    A: AddressMode + Into<u16> + 'static,
    P: PIOExt + FunctionConfig,
    SMI: StateMachineIndex,
    SDA: PinId,
    SCL: PinId,
    Function<P>: ValidPinMode<SDA> + ValidPinMode<SCL>,
{
    type Error = Error;

    fn read(&mut self, address: A, buffer: &mut [u8]) -> Result<(), Self::Error> {
        let mut res = self.setup(address, true, false);
        if res.is_ok() {
            res = self.read(buffer);
        }
        self.stop();
        res
    }
}

impl<A, P, SMI, SDA, SCL> i2c::WriteIter<A> for I2C<'_, P, SMI, SDA, SCL>
where
    A: AddressMode + Into<u16> + 'static,
    P: PIOExt + FunctionConfig,
    SMI: StateMachineIndex,
    SDA: PinId,
    SCL: PinId,
    Function<P>: ValidPinMode<SDA> + ValidPinMode<SCL>,
{
    type Error = Error;

    fn write<B>(&mut self, address: A, bytes: B) -> Result<(), Self::Error>
    where
        B: IntoIterator<Item = u8>,
    {
        let mut res = self.setup(address, false, false);
        if res.is_ok() {
            res = self.write(bytes);
        }
        self.stop();
        res
    }
}
impl<A, P, SMI, SDA, SCL> i2c::Write<A> for I2C<'_, P, SMI, SDA, SCL>
where
    A: AddressMode + Into<u16> + 'static,
    P: PIOExt + FunctionConfig,
    SMI: StateMachineIndex,
    SDA: PinId,
    SCL: PinId,
    Function<P>: ValidPinMode<SDA> + ValidPinMode<SCL>,
{
    type Error = Error;

    fn write(&mut self, address: A, buffer: &[u8]) -> Result<(), Self::Error> {
        <Self as i2c::WriteIter<A>>::write(self, address, buffer.iter().cloned())
    }
}

impl<A, P, SMI, SDA, SCL> i2c::WriteIterRead<A> for I2C<'_, P, SMI, SDA, SCL>
where
    A: AddressMode + Into<u16> + Clone + 'static,
    P: PIOExt + FunctionConfig,
    SMI: StateMachineIndex,
    SDA: PinId,
    SCL: PinId,
    Function<P>: ValidPinMode<SDA> + ValidPinMode<SCL>,
{
    type Error = Error;

    fn write_iter_read<B>(
        &mut self,
        address: A,
        bytes: B,
        buffer: &mut [u8],
    ) -> Result<(), Self::Error>
    where
        B: IntoIterator<Item = u8>,
    {
        let mut res = self.setup(address.clone(), false, false);
        if res.is_ok() {
            res = self.write(bytes);
        }
        if res.is_ok() {
            res = self.setup(address, true, true);
        }
        if res.is_ok() {
            res = self.read(buffer);
        }
        self.stop();
        res
    }
}
impl<A, P, SMI, SDA, SCL> i2c::WriteRead<A> for I2C<'_, P, SMI, SDA, SCL>
where
    A: AddressMode + Into<u16> + Clone + 'static,
    P: PIOExt + FunctionConfig,
    SMI: StateMachineIndex,
    SDA: PinId,
    SCL: PinId,
    Function<P>: ValidPinMode<SDA> + ValidPinMode<SCL>,
{
    type Error = Error;

    fn write_read(
        &mut self,
        address: A,
        bytes: &[u8],
        buffer: &mut [u8],
    ) -> Result<(), Self::Error> {
        <Self as i2c::WriteIterRead<A>>::write_iter_read(
            self,
            address,
            bytes.iter().cloned(),
            buffer,
        )
    }
}

impl<A, P, SMI, SDA, SCL> i2c::TransactionalIter<A> for I2C<'_, P, SMI, SDA, SCL>
where
    A: AddressMode + Into<u16> + Clone + 'static,
    P: PIOExt + FunctionConfig,
    SMI: StateMachineIndex,
    SDA: PinId,
    SCL: PinId,
    Function<P>: ValidPinMode<SDA> + ValidPinMode<SCL>,
{
    type Error = Error;

    fn exec_iter<'a, O>(&mut self, address: A, operations: O) -> Result<(), Self::Error>
    where
        O: IntoIterator<Item = Operation<'a>>,
    {
        let mut res = Ok(());
        let mut first = true;
        for op in operations {
            match op {
                Operation::Read(buf) => {
                    res = self.setup(address.clone(), true, !first);
                    if res.is_ok() {
                        res = self.read(buf);
                    }
                }
                Operation::Write(buf) => {
                    res = self.setup(address.clone(), false, !first);
                    if res.is_ok() {
                        res = self.write(buf.iter().cloned());
                    }
                }
            };
            if res.is_err() {
                break;
            }
            first = false;
        }
        self.stop();
        res
    }
}

impl<A, P, SMI, SDA, SCL> i2c::Transactional<A> for I2C<'_, P, SMI, SDA, SCL>
where
    A: AddressMode + Into<u16> + Clone + 'static,
    P: PIOExt + FunctionConfig,
    SMI: StateMachineIndex,
    SDA: PinId,
    SCL: PinId,
    Function<P>: ValidPinMode<SDA> + ValidPinMode<SCL>,
{
    type Error = Error;

    fn exec<'a>(
        &mut self,
        address: A,
        operations: &mut [Operation<'a>],
    ) -> Result<(), Self::Error> {
        let mut res = Ok(());
        let mut first = true;
        for op in operations {
            match op {
                Operation::Read(buf) => {
                    res = self.setup(address.clone(), true, !first);
                    if res.is_ok() {
                        res = self.read(buf);
                    }
                }
                Operation::Write(buf) => {
                    res = self.setup(address.clone(), false, !first);
                    if res.is_ok() {
                        res = self.write(buf.iter().cloned());
                    }
                }
            };
            if res.is_err() {
                break;
            }
            first = false;
        }
        self.stop();
        res
    }
}

#[cfg(feature = "eh1_0_alpha")]
mod eh1_0_alpha {
    use eh1_0_alpha::i2c::{blocking::Operation, AddressMode, ErrorKind, NoAcknowledgeSource};

    use crate::Error;

    use super::{Function, FunctionConfig, PIOExt, PinId, StateMachineIndex, ValidPinMode, I2C};

    impl eh1_0_alpha::i2c::Error for super::Error {
        fn kind(&self) -> ErrorKind {
            match self {
                Error::NoAcknowledgeAddress => {
                    ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address)
                }
                Error::NoAcknowledgeData => ErrorKind::NoAcknowledge(NoAcknowledgeSource::Data),
            }
        }
    }

    impl<P, SMI, SDA, SCL> eh1_0_alpha::i2c::ErrorType for I2C<'_, P, SMI, SDA, SCL>
    where
        P: PIOExt + FunctionConfig,
        SMI: StateMachineIndex,
        SDA: PinId,
        SCL: PinId,
        Function<P>: ValidPinMode<SDA> + ValidPinMode<SCL>,
    {
        type Error = super::Error;
    }

    impl<A, P, SMI, SDA, SCL> eh1_0_alpha::i2c::blocking::I2c<A> for I2C<'_, P, SMI, SDA, SCL>
    where
        A: AddressMode + Into<u16> + Clone + 'static,
        P: PIOExt + FunctionConfig,
        SMI: StateMachineIndex,
        SDA: PinId,
        SCL: PinId,
        Function<P>: ValidPinMode<SDA> + ValidPinMode<SCL>,
    {
        fn read(&mut self, address: A, buffer: &mut [u8]) -> Result<(), Self::Error> {
            let mut res = self.setup(address, true, false);
            if res.is_ok() {
                res = self.read(buffer);
            }
            self.stop();
            res
        }

        fn write(&mut self, address: A, bytes: &[u8]) -> Result<(), Self::Error> {
            self.write_iter(address, bytes.into_iter().cloned())
        }

        fn write_iter<B>(&mut self, address: A, bytes: B) -> Result<(), Self::Error>
        where
            B: IntoIterator<Item = u8>,
        {
            let mut res = self.setup(address, false, false);
            if res.is_ok() {
                res = self.write(bytes);
            }
            self.stop();
            res
        }

        fn write_read(
            &mut self,
            address: A,
            bytes: &[u8],
            buffer: &mut [u8],
        ) -> Result<(), Self::Error> {
            self.write_iter_read(address, bytes.into_iter().cloned(), buffer)
        }

        fn write_iter_read<B>(
            &mut self,
            address: A,
            bytes: B,
            buffer: &mut [u8],
        ) -> Result<(), Self::Error>
        where
            B: IntoIterator<Item = u8>,
        {
            let mut res = self.setup(address.clone(), false, false);
            if res.is_ok() {
                res = self.write(bytes);
            }
            if res.is_ok() {
                res = self.setup(address, true, true);
            }
            if res.is_ok() {
                res = self.read(buffer);
            }
            self.stop();
            res
        }

        fn transaction<'a>(
            &mut self,
            address: A,
            operations: &mut [Operation<'a>],
        ) -> Result<(), Self::Error> {
            let mut res = Ok(());
            let mut first = true;
            for op in operations {
                match op {
                    Operation::Read(buf) => {
                        res = self.setup(address.clone(), true, !first);
                        if res.is_ok() {
                            res = self.read(buf);
                        }
                    }
                    Operation::Write(buf) => {
                        res = self.setup(address.clone(), false, !first);
                        if res.is_ok() {
                            res = self.write(buf.iter().cloned());
                        }
                    }
                };
                if res.is_err() {
                    break;
                }
                first = false;
            }
            self.stop();
            res
        }

        fn transaction_iter<'a, O>(&mut self, address: A, operations: O) -> Result<(), Self::Error>
        where
            O: IntoIterator<Item = Operation<'a>>,
        {
            let mut res = Ok(());
            let mut first = true;
            for op in operations {
                match op {
                    Operation::Read(buf) => {
                        res = self.setup(address.clone(), true, !first);
                        if res.is_ok() {
                            res = self.read(buf);
                        }
                    }
                    Operation::Write(buf) => {
                        res = self.setup(address.clone(), false, !first);
                        if res.is_ok() {
                            res = self.write(buf.iter().cloned());
                        }
                    }
                };
                if res.is_err() {
                    break;
                }
                first = false;
            }
            self.stop();
            res
        }
    }
}
