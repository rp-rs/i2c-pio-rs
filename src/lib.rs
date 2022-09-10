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

use embedded_hal::blocking::i2c::{self, AddressMode, Operation, TenBitAddress};
use fugit::HertzU32;
use pio::{Instruction, InstructionOperands, SideSet};
use rp2040_hal::{
    gpio::{Disabled, DisabledConfig, Function, FunctionConfig, Pin, PinId, ValidPinMode},
    pio::{
        PIOExt, PinDir, PinState, Rx, ShiftDirection, StateMachine, StateMachineIndex, Tx,
        UninitStateMachine, ValidStateMachine, PIO,
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
pub struct Error;

/// Instance of I2C Controller.
pub struct I2C<'pio, P, SM, SDA, SCL>
where
    P: PIOExt + FunctionConfig,
    SM: ValidStateMachine<PIO = P>,
    SDA: PinId,
    SCL: PinId,
    Function<P>: ValidPinMode<SDA> + ValidPinMode<SCL>,
{
    pio: &'pio mut PIO<P>,
    sm: StateMachine<SM, rp2040_hal::pio::Running>,
    tx: Tx<SM>,
    rx: Rx<SM>,
    _sda: Pin<SDA, Function<P>>,
    _scl: Pin<SCL, Function<P>>,
}

impl<'pio, P, SM, SDA, SCL> I2C<'pio, P, (P, SM), SDA, SCL>
where
    P: PIOExt + FunctionConfig,
    SM: StateMachineIndex,
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
        sm: UninitStateMachine<(P, SM)>,
        bus_freq: HertzU32,
        clock_freq: HertzU32,
    ) -> I2C<'pio, P, (P, SM), SDA, SCL>
    where
        Function<P>: ValidPinMode<SDA> + ValidPinMode<SCL>,
    {
        // prepare the PIO program
        let mut a = pio::Assembler::<32>::new_with_side_set(SIDESET);

        let mut handle_nack = a.label();
        let mut send_byte = a.label();
        let mut bitloop = a.label();
        let mut wrap_target = a.label();
        let mut wrap_source = a.label();
        let mut do_exec = a.label();

        a.bind(&mut &mut handle_nack);
        // continue if NAK was expected
        a.jmp(pio::JmpCondition::YDecNonZero, &mut wrap_target);
        // Otherwise stop, ask for help (raises the irq line (0+SM::id())%4)
        a.irq(false, true, 0, true);

        a.bind(&mut send_byte);
        // Unpack Final
        a.out(pio::OutDestination::Y, 1);
        // loop 8 times
        a.set(pio::SetDestination::X, 7);

        // Send 1 byte
        a.bind(&mut bitloop);
        // Serialize write data (all-ones is reading)
        a.out_with_delay(pio::OutDestination::PINDIRS, 1, 7);
        // SCL rising edge
        a.nop_with_delay_and_side_set(2, 1);
        // Allow clock to be stretched
        a.wait_with_delay(1, pio::WaitSource::GPIO, SCL::DYN.num, false, 4);
        // Sample read data in middle of SCL pulse
        a.in_with_delay(pio::InSource::PINS, 1, 7);
        // SCL falling edge
        a.jmp_with_delay_and_side_set(pio::JmpCondition::XDecNonZero, &mut bitloop, 7, 0);

        // handle ACK pulse
        // On reads, we provide the ACK
        a.out_with_delay(pio::OutDestination::PINDIRS, 1, 7);
        // SCL risin edge
        a.nop_with_delay_and_side_set(7, 1);
        // Allow clock to be stretched
        a.wait_with_delay(1, pio::WaitSource::GPIO, SCL::DYN.num, false, 7);
        // Test SDA for ACK/NACK, fall through if ACK
        a.jmp_with_delay_and_side_set(pio::JmpCondition::PinHigh, &mut handle_nack, 2, 0);

        a.bind(&mut wrap_target);
        // Unpack Instr count
        a.out(pio::OutDestination::X, 6);
        // Instr == 0, this is a data record
        a.jmp(pio::JmpCondition::XIsZero, &mut send_byte);
        // Instr > 0, remainder of this OSR is invalid
        a.out(pio::OutDestination::NULL, 32);

        a.bind(&mut do_exec);
        // Execute one instruction per FIFO word
        a.out(pio::OutDestination::EXEC, 16);
        a.jmp(pio::JmpCondition::XDecNonZero, &mut do_exec);
        a.bind(&mut wrap_source);

        let program = a.assemble_with_wrap(wrap_source, wrap_target);

        // Install the program into PIO instruction memory.
        let installed = pio.install(&program).unwrap();
        let wrap_target = installed.wrap_target();

        // Configure the PIO state machine.
        let bit_freq = 32 * bus_freq;
        let mut int = clock_freq / bit_freq;
        let rem = clock_freq - (int * bit_freq);
        let frac = (rem * 256) / bit_freq;

        assert!(
            (1..=65536).contains(&int) && (int != 65536 || frac == 0),
            "The ratio between the bus frequency and the system clock must be within [1.0, 65536.0]."
        );

        // 65536.0 is represented as 0 in the pio's clock divider
        if int == 65536 {
            int = 0;
        }
        // Using lossy conversion because range have been checked
        let int: u16 = int as u16;
        let frac: u8 = frac as u8;

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
            .autopush(true)
            .push_threshold(8)
            // clock config
            .clock_divisor_fixed_point(int, frac)
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
}

impl<P, SM, SDA, SCL> I2C<'_, P, SM, SDA, SCL>
where
    P: PIOExt + FunctionConfig,
    SM: ValidStateMachine<PIO = P>,
    SDA: PinId,
    SCL: PinId,
    Function<P>: ValidPinMode<SDA> + ValidPinMode<SCL>,
{
    fn has_errored(&mut self) -> bool {
        let mask = 1 << SM::id();
        self.pio.get_irq_raw() & mask != 0
    }

    fn put(&mut self, data: u16) {
        while !self.tx.write_u16_replicated(data) {}
    }

    fn put_data(&mut self, data: u8, read_ack: bool, last: bool) {
        let final_field = if last { FINAL_BIT } else { 0 };
        let nak_field = if read_ack { NAK_BIT } else { 0 };
        let data_field = u16::from(data) << DATA_OFFSET;

        let word = final_field | data_field | nak_field;
        while !self.tx.write_u16_replicated(word) {}
    }

    fn put_instr_sequence<T, U>(&mut self, seq: T)
    where
        T: IntoIterator<IntoIter = U>,
        U: Iterator<Item = Instruction> + ExactSizeIterator,
    {
        assert!(
            self.tx.has_stalled(),
            "put_instr may only be called when the state machine is idle"
        );

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
        self.put_instr_sequence([SC0SD0, SC1SD0, SC1SD1])
    }

    fn restart(&mut self) {
        self.put_instr_sequence([SC0SD1, SC1SD1, SC1SD0, SC0SD0])
    }

    fn wait_idle(&mut self) {
        self.tx.clear_stalled_flag();
        while (!self.tx.has_stalled() || !self.tx.is_empty()) && !self.has_errored() {
            // discard rx fifo to a prevent RX stall
            let _ = self.rx.read();
        }
    }

    fn resume_after_error(&mut self) {
        // drain tx_fifo
        self.tx.drain_fifo();
        self.sm.restart();
        self.pio.clear_irq(1 << SM::id());
    }

    fn setup<A>(&mut self, address: A, read: bool, do_restart: bool)
    where
        A: AddressMode + Into<u16> + 'static,
    {
        // TODO: validate addr
        let address: u16 = address.into();

        // send start condition
        if !do_restart {
            self.start();
        } else {
            self.restart();
        }

        // flush read fifo
        assert!(self.rx.read().is_none(), "rx FIFO shall be empty");

        let read_flag = if read { 1 } else { 0 };

        // send address
        if core::any::TypeId::of::<A>() == core::any::TypeId::of::<TenBitAddress>() {
            let addr_hi = 0xF0 | ((address >> 7) & 0x6) | read_flag;
            let addr_lo = address & 0xFF;

            self.put_data(addr_hi as u8, true, false);
            self.put_data(addr_lo as u8, true, false);
        } else {
            let addr = (address << 1) | read_flag;
            self.put_data(addr as u8, true, false);
        }
    }

    fn read<A>(&mut self, buffer: &mut [u8]) {
        let mut tx_remain = buffer.len();
        let mut iter = buffer.iter_mut();
        // It's kind of an abuse but it'll do for now
        let mut ignore_cnt = core::mem::size_of::<A>();

        while iter.len() != 0 && !self.has_errored() {
            if !self.tx.is_full() && tx_remain > 0 {
                tx_remain -= 1;
                let last = tx_remain == 0;
                self.put_data(0xFF, last, last);
            }
            if let Some(data) = self.rx.read() {
                if ignore_cnt > 0 {
                    ignore_cnt -= 1;
                } else if let Some(byte) = iter.next() {
                    *byte = (data & 0xFF) as u8;
                }
            }
        }
    }

    fn write<B>(&mut self, buffer: B)
    where
        B: IntoIterator<Item = u8>,
    {
        let mut iter = buffer.into_iter().peekable();
        while let Some(byte) = iter.next() {
            if self.has_errored() {
                break;
            }
            self.put_data(byte, true, iter.peek().is_none());
            let _ = self.rx.read();
        }
        self.wait_idle();
        // flush RX FIFO
        while self.rx.read().is_some() {}
    }

    fn finish(&mut self) -> Result<(), Error> {
        let res = if self.has_errored() {
            self.resume_after_error();
            self.wait_idle();
            Err(Error)
        } else {
            Ok(())
        };
        self.stop();
        res
    }
}

impl<A, P, SM, SDA, SCL> i2c::Read<A> for I2C<'_, P, SM, SDA, SCL>
where
    A: AddressMode + Into<u16> + 'static,
    P: PIOExt + FunctionConfig,
    SM: ValidStateMachine<PIO = P>,
    SDA: PinId,
    SCL: PinId,
    Function<P>: ValidPinMode<SDA> + ValidPinMode<SCL>,
{
    type Error = Error;

    fn read(&mut self, address: A, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.setup(address, true, false);
        self.read::<A>(buffer);
        self.finish()
    }
}

impl<A, P, SM, SDA, SCL> i2c::WriteIter<A> for I2C<'_, P, SM, SDA, SCL>
where
    A: Copy + AddressMode + Into<u16> + 'static,
    P: PIOExt + FunctionConfig,
    SM: ValidStateMachine<PIO = P>,
    SDA: PinId,
    SCL: PinId,
    Function<P>: ValidPinMode<SDA> + ValidPinMode<SCL>,
{
    type Error = Error;

    fn write<B>(&mut self, address: A, bytes: B) -> Result<(), Self::Error>
    where
        B: IntoIterator<Item = u8>,
    {
        self.setup(address, false, false);
        self.write(bytes);
        self.finish()
    }
}
impl<A, P, SM, SDA, SCL> i2c::Write<A> for I2C<'_, P, SM, SDA, SCL>
where
    A: AddressMode + Copy + Into<u16> + 'static,
    P: PIOExt + FunctionConfig,
    SM: ValidStateMachine<PIO = P>,
    SDA: PinId,
    SCL: PinId,
    Function<P>: ValidPinMode<SDA> + ValidPinMode<SCL>,
{
    type Error = Error;

    fn write(&mut self, address: A, buffer: &[u8]) -> Result<(), Self::Error> {
        <Self as i2c::WriteIter<A>>::write(self, address, buffer.iter().cloned())
    }
}

impl<A, P, SM, SDA, SCL> i2c::WriteIterRead<A> for I2C<'_, P, SM, SDA, SCL>
where
    A: Copy + AddressMode + Into<u16> + 'static,
    P: PIOExt + FunctionConfig,
    SM: ValidStateMachine<PIO = P>,
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
        self.setup(address, false, false);
        self.write(bytes);
        if !self.has_errored() {
            self.setup(address, true, true);
            self.read::<A>(buffer);
        }
        self.finish()
    }
}
impl<A, P, SM, SDA, SCL> i2c::WriteRead<A> for I2C<'_, P, SM, SDA, SCL>
where
    A: Copy + AddressMode + Into<u16> + 'static,
    P: PIOExt + FunctionConfig,
    SM: ValidStateMachine<PIO = P>,
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

impl<A, P, SM, SDA, SCL> i2c::TransactionalIter<A> for I2C<'_, P, SM, SDA, SCL>
where
    A: Copy + AddressMode + Into<u16> + 'static,
    P: PIOExt + FunctionConfig,
    SM: ValidStateMachine<PIO = P>,
    SDA: PinId,
    SCL: PinId,
    Function<P>: ValidPinMode<SDA> + ValidPinMode<SCL>,
{
    type Error = Error;

    fn exec_iter<'a, O>(&mut self, address: A, operations: O) -> Result<(), Self::Error>
    where
        O: IntoIterator<Item = Operation<'a>>,
    {
        let mut first = true;
        for op in operations {
            match op {
                Operation::Read(buf) => {
                    self.setup(address, true, !first);
                    self.read::<A>(buf);
                }
                Operation::Write(buf) => {
                    self.setup(address, false, !first);
                    self.write(buf.iter().cloned());
                }
            };
            first = false;
            if self.has_errored() {
                break;
            }
        }
        self.finish()
    }
}

impl<A, P, SM, SDA, SCL> i2c::Transactional<A> for I2C<'_, P, SM, SDA, SCL>
where
    A: Copy + AddressMode + Into<u16> + 'static,
    P: PIOExt + FunctionConfig,
    SM: ValidStateMachine<PIO = P>,
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
        let mut first = true;
        for op in operations {
            match op {
                Operation::Read(buf) => {
                    self.setup(address, true, !first);
                    self.read::<A>(buf);
                }
                Operation::Write(buf) => {
                    self.setup(address, false, !first);
                    self.write(buf.iter().cloned());
                }
            };
            first = false;
            if self.has_errored() {
                break;
            }
        }
        self.finish()
    }
}

#[cfg(feature = "eh1_0_alpha")]
mod eh1_0_alpha {
    use super::Error;
    use super::I2C;

    impl<A, P, SM, SDA, SCL> eh1::Write for I2C<'_, P, SM, SDA, SCL>
    where
        A: Copy + AddressMode + Into<u16> + 'static,
        P: PIOExt + FunctionConfig,
        SM: ValidStateMachine<PIO = P>,
        SDA: PinId,
        SCL: PinId,
        Function<P>: ValidPinMode<SDA> + ValidPinMode<SCL>,
    {
        type Error = Error;

        fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
            Write::write(self, addr, bytes)
        }
    }
    impl<T: Deref<Target = Block>, PINS> eh1::WriteRead for I2C<T, PINS, Controller> {
        type Error = Error;

        fn write_read(&mut self, addr: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Error> {
            WriteRead::write_read(self, addr, bytes, buffer)
        }
    }
    impl<T: Deref<Target = Block>, PINS> eh1::Read for I2C<T, PINS, Controller> {
        type Error = Error;

        fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Error> {
            Read::read(self, addr, buffer)
        }
    }
}
