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
use core::{
    cell::RefCell,
    iter::{once, Peekable},
};

use either::Either::{Left, Right};
use fugit::HertzU32;
use heapless::Deque;
use i2c_cmd::{restart, start, CmdWord, Data};
use pio::Instruction;
use rp2040_hal::{
    gpio::{
        AnyPin, Function, FunctionNull, OutputEnableOverride, Pin, PinId, PullType, PullUp,
        ValidFunction,
    },
    pio::{
        PIOExt, PinDir, PinState, PioIRQ, Running, Rx, ShiftDirection, StateMachine,
        StateMachineIndex, Tx, UninitStateMachine, PIO,
    },
};

use crate::i2c_cmd::stop;

mod eh0_2;
mod eh1_0;
mod i2c_cmd;

mod utils;

/// Length of an address.
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AddressLength {
    _7,
    _10,
}
pub trait ValidAddressMode:
    Copy + Into<u16> + embedded_hal::i2c::AddressMode + embedded_hal_0_2::blocking::i2c::AddressMode
{
    fn address_len() -> AddressLength;
}
macro_rules! impl_valid_addr {
    ($t:path => $e:expr) => {
        impl ValidAddressMode for $t {
            fn address_len() -> AddressLength {
                $e
            }
        }
    };
}
impl_valid_addr!(u8 => AddressLength::_7);
impl_valid_addr!(u16 => AddressLength::_10);
// `embedded_hal`s’ SevenBitAddress and TenBitAddress are aliases to u8 and u16 respectively.
//impl_valid_addr!(embedded_hal::i2c::SevenBitAddress => AddressLength::_7);
//impl_valid_addr!(embedded_hal::i2c::TenBitAddress => AddressLength::_10);
//impl_valid_addr!(embedded_hal_0_2::blocking::i2c::SevenBitAddress => AddressLength::_7);
//impl_valid_addr!(embedded_hal_0_2::blocking::i2c::TenBitAddress => AddressLength::_10);

fn setup<'b, A: ValidAddressMode>(
    address: A,
    read: bool,
    do_restart: bool,
) -> impl Iterator<Item = CmdWord<'b>> {
    let read_flag = if read { 1 } else { 0 };
    let address: u16 = address.into();
    let address = match A::address_len() {
        AddressLength::_7 => {
            let address_and_flag = ((address as u8) << 1) | read_flag;
            Left(once(address_and_flag).map(CmdWord::address))
        }
        AddressLength::_10 => {
            let addr_hi = 0xF0 | (((address & 0x0700) >> 7) as u8);
            let addr_lo = (address & 0xFF) as u8;

            Right(if read {
                let addr_write = [addr_hi, addr_lo].into_iter().map(CmdWord::address);
                let addr_read = once(addr_hi | read_flag).map(CmdWord::address);
                Left(addr_write.chain(restart()).chain(addr_read))
            } else {
                let address = [addr_hi | read_flag, addr_lo].into_iter();
                Right(address.map(CmdWord::address))
            })
        }
    };

    if do_restart {
        Left(restart())
    } else {
        Right(start())
    }
    .chain(address)
}

#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    NoAcknowledgeAddress,
    NoAcknowledgeData,
    BusContention,
}

/// Instance of I2C Controller.
pub struct I2C<'pio, P, SMI, SDA, SCL>
where
    P: PIOExt,
    SMI: StateMachineIndex,
    SDA: AnyPin,
    SCL: AnyPin,
{
    pio: &'pio mut PIO<P>,
    sm: StateMachine<(P, SMI), rp2040_hal::pio::Running>,
    tx: Tx<(P, SMI)>,
    rx: Rx<(P, SMI)>,
    sda: (Pin<SDA::Id, P::PinFunction, PullUp>, OutputEnableOverride),
    scl: (Pin<SCL::Id, P::PinFunction, PullUp>, OutputEnableOverride),
}

impl<'pio, P, SMI, SDA, SCL> I2C<'pio, P, SMI, SDA, SCL>
where
    P: PIOExt,
    SMI: StateMachineIndex,
    SDA: AnyPin,
    SCL: AnyPin,
{
    /// Creates a new instance of this driver.
    ///
    /// Note: the PIO must have been reset before using this driver.
    pub fn new(
        pio: &'pio mut PIO<P>,
        sda: SDA,
        scl: SCL,
        sm: UninitStateMachine<(P, SMI)>,
        bus_freq: HertzU32,
        clock_freq: HertzU32,
    ) -> Self
    where
        SDA: AnyPin<Function = FunctionNull>,
        SDA::Id: ValidFunction<P::PinFunction>,
        SCL: AnyPin<Function = FunctionNull>,
        SCL::Id: ValidFunction<P::PinFunction>,
    {
        let (sda, scl): (SDA::Type, SCL::Type) = (sda.into(), scl.into());

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
        program.code[7] |= u16::from(scl.id().num);
        program.code[12] |= u16::from(scl.id().num);

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
        let (mut sm, rx, tx) = rp2040_hal::pio::PIOBuilder::from_installed_program(installed)
            // use both RX & TX FIFO
            .buffers(rp2040_hal::pio::Buffers::RxTx)
            // Pin configuration
            .set_pins(sda.id().num, 1)
            .out_pins(sda.id().num, 1)
            .in_pin_base(sda.id().num)
            .side_set_pin_base(scl.id().num)
            .jmp_pin(sda.id().num)
            // OSR config
            .out_shift_direction(ShiftDirection::Left)
            .autopull(true)
            .pull_threshold(16)
            // ISR config
            .in_shift_direction(ShiftDirection::Left)
            .push_threshold(8)
            // clock config
            .clock_divisor_fixed_point(int, frac)
            .build(sm);

        // enable pull up on SDA & SCL: idle bus
        let sda: Pin<_, _, PullUp> = sda.into_pull_type();
        let scl: Pin<_, _, PullUp> = scl.into_pull_type();
        let sda_override = sda.get_output_enable_override();
        let scl_override = scl.get_output_enable_override();

        // This will pull the bus high for a little bit of time
        sm.set_pins([
            (scl.id().num, PinState::High),
            (sda.id().num, PinState::High),
        ]);
        sm.set_pindirs([
            (scl.id().num, PinDir::Output),
            (sda.id().num, PinDir::Output),
        ]);

        // attach SDA pin to pio
        let mut sda: Pin<SDA::Id, P::PinFunction, PullUp> = sda.into_function();
        // configure SDA pin as inverted
        sda.set_output_enable_override(OutputEnableOverride::Invert);

        // attach SCL pin to pio
        let mut scl: Pin<SCL::Id, P::PinFunction, PullUp> = scl.into_function();
        // configure SCL pin as inverted
        scl.set_output_enable_override(OutputEnableOverride::Invert);

        // the PIO now keeps the pin as Input, we can set the pin state to Low.
        sm.set_pins([(sda.id().num, PinState::Low), (scl.id().num, PinState::Low)]);

        // Set the state machine on the entry point.
        sm.exec_instruction(pio::Instruction {
            operands: pio::InstructionOperands::JMP {
                condition: pio::JmpCondition::Always,
                address: wrap_target,
            },
            delay: 0,
            side_set: None,
        });

        // enable
        let sm = sm.start();

        Self {
            pio,
            sm,
            tx,
            rx,
            sda: (sda, sda_override),
            scl: (scl, scl_override),
        }
    }

    fn has_irq(pio: &mut PIO<P>) -> bool {
        let mask = 1 << SMI::id();
        pio.get_irq_raw() & mask != 0
    }

    /// While this method contains some loop, none of them are expected to run for more than a few
    /// cycles. This works for both sync & async operations.
    fn err_with(
        pio: &mut PIO<P>,
        sm: &mut StateMachine<(P, SMI), Running>,
        rx: &mut Rx<(P, SMI)>,
        tx: &mut Tx<(P, SMI)>,
    ) {
        // Clear Tx FiFo
        sm.drain_tx_fifo();
        // clear RX FiFo
        while rx.read().is_some() {}
        // wait for the state machine to either stall on pull or block on irq
        tx.clear_stalled_flag();
        while !(tx.has_stalled() || Self::has_irq(pio)) {}

        // Clear OSR
        if Self::has_irq(pio) {
            sm.exec_instruction(Instruction {
                operands: pio::InstructionOperands::OUT {
                    destination: pio::OutDestination::NULL,
                    bit_count: 16,
                },
                delay: 0,
                side_set: None,
            });
            // resume pio driver
            pio.clear_irq(1 << SMI::id());
        }
        // generate stop condition
        Self::generate_stop(tx);
    }

    fn generate_stop(tx: &mut Tx<(P, SMI)>) {
        // this driver checks for acknowledge error and/or expects data back, so by the time a stop
        // is generated, the tx fifo should be empty.
        assert!(tx.is_empty(), "TX FIFO is empty");

        stop().for_each(|encoded| {
            tx.write_u16_replicated(encoded);
        });
        tx.clear_stalled_flag();
        while !tx.has_stalled() {}
    }

    // - TX FIFO depth (cmd waiting to be sent)
    // - OSR
    // - RX FIFO input waiting to be processed
    fn process_queue_steps<'b>(
        &mut self,
        output: &mut Peekable<impl Iterator<Item = CmdWord<'b>>>,
        input: &mut Deque<Data<'b>, 9>,
    ) -> Result<(), Error> {
        // if there is room in the tx fifo
        if !self.tx.is_full() {
            if let Some(mut word) = output.next() {
                let last = matches!(
                    (&mut word, output.peek()),
                    (CmdWord::Data(_), None) | (CmdWord::Data(_), Some(CmdWord::Raw(_)))
                );
                let word_u16 = word.encode(last);
                self.tx.write_u16_replicated(word_u16);
                if let CmdWord::Data(d) = word {
                    input.push_back(d).expect("`input` is not full");
                }
            }
        }

        if let Some(word) = self.rx.read() {
            let word = (word & 0xFF) as u8;
            if let Some(d) = input.pop_front() {
                match d.byte {
                    Left(exp) if word != exp => {
                        Self::err_with(self.pio, &mut self.sm, &mut self.rx, &mut self.tx);
                        return Err(Error::BusContention);
                    }
                    Right(inp) => *inp = word,
                    _ => {}
                }
            }
        } else if Self::has_irq(self.pio) {
            // the byte that err’ed isn’t in the rx fifo. Once we’re done clearing them, we
            // know the head of the queue is the byte that failed.
            let Some(d) = input.pop_front() else {
                unreachable!("There cannot be a failure without a transmission")
            };
            Self::err_with(self.pio, &mut self.sm, &mut self.rx, &mut self.tx);
            return Err(if d.is_address {
                Error::NoAcknowledgeAddress
            } else {
                Error::NoAcknowledgeData
            });
        }
        Ok(())
    }

    fn process_queue<'b>(
        &mut self,
        queue: impl IntoIterator<Item = CmdWord<'b>>,
    ) -> Result<(), Error> {
        let mut output = queue.into_iter().peekable();
        let mut input = Deque::new();

        // while we’re not done sending/receiving
        while output.peek().is_some() || !input.is_empty() {
            self.process_queue_steps(&mut output, &mut input)?;
        }
        Ok(())
    }

    async fn receive(
        in_flight: &RefCell<Deque<Data<'_>, 10>>,
        rx: &mut Rx<(P, SMI)>,
    ) -> Result<(), Error> {
        while !in_flight.borrow().is_empty() {
            let word = (rx.async_read(PioIRQ::Irq0).await & 0xFF) as u8;
            let Some(expected) = in_flight.borrow_mut().pop_front() else {
                unreachable!("We cannot receive more data than requested");
            };
            //defmt::trace!("received: {}->{:x}", expected, word);
            match expected.byte {
                Left(exp) if word != exp => {
                    return Err(Error::BusContention);
                }
                Right(inp) => *inp = word,
                _ => {}
            }
        }
        Ok(())
    }

    async fn transmit<'b>(
        in_flight: &RefCell<Deque<Data<'b>, 10>>,
        queue: impl IntoIterator<Item = CmdWord<'b>>,
        tx: &mut Tx<(P, SMI)>,
    ) {
        let mut output = queue.into_iter().peekable();
        while let Some(mut word) = output.next() {
            let last = matches!(
                (&mut word, output.peek()),
                (CmdWord::Data(_), None) | (CmdWord::Data(_), Some(CmdWord::Raw(_)))
            );
            let word_u16 = word.encode(last);
            tx.async_write_u16_replicated(PioIRQ::Irq0, word_u16).await;
            if let CmdWord::Data(d) = word {
                in_flight
                    .borrow_mut()
                    .push_back(d)
                    .expect("`input` is full");
            }
        }
    }

    async fn process_queue_async<'b>(
        &mut self,
        queue: impl IntoIterator<Item = CmdWord<'b>>,
    ) -> Result<(), Error> {
        use futures::FutureExt;
        // the Deque size should be { PIO::TX_FIFO_SIZE + 1 } but we can’t use const generics in
        // there yet.
        let in_flight = RefCell::new(Deque::new());

        // wrap the TX in a RefCell to allow for mutable borrows in async closures
        // There might be better solutions to this, but this is the simplest one that works.
        let ref_tx = RefCell::new(&mut self.tx);

        assert_eq!(self.rx.read(), None, "RX FIFO should be empty");

        #[allow(clippy::await_holding_refcell_ref)]
        let fut = core::pin::pin!(async {
            // this refcell borrow is kept through an await point. The other reference is passed to
            // the cancel guard which is only used *if* the future is cancelled.
            let tx = &mut *ref_tx.borrow_mut();
            let rx = &mut self.rx;
            let mut irq = self.pio.irq0();
            futures::select_biased! {
                _ = futures::future::join(
                    Self::transmit(&in_flight, queue, tx),
                    Self::receive(&in_flight, rx)
                ).fuse() => Ok(()),
                _ = irq.sm_interrupt(SMI::id() as u8).fuse() => {
                    // the byte that err’ed isn’t in the rx fifo. Once we’re done clearing them, we
                    // know the head of the queue is the byte that failed.
                    let mut in_flight = in_flight.borrow_mut();

                    let Some(d) = in_flight.pop_front() else {
                        unreachable!("There cannot be a failure without a transmission")
                    };

                    Self::err_with(self.pio, &mut self.sm, rx, tx);
                    // Purge the rx fifo
                    while rx.read().is_some() {}
                    Err(if d.is_address {
                        Error::NoAcknowledgeAddress
                    } else {
                        Error::NoAcknowledgeData
                    })
                }
            }
        });

        // wrap all the futures in a cancel guard to make sure a Stop condition is generated and
        // that the state machine is left in a clean state.
        crate::utils::CancelGuard::new(&ref_tx, fut, |me: &RefCell<&mut Tx<(P, SMI)>>| {
            Self::generate_stop(*me.borrow_mut());
        })
        .await
    }
}

impl<P, SMI, SDA, SCL> I2C<'_, P, SMI, SDA, SCL>
where
    P: PIOExt,
    SMI: StateMachineIndex,
    SDA: AnyPin,
    SDA::Id: ValidFunction<SDA::Function>,
    SCL: AnyPin,
    SCL::Id: ValidFunction<SCL::Function>,
{
    fn reset_pin<I, F, T>(
        (mut pin, override_): (Pin<I, P::PinFunction, PullUp>, OutputEnableOverride),
    ) -> Pin<I, F, T>
    where
        I: PinId,
        F: Function,
        T: PullType,
        I: ValidFunction<F>,
    {
        // Prevent glitches during reconfiguration
        pin.set_output_enable_override(OutputEnableOverride::Disable);
        // reconfigure the pin
        let mut pin = pin.reconfigure();
        // revert to normal operation
        pin.set_output_enable_override(override_);
        pin
    }

    /// Frees the state machine and pins.
    #[allow(clippy::type_complexity)]
    pub fn free(self) -> ((SDA::Type, SCL::Type), UninitStateMachine<(P, SMI)>) {
        let Self {
            pio,
            sm,
            tx,
            rx,
            sda,
            scl,
            ..
        } = self;
        let (uninit, program) = sm.uninit(rx, tx);
        pio.uninstall(program);

        let scl = Self::reset_pin(scl);
        let sda = Self::reset_pin(sda);

        ((sda, scl), uninit)
    }
}
