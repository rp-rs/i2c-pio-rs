use either::Either::{Left, Right};
use embedded_hal::i2c::{ErrorKind, NoAcknowledgeSource, Operation};

use crate::{CmdWord, Error, ValidAddressMode};

use super::{AnyPin, PIOExt, StateMachineIndex, I2C};

impl<P, SMI, SDA, SCL> I2C<'_, P, SMI, SDA, SCL>
where
    P: PIOExt,
    SMI: StateMachineIndex,
    SDA: AnyPin,
    SCL: AnyPin,
{
    pub fn write_iter<B, A>(&mut self, address: A, bytes: B) -> Result<(), Error>
    where
        A: ValidAddressMode,
        B: IntoIterator<Item = u8> + Clone,
    {
        self.process_queue(
            super::setup(address, false, false).chain(bytes.into_iter().map(CmdWord::write)),
        )?;
        self.generate_stop();
        Ok(())
    }

    pub fn write_iter_read<A, B>(
        &mut self,
        address: A,
        bytes: B,
        buffer: &mut [u8],
    ) -> Result<(), Error>
    where
        A: ValidAddressMode,
        B: IntoIterator<Item = u8>,
    {
        self.process_queue(
            super::setup(address, false, false)
                .chain(bytes.into_iter().map(CmdWord::write))
                .chain(super::setup(address, true, true))
                .chain(buffer.iter_mut().map(CmdWord::read)),
        )?;
        self.generate_stop();
        Ok(())
    }

    pub fn transaction_iter<'a, A, O>(&mut self, address: A, operations: O) -> Result<(), Error>
    where
        A: ValidAddressMode,
        O: IntoIterator<Item = Operation<'a>>,
    {
        let mut first = true;
        for op in operations {
            let iter = match op {
                Operation::Read(buf) => Left(
                    super::setup(address, true, !first).chain(buf.iter_mut().map(CmdWord::read)),
                ),
                Operation::Write(buf) => Right(
                    super::setup(address, false, !first)
                        .chain(buf.iter().cloned().map(CmdWord::write)),
                ),
            };
            self.process_queue(iter)?;
            first = false;
        }
        self.generate_stop();
        Ok(())
    }
}

impl embedded_hal::i2c::Error for super::Error {
    fn kind(&self) -> ErrorKind {
        match self {
            Error::NoAcknowledgeAddress => ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address),
            Error::NoAcknowledgeData => ErrorKind::NoAcknowledge(NoAcknowledgeSource::Data),
            Error::BusContention => ErrorKind::ArbitrationLoss,
        }
    }
}

impl<P, SMI, SDA, SCL> embedded_hal::i2c::ErrorType for I2C<'_, P, SMI, SDA, SCL>
where
    P: PIOExt,
    SMI: StateMachineIndex,
    SDA: AnyPin,
    SCL: AnyPin,
{
    type Error = super::Error;
}

impl<A, P, SMI, SDA, SCL> embedded_hal::i2c::I2c<A> for I2C<'_, P, SMI, SDA, SCL>
where
    A: ValidAddressMode,
    P: PIOExt,
    SMI: StateMachineIndex,
    SDA: AnyPin,
    SCL: AnyPin,
{
    fn transaction(
        &mut self,
        address: A,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        let mut first = true;
        for op in operations {
            let iter = match op {
                Operation::Read(buf) => Left(
                    super::setup(address, true, !first).chain(buf.iter_mut().map(CmdWord::read)),
                ),
                Operation::Write(buf) => Right(
                    super::setup(address, false, !first)
                        .chain(buf.iter().cloned().map(CmdWord::write)),
                ),
            };
            self.process_queue(iter)?;
            first = false;
        }
        self.generate_stop();
        Ok(())
    }
}
