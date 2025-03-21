use crate::*;
use embedded_hal_0_2::blocking::i2c::{self, Operation};
impl<A, P, SMI, SDA, SCL> i2c::Read<A> for I2C<'_, P, SMI, SDA, SCL>
where
    A: ValidAddressMode,
    P: PIOExt,
    SMI: StateMachineIndex,
    SDA: AnyPin,
    SCL: AnyPin,
{
    type Error = Error;

    fn read(&mut self, address: A, buffer: &mut [u8]) -> Result<(), Self::Error> {
        let iter = super::setup(address, true, false).chain(buffer.iter_mut().map(CmdWord::read));
        self.process_queue(iter)?;
        Self::generate_stop(&mut self.tx);
        Ok(())
    }
}

impl<A, P, SMI, SDA, SCL> i2c::WriteIter<A> for I2C<'_, P, SMI, SDA, SCL>
where
    A: ValidAddressMode,
    P: PIOExt,
    SMI: StateMachineIndex,
    SDA: AnyPin,
    SCL: AnyPin,
{
    type Error = Error;

    fn write<B>(&mut self, address: A, bytes: B) -> Result<(), Self::Error>
    where
        B: IntoIterator<Item = u8>,
    {
        let iter = super::setup(address, false, false).chain(bytes.into_iter().map(CmdWord::write));
        self.process_queue(iter)?;
        Self::generate_stop(&mut self.tx);
        Ok(())
    }
}
impl<A, P, SMI, SDA, SCL> i2c::Write<A> for I2C<'_, P, SMI, SDA, SCL>
where
    A: ValidAddressMode,
    P: PIOExt,
    SMI: StateMachineIndex,
    SDA: AnyPin,
    SCL: AnyPin,
{
    type Error = Error;

    fn write(&mut self, address: A, buffer: &[u8]) -> Result<(), Self::Error> {
        <Self as i2c::WriteIter<A>>::write(self, address, buffer.iter().cloned())
    }
}

impl<A, P, SMI, SDA, SCL> i2c::WriteIterRead<A> for I2C<'_, P, SMI, SDA, SCL>
where
    A: ValidAddressMode,
    P: PIOExt,
    SMI: StateMachineIndex,
    SDA: AnyPin,
    SCL: AnyPin,
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
        self.process_queue(
            super::setup(address, false, false)
                .chain(bytes.into_iter().map(CmdWord::write))
                .chain(super::setup(address, true, true))
                .chain(buffer.iter_mut().map(CmdWord::read)),
        )?;
        Self::generate_stop(&mut self.tx);
        Ok(())
    }
}
impl<A, P, SMI, SDA, SCL> i2c::WriteRead<A> for I2C<'_, P, SMI, SDA, SCL>
where
    A: ValidAddressMode,
    P: PIOExt,
    SMI: StateMachineIndex,
    SDA: AnyPin,
    SCL: AnyPin,
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
    A: ValidAddressMode,
    P: PIOExt,
    SMI: StateMachineIndex,
    SDA: AnyPin,
    SCL: AnyPin,
{
    type Error = Error;

    fn exec_iter<'a, O>(&mut self, address: A, operations: O) -> Result<(), Self::Error>
    where
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
        Self::generate_stop(&mut self.tx);
        Ok(())
    }
}

impl<A, P, SMI, SDA, SCL> i2c::Transactional<A> for I2C<'_, P, SMI, SDA, SCL>
where
    A: ValidAddressMode,
    P: PIOExt,
    SMI: StateMachineIndex,
    SDA: AnyPin,
    SCL: AnyPin,
{
    type Error = Error;

    fn exec(&mut self, address: A, operations: &mut [Operation<'_>]) -> Result<(), Self::Error> {
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
        Self::generate_stop(&mut self.tx);
        Ok(())
    }
}
