
#![no_std]



use embedded_hal as hal;
use hal::digital::v2::OutputPin;
use embedded_hal::blocking::delay::DelayMs;


/// Errors in this crate
#[derive(Debug)]
pub enum Error<CommE, PinE> {
    /// Sensor communication error
    Comm(CommE),
    /// Pin setting error
    Pin(PinE),

    /// Unrecognized chip ID
    UnknownChipId,
    /// Sensor not responding
    Unresponsive,
}

pub struct PMW3901<SPI, CSN> {
    /// the SPI port to use when communicating
    spi: SPI,
    /// the Chip Select pin (GPIO output) to use when communicating
    csn: CSN,
}

impl<SPI, CSN, CommE, PinE> PMW3901<SPI, CSN>
    where
        SPI: hal::blocking::spi::Write<u8, Error = CommE>
        + hal::blocking::spi::Transfer<u8, Error = CommE>,
        CSN: OutputPin<Error = PinE>,
{
    /// Combined with register address for reading single byte register
    const DIR_READ: u8 = 0x80; // same as 1<<7  // TODO necessary?

    pub fn new(spi: SPI, csn: CSN) -> Self {
        let mut inst = Self { spi: spi, csn: csn };
        //ensure that the device is initially deselected
        let _ = inst.csn.set_high();
        inst
    }

    /// Initialize this device
    pub fn init(&mut self, delay_source: &mut impl DelayMs<u8>) -> Result<(), Error<CommE, PinE>> {
        //perform a soft reset
        self.register_write(Registers::PowerUpReset as u8, 0x5A)?;

        delay_source.delay_ms(3);

        // verify chip IDs
        let cid = self.register_read(Registers::ProductId)?;
        let inv_cid = self.register_read(Registers::InverseProductId)?;

        if !(0x49 == cid && 0xB8 == inv_cid) {
            return Err(Error::UnknownChipId);
        }

        self.write_optimization_registers()?;

        Ok(())
    }

    pub fn get_motion(&mut self) -> Result< (i16, i16), Error<CommE, PinE>> {
        //read the motion block in one swoop
        let mut block: [u8; 5] = [0; 5];
        // TODO verify that this sensor supports block reads
        self.read_block(Registers::Motion, &mut block)?;
        let dx = ((block[2] as u16) << 8) | (block[1] as u16);
        let dy = ((block[4] as u16) << 8) | (block[3] as u16);
        //TODO verify sign conversion is as expected
        Ok((dx as i16, dy as i16))
    }

    /// Read a single register's value
    pub fn register_read(&mut self, reg: Registers) -> Result<u8, Error<CommE, PinE>> {
        let mut block: [u8; 2] = [0; 2];
        self.read_block(reg, &mut block)?;
        #[cfg(feature = "rttdebug")]
        rprintln!("read reg 0x{:x} {:x?} ", reg, block[1]);

        Ok(block[1])
    }

    /// Write a value to a single register
    pub fn register_write(&mut self, reg: u8, val: u8) -> Result<(), Error<CommE, PinE>> {
        let block: [u8; 2] = [reg, val];
        self.write_block(&block)?;
        Ok(())
    }

    /// Read enough data to fill a provided buffer
    pub fn read_block(&mut self, reg: Registers, buffer: &mut [u8]) -> Result<(), Error<CommE, PinE>> {
        buffer[0] = reg as u8 | Self::DIR_READ;
        self.csn.set_low().map_err(Error::Pin)?;
        let rc = self.spi.transfer(buffer);
        self.csn.set_high().map_err(Error::Pin)?;
        rc.map_err(Error::Comm)?;

        Ok(())
    }

    /// Write a block to the device
    fn write_block(&mut self, block: &[u8]) -> Result<(), Error<CommE, PinE>> {
        #[cfg(feature = "rttdebug")]
        rprintln!("write {:x?} ", block);

        self.csn.set_low().map_err(Error::Pin)?;
        let rc = self.spi.write(block);
        self.csn.set_high().map_err(Error::Pin)?;
        rc.map_err(Error::Comm)?;

        Ok(())
    }

    /// Write a sequence to optimization registers to optimize performance,
    /// as advised by the datasheet.
    fn write_optimization_registers(&mut self) -> Result<(), Error<CommE, PinE>> {
        self.register_write(0x7F, 0x00)?;
        self.register_write(0x61, 0xAD)?;
        self.register_write(0x7F, 0x03)?;
        self.register_write(0x40, 0x00)?;
        self.register_write(0x7F, 0x05)?;
        self.register_write(0x41, 0xB3)?;
        self.register_write(0x43, 0xF1)?;
        self.register_write(0x45, 0x14)?;
        self.register_write(0x5B, 0x32)?;
        self.register_write(0x5F, 0x34)?;
        self.register_write(0x7B, 0x08)?;
        self.register_write(0x7F, 0x06)?;
        self.register_write(0x44, 0x1B)?;
        self.register_write(0x40, 0xBF)?;
        self.register_write(0x4E, 0x3F)?;

        Ok(())
    }
}

#[repr(u8)]
pub enum Registers {
    ProductId = 0x00,
    RevisionId = 0x01,
    Motion = 0x02,
    DeltaXL = 0x03,
    DeltaXH = 0x04,
    DeltaYL = 0x05,
    DeltaYH = 0x06,
    Squal = 0x07,
    RawDataSum = 0x08,
    MaximumRawData = 0x09,
    MinimumRawData = 0x0A,
    ShutterLower = 0x0B,
    ShutterUpper = 0x0C,
    Observation = 0x15,
    MotionBurst = 0x16,

    PowerUpReset = 0x3A,
    Shutdown = 0x3B,

    RawDataGrab = 0x58,
    RawDataGrabStatus = 0x59,

    InverseProductId = 0x5F,
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
