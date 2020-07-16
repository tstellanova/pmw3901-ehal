/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

#![no_std]

//!
//! An embedded-hal no-std driver for the PMW3901 optical flow sensor.
//!

use embedded_hal as hal;
use hal::digital::v2::OutputPin;
use embedded_hal::blocking::delay::DelayMs;

#[cfg(feature = "rttdebug")]
use panic_rtt_core::rprintln;

/// Errors in this crate
#[derive(Debug)]
pub enum Error<CommE, PinE> {
    /// Sensor communication error
    Comm(CommE),
    /// Pin setting error
    Pin(PinE),

    /// Poor signal quality / insufficient light or texture
    NoSignal,
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


    pub fn new(spi: SPI, csn: CSN) -> Self {
        let mut inst = Self { spi: spi, csn: csn };
        //ensure that the device is initially deselected
        let _ = inst.csn.set_high();
        inst
    }

    /// Initialize this device
    pub fn init(&mut self, delay_source: &mut impl DelayMs<u32>) -> Result<(), Error<CommE, PinE>> {
        //perform a soft reset
        self.register_write(Register::PowerUpReset as u8, 0x5A)?;
        delay_source.delay_ms(3000);

        // verify chip IDs
        let cid = self.register_read(Register::ProductId)?;
        let inv_cid = self.register_read(Register::InverseProductId)?;

        if !(Self::PRODUCT_ID == cid && Self::INV_PRODUCT_ID == inv_cid) {
            #[cfg(feature = "rttdebug")]
            rprintln!("unknown cid 0x{:x} inv_cid 0x{:x} ", cid, inv_cid);
            return Err(Error::UnknownChipId);
        }

        self.write_config_optimizations()?;
        // send an initial to the sensor: this is allowed to fail (squal == 0)
        let _ = self.get_motion();

        Ok(())
    }

    /// Below this threshold we assume the quality of flow measurement is poor
    const SQUAL_THRESHOLD: u8 = 5;
    const DIR_READ: u8 = 0x7f;
    const MOTION_READ_BLOCK: [u8; 12] = [
        Self::DIR_READ & (Register::Motion as u8), 0,
        Self::DIR_READ & (Register::DeltaXL as u8), 0,
        Self::DIR_READ & (Register::DeltaXH as u8), 0,
        Self::DIR_READ & (Register::DeltaYL as u8), 0,
        Self::DIR_READ & (Register::DeltaYH as u8), 0,
        Self::DIR_READ & (Register::Squal as u8), 0
    ];

    /// Main method for obtaining the (dx, dy) flow
    pub fn get_motion(&mut self) -> Result< (i16, i16), Error<CommE, PinE>> {
        //read the motion block all at once
        let mut block: [u8; 12] = Self::MOTION_READ_BLOCK;
        self.transfer_sequence(&mut block)?;
        #[cfg(feature = "rttdebug")]
        rprintln!("block: {:?}", block);

        let squal = block[11];
        if squal < Self::SQUAL_THRESHOLD {
            return Err(Error::NoSignal);
        }
        let dx = ((block[5] as i16) << 8) | (block[3] as i16);
        let dy = ((block[9] as i16) << 8) | (block[7] as i16);

        Ok((dx, dy))
    }

    /// Read a single register's value
    pub fn register_read(&mut self, reg: Register) -> Result<u8, Error<CommE, PinE>> {
        let mut block: [u8; 2] = [reg as u8, 0];
        self.transfer_sequence(&mut block)?;
        // self.read_block(reg, &mut block)?;
        Ok(block[1])
    }

    /// Write a value to a single register
    pub fn register_write(&mut self, reg: u8, val: u8) -> Result<(), Error<CommE, PinE>> {
        let block: [u8; 2] = [reg, val];
        self.write_block(&block)?;
        Ok(())
    }

    /// transfer a series of bytes to the sensor
    fn transfer_sequence(&mut self,  buffer: &mut [u8]) -> Result<(), Error<CommE, PinE>> {
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

    /// Write a sequence to undocumented registers in order to optimize performance,
    /// as advised by datasheet section 8.2.
    fn write_config_optimizations(&mut self) -> Result<(), Error<CommE, PinE>> {
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

    /// supported product IDs
    const PRODUCT_ID: u8 = 0x49;
    const INV_PRODUCT_ID: u8  = 0xB6;
}



#[repr(u8)]
#[derive(Copy, Clone, Debug)]
pub enum Register {
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


