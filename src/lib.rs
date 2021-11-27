//! Winsen Infrared CO2 Module MH-Z19 / MH-Z19B / MH-Z14 driver.
//!
//! [MH-Z19 Datasheet](https://www.winsen-sensor.com/d/files/PDF/Infrared%20Gas%20Sensor/NDIR%20CO2%20SENSOR/MH-Z19%20CO2%20Ver1.0.pdf)
//!
//! [MH-Z19B Datasheet](https://www.winsen-sensor.com/d/files/infrared-gas-sensor/mh-z19b-co2-ver1_0.pdf)
//!
//! [MH-Z14 Datasheet](https://www.winsen-sensor.com/d/files/infrared-gas-sensor/mh-z14a_co2-manual-v1_01.pdf)


#![cfg_attr(not(feature = "std"), no_std)]
#![cfg_attr(docsrs, feature(doc_cfg))]

use embedded_hal::serial::{Read, Write};

pub trait MonotonicCounter {
    /// Returns a measurement of the monotonic counter
    fn value(&self) -> u32;

    /// Returns the frequency at which the monotonic counter is operating at
    fn frequency(&self) -> u32;
}

#[cfg(feature = "std")]
impl MonotonicCounter for std::time::Instant {
    fn value(&self) -> u32 {
        (self.elapsed().as_millis() & 0xffffffff) as u32
    }

    fn frequency(&self) -> u32 {
        1000
    }
}

pub struct WinsenSensor<S, C> {
    serial: S,
    counter: C,
}

impl<E, S: Read<u8, Error=E> + Write<u8, Error=E>, C: MonotonicCounter> WinsenSensor<S, C> {
    pub fn new(serial: S, counter: C) -> Self {
        Self {
            serial,
            counter,
        }
    }

    pub fn free(self) -> (S, C) {
        (self.serial, self.counter)
    }

    pub fn probe(&mut self) -> Result<bool, E> {
        // Use "Read CO2 concentration" command, ignore the result

        // MH-Z19B requires two attempts to be made in order to skip the bootloader prompt
        for _ in 0..2 {
            match self.simple_command(0x86, &[]) {
                Ok(_) => return Ok(true),
                Err(Error::Serial(e)) => return Err(e),
                // First packets can be corrupted, so ignore protocol errors
                Err(_) => {},
            }
        }

        // Final try
        match self.simple_command(0x86, &[]) {
            Ok(_) => Ok(true),
            Err(Error::Serial(e)) => Err(e),
            Err(_) => Ok(false),
        }
    }

    fn send_packet(&mut self, command: u8, payload: &[u8]) -> Result<(), Error<E>> {
        assert!(payload.len() <= 5);

        let mut buffer = [0u8; 9];
        buffer[0] = 0xff;
        buffer[1] = 0x01;
        buffer[2] = command;
        buffer[3..3+payload.len()].copy_from_slice(payload);
        buffer[8] = checksum(&buffer[..8]);

        let t0 = self.counter.value();
        let dt = self.counter.frequency() / 10; // 100ms in clock ticks

        for b in &buffer {
            if (self.counter.value().wrapping_sub(t0)) >= dt {
                return Err(Error::Timeout);
            }
            match self.serial.write(*b) {
                Ok(()) => continue,
                Err(nb::Error::Other(e)) => return Err(Error::Serial(e)),
                Err(nb::Error::WouldBlock) => {},
            }
        }
        Ok(())
    }

    fn receive_packet(&mut self, command: u8, response: &mut [u8]) -> Result<(), Error<E>> {
        assert!(response.len() <= 6);

        let mut buffer = [0u8; 9];
        buffer[0] = 0xff;

        let t0 = self.counter.value();
        let dt = self.counter.frequency() / 10; // 100ms in clock ticks

        // Wait for 0xff
        loop {
            if (self.counter.value().wrapping_sub(t0)) >= dt {
                return Err(Error::Timeout);
            }
            match self.serial.read() {
                Ok(b) => {
                    if b == 0xff {
                        break;
                    }
                },
                Err(nb::Error::Other(e)) => return Err(Error::Serial(e)),
                Err(nb::Error::WouldBlock) => {},
            }
        }

        // Read the rest of the response
        for b in &mut buffer[1..] {
            if (self.counter.value().wrapping_sub(t0)) >= dt {
                return Err(Error::Timeout);
            }
            match self.serial.read() {
                Ok(byte) => {
                    *b = byte;
                    continue;
                },
                Err(nb::Error::Other(e)) => return Err(Error::Serial(e)),
                Err(nb::Error::WouldBlock) => {},
            }
        }

        // Verify checksum
        if checksum(&buffer) != 0 {
            return Err(Error::WrongChecksum);
        }

        // Check packet type
        if buffer[1] != command {
            return Err(Error::WrongPacketType);
        }

        response.copy_from_slice(&buffer[2..2+response.len()]);

        Ok(())
    }

    fn command(&mut self, command: u8, payload: &[u8], response: &mut [u8]) -> Result<(), Error<E>> {
        self.send_packet(command, payload)?;
        self.receive_packet(command, response)
    }

    fn simple_command(&mut self, command: u8, payload: &[u8]) -> Result<(), Error<E>> {
        self.send_packet(command, payload)?;
        self.receive_packet(command, &mut [])
    }

    /// Read the CO2 gas concentration in ppm
    pub fn read_co2_concentration(&mut self) -> Result<u16, Error<E>> {
        let mut buf = [0; 2];
        self.command(0x86, &[], &mut buf)?;
        Ok(u16::from_be_bytes(buf))
    }

    /// Enable or disable Automatic Baseline Correction (ABC)
    pub fn set_automatic_baseline_correction(&mut self, enabled: bool) -> Result<(), Error<E>> {
        let param = if enabled {
            0xA0
        } else {
            0x00
        };
        self.simple_command(0x79, &[param])
    }

    /// Perform zero point calibration
    ///
    /// For MH-Z19B zero point is 400ppm, please make sure the sensor has
    /// been worked under 400ppm for over 20 minutes
    pub fn calibrate_zero_point(&mut self) -> Result<(), Error<E>> {
        self.simple_command(0x87, &[])
    }

    /// Perform span point calibration
    ///
    /// Quoting the datasheet: "Note: Pls do ZERO calibration before span calibration
    /// Please make sure the sensor worked under a certain level co2 for over 20 minutes.
    ///
    /// Suggest using 2000ppm as span, at least 1000ppm"
    pub fn calibrate_span_point(&mut self, span: u16) -> Result<(), Error<E>> {
        let span = span.to_be_bytes();
        self.simple_command(0x88, &span)
    }

    /// Set the sensor detection range (MH-Z19B only).
    ///
    /// Quoting the datasheet: "Detection range is 2000 or 5000ppm"
    pub fn set_detection_range(&mut self, range: u32) -> Result<(), Error<E>> {
        // Note that the actual format differs from what is specified in the datasheet,
        // at least for MH-Z19B
        let range = range.to_be_bytes();
        let mut payload = [0; 5];
        payload[1..].copy_from_slice(&range);

        self.simple_command(0x99, &payload)
    }

    #[cfg(feature = "experimental")]
    #[cfg_attr(docsrs, doc(cfg(feature = "experimental")))]
    /// Get the sensor detection range
    pub fn get_detection_range(&mut self) -> Result<u32, Error<E>> {
        let mut buf = [0; 4];
        self.command(0x9b, &[], &mut buf)?;
        Ok(u32::from_be_bytes(buf))
    }

    #[cfg(feature = "experimental")]
    #[cfg_attr(docsrs, doc(cfg(feature = "experimental")))]
    /// Perform sensor reset
    ///
    /// This command resets sensor MCU
    pub fn reset(&mut self) -> Result<(), Error<E>> {
        self.send_packet(0x8d, &[])
    }

    #[cfg(feature = "experimental")]
    #[cfg_attr(docsrs, doc(cfg(feature = "experimental")))]
    /// Get CO2 concentration bounds used for the analog output
    pub fn get_analog_bounds(&mut self) -> Result<(u16, u16), Error<E>> {
        let mut buf = [0; 4];
        self.command(0xa5, &[], &mut buf)?;
        let high = u16::from_be_bytes([buf[0], buf[1]]);
        let low = u16::from_be_bytes([buf[2], buf[3]]);
        Ok((high, low))
    }

    #[cfg(feature = "experimental")]
    #[cfg_attr(docsrs, doc(cfg(feature = "experimental")))]
    /// Get firmware version string
    pub fn get_firmware_version(&mut self) -> Result<[u8; 4], Error<E>> {
        let mut buf = [0; 4];
        self.command(0xa0, &[], &mut buf)?;
        Ok(buf)
    }
}

#[derive(Debug, PartialEq)]
pub enum Error<E> {
    /// Request timed out
    Timeout,
    /// Underlying serial port error
    Serial(E),
    /// Packet of bytes has the wrong checksum
    WrongChecksum,
    /// The packet type is not the one excepting (eg must be 0x86 when reading gas concentration)
    WrongPacketType,
}

fn checksum(payload: &[u8]) -> u8 {
    !payload.iter().fold(0u8, |sum, c| sum.wrapping_add(*c))
}
