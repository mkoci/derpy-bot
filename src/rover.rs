//! RoverC Pro motor driver over I2C.
//!
//! The RoverC Pro has an STM32 motor controller at I2C address 0x38.
//! It drives 4 mecanum wheels. Each motor speed is a signed byte (-127..=127).
//!
//! Register map (each register is one byte, signed):
//!   0x00 - front-left motor
//!   0x01 - front-right motor
//!   0x02 - rear-left motor
//!   0x03 - rear-right motor
//!
//! Positive = forward, Negative = backward.

use esp_idf_hal::i2c::I2cDriver;

const ROVER_I2C_ADDR: u8 = 0x38;

/// Speeds for the four mecanum wheels.
#[derive(Debug, Clone, Copy, Default)]
pub struct WheelSpeeds {
    pub front_left: i8,
    pub front_right: i8,
    pub rear_left: i8,
    pub rear_right: i8,
}

pub struct RoverC<'a> {
    i2c: I2cDriver<'a>,
}

impl<'a> RoverC<'a> {
    pub fn new(i2c: I2cDriver<'a>) -> Self {
        Self { i2c }
    }

    /// Send wheel speeds to the RoverC Pro motor controller.
    pub fn set_speeds(&mut self, speeds: WheelSpeeds) -> anyhow::Result<()> {
        let data = [
            speeds.front_left as u8,
            speeds.front_right as u8,
            speeds.rear_left as u8,
            speeds.rear_right as u8,
        ];
        self.i2c
            .write(ROVER_I2C_ADDR, &[0x00, data[0], data[1], data[2], data[3]], 100)
            .map_err(|e| anyhow::anyhow!("I2C write failed: {:?}", e))
    }

    /// Emergency stop — all motors to zero.
    pub fn stop(&mut self) -> anyhow::Result<()> {
        self.set_speeds(WheelSpeeds::default())
    }
}
