mod dualsense;
mod mecanum;
mod rover;

use esp_idf_hal::i2c::{I2cConfig, I2cDriver};
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::prelude::*;
use esp_idf_svc::log::EspLogger;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use log::{info, warn};
use std::thread;
use std::time::Duration;

use mecanum::{compute_wheel_speeds, DriveInput};
use rover::RoverC;

/// Max motor speed (0-127). Start conservative!
const MAX_SPEED: i8 = 80;

/// Control loop frequency
const LOOP_INTERVAL: Duration = Duration::from_millis(20); // 50Hz

fn main() -> anyhow::Result<()> {
    // Bind the ESP-IDF patches (required for std)
    esp_idf_svc::sys::link_patches();

    // Set up logging
    EspLogger::initialize_default();
    info!("=== DerpyBot starting up! ===");

    // Initialize NVS — required by Bluetooth for storing calibration and pairing data
    let _nvs = EspDefaultNvsPartition::take()?;
    info!("NVS initialized");

    // Take peripherals
    let peripherals = Peripherals::take()?;

    // Set up I2C for the RoverC Pro
    // M5StickC PLUS2: SDA=GPIO0, SCL=GPIO26
    let i2c_config = I2cConfig::new().baudrate(100.kHz().into());
    let i2c = I2cDriver::new(
        peripherals.i2c0,
        peripherals.pins.gpio0,  // SDA
        peripherals.pins.gpio26, // SCL
        &i2c_config,
    )?;

    let mut rover = RoverC::new(i2c);

    // Make sure motors are stopped at startup
    rover.stop()?;
    info!("RoverC Pro motor driver ready");

    // Initialize Bluetooth and start scanning for DualSense
    dualsense::init()?;

    info!("Entering control loop...");

    let mut was_connected = false;

    loop {
        let connected = dualsense::is_connected();

        // Log connection state changes
        if connected && !was_connected {
            info!("Controller connected! Ready to drive.");
        } else if !connected && was_connected {
            warn!("Controller disconnected! Stopping motors.");
            rover.stop()?;
        }
        was_connected = connected;

        if connected {
            let pad = dualsense::get_state();

            // PS button = emergency stop
            if pad.ps_button {
                rover.stop()?;
            } else {
                let input = DriveInput {
                    strafe_x: pad.left_x,
                    strafe_y: pad.left_y,
                    rotation: pad.right_x,
                };

                // L2/R2 as speed boost: base is MAX_SPEED, R2 boosts to 127
                let speed = if pad.r2 > 0.1 {
                    let boost = MAX_SPEED as f32 + (127.0 - MAX_SPEED as f32) * pad.r2;
                    boost as i8
                } else if pad.l2 > 0.1 {
                    // L2 = slow mode (half speed)
                    (MAX_SPEED as f32 * 0.5) as i8
                } else {
                    MAX_SPEED
                };

                let speeds = compute_wheel_speeds(input, speed);
                rover.set_speeds(speeds)?;
            }
        }

        thread::sleep(LOOP_INTERVAL);
    }
}
