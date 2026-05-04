//! Mecanum wheel kinematics.
//!
//! Translates joystick input (strafe_x, strafe_y, rotation) into
//! individual wheel speeds for a 4-wheel mecanum drive.
//!
//! Wheel layout (top view, front is up):
//!
//!     FL \\  // FR
//!          \\/
//!          /\\
//!     RL //  \\ RR
//!
//! The standard mecanum equations:
//!   FL = drive_y + drive_x + rotation
//!   FR = drive_y - drive_x - rotation
//!   RL = drive_y - drive_x + rotation
//!   RR = drive_y + drive_x - rotation

use crate::rover::WheelSpeeds;

/// Joystick input normalized to -1.0..=1.0 for each axis.
#[derive(Debug, Clone, Copy, Default)]
pub struct DriveInput {
    /// Left stick X: strafe left/right (-1 = left, +1 = right)
    pub strafe_x: f32,
    /// Left stick Y: drive forward/backward (-1 = backward, +1 = forward)
    pub strafe_y: f32,
    /// Right stick X: rotate (-1 = CCW, +1 = CW)
    pub rotation: f32,
}

/// Convert drive input to wheel speeds using mecanum kinematics.
pub fn compute_wheel_speeds(input: DriveInput, max_speed: i8) -> WheelSpeeds {
    let max = max_speed as f32;

    let fl = input.strafe_y + input.strafe_x + input.rotation;
    let fr = input.strafe_y - input.strafe_x - input.rotation;
    let rl = input.strafe_y - input.strafe_x + input.rotation;
    let rr = input.strafe_y + input.strafe_x - input.rotation;

    // Find the largest magnitude to normalize if any exceed 1.0
    let largest = fl.abs().max(fr.abs()).max(rl.abs()).max(rr.abs()).max(1.0);

    WheelSpeeds {
        front_left: ((fl / largest) * max) as i8,
        front_right: ((fr / largest) * max) as i8,
        rear_left: ((rl / largest) * max) as i8,
        rear_right: ((rr / largest) * max) as i8,
    }
}
