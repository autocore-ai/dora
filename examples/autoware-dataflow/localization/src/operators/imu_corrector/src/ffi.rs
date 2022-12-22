#![warn(unsafe_op_in_unsafe_fn)]

use dora_operator_api::{self, DoraOutputSender};
use ffi::SendOutputResult;

#[cxx::bridge]
#[allow(unsafe_op_in_unsafe_fn)]
pub mod ffi {
    struct OnInputResult {
        error: String,
        stop: bool,
    }

    struct SendOutputResult {
        error: String,
    }
    struct ImuCorrectorConfig {
        angular_velocity_offset_x: f64, // [rad/s]
        angular_velocity_offset_y: f64,
        angular_velocity_offset_z: f64,
        angular_velocity_stddev_xx: f64,    // [rad/s]
        angular_velocity_stddev_yy: f64,
        angular_velocity_stddev_zz: f64,
    }

    extern "Rust" {
        type OutputSender<'a, 'b>;
        fn send_output(sender: &mut OutputSender, id: &str, data: &[u8]) -> SendOutputResult;
    }

    unsafe extern "C++" {
        include!("imu_corrector/imu_corrector.hpp");
        // include!("imu_corrector/include/imu_corrector/imu_corrector.hpp");

        type ImuCorrector;
        fn new_operator(cfg: &ImuCorrectorConfig) -> UniquePtr<ImuCorrector>;
        fn on_input(
            op: Pin<&mut ImuCorrector>,
            id: &str,
            data: &[u8],
            output_sender: &mut OutputSender,
        ) -> OnInputResult;
    }
}

pub struct OutputSender<'a, 'b>(pub &'a mut DoraOutputSender<'b>);

fn send_output(sender: &mut OutputSender, id: &str, data: &[u8]) -> SendOutputResult {
    let error = sender
        .0
        .send(id.into(), data.to_owned())
        .err()
        .unwrap_or_default();
    SendOutputResult { error }
}
