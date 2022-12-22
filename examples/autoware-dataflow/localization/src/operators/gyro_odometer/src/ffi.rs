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

    struct GyroOdometerConfig {
        output_frame: String,
        msg_timeout_sec: f64,
    }

    extern "Rust" {
        type OutputSender<'a, 'b>;
        fn send_output(sender: &mut OutputSender, id: &str, data: &[u8]) -> SendOutputResult;
    }

    unsafe extern "C++" {
        include!("gyro_odometer/gyro_odometer.hpp");
        // include!("gyro_odometer/include/gyro_odometer/gyro_odometer.hpp");

        type GyroOdometer;
        fn new_operator(cfg: &GyroOdometerConfig) -> UniquePtr<GyroOdometer>;
        fn on_input(
            op: Pin<&mut GyroOdometer>,
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
