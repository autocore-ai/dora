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
    struct EKFLocalizerConfig {
        // Common params
        show_debug_info: bool,
        enable_yaw_bias_estimation: bool,
        pose_frame_id: String,
        predict_frequency: f64,
        tf_rate: f64,
        extend_state_step: i32,
        // For pose measurement
        pose_additional_delay: f64,
        pose_measure_uncertainty_time: f64,
        pose_smoothing_steps: i32,
        pose_gate_dist: f64,
        // For twist measurement
        twist_additional_delay: f64,
        twist_smoothing_steps: i32,
        twist_gate_dist: f64,
        // For process model
        proc_stddev_yaw_c: f64,
        proc_stddev_vx_c: f64,
        proc_stddev_wz_c: f64,
    }

    extern "Rust" {
        type OutputSender<'a, 'b>;
        fn send_output(sender: &mut OutputSender, id: &str, data: &[u8]) -> SendOutputResult;
    }

    unsafe extern "C++" {
        include!("ekf_localizer/ekf_localizer.hpp");
        // include!("ekf_localizer/include/ekf_localizer/ekf_localizer.hpp");

        type EKFLocalizer;
        fn new_operator(cfg: &EKFLocalizerConfig) -> UniquePtr<EKFLocalizer>;
        fn on_input(
            op: Pin<&mut EKFLocalizer>,
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
