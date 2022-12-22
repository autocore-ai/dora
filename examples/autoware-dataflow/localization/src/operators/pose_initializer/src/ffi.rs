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
    struct PoseInitializerConfig {
        // The particle covariance of fitted gnss pose
        gnss_particle_covariance: [f64; 36],
        // The covariance of output pose
        output_pose_covariance: [f64; 36],
    }

    extern "Rust" {
        type OutputSender<'a, 'b>;
        fn send_output(sender: &mut OutputSender, id: &str, data: &[u8]) -> SendOutputResult;
    }

    unsafe extern "C++" {
        include!("pose_initializer/pose_initializer.hpp");
        // include!("pose_initializer/include/pose_initializer/pose_initializer.hpp");

        type PoseInitializer;
        fn new_operator(cfg: &PoseInitializerConfig) -> UniquePtr<PoseInitializer>;
        fn on_input(
            op: Pin<&mut PoseInitializer>,
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
