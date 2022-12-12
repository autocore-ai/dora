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

    struct NdtLocalizerConfig {
        base_frame: String,
        trans_epsilon: f32,
        step_size: f32,
        resolution: f32,
        max_iterations: f32,
        converged_param_transform_probability: f32,
    }

    extern "Rust" {
        type OutputSender<'a, 'b>;
        fn send_output(sender: &mut OutputSender, id: &str, data: &[u8]) -> SendOutputResult;
    }

    unsafe extern "C++" {
        include!("ndt_localizer/ndt_localizer.hpp");
        // include!("ndt_localizer/include/ndt_localizer/ndt_localizer.hpp");

        type NdtLocalizer;
        fn new_operator(cfg: &NdtLocalizerConfig) -> UniquePtr<NdtLocalizer>;
        fn on_input(
            op: Pin<&mut NdtLocalizer>,
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
