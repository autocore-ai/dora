mod ffi;
use ffi::OutputSender;
use dora_operator_api::{self, register_operator, DoraOperator, DoraOutputSender, DoraStatus};

register_operator!(EKFLocalizerWrapper);

struct EKFLocalizerWrapper {
    operator: cxx::UniquePtr<ffi::ffi::EKFLocalizer>,
}

impl Default for EKFLocalizerWrapper {
    fn default() -> Self {
        let cfg = ffi::ffi::EKFLocalizerConfig {
            // Common params
            show_debug_info: false,
            enable_yaw_bias_estimation: true,
            pose_frame_id: String::from("map"),
            predict_frequency: 50.0,
            tf_rate: 50.0,
            extend_state_step: 50,
            // For pose measurement
            pose_additional_delay: 0.0,
            pose_measure_uncertainty_time: 0.01,
            pose_smoothing_steps: 5,
            pose_gate_dist: 10000.0,
            // For twist measurement
            twist_additional_delay: 0.0,
            twist_smoothing_steps: 2,
            twist_gate_dist: 10000.0,
            // For process model
            proc_stddev_yaw_c: 0.005,
            proc_stddev_vx_c: 10.0,
            proc_stddev_wz_c: 5.0,
        };
        Self {
            operator: ffi::ffi::new_operator(&cfg),
        }
    }
}

impl DoraOperator for EKFLocalizerWrapper {
    fn on_input(
        &mut self,
        id: &str,
        data: &[u8],
        output_sender: &mut DoraOutputSender,
    ) -> Result<DoraStatus, std::string::String> {
        let operator = self.operator.as_mut().unwrap();
        let mut output_sender = OutputSender(output_sender);

        // Distinguish different inputs according to its ID
        let result = ffi::ffi::on_input(operator, id, data, &mut output_sender);

        if result.error.is_empty() {
            Ok(match result.stop {
                false => DoraStatus::Continue,
                true => DoraStatus::Stop,
            })
        } else {
            Err(result.error)
        }
    }
}

