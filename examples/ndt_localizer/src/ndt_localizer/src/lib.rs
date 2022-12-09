mod ffi;
use ffi::OutputSender;
use dora_operator_api::{self, register_operator, DoraOperator, DoraOutputSender, DoraStatus};

register_operator!(NdtLocalizerWrapper);

struct NdtLocalizerWrapper {
    operator: cxx::UniquePtr<ffi::ffi::NdtLocalizer>,
}

impl Default for NdtLocalizerWrapper {
    fn default() -> Self {
        let cfg = ffi::ffi::NdtLocalizerConfig {
            base_frame: String::from("base_link"),
            trans_epsilon: 0.05,
            step_size: 0.1,
            resolution: 2.0,
            max_iterations: 30.0,
            converged_param_transform_probability: 3.0,
        };
        Self {
            operator: ffi::ffi::new_operator(&cfg),
        }
    }
}

impl DoraOperator for NdtLocalizerWrapper {
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

