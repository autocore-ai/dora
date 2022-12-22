mod ffi;
use ffi::OutputSender;
use dora_operator_api::{self, register_operator, DoraOperator, DoraOutputSender, DoraStatus};

register_operator!(MapHeightFitterWrapper);

struct MapHeightFitterWrapper {
    operator: cxx::UniquePtr<ffi::ffi::MapHeightFitter>,
}

impl Default for MapHeightFitterWrapper {
    fn default() -> Self {
        let cfg = ffi::ffi::MapHeightFitterConfig {
            gnss_pose_timeout: 3.0,
        };
        Self {
            operator: ffi::ffi::new_operator(&cfg),
        }
    }
}

impl DoraOperator for MapHeightFitterWrapper {
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

