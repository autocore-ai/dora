mod ffi;
use ffi::OutputSender;
use dora_operator_api::{self, register_operator, DoraOperator, DoraOutputSender, DoraStatus};

register_operator!(ImuCorrectorWrapper);

struct ImuCorrectorWrapper {
    operator: cxx::UniquePtr<ffi::ffi::ImuCorrector>,
}

impl Default for ImuCorrectorWrapper {
    fn default() -> Self {
        let cfg = ffi::ffi::ImuCorrectorConfig {
            angular_velocity_offset_x: 0.0,
            angular_velocity_offset_y: 0.0,
            angular_velocity_offset_z: 0.0,
            angular_velocity_stddev_xx: 0.03,
            angular_velocity_stddev_yy: 0.03,
            angular_velocity_stddev_zz: 0.03,
        };
        Self {
            operator: ffi::ffi::new_operator(&cfg),
        }
    }
}

impl DoraOperator for ImuCorrectorWrapper {
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

