mod ffi;
use ffi::OutputSender;
use dora_operator_api::{self, register_operator, DoraOperator, DoraOutputSender, DoraStatus};

register_operator!(VoxelGridFilterWrapper);

struct VoxelGridFilterWrapper {
    operator: cxx::UniquePtr<ffi::ffi::VoxelGridFilter>,
}

impl Default for VoxelGridFilterWrapper {
    fn default() -> Self {
        let cfg = ffi::ffi::VoxelGridFilterConfig {
            leaf_size: 3.0,
            output_log: false,
        };
        Self {
            operator: ffi::ffi::new_operator(&cfg),
        }
    }
}

impl DoraOperator for VoxelGridFilterWrapper {
    fn on_input(
        &mut self,
        id: &str,
        data: &[u8],
        output_sender: &mut DoraOutputSender,
    ) -> Result<DoraStatus, std::string::String> {
        let operator = self.operator.as_mut().unwrap();
        let mut output_sender = OutputSender(output_sender);

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

