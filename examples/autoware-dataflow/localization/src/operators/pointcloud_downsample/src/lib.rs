mod ffi;
use ffi::OutputSender;
use dora_operator_api::{self, register_operator, DoraOperator, DoraOutputSender, DoraStatus};

register_operator!(PointcloudDownsampleWrapper);

struct PointcloudDownsampleWrapper {
    operator: cxx::UniquePtr<ffi::ffi::PointcloudDownsample>,
}

impl Default for PointcloudDownsampleWrapper {
    fn default() -> Self {
        let downsample_cfg = ffi::ffi::PointcloudDownsampleConfig {
            input_frame: String::from("base_link"),
            output_frame: String::from("base_link"),
        };
        let crop_box_cfg = ffi::ffi::CropBoxFilterConfig {
            min_x: -60.0,
            max_x: 60.0,
            min_y: -60.0,
            max_y: 60.0,
            min_z: -30.0,
            max_z: 30.0,
            negative: false,
        };
        let voxel_grid_downsample_cfg = ffi::ffi::VoxelGridDownsampleFilterConfig {
            voxel_size_x: 3.0,
            voxel_size_y: 3.0,
            voxel_size_z: 3.0,
        };
        let random_downsample_cfg = ffi::ffi::RandomDownsampleFilterConfig {
            sample_num: 1500,
        };
        
        Self {
            operator: ffi::ffi::new_operator(
                &downsample_cfg, &crop_box_cfg, &voxel_grid_downsample_cfg, &random_downsample_cfg
            ),
        }
    }
}

impl DoraOperator for PointcloudDownsampleWrapper {
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

