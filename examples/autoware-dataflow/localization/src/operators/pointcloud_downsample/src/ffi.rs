#![warn(unsafe_op_in_unsafe_fn)]

use dora_operator_api::{self, DoraOutputSender};
use ffi::SendOutputResult;

// #[cxx::bridge(namespace = "pointcloud_downsample")]
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
    struct PointcloudDownsampleConfig {
        // requred input frame and output frame
        input_frame: String,
        output_frame: String,
    }
    struct CropBoxFilterConfig {
        min_x: f32,
        max_x: f32,
        min_y: f32,
        max_y: f32,
        min_z: f32,
        max_z: f32,
        negative: bool,
    }
    struct VoxelGridDownsampleFilterConfig {
        voxel_size_x: f64,
        voxel_size_y: f64,
        voxel_size_z: f64,
    }
    struct RandomDownsampleFilterConfig {
        sample_num: i64,
    }

    extern "Rust" {
        type OutputSender<'a, 'b>;
        fn send_output(sender: &mut OutputSender, id: &str, data: &[u8]) -> SendOutputResult;
    }

    unsafe extern "C++" {
        include!("pointcloud_downsample/pointcloud_downsample.hpp");
        // include!("pointcloud_downsample/include/pointcloud_downsample/pointcloud_downsample.hpp");

        type PointcloudDownsample;
        fn new_operator(
            downsample_cfg: &PointcloudDownsampleConfig,
            crop_box_cfg: &CropBoxFilterConfig,
            voxel_grid_downsample_cfg: &VoxelGridDownsampleFilterConfig,
            random_downsample_cfg: &RandomDownsampleFilterConfig
        ) -> UniquePtr<PointcloudDownsample>;
        fn on_input(
            op: Pin<&mut PointcloudDownsample>,
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
