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
    struct NDTScanMatcherConfig {
        // Vehicle reference frame
        base_frame: String,
        // Subscriber queue size
        // input_sensor_points_queue_size: i32,
        // NDT implementation type (0=PCL_GENERIC, 1=PCL_MODIFIED, 2=OMP)
        ndt_implement_type: i32,
        // The maximum difference between two consecutive transformations in order to consider convergence
        trans_epsilon: f64,
        // The newton line search maximum step length
        step_size: f64,
        // The ND voxel grid resolution
        resolution: f64,
        // The number of iterations required to calculate alignment
        max_iterations: i32,
        // Converged param type (0=TRANSFORM_PROBABILITY, 1=NEAREST_VOXEL_TRANSFORMATION_LIKELIHOOD)
        // NEAREST_VOXEL_TRANSFORMATION_LIKELIHOOD is only available when NDTImplementType::OMP is selected
        converged_param_type: i32,
        // Threshold for deciding whether to trust the estimation result if converged_param_type is 0
        converged_param_transform_probability: f64, 
        // Threshold for deciding whether to trust the estimation result if converged_param_type is 1
        converged_param_nearest_voxel_transformation_likelihood: f64,
        // The number of particles to estimate initial pose
        initial_estimate_particles_num: i32,
        // Tolerance of timestamp difference between initial_pose and sensor pointcloud. [sec]
        initial_pose_timeout_sec: f64,
        // Tolerance of distance difference between two initial poses used for linear interpolation. [m]
        initial_pose_distance_tolerance_m: f64,
        // neighborhood search method in OMP (0=KDTREE, 1=DIRECT26, 2=DIRECT7, 3=DIRECT1)
        omp_neighborhood_search_method: i32,
        // Number of threads used for parallel computing
        omp_num_threads: i32,
        // The covariance of output pose
        output_pose_covariance: [f64; 36],
        // Regularization switch
        regularization_enabled: bool,
        // Regularization scale factor
        regularization_scale_factor: f64,
    }

    extern "Rust" {
        type OutputSender<'a, 'b>;
        fn send_output(sender: &mut OutputSender, id: &str, data: &[u8]) -> SendOutputResult;
    }

    unsafe extern "C++" {
        include!("ndt_scan_matcher/ndt_scan_matcher.hpp");
        // include!("ndt_scan_matcher/include/ndt_scan_matcher/ndt_scan_matcher.hpp");

        type NDTScanMatcher;
        fn new_operator(cfg: &NDTScanMatcherConfig) -> UniquePtr<NDTScanMatcher>;
        fn on_input(
            op: Pin<&mut NDTScanMatcher>,
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
