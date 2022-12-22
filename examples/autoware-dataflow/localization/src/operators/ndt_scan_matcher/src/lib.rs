mod ffi;
use ffi::OutputSender;
use dora_operator_api::{self, register_operator, DoraOperator, DoraOutputSender, DoraStatus};

register_operator!(NDTScanMatcherWrapper);

struct NDTScanMatcherWrapper {
    operator: cxx::UniquePtr<ffi::ffi::NDTScanMatcher>,
}

impl Default for NDTScanMatcherWrapper {
    fn default() -> Self {
        let cfg = ffi::ffi::NDTScanMatcherConfig {
            base_frame: String::from("base_link"),
            ndt_implement_type: 2,
            trans_epsilon: 0.01,
            step_size: 0.1,
            resolution: 2.0,
            max_iterations: 30,
            converged_param_type: 1,
            converged_param_transform_probability: 3.0,
            converged_param_nearest_voxel_transformation_likelihood: 2.3,
            initial_estimate_particles_num: 100,
            initial_pose_timeout_sec: 1.0,
            initial_pose_distance_tolerance_m: 10.0,
            omp_neighborhood_search_method: 0,
            omp_num_threads: 4,
            output_pose_covariance: 
                [
                    0.0225, 0.0,   0.0,   0.0,      0.0,      0.0,
                    0.0,   0.0225, 0.0,   0.0,      0.0,      0.0,
                    0.0,   0.0,   0.0225, 0.0,      0.0,      0.0,
                    0.0,   0.0,   0.0,   0.000625, 0.0,      0.0,
                    0.0,   0.0,   0.0,   0.0,      0.000625, 0.0,
                    0.0,   0.0,   0.0,   0.0,      0.0,      0.000625,
                ],
            regularization_enabled: false,
            regularization_scale_factor: 0.01,
        };
        Self {
            operator: ffi::ffi::new_operator(&cfg),
        }
    }
}

impl DoraOperator for NDTScanMatcherWrapper {
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

