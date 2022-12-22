mod ffi;
use ffi::OutputSender;
use dora_operator_api::{self, register_operator, DoraOperator, DoraOutputSender, DoraStatus};

register_operator!(GNSSPoserWrapper);

struct GNSSPoserWrapper {
    operator: cxx::UniquePtr<ffi::ffi::GNSSPoser>,
}

impl Default for GNSSPoserWrapper {
    fn default() -> Self {
        let cfg = ffi::ffi::GNSSPoserConfig {
            base_frame: String::from("base_link"),
            gnss_base_frame: String::from("gnss_base_link"),
            gnss_antenna_frame: String::from("gnss_link"),
            map_frame: String::from("map"),
            // Coordinate system type (0:UTM, 1:MGRS, 2:PLANE, 3:LocalCartesianWGS84, 4:LocalCartesianUTM)
            coordinate_system: 0,
            // Position buffer capacity
            buff_epoch: 1,
            // Is using gnss_ins_orientation
            use_gnss_ins_orientation: false,
            // Plane zone number
            plane_zone: 9,
            // Latitude of local origin
            latitude: 40.81187906,
            // Longitude of local origin
            longitude: 29.35810110,
            // Altitude of local origin
            altitude: 47.62,
        };
        Self {
            operator: ffi::ffi::new_operator(&cfg),
        }
    }
}

impl DoraOperator for GNSSPoserWrapper {
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

