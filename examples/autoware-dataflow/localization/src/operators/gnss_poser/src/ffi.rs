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
    struct GNSSPoserConfig {
        // Vehicle reference frame
        base_frame: String,
        gnss_base_frame: String,
        gnss_antenna_frame: String,
        map_frame: String,
        // Coordinate system type (0:UTM, 1:MGRS, 2:PLANE, 3:LocalCartesianWGS84, 4:LocalCartesianUTM)
        coordinate_system: i32,
        // Position buffer capacity
        buff_epoch: i32,
        // Is using gnss_ins_orientation
        use_gnss_ins_orientation: bool,
        // Plane zone number
        plane_zone: i32,
        // Latitude of local origin
        latitude: f64,
        // Longitude of local origin
        longitude: f64,
        // Altitude of local origin
        altitude: f64,
    }

    extern "Rust" {
        type OutputSender<'a, 'b>;
        fn send_output(sender: &mut OutputSender, id: &str, data: &[u8]) -> SendOutputResult;
    }

    unsafe extern "C++" {
        include!("gnss_poser/gnss_poser.hpp");
        // include!("gnss_poser/include/gnss_poser/gnss_poser.hpp");

        type GNSSPoser;
        fn new_operator(cfg: &GNSSPoserConfig) -> UniquePtr<GNSSPoser>;
        fn on_input(
            op: Pin<&mut GNSSPoser>,
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
