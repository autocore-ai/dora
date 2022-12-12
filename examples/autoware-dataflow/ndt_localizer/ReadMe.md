# Brief description

Taking [a clean and simple NDT localizer](https://github.com/AbangLZU/ndt_localizer) as an example, ndt_localizer removes the blocks in the node migration and compilation processes and uses necessary auxiliary software packages to form a minimum closed loop for testing.

The changes to the node migration process mainly cover the following aspects:

1. The source code in CMakeLists.txt, .hpp files, and .cpp files is modified. 

2. The Dora interface is used, and ffi.rs is defined based on the source code in the launch and cfgs folders of the original project.

3. Message serialization and deserialization are implemented in C++.

4. The compilation process is defined in C++ and Rust, and blocks in compilation are removed. Note that the dependencies on ROS dynamic libraries are retained, which will be addressed in a later version.

5. Auxiliary nodes are added to ensure the minimum closed loop, including manual sending of coordinate transformation information.

You can compare the code in the original project with that in the current directory for details of changes. The original project uses ROS1 code, and the type of shared_ptr in use is boost::shared_ptr. However, member variables of this pointer type are private and cannot be serialized by using cereal. Therefore, ROS1 code needs to be changed to ROS2 code.

# Usage

Use the following syntax to compile code and run ndt_localizer:

```bash
# Compile code
cd dora
source /opt/ros/galactic/setup.bash
# cargo build --package dora-coordinator # run this cmd if needed
cargo run --example ndt-dataflow
# Run ndt_localizer
cd examples/autoware-dataflow/ndt_localizer
source install/setup.bash
../../../target/debug/dora-coordinator --run-dataflow dataflow.yml --runtime ../../../target/debug/dora-runtime
```

Although example/ndt_localizer/run.rs stores executable code, `source install/setup.bash` cannot be effectively invoked in the script. Therefore, you need to manually run ndt_localizer.