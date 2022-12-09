Notice: At present, the document is written in Chinese, and will be changed to English later

## 简要说明
ndt_localizer 以[简化版的ndt算法](https://github.com/AbangLZU/ndt_localizer)为例, 走通节点迁移和编译流程. 并添加必要的辅助软件包, 构成最小测试闭环.

节点迁移的改动主要涉及如下方面:

1. CMakeLists.txt, xxx.hpp, xxx.cpp 等源码改写; 

2. Dora接口使用, 根据源码中的 launch 和 config 文件定义ffi.rs;

3. C++ 侧消息序列化和反序列化工作;

4. C++ 和 Rust 混合编译流程编写; (走通编译流程,但遗留对ros动态库的依赖, 留待后续解决)

5. 添加辅助节点, 保证构成最小闭环, 包括手动发送坐标转换信息.

具体改动内容可以对比原始工程和本目录下代码. 需要特别说明的是原始工程是ros1代码, 共享指针(shared_ptr)使用的是 boost::shared_ptr, 但 boost 指针的成员变量都是私有的, 没法用 cereal 进行序列化, 所以要改成ros2的代码.

## 使用方式

编译和使用方式如下

```bash
# 编译
cd dora
source /opt/ros/galactic/setup.bash
# cargo build --package dora-coordinator # run this cmd if needed
cargo run --example ntd-dataflow
# 运行
cd example/ndt_localizer
source install/setup.bash
../../target/debug/dora-coordinator --run-dataflow dataflow.yml --runtime ../../target/debug/dora-runtime
```

虽然在 example/ndt_localizer/run.rs中编写了直接运行的代码, 但因为脚本中无法有效调用`source install/setup.bash`, 所以需要手动运行.
