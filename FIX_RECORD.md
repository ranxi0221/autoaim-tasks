# FIX_RECORD.md — 自瞄练习开发记录

> **开始日期**: 2026-08-07

---

## 一、仓库简介

本仓库 `sp_vision_ws` 是基于 [TongjiSuperPower/sp_vision_25](https://github.com/TongjiSuperPower/sp_vision_25) 改造的 ROS2 工作区，目标是将原独立 CMake 程序改装成 ROS2 包，从 rosbag 回放数据跑通完整自瞄链路（检测 → 解算 → 跟踪），并在 PlotJuggler 中验证结果。

**目录结构**:
```
sp_vision_ws/
├── FIX_RECORD.md          # 本文件
├── shots/                 # 截图/录屏
├── src/
│   ├── sp_vision/         # 原 sp_vision_25 源码（一个 ROS2 包）
│   └── autoaim_msgs/      # 自定义消息包（待建）
├── build/                 # 自动生成，不进 git
├── install/               # 自动生成，不进 git
└── log/                   # 自动生成，不进 git
```

**分支**: `feature/tiexuejuan-ros-migration`

---

## 二、里程碑记录

### M1 — ROS 化编译 ✅ 完成

**日期**: 2026-08-07 ~ 2026-08-09

**完成了什么**:
- 创建了 `autoaim_msgs` 自定义消息包，定义 `Orienta.msg`（8 个 float32），`ros2 interface show` 验证通过
- 将 `sp_vision` 从独立 CMake 项目改造成 `ament_cmake` ROS2 包：
  - 新建 `package.xml`
  - 改造 `CMakeLists.txt`：ament_cmake 支持、install 规则
  - 适配系统 OpenCV4（`${OpenCV_LIBS}` → opencv_* target）
  - 条件编译：OpenVINO / Ceres / sentry 按依赖可用性自动开关
- `colcon build` 全量编译通过（~1分30秒），33 个可执行文件安装到位
- `ros2 run sp_vision standard --help` 正常运行

**修改的文件**:
- `package.xml` — 新建
- `CMakeLists.txt` — ament 改造、OpenCV4 兼容、OpenVINO 路径适配、条件编译、install 规则
- `io/CMakeLists.txt` — sp_msgs → autoaim_msgs
- `tasks/auto_aim/CMakeLists.txt` / `auto_buff/CMakeLists.txt` / `omniperception/CMakeLists.txt` — OpenVINO 路径

**遇到的主要问题**:
1. **OpenCV `${OpenCV_LIBS}` 为空**: Ubuntu 22.04 的 OpenCV4 cmake 不再设置此变量，改为手动指定 `opencv_core opencv_imgproc ...` targets
2. **pip 版 OpenVINO (ABI=0) 与系统库 (ABI=1) 不兼容**: 这是一个经典的 C++ ABI 问题。pip 安装的 OpenVINO 用旧 GCC ABI 编译，其 cmake 通过 `add_definitions(-D_GLIBCXX_USE_CXX11_ABI=0)` 全局污染所有编译单元，导致链接时与 ABI=1 的系统库（OpenCV/yaml-cpp/fmt）符号不匹配
3. **Intel APT 版 OpenVINO 解决**: 安装 `openvino-2024.6.0` 从 Intel APT 源，它用 ABI=1 编译，与系统库一致
4. **缺少 `opencv_dnn`**: 代码用到 `cv::dnn::Net` 等 DNN 模块函数，补充到 OpenCV_LIBS
5. **fmt 包自带 `--as-needed`**: 影响链接顺序，通过 `set_target_properties` 清除

**怎么排查和解决的**:
- 用 `nm -D` 对比 `.o` 文件和 `.so` 库的符号（旧 ABI 用 `Ss`/`KSs`，新 ABI 用 `__cxx11::basic_string`）
- 用 `cat -A` 查看 CMake flags.make 确认 `CXX_DEFINES` 实际值
- 追踪 OpenVINO cmake 文件中的 `add_definitions` 全局调用和 `INTERFACE_COMPILE_DEFINITIONS` target 级传播
- 最终方案：安装 Intel APT 版 OpenVINO（ABI=1），删除所有 ABI hack

**依赖安装**:
```bash
# 基础依赖
sudo apt-get install -y libopencv-dev libfmt-dev libeigen3-dev libspdlog-dev \
  libyaml-cpp-dev libusb-1.0-0-dev nlohmann-json3-dev libceres-dev

# OpenVINO (Intel APT, ABI=1)
wget -qO- https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB \
  | sudo gpg --dearmor --yes -o /usr/share/keyrings/intel-sw.gpg
echo "deb [signed-by=/usr/share/keyrings/intel-sw.gpg] https://apt.repos.intel.com/openvino/2024 ubuntu22 main" \
  | sudo tee /etc/apt/sources.list.d/intel-openvino.list
sudo apt update && sudo apt install -y openvino
```

---

### M2 — 回放订阅配对 ✅ 完成

**日期**: 2026-08-09

**完成了什么**:
- 新建 ROS2 节点 `src/bag_replay_node.cpp`
- 同时订阅 `/image_raw`（sensor_msgs/Image）和 `/imu/quaternion`（autoaim_msgs/Orienta）
- 用 `message_filters::ApproximateTime` 做两路消息时间戳同步配对
- 用 `cv_bridge` 将 ROS Image 转换为 `cv::Mat`（BGR8）
- 从 Orienta 提取 `Eigen::Quaterniond` 四元数
- 从 `header.stamp.nanosec` 还原 `steady_clock::time_point`
- 独立计数 image/quat/paired 帧数，打印配对信息（时间戳、四元数、图像尺寸、帧间隔 `dt`）

**验证方法**:
```bash
# 终端1: 启动订阅节点
source install/setup.bash && ros2 run sp_vision bag_replay_node

# 终端2: 播放 bag
ros2 bag play bags/move_translate_bag --loop

# 预期输出:
# [bag_replay_node]: PAIRED [frame N] stamp=... | q=(...) | img=1280x1024 | dt=...ms
# 结束时 image 数 = quat 数 = paired 数（同帧同序）
```

**新增的文件**:
- `src/bag_replay_node.cpp` — M2 订阅配对节点

**修改的文件**:
- `CMakeLists.txt` — 添加 `bag_replay_node` 可执行文件及其 ROS2 依赖

**关键设计决策**:
- 选用 `ApproximateTime` 而非 `ExactTime` 同步策略（ExactTime 在 Humble 有模板参数 bug）
- 节点启动时不依赖 OpenVINO 或配置文件，纯粹的纯数据订阅（算法部分留给 M3/M4）

---

