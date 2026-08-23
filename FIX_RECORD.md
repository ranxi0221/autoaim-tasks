# FIX_RECORD.md — 自瞄练习开发记录

> **开始日期**: 2026-08-02

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
│   └── autoaim_msgs/      # 自定义消息包（已建，Orienta.msg）
├── build/                 # 自动生成，不进 git
├── install/               # 自动生成，不进 git
└── log/                   # 自动生成，不进 git
```

**分支**: `feature/tiexuejuan-ros-migration`

---

## 二、里程碑记录

## M1 — ROS 化编译 ✅ 完成

**日期**: 2026-08-02 ~ 2026-08-04

**完成了什么**:
- 创建了 `autoaim_msgs` 自定义消息包，定义 `Orienta.msg`（8 个 float32），`ros2 interface show` 验证通过
- 将 `sp_vision` 从独立 CMake 项目改造成 `ament_cmake` ROS2 包：
  - 新建 `package.xml`
  - 改造 `CMakeLists.txt`：ament_cmake 支持、install 规则
  - 适配系统 OpenCV4（`${OpenCV_LIBS}` → opencv_* target）
  - 条件编译：OpenVINO / Ceres / sentry 按依赖可用性自动开关

**怎么验证的**: `colcon build` 全量编译通过（~1分30秒），33 个可执行文件安装到位；`ros2 pkg list | grep sp_vision` 能看到两个包；`ros2 run sp_vision standard --help` 正常运行。

![colcon build 成功](shots/colcon_build.png)

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

**还没解决什么**: 无，M1 目标全部达成。

---

## M2 — 回放订阅配对 ✅ 完成

**日期**: 2026-08-08 ~ 2026-08-09

**完成了什么**:
- 新建 ROS2 节点 `src/bag_replay_node.cpp`（`ros2 run sp_vision bag_replay_node`）
- 同时订阅 `/image_raw`（sensor_msgs/Image）和 `/imu/quaternion`（autoaim_msgs/Orienta），QoS 用 `SensorDataQoS` 与 bag 录制端匹配
- 按"同帧同序"规则做顺序配对：两路各自暂存一帧，一图 + 一四元数到齐即配对
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

**验证结果**:
- 12s 回放内完成 317 帧配对，结束统计 image = quat = paired，同帧同序无错配
- 相邻帧 `dt` 分布 16~75ms，与 bag 帧率一致
- 图像正确解码为 1280×1024 BGR8
- 四元数 `w≈0.981`，云台姿态正常（bag 场景为运动云台）

**新增的文件**:
- `src/bag_replay_node.cpp` — M2 订阅配对节点
- `LEARNING_GUIDE.md` — 学习指南（含新手教程：双终端操作、rosbag 通俗解释、输出逐字段详解、常见问题排查）

**修改的文件**:
- `CMakeLists.txt` — 添加 `bag_replay_node` 可执行文件及其 ROS2 依赖

**关键设计决策**:
- **配对方案从 message_filters 换成顺序配对**：最初计划用 `message_filters::ApproximateTime` 按时间戳同步两路消息，实测发现 `Orienta.msg` 没有 `header` 字段，message_filters 拿不到四元数消息的时间戳，无法同步。由于录制端保证同帧同序发出（任务书 §三），改为顺序配对（一图 + 一四元数即配对），实现更简单且实测无错配。
- 节点启动时不依赖 OpenVINO 或配置文件，纯数据订阅（算法部分留给 M3/M4）

**还没解决什么**: 无，M2 目标全部达成。

---

## M3 — 检测出装甲板 ✅ 完成

**日期**: 2026-08-19 ~ 2026-08-23

**完成了什么**:
- `bag_replay_node` 接入 YOLO 检测器（yolo11 模型，OpenVINO CPU 推理），每个配对帧直接送进 `detector_->detect()`
- 对检测结果做分类统计（颜色/名称/类型），节点退出时打印汇总
- 每 30 帧把检测框自绘到图像上（绿色顶点 + 中心 + 分类文字），存 `shots/m3_frameNNNN_armorsN.jpg`——这是无显示器环境下替代"可视化窗口"的验收证据
- `configs/standard3.yaml` 填入任务书 §5.1 的 6mm 档相机内参/畸变、相机→云台外参、云台→IMU 外参；`device` 由 GPU 改为 CPU

**怎么验证的**:
```bash
# 终端1: 启动节点（cwd 必须是 src/sp_vision，配置/模型路径是相对路径）
source install/setup.bash && cd src/sp_vision && ros2 run sp_vision bag_replay_node

# 终端2: 回放 bag
source install/setup.bash && ros2 bag play bags/move_translate_bag --loop
```

2026-08-23 用当前代码重跑验收，一次运行的结果：

- 配对 3564 帧（跨 3 个 bag loop），检出 2982 次
- **分类 100% 一致为 `blue/one/big`**——bag 场景是单块蓝方大装甲板，无颜色/类型闪烁，分类正确
- 帧级检出率 84%（2982/3564）；`armors=0` 的 582 帧集中在云台摆离目标的方向段，目标一进入视野就连续稳定检出（见日志 `q=(...)` 与 `armors=1` 的对应关系）
- 截图程序化验证：`armors1` 截图含约 5000 个绿色标记像素（检测框确实画上），`armors0` 截图无标记，与文件名一致

![M3 检测框示例（第510帧）](shots/m3_frame0510_armors1.jpg)

![M3 无目标帧示例（第90帧）](shots/m3_frame0090_armors0.jpg)

**修改的文件**:
- `src/bag_replay_node.cpp` — 接入 YOLO 检测、分类统计、截图保存
- `CMakeLists.txt` — `bag_replay_node` 增加链接 `auto_aim tools io fmt yaml-cpp`
- `configs/standard3.yaml` — 6mm 档内外参（任务书 §5.1）、`device: CPU`

**遇到的主要问题**:
1. **GPU 推理跑不通**：`device: GPU` 时 OpenVINO GPU 插件在本机不可用，初始化失败 → 改 `device: CPU` 后推理正常
2. **无显示器环境看不到检测效果**：YOLO debug 模式的 `imshow` 窗口弹不出来 → 在节点里自绘检测框并每 30 帧存一张截图到 `shots/`，作为可视化验收证据（任务书 §八 要求的 M3 检测截图即来自这里）
3. **bag 播放端忽略 `/imu/quaternion`**：play 日志报 `Ignoring a topic '/imu/quaternion', reason: package 'autoaim_msgs' not found`——播放终端只 source 了 `/opt/ros/humble`，没有 source 工作区的 `install/setup.bash`，找不到 `autoaim_msgs` 无法反序列化 → 两个终端都要 source 工作区环境（这正是任务书 §5.2 警告的场景，实测踩中）
4. **`--loop` 边界出现负 dt、计数不等**：loop 回绕时 stamp 从 ~975ms 跳回 0，`dt` 出现负值；节点在播放中途启动时 image/quat 计数也会不齐 → 顺序配对的"pending 覆盖"策略保证配对不串帧，负 dt 只出现在 loop 边界。M4 的 EKF 使用时需要注意（或改为单次播放）

**还没解决什么**:
- GPU 推理未跑通，当前为 CPU 推理；CPU 推理的实时性是否满足 M4 需求待验证
- bag 里只有一块蓝方大装甲板，其他颜色/类型的分类正确性未验证
- 无显示器环境看不到实时窗口，可视化以截图为准（真机调试可用 debug 窗口）

---

## 三、项目说明

（按任务书 §4.3，做到 M3 后开始稳定维护）

**项目功能**: 从 rosbag 回放订阅 `/image_raw` + `/imu/quaternion`，同帧同序配对后送 YOLO 检测装甲板，输出分类统计并定期保存检测截图。已跑通 M1 编译 → M2 配对 → M3 检测；M4 解算+跟踪、M5 PlotJuggler 曲线待做。

**依赖**: Ubuntu 22.04 + ROS2 Humble；系统库 `libopencv-dev libfmt-dev libeigen3-dev libspdlog-dev libyaml-cpp-dev libusb-1.0-0-dev nlohmann-json3-dev libceres-dev`；OpenVINO（Intel APT 版，ABI=1）；ROS 包 `rclcpp sensor_msgs cv_bridge autoaim_msgs`。

**输入源**: `bags/move_translate_bag`（运动云台 + 静止目标场景）。两个话题：`/image_raw`（sensor_msgs/Image，1280×1024 BGR8）和 `/imu/quaternion`（autoaim_msgs/Orienta），同帧同序、reliable QoS，每循环 1247 帧。

**输出话题或结果**: 目前节点不发话题。终端输出每条配对信息（`PAIRED [frame N] stamp/q/img/dt/armors`）和退出时的分类统计；检测截图存 `shots/`。（M5 计划把目标位置等关键量发成话题给 PlotJuggler 画曲线）

**参数入口**: `configs/standard3.yaml`（模型路径、`device`、`min_confidence`、`use_traditional`、6mm 相机内外参、相机参数等）；运行时 `ros2 run sp_vision bag_replay_node <config_path>`，默认 `configs/standard3.yaml`。

**当前已知局限**: CPU 推理；无显示器环境无实时可视化窗口；`--loop` 边界有负 dt；节点中途启动时计数不等但配对正确；仅验证过单一蓝方大装甲板场景。

---

