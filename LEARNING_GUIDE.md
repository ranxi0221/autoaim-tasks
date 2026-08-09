# 自瞄练习项目学习指南

> 跟着代码走，理解每一步做了什么、为什么这样做

---

## 一、这项目是做什么的

**一句话**：把 `sp_vision_25`（同济 SuperPower 战队的 RoboMaster 自瞄算法）从"读真实相机 + 下位机"改成"读 rosbag 录好的数据包"，在离线环境下跑通 检测 → 解算 → 跟踪。

**原本的数据流**（`standard.cpp` 主循环）：
```cpp
camera.read(img, t);              // 1. 从工业相机取一帧图像 + 时间戳
q = cboard.imu_at(t - 1ms);       // 2. 从下位机取该时刻云台姿态四元数
solver.set_R_gimbal2world(q);     // 3. 用四元数建立 gimbal→world 坐标变换
auto armors  = detector.detect(img);   // 4. 神经网络检测装甲板
auto targets = tracker.track(armors, t); // 5. PnP 解算 + EKF 跟踪 + 预测
auto command = aimer.aim(targets, t, ...); // 6. 计算云台控制指令
cboard.send(command);             // 7. 发给下位机执行
```

**改造后的数据流**（我们做的）：
```
rosbag → /image_raw (sensor_msgs/Image)  ──┐
                                            ├─ message_filters 配对 ──→ 算法链路
rosbag → /imu/quaternion (Orienta)        ──┘
```

**5 个里程碑**：

| | 目标 | 自检方法 |
|---|---|---|
| M1 | 把原项目改成 ROS2 包，能编译 | `colcon build` 通过，`ros2 run` 能启动 |
| M2 | 从 bag 订阅图像+姿态并配对 | 播 bag 时打印帧数/时间戳，数量对得上 |
| M3 | 检测出装甲板 | 可视化窗口稳定框住装甲板 |
| M4 | 解算 + 跟踪 | 得到世界系目标位置，tracker 正常切换 |
| M5 | PlotJuggler 曲线 | 运动云台下世界系目标不漂移 |

---

## 二、前置概念：ROS2 工作区是什么

ROS2 工作区**就是一个普通文件夹**，不是特殊工具创建的。它有一个约定俗成的结构：

```
sp_vision_ws/          ← 工作区根目录（git 仓库也在这里）
├── src/               ← 你手动建，放所有"包"
│   ├── sp_vision/     ← 一个包（package）
│   └── autoaim_msgs/  ← 另一个包
├── build/             ← colcon build 自动生成（中间产物）
├── install/           ← colcon build 自动生成（最终结果）
└── log/               ← 编译日志
```

**关键概念**：

- **包（package）**：有 `package.xml` 文件的目录，是编译的最小单元
- **`colcon build`**：ROS2 的编译命令，必须在工作区**根目录**运行。它会自动扫描 `src/` 下所有包一起编译
- **`source install/setup.bash`**：把编译结果"注册"到当前终端环境。每次新开终端都要执行一次，之后 `ros2 run` 才能找到你的节点
- **`ros2 run <包名> <可执行文件名>`**：启动一个 ROS2 节点

---

## 三、M1：把独立 CMake 项目改成 ROS2 包

### 3.1 原项目怎么编译的

`sp_vision_25` 原本是一个**标准 CMake 项目**，不依赖 ROS：

```cmake
# 原来的 CMakeLists.txt 骨架
cmake_minimum_required(VERSION 3.16.3)
project(sp_vision)

find_package(OpenCV REQUIRED)     # 找依赖库
find_package(fmt REQUIRED)
...

add_executable(standard src/standard.cpp)          # 声明可执行文件
target_link_libraries(standard ${OpenCV_LIBS} ...) # 链接库
```

编译方式：
```bash
cmake -B build && make -C build    # 传统 cmake 两步走
```

### 3.2 要变成 ROS2 包，需要加三样东西

#### ① `package.xml` — 包的"身份证"

告诉 ROS2 这个包叫什么、依赖什么：

```xml
<package format="3">
  <name>sp_vision</name>                        <!-- 包名 = ros2 run 时用的名字 -->
  <version>0.1.0</version>

  <buildtool_depend>ament_cmake</buildtool_depend>  <!-- 用 ament 编译系统替代原生 cmake -->

  <depend>rclcpp</depend>           <!-- ROS2 C++ 客户端库 -->
  <depend>sensor_msgs</depend>      <!-- 图像等标准消息类型 -->
  <depend>cv_bridge</depend>        <!-- ROS Image ↔ OpenCV Mat 转换 -->
  <depend>autoaim_msgs</depend>     <!-- 我们自定义的消息类型 -->

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

#### ② CMakeLists.txt 的头尾两行

```cmake
find_package(ament_cmake REQUIRED)   // 加在头部：引入 ament 编译系统
...
ament_package()                      // 加在尾部：注册为 ament 包
```

#### ③ `install(TARGETS ...)` 指令

原项目用 `add_executable(xxx ...)` 声明了可执行文件，但**没有 install 规则**。ROS2 只在 `install/` 目录下搜索可执行文件，所以必须加：

```cmake
install(TARGETS
  standard              # 原版自瞄主程序
  bag_replay_node       # M2 新建的回放节点
  ...                   # 其他 30+ 个可执行文件
  DESTINATION lib/${PROJECT_NAME}   # 安装到 install/sp_vision/lib/sp_vision/
)
```

没有这条规则 = `ros2 run` 报 "Package not found"。

### 3.3 我们做了什么改动

| 文件 | 改动 | 原因 |
|---|---|---|
| `package.xml` | **新建** | ROS2 包必须的身份证 |
| `CMakeLists.txt` | 加 ament 头尾、install 规则、条件编译、OpenCV4 兼容 | 让 colcon 能编译+安装 |
| `io/CMakeLists.txt` | `sp_msgs` → 跳过（我们不编译哨兵模块） | sp_msgs 不存在，也不需要 |
| `tasks/*/CMakeLists.txt` | 去掉硬编码的 OpenVINO 路径 | 系统路径下 CMake 自动能找到 |

---

## 四、自定义消息包 autoaim_msgs

### 4.1 为什么需要它

bag 里 `/imu/quaternion` 话题的类型叫 `autoaim_msgs/msg/Orienta`，这是录制方自定义的消息。ROS2 要**反序列化**（把二进制字节还原成结构体）这个消息，必须有一个包能"生成"这个类型。

没有这个包 → 编译不过 + 收不到消息。

### 4.2 结构

```
autoaim_msgs/
├── package.xml           # 包信息，标记为 rosidl_interface_packages
├── CMakeLists.txt         # 用 rosidl_generate_interfaces 生成消息代码
└── msg/
    └── Orienta.msg        # 字段必须和 bag 录制方完全一致
```

**Orienta.msg** 的内容：
```
float32 w       # 四元数实部
float32 x       # 四元数虚部 x
float32 y       # 四元数虚部 y
float32 z       # 四元数虚部 z
float32 dm_w    # 以下 dm_* 恒为 0，录制方预留的字段
float32 dm_x
float32 dm_y
float32 dm_z
```

**为什么字段必须一模一样**：ROS2 的序列化是按字段顺序和类型严格计算的。哪怕多一个或少一个 float32，二进制数据就对不齐，反序列化就会失败。

### 4.3 消息包的关键配置

`package.xml` 里三个容易漏的点：
```xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>  <!-- 编译时生成代码 -->
<exec_depend>rosidl_default_runtime</exec_depend>              <!-- 运行时需要 -->
<member_of_group>rosidl_interface_packages</member_of_group>   <!-- 标记为消息包 -->
```

`CMakeLists.txt` 里：
```cmake
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} "msg/Orienta.msg")
```

编译后 ROS2 自动生成 `autoaim_msgs/msg/orienta.hpp`，之后 `sp_vision` 包就可以 `#include` 它了。

---

## 五、M1 最坑的问题：C++ ABI 不兼容

### 5.1 什么是 C++ ABI

C++ 编译器在编译 `std::string` 时，会把函数名"改编"(mangle) 成唯一的符号，供链接器使用。

GCC 5 之后，有两种互不兼容的 ABI：

| | 旧 ABI | 新 ABI（C++11） |
|---|---|---|
| `std::string` 的符号特征 | `Ss` / `KSs` | `__cxx11::basic_string` |
| 宏控制 | `_GLIBCXX_USE_CXX11_ABI=0` | `_GLIBCXX_USE_CXX11_ABI=1`（默认） |
| Ubuntu 22.04 系统库 | ❌ | ✅ OpenCV、yaml-cpp、fmt 都用这个 |

**关键**：ABI 不同的库**不能互相调用**——链接器会报 `undefined reference`，因为两边对同一个函数生成了不同的符号名。

### 5.2 我们的两难处境

```
pip 版 OpenVINO (ABI=0)  ←→  系统库 OpenCV / yaml-cpp / fmt (ABI=1)
         ↓                               ↓
      旧 string                       新 string
    符号: RKSs                    符号: RKNSt7__cxx1112basic_string...
```

OpenVINO 通过 cmake 全局设置 `_GLIBCXX_USE_CXX11_ABI=0`：
```cmake
# OpenVINOConfig.cmake 内部：
set(OpenVINO_GLIBCXX_USE_CXX11_ABI "0")
add_definitions(-D_GLIBCXX_USE_CXX11_ABI=0)   # 这一行影响项目里所有 .cpp！
```

结果：所有 `.cpp` 都用旧 ABI 编译 → 链接系统库时找不到新 ABI 符号 → 编译失败。

### 5.3 怎么排查的

**Step 1**: 看编译命令里的实际宏定义
```bash
cat build/sp_vision/io/CMakeFiles/io.dir/flags.make | grep CXX_DEFINES
# 输出: CXX_DEFINES = -D_GLIBCXX_USE_CXX11_ABI=0    ← 全局污染！
```

**Step 2**: 对比符号
```bash
# 编译出的 .o 文件期望什么符号？
nm build/sp_vision/io/CMakeFiles/io.dir/camera.cpp.o | grep LoadFile
# 输出: U _ZN4YAML8LoadFileERKSs       ← KSs = 旧 ABI

# 系统的 yaml-cpp.so 提供什么符号？
nm -D /usr/lib/x86_64-linux-gnu/libyaml-cpp.so | grep LoadFile
# 输出: T _ZN4YAML8LoadFileERKNSt7__cxx1112basic_string...  ← __cxx11 = 新 ABI
```

**结论**：`.o` 期望旧符号，`.so` 提供新符号 → 不匹配 → 链接失败。

**Step 3**: 追踪来源
```bash
grep -r "GLIBCXX_USE_CXX11_ABI" ~/.local/lib/python3.10/site-packages/openvino/cmake/
# 找到: OpenVINOConfig.cmake 有 add_definitions(-D_GLIBCXX_USE_CXX11_ABI=0)
# 找到: OpenVINOTargets.cmake 有 INTERFACE_COMPILE_DEFINITIONS "_GLIBCXX_USE_CXX11_ABI=0;..."
```

**根因**：`add_definitions()` 是 CMake 的**全局命令**，影响当前目录及所有子目录的全部 target。`INTERFACE_COMPILE_DEFINITIONS` 则通过 `target_link_libraries` 链传播。

### 5.4 解决方案的演进

| 尝试 | 做法 | 结果 |
|---|---|---|
| 1 | `remove_definitions(-D_GLIBCXX_USE_CXX11_ABI=0)` | ❌ `add_definitions` 在 OpenVINO cmake 内部执行，外部 remove 无法抵消 |
| 2 | `add_definitions(-D_GLIBCXX_USE_CXX11_ABI=1)` 覆盖 | ❌ 两个 define 同时存在，顺序不确定 |
| 3 | 改 `target_compile_definitions(auto_aim PRIVATE ...)` | ❌ auto_aim 自己 ABI=0，但调用方 ABI=1，API 用了 std::string 导致不匹配 |
| 4 | 改 `target_link_libraries(auto_aim PRIVATE openvino::runtime)` | ❌ PRIVATE 阻止传播但 auto_aim 自身仍需 ABI=0 |
| 5 | 直接改 OpenVINOTargets.cmake 删掉 ABI=0 | ❌ 只能删 INTERFACE 级别，`add_definitions` 的全局定义仍在 |
| 6 | **换 Intel APT 版 OpenVINO** | ✅ 它在 Ubuntu 22.04 上用 ABI=1 编译，与系统库一致 |

**最终方案**：安装 Intel 官方 APT 源的 OpenVINO（ABI=1），问题彻底消失：
```bash
wget -qO- https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB \
  | sudo gpg --dearmor --yes -o /usr/share/keyrings/intel-sw.gpg
echo "deb [signed-by=/usr/share/keyrings/intel-sw.gpg] https://apt.repos.intel.com/openvino/2024 ubuntu22 main" \
  | sudo tee /etc/apt/sources.list.d/intel-openvino.list
sudo apt update && sudo apt install -y openvino
```

---

## 六、M1 其他踩坑记录

### 6.1 OpenCV4 的 `${OpenCV_LIBS}` 为空

原代码里大量使用 `${OpenCV_LIBS}` 变量来链接 OpenCV，但在 Ubuntu 22.04 的 OpenCV 4.5.4 中，cmake 不再设置这个变量。

**解决**：改为显式列出需要的 opencv 组件：
```cmake
set(OpenCV_LIBS opencv_core opencv_imgproc opencv_highgui
    opencv_imgcodecs opencv_videoio opencv_calib3d opencv_dnn)
```

### 6.2 缺少 `opencv_dnn`

代码里用到 `cv::dnn::Net`、`cv::dnn::blobFromImage` 等 DNN 模块函数，一开始漏了 `opencv_dnn`，链接时报 `undefined reference to cv::dnn::dnn4_v20211004::...`。

### 6.3 fmt 包的 `--as-needed` 问题

Ubuntu 22.04 的 `libfmt-dev` 包的 cmake 配置里，`fmt::fmt` target 的 `INTERFACE_LINK_LIBRARIES` 包含 `-Wl,--as-needed`。这个 flag 在链接命令中间出现，导致排在它后面的库可能被跳过。

**解决**：
```cmake
get_target_property(_fmt_iface_libs fmt::fmt INTERFACE_LINK_LIBRARIES)
string(REPLACE "-Wl,--as-needed" "" _fmt_fixed "${_fmt_iface_libs}")
set_target_properties(fmt::fmt PROPERTIES INTERFACE_LINK_LIBRARIES "${_fmt_fixed}")
```

---

## 七、M2：从 rosbag 订阅数据并配对

### 7.1 目标

创建一个 ROS2 节点，同时订阅 bag 里的两个话题，把同一帧的图像和四元数**按时间戳配对**，组成 `(图像, 四元数, 时间戳)` 三元组。

### 7.2 rosbag 数据契约

bag 里只有两个话题，逐帧一一对应：

| 话题 | 消息类型 | 内容 | 备注 |
|---|---|---|---|
| `/image_raw` | `sensor_msgs/msg/Image` | BGR8, 1280×1024 | `header.stamp` 带时间戳 |
| `/imu/quaternion` | `autoaim_msgs/msg/Orienta` | 云台姿态四元数 | **无 header**，但和图像同序发出 |

**关键规则**：
- 同一帧的图像和四元数**同帧同序**发出 —— 这意味着按接收顺序配对即可
- `header.stamp` 是**单调时钟纳秒值**（`steady_clock`），**不是日期时间**。它的绝对值无意义，只用于：① 图像-四元数配对 ② 算相邻帧时间差 `dt`

### 7.3 bag_replay_node 的实现

```cpp
class BagReplayNode : public rclcpp::Node
{
  // 1. 两个独立订阅：各自计数
  image_sub_  = create_subscription<Image>("/image_raw", ...);
  quat_sub_   = create_subscription<Orienta>("/imu/quaternion", ...);

  // 2. message_filters 同步配对
  sync_image_sub_.subscribe(this, "/image_raw");
  sync_quat_sub_. subscribe(this, "/imu/quaternion");
  sync_ = make_shared<Sync>(ApproximateTime(10), sync_image_sub_, sync_quat_sub_);

  // 3. 配对回调
  void paired_callback(Image::SharedPtr img, Orienta::SharedPtr quat) {
    cv::Mat cv_img = cv_bridge::toCvCopy(img, "bgr8")->image;   // ROS Image → cv::Mat
    Eigen::Quaterniond q(quat->w, quat->x, quat->y, quat->z);   // → 四元数
    auto t = steady_clock::time_point(nanoseconds(img->header.stamp.nanosec)); // → 时间点
    // 打印验证
  }
};
```

### 7.4 关键技术点

**`cv_bridge`**：ROS 图像消息 ↔ OpenCV Mat 的桥梁
```cpp
cv::Mat img = cv_bridge::toCvCopy(ros_msg, "bgr8")->image;
```
BGR8 编码要和 bag 录制时一致。

**`message_filters::ApproximateTime`**：按时间戳近似匹配两路消息。bag 中同帧同序发出，近似匹配足够准确。

> 为什么不用 `ExactTime`？ROS2 Humble 的 `ExactTime` 策略对 2 路输入有模板参数 bug（内部使用 9 路 Signal9，但回调只接受 2 个参数导致编译失败）。

**`header.stamp.nanosec`**：bag 里用的时间戳是 `steady_clock` 的纳秒计数值，不是 Unix 时间。还原方式：
```cpp
auto t = std::chrono::steady_clock::time_point(
    std::chrono::nanoseconds(stamp_ns));
```

### 7.5 验证方式

```bash
# 终端 1：启动订阅节点
source install/setup.bash
ros2 run sp_vision bag_replay_node

# 终端 2：回放 bag
ros2 bag play bags/move_translate_bag --loop

# 预期输出：
# PAIRED [frame 1] stamp=... | q=(0.707,0.000,0.707,0.000) | img=1280x1024 | dt=33.33ms
# PAIRED [frame 2] ...
# 结束时: image=N, quat=N, paired=N （三个数相等 = 配对正确）
```

---

## 八、当前文件结构总览

```
sp_vision_ws/
├── LEARNING_GUIDE.md       ← 本学习指南
├── FIX_RECORD.md           ← 开发记录 / 踩坑笔记
├── .gitignore              ← 排除 build/install/log/大文件
├── shots/                  ← 截图目录
└── src/
    ├── autoaim_msgs/       ← 自建消息包 (M1)
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── msg/Orienta.msg
    └── sp_vision/          ← 改造后的自瞄源码
        ├── CMakeLists.txt  ← 主要修改：ament 化 + 条件编译 + install
        ├── package.xml     ← 新建
        ├── src/
        │   ├── standard.cpp          ← 原版主程序（读相机+下位机）
        │   └── bag_replay_node.cpp   ← M2 新建：ROS2 订阅配对节点
        ├── tasks/auto_aim/ ← 自瞄算法（检测/解算/跟踪/决策）
        ├── tasks/auto_buff/← 打符算法
        ├── io/             ← 硬件抽象层（相机/下位机/串口）
        └── tools/          ← 工具库（EKF/PNP/数学/日志）
```

---

## 九、依赖安装速查

```bash
# 基础系统库
sudo apt-get install -y libopencv-dev libfmt-dev libeigen3-dev \
  libspdlog-dev libyaml-cpp-dev libusb-1.0-0-dev nlohmann-json3-dev \
  libceres-dev

# ROS2 包
sudo apt-get install -y ros-humble-cv-bridge ros-humble-message-filters

# OpenVINO (Intel APT, ABI=1)
wget -qO- https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB \
  | sudo gpg --dearmor --yes -o /usr/share/keyrings/intel-sw.gpg
echo "deb [signed-by=/usr/share/keyrings/intel-sw.gpg] https://apt.repos.intel.com/openvino/2024 ubuntu22 main" \
  | sudo tee /etc/apt/sources.list.d/intel-openvino.list
sudo apt update && sudo apt install -y openvino
```
