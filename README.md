# autoaim-tasks

26 届暑期自瞄练习项目仓库 · 基于 TongjiSuperPower / sp_vision_25 的 ROS2 + rosbag 回放改造。
**M1~M5 五个里程碑已全部跑通**（检测 → 解算 → 跟踪 → PlotJuggler 验证）。

---

## 一、我的文件都在哪里（仓库文件清单）

### 根目录

| 文件/目录 | 是什么 |
| --- | --- |
| `README.md` | 本文件：仓库说明 + 学习指南 |
| `FIX_RECORD.md` | **学习/开发记录（验收核心）**，M1~M5 每个里程碑怎么做的、踩了什么坑、怎么排查，都写在里面 |
| `暑期自瞄练习项目.md` | 练习任务书（需求与验收标准原文） |
| `LEARNING_GUIDE.md` | 深度学习指南（ROS2 工作区、colcon、rosbag 等概念的逐字讲解，M2 时写的） |
| `src/` | 所有包（代码），见下节 |
| `shots/` | **证据目录**：colcon build 截图、M3 检测截图 118 张、M5 曲线截图、20s 录屏 |
| `bags/` | rosbag 数据包（2~5GB，**不进 git**） |
| `build/ install/ log/` | `colcon build` 自动生成（**不进 git**），删掉随时可以重新编译生成 |
| `move_translate_bag.zip` | bag 的压缩备份（**不进 git**） |
| `截图 2026-08-23 22-02-33.png` | 最新一张 PlotJuggler 截图（散落在根目录，可选归档进 `shots/`） |
| `.gitignore` 等隐藏文件 | git 配置：声明哪些文件不进仓库 |

### `src/sp_vision/` — 自瞄主包

| 文件/目录 | 是什么 |
| --- | --- |
| `package.xml` | 包定义（依赖声明） |
| `CMakeLists.txt` | 编译规则（ament_cmake 化改造的成果，M1） |
| `src/bag_replay_node.cpp` | **我们的核心节点**：订阅 bag → 配对 → 检测 → 解算 → 跟踪 → 发布 `/target/state`（M2~M5 全部在这一个文件里） |
| `src/standard.cpp` 等 | 原项目的真机入口程序（真车上跑用，练习中没动） |
| `tasks/auto_aim/` | 算法层：`detector/yolo`（检测）、`solver`（PnP 解算）、`tracker`（EKF 跟踪状态机）、`target`（整车状态）、`aimer/planner`（决策，未接入） |
| `tools/` | 工具层：`extended_kalman_filter`、`img_tools`（画框）、`math_tools`（欧拉角等）、`logger`、`plotter` |
| `io/` | 硬件抽象层（相机驱动、下位机通信，练习中没用到） |
| `assets/` | 模型权重（yolo11.xml/bin、tiny_resnet.onnx 等） |
| `configs/standard3.yaml` | **配置入口**：6mm 相机内外参、模型路径、device、enemy_color、tracker 参数 |
| `calibration/ tests/` | 标定程序和模块测试（原项目自带） |

### `src/autoaim_msgs/` — 自定义消息包

| 文件/目录 | 是什么 |
| --- | --- |
| `msg/Orienta.msg` | 云台姿态四元数（bag 里 `/imu/quaternion` 的类型契约，M1 自建） |
| `msg/TargetState.msg` | 目标状态（世界系位置/速度/装甲板位置/状态等 16 字段，M5 自建） |
| `package.xml` / `CMakeLists.txt` | 消息包的编译配置（用 `rosidl_generate_interfaces` 生成消息类型） |

### 我的"视频"在哪里

| 想看什么 | 在哪 |
| --- | --- |
| 20 秒验收录屏（识别画面 + 曲线同屏） | `shots/m5_recording.webm` |
| bag 原始数据（1247 帧"视频"） | `bags/move_translate_bag/`（db3 文件） |
| bag 的压缩备份 | 根目录 `move_translate_bag.zip` |
| 每一帧的检测画面截图 | `shots/m3_frame*.jpg` / `shots/m4_frame*.jpg` |
| PlotJuggler 曲线截图 | `shots/plotjuggler_curves.png` |

---

## 二、学习指南

### 1. 这个项目在做什么

把 sp_vision_25（同济 SuperPower 战队的 RoboMaster 自瞄算法）从"读真实相机 + 下位机"改成"读 rosbag 录好的数据包"，离线跑通完整自瞄链路：

```text
rosbag 回放
  ├── /image_raw         1280×1024 图像 ──┐
  └── /imu/quaternion    云台姿态四元数 ──┤ 同帧同序配对
                                          ↓
                       bag_replay_node（ROS2 节点）
                          ① YOLO 检测装甲板（M3）
                          ② PnP 解算 + EKF 跟踪（M4）
                          ③ 发布 /target/state（M5）
                                          ↓
                       PlotJuggler 实时曲线（M5 验收）
```

### 2. 五分钟跑起来（三终端）

```bash
# T1 节点（cwd 必须在 src/sp_vision，配置/模型是相对路径）
cd ~/sp_vision_ws/src/sp_vision
source ~/sp_vision_ws/install/setup.bash
ros2 run sp_vision bag_replay_node

# T2 回放 bag（两个终端都要 source，否则报 autoaim_msgs not found）
source ~/sp_vision_ws/install/setup.bash
ros2 bag play ~/sp_vision_ws/bags/move_translate_bag [--loop]

# T3 曲线（必须过滤海康 MVS 的 LD_LIBRARY_PATH，否则 Qt 崩溃）
source ~/sp_vision_ws/install/setup.bash
export LD_LIBRARY_PATH=$(printf '%s' "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v '/opt/MVS' | paste -sd: -)
ros2 run plotjuggler plotjuggler
```

PlotJuggler 操作顺序：**先起 T1/T2，再点 ROS2 Topic Subscriber**（话题列表是点击时的快照）→ 勾选 `/target/state` → 把 `x/y/z`（目标旋转中心）和 `armor_x/y/z`（可见装甲板）拖进图区。

### 3. 每个里程碑我学到了什么

| 里程碑 | 做了什么 | 学到的核心知识 |
| --- | --- | --- |
| **M1 ROS 化编译** | 原 CMake 项目改造成 ament_cmake 包，`colcon build` 通过 | ROS2 工作区/包结构；C++ ABI 不匹配（pip OpenVINO ABI=0 vs 系统库 ABI=1，换 Intel APT 版解决）；OpenCV4 的 `${OpenCV_LIBS}` 已废弃 |
| **M2 回放订阅配对** | 节点订阅两个话题，同帧同序配对 | 话题/消息/QoS（reliable）；`Orienta.msg` 没有 header，无法用 message_filters 时间同步 → 顺序配对；时间戳是单调时钟纳秒，只用于配对和算 dt |
| **M3 检测出装甲板** | 接入 YOLO（OpenVINO 推理），填 6mm 内外参 | OpenVINO 模型加载与推理；相机内外参的作用（PnP 的前置）；无显示器环境用截图代替窗口 |
| **M4 解算 + 跟踪** | 接入 solver（PnP）+ tracker（EKF 状态机） | PnP 解算原理与坐标变换链（camera→gimbal→world 的 sandwich 补偿）；EKF 估计整车状态；跟踪状态机（detecting 5 帧→tracking，temp_lost 15 帧→lost）；`enemy_color` 配错会静默过滤掉所有目标 |
| **M5 PlotJuggler** | 发布 `/target/state`，曲线验证不漂移 | 自定义消息包扩展；话题发布；用曲线做定量验收（回归斜率 ≈0.1mm/帧 = 无系统性漂移）；target（旋转中心，平滑）与 armor（可见板，振荡）的对照读图 |

### 4. 常见坑速查

| 现象 | 原因 | 解法 |
| --- | --- | --- |
| 报 `package 'autoaim_msgs' not found` | 终端没 source 工作区 | 每个终端 `source install/setup.bash` |
| PlotJuggler 启动崩溃（Qt 平台插件报错） | 海康 MVS SDK 的 `/opt/MVS/bin` 污染 `LD_LIBRARY_PATH` | 启动前过滤 `/opt/MVS` 路径（见上 T3 命令） |
| `dt` 出现负值 | `--loop` 回绕时时间戳跳回 0 | 正常现象；正式验收用单次播放 |
| 一直没有目标输出 | `enemy_color` 配成 red 而场景是蓝方 | `configs/standard3.yaml` 改 `enemy_color: "blue"` |
| 节点起不来/YOLO 加载失败 | cwd 不在 `src/sp_vision`，相对路径失效 | 先 `cd src/sp_vision` 再 `ros2 run` |
| 曲线跟不上、回放掉帧 | CPU 推理 ~31fps < bag 46.5fps | 加 `--rate 0.3` 放慢，或攻克 GPU 推理 |
| PlotJuggler 话题列表是空的 | 列表是点击订阅按钮时的快照 | 先起节点和 bag，再点 ROS2 Topic Subscriber |

### 5. 验收证据清单（任务书 §八）

| 证据 | 位置 |
| --- | --- |
| colcon build 成功截图 | `shots/colcon_build.png` |
| M3 检测框截图（118 张） | `shots/m3_frame*.jpg` |
| M4 跟踪 + 重投影截图 | `shots/m4_frame*.jpg` |
| M5 PlotJuggler 曲线截图 | `shots/plotjuggler_curves.png` |
| 10~30s 录屏（画面 + 曲线同屏） | `shots/m5_recording.webm` |
| 开发记录（验收核心） | `FIX_RECORD.md` |

---

## 三、仓库结构速览

```
sp_vision_ws/
├── README.md                # 本文件（学习指南）
├── FIX_RECORD.md            # 开发记录（验收核心）
├── 暑期自瞄练习项目.md        # 任务书
├── LEARNING_GUIDE.md        # 深度学习指南
├── shots/                   # 证据：截图 + 录屏
├── bags/                    # rosbag 数据（不进 git）
├── src/
│   ├── sp_vision/           # 自瞄主包（原 sp_vision_25 改造）
│   │   ├── src/bag_replay_node.cpp   # 我们的节点（M2~M5）
│   │   ├── tasks/auto_aim/  # 检测/解算/跟踪算法
│   │   ├── configs/standard3.yaml    # 配置入口
│   │   └── assets/          # 模型权重
│   └── autoaim_msgs/        # 自定义消息包
│       └── msg/             # Orienta.msg + TargetState.msg
├── build/ install/ log/     # 编译产物（不进 git）
└── move_translate_bag.zip   # bag 压缩备份（不进 git）
```

详细开发过程、每个坑的排查记录见 `FIX_RECORD.md`；ROS2 基础概念讲解见 `LEARNING_GUIDE.md`；任务要求原文见 `暑期自瞄练习项目.md`。
