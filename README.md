# autoaim-tasks

26 届暑期自瞄练习项目仓库 · 基于 TongjiSuperPower / sp_vision_25 的 ROS2 + rosbag 回放改造。

## 仓库内容

| 路径 | 说明 |
| --- | --- |
| `src/sp_vision/` | 原 sp_vision_25 内容（ROS2 化改造后的自瞄包） |
| `src/autoaim_msgs/` | 订阅 `/imu/quaternion` 所需的消息包（自定义 `Orienta`） |
| `FIX_RECORD.md` | 学习/开发记录（验收核心，按 M1–M5 里程碑记录） |
| `暑期自瞄练习项目.md` | 练习任务书（需求与验收标准原文） |
| `shots/` | 截图/录屏证据目录 |

## 练习目标

把 sp_vision 改造成 ROS2 包，从 rosbag 订阅「图像 + 云台姿态」按时间戳配对，跑通 检测 → 解算 → 跟踪 完整自瞄链路，并用 PlotJuggler 观察世界系目标稳定性。

## 里程碑

- M1 ROS 化编译（`colcon build` 通过、能 `ros2 run`）
- M2 回放订阅配对（拿到 `(图像, 四元数, 时间戳)`）
- M3 检测出装甲板
- M4 解算 + 跟踪
- M5 PlotJuggler 曲线验证（运动云台下世界系目标不漂移）

详细要求与数据契约见 `暑期自瞄练习项目.md`，开发记录见 `FIX_RECORD.md`。
