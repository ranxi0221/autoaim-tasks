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

---

