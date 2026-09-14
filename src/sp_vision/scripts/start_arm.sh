#!/bin/bash
# 摆臂一键启动：等待下位机串口(/dev/gimbal)与海康相机(2bdf)就绪后，启动 standard_mpc
# 用法（在任意目录）：~/sp_vision_ws/src/sp_vision/scripts/start_arm.sh
set -u

SP_VISION_DIR="$(dirname "$(readlink -f "$0")")/.."   # scripts/ 的上一级 = src/sp_vision
cd "$SP_VISION_DIR" || exit 1

set +u   # ROS setup 脚本会引用未定义变量，不能在其执行期间保持 set -u
source /opt/ros/humble/setup.bash
source "$SP_VISION_DIR/../../install/setup.bash"
set -u

echo "[start_arm] 等待下位机串口 /dev/gimbal ...（未出现 = 检查两根线/电源）"
while [ ! -e /dev/gimbal ]; do sleep 1; done
echo "[start_arm] 串口就绪"

echo "[start_arm] 等待相机 (2bdf) ..."
while ! lsusb | grep -qi "2bdf"; do sleep 1; done
echo "[start_arm] 相机就绪"

export GDK_BACKEND=x11   # Wayland 下 OpenCV 窗口兼容
exec ros2 run sp_vision standard_mpc configs/arm_deploy.yaml
