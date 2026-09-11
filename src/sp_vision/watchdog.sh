#!/bin/bash
# standard_mpc 看门狗：进程退出（崩溃/被 kill）后自动重启。
# 注意：configs/assets 都是相对路径，必须先 cd 到本脚本所在目录（src/sp_vision）。
set -u

SP_VISION_DIR="$(dirname "$(readlink -f "$0")")"
cd "$SP_VISION_DIR" || exit 1

source /opt/ros/humble/setup.bash
source "$SP_VISION_DIR/../../install/setup.bash"

mkdir -p logs

while true; do
  echo "[watchdog] $(date '+%F %T') starting standard_mpc ..."
  ros2 run sp_vision standard_mpc configs/arm_deploy.yaml >> "logs/watchdog_$(date +%F).log" 2>&1
  code=$?
  echo "[watchdog] $(date '+%F %T') standard_mpc exited with code=$code, restart in 3s"
  sleep 3
done
