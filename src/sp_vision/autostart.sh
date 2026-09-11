#!/bin/bash
# 开机自启入口（由 ~/.config/autostart/sp_vision.desktop 调用，Exec 须为本文件绝对路径）
# 原版路径写死 ~/Desktop/sp_vision_25/，改为按脚本自身位置推导，可随工作区任意放置。
sleep 5
SP_VISION_DIR="$(dirname "$(readlink -f "$0")")"
cd "$SP_VISION_DIR" || exit 1
screen \
    -L \
    -Logfile logs/$(date "+%Y-%m-%d_%H-%M-%S").screenlog \
    -d \
    -m \
    bash -c "./watchdog.sh"
