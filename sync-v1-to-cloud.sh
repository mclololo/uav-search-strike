#!/usr/bin/env bash
# 将 v1 控制稳定性修复同步到 GPUFree 云服务器
# 用法：在本地 WSL/Git Bash 中运行：bash sync-v1-to-cloud.sh
# 注意：本脚本只同步 src/mission_pkg/ 中 v1 改动的文件，不会覆盖云端的 .pt/.pth 权重。

set -euo pipefail

HOST="root@120.209.70.195"
PORT="30287"
REMOTE_BASE="/home/ubuntu/catkin_ws/src/mission_pkg"

FILES=(
  "config/full_competition.xml"
  "config/full_competition_blue.xml"
  "config/full_competition_purple.xml"
  "config/full_competition_red.xml"
  "config/hit_balloon.xml"
  "include/mavros_cnt.h"
  "include/nodes/action/visual_servo.h"
  "launch/llm_mission.launch"
  "src/bt_ros.cpp"
  "src/mavros_cnt.cpp"
  "src/nodes/action/visual_servo.cpp"
)

echo "===== 同步 v1 控制稳定性修复到 GPUFree ====="
echo "目标：${HOST}:${PORT}${REMOTE_BASE}"
echo ""

for f in "${FILES[@]}"; do
  src="src/mission_pkg/${f}"
  dst="${REMOTE_BASE}/${f}"
  echo "-> scp ${src} -> ${dst}"
  scp -P "${PORT}" "${src}" "${HOST}:${dst}"
done

echo ""
echo "===== 修复文件所有权 ====="
ssh -p "${PORT}" "${HOST}" "chown -R ubuntu:ubuntu ${REMOTE_BASE}"

echo ""
echo "===== 同步完成 ====="
echo "请在云服务器执行以下操作："
echo "  1. cd /home/ubuntu/catkin_ws"
echo "  2. catkin_make"
echo "  3. source devel/setup.bash"
echo "  4. 重新启动 roslaunch mission_pkg llm_mission.launch"
echo ""
echo "注意：/home/ubuntu/model/ 下的 mission_once.py / mission_prompt.txt / system_prompt.txt"
echo "      保持当前感知分支版本，不要被 v1 的旧版本覆盖。"
