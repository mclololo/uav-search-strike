#!/usr/bin/env bash
# 将本次二维码/ArUco 检测鲁棒性修复同步到 GPUFree
set -euo pipefail

HOST="root@120.209.70.195"
PORT="30287"

FILES=(
  "src/mission_pkg/scripts/aruco_detect_node.py:/home/ubuntu/catkin_ws/src/mission_pkg/scripts/aruco_detect_node.py"
  "src/mission_pkg/launch/aruco_detect.launch:/home/ubuntu/catkin_ws/src/mission_pkg/launch/aruco_detect.launch"
)

echo "===== 同步二维码/ArUco 鲁棒性修复到 GPUFree ====="

for pair in "${FILES[@]}"; do
  src="${pair%%:*}"
  dst="${pair##*:}"
  echo "-> scp ${src} -> ${dst}"
  scp -P "${PORT}" "${src}" "${HOST}:${dst}"
done

echo ""
echo "===== 修复文件所有权 ====="
ssh -p "${PORT}" "${HOST}" "chown -R ubuntu:ubuntu /home/ubuntu/catkin_ws/src/mission_pkg"

echo ""
echo "===== 同步完成 ====="
echo "请在云服务器执行：cd /home/ubuntu/catkin_ws && catkin_make"
