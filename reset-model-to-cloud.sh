#!/usr/bin/env bash
# 将本地 02caeff 版本的 model 文件夹文件同步到 GPUFree 云端
# 用法：bash reset-model-to-cloud.sh

set -euo pipefail

HOST="root@120.209.70.195"
PORT="30287"
SRC_DIR="云服务器上model文件夹里要改动的代码"

FILES=(
  "${SRC_DIR}/mission_once.py:/home/ubuntu/model/mission_once.py"
  "${SRC_DIR}/mission_prompt.txt:/home/ubuntu/model/prompts/mission_prompt.txt"
  "${SRC_DIR}/system_prompt.txt:/home/ubuntu/model/prompts/system_prompt.txt"
)

echo "===== 同步 model 文件夹到 GPUFree（02caeff 版本） ====="

for pair in "${FILES[@]}"; do
  src="${pair%%:*}"
  dst="${pair##*:}"
  echo "-> scp ${src} -> ${dst}"
  scp -P "${PORT}" "${src}" "${HOST}:${dst}"
done

echo ""
echo "===== 修复文件所有权 ====="
ssh -p "${PORT}" "${HOST}" "chown -R ubuntu:ubuntu /home/ubuntu/model"

echo ""
echo "===== 同步完成 ====="
