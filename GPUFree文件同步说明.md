# GPUFree 文件同步说明

## 背景

以下文件最初是从 GPUFree 云服务器下载到本地后修改的，主要用于大模型提示词与一次性任务生成。它们**不完全是普通 ROS 包代码**，合并分支后，其他成员拉取代码时需要注意：如果这些文件有更新，必须手动同步回 GPUFree 对应路径，否则云服务器上运行的是旧版本。

## 文件对应关系

| 本地文件（仓库内路径） | GPUFree 目标路径 | 说明 |
|---|---|---|
| [`mission_once.py`](mission_once.py) | `/home/ubuntu/model/mission_once.py` | 大模型一次性任务生成脚本 |
| [`prompts/system_prompt.txt`](prompts/system_prompt.txt) | `/home/ubuntu/model/prompts/system_prompt.txt` | 系统提示词 |
| [`prompts/mission_prompt.txt`](prompts/mission_prompt.txt) | `/home/ubuntu/model/prompts/mission_prompt.txt` | 任务提示词 |

## 同步命令（本地 → GPUFree）

在本地仓库根目录执行：

```bash
# mission_once.py
scp -P 30287 mission_once.py root@120.209.70.195:/home/ubuntu/model/mission_once.py

# system_prompt.txt
scp -P 30287 prompts/system_prompt.txt root@120.209.70.195:/home/ubuntu/model/prompts/system_prompt.txt

# mission_prompt.txt
scp -P 30287 prompts/mission_prompt.txt root@120.209.70.195:/home/ubuntu/model/prompts/mission_prompt.txt
```

如果上传后文件所有者不是 `ubuntu`，可以在 GPUFree 上执行：

```bash
ssh -p 30287 root@120.209.70.195 \
  "chown ubuntu:ubuntu /home/ubuntu/model/mission_once.py \
   /home/ubuntu/model/prompts/system_prompt.txt \
   /home/ubuntu/model/prompts/mission_prompt.txt"
```

## 注意事项

1. **合并前确认最终版本**：在把分支合并到 `main` 之前，确保本地这三个文件就是你希望同步到 GPUFree 的最终版本。
2. **prompt 文件已放入 `prompts/` 目录**：本地 [`mission_once.py`](mission_once.py) 从 `prompts/mission_prompt.txt` 加载提示词，请保持该目录结构。
3. **不要提交大模型权重**：`/home/ubuntu/model/FM9G4B-V` 等大模型文件不应提交到 git，也不在本文同步范围内。

## 反向同步（GPUFree → 本地）

如果需要把云服务器上的当前版本下载回本地对比，可执行：

```bash
scp -P 30287 root@120.209.70.195:/home/ubuntu/model/mission_once.py ./mission_once.py
scp -P 30287 root@120.209.70.195:/home/ubuntu/model/prompts/system_prompt.txt ./prompts/system_prompt.txt
scp -P 30287 root@120.209.70.195:/home/ubuntu/model/prompts/mission_prompt.txt ./prompts/mission_prompt.txt
```
