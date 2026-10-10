# GPUFree 文件同步说明

## 背景

本地仓库根目录对应一个云端 catkin 工作区。新成员应先按 [组员克隆与云端运行](组员克隆与云端运行.md) 部署到新的版本目录；本文主要说明已有 `/home/ubuntu/catkin_ws` 的差异同步。当前实例 SSH 端口为 `30624`，实例重建或端口变更后以平台信息为准。

发布分支为 `feature/perception`，代码和 FCOS 检测头一同提交，检测头由 Git LFS 分发。2026-10-09 的活动云端工作区尚未应用本次启动路径调整。

任务解析代码统一放在 `云服务器上model文件夹里要改动的代码/`。当前启动脚本直接运行这里的脚本和提示词；下表的 `/home/ubuntu/model` 目标仅供旧入口兼容，正常新版本部署无需更新那三份旧文件。

## 文件对应关系

| 本地文件（仓库内路径） | GPUFree 目标路径 | 说明 |
|---|---|---|
| `云服务器上model文件夹里要改动的代码/mission_once.py` | `/home/ubuntu/model/mission_once.py` | 大模型一次性任务生成脚本 |
| `云服务器上model文件夹里要改动的代码/system_prompt.txt` | `/home/ubuntu/model/prompts/system_prompt.txt` | 系统提示词 |
| `云服务器上model文件夹里要改动的代码/mission_prompt.txt` | `/home/ubuntu/model/prompts/mission_prompt.txt` | 任务提示词 |
| `sim_env_start.sh` | `/home/ubuntu/catkin_ws/sim_env_start.sh` | FM9G 文件路径相对于仓库根目录 |
| `tools/fm9g_grounding/` | `/home/ubuntu/catkin_ws/tools/fm9g_grounding/` | 九格检测、采集和训练源码 |
| `tools/fm9g_grounding/deployment/fcos-recovered-20261008/epoch-04/head.safetensors` | 同一仓库相对路径 | FCOS 检测头，由 Git LFS 管理 |

云端实例预装的基础模型路径为 `/home/ubuntu/model/FM9G4B-V`；YOLO 权重位于 `src/object_det/models/best.pt`。这两类服务器文件不在 Git 同步范围内，必须保留，不能复制、删除或覆盖。FM9G 模式不读取 YOLO `best.pt`。

ROS 一方源码包括 `src/mission_pkg`、`src/object_det`、`src/common_msgs`、`src/sensor_pkg`；EGO 的改动包括 `plan_manage` 下的 `ego_replan_fsm.cpp`、`planner_manager.cpp`，以及 `plan_env` 下的 `grid_map.h`、`grid_map.cpp`。同步这些代码也必须先检查差异，不能直接删除或替换整个 `src/`。工作区原来的 `build/`、`devel/`、PX4 和 RflySim SDK 不从本地复制。

`sim_env_start.sh` 现在默认运行工作区自身的 `tools/fm9g_grounding/ros_fcos_node.py`，默认检测头是上述版本目录，不再依赖实验目录的 `scripts/speed_head_20260923` 或 `deployment/current`。独立 Python 默认 `/home/ubuntu/.conda/envs/fm9g-train/bin/python`；可通过 `FM9G_NODE`、`FM9G_HEAD`、`FM9G_PYTHON` 显式覆盖。云端原有实验目录可以保留用于训练和历史回溯。

## 其他成员的九格运行准备

LLM 默认入口也是工作区内的 `mission_once.py`，`MISSION_PROMPT_PATH` 指向仓库 `mission_prompt.txt`，`MISSION_MODEL_BASE` 默认使用服务器基础模型。可通过 `LLM_NODE` 覆盖语言入口。独立版本目录里的代码与旧工作区互不覆盖。

安装 Git LFS 后，在本地 Git clone 中运行 `git lfs install`、`git lfs pull`。确认 `head.safetensors` 是实际约 29.6 MB 文件，而不是以 `version https://git-lfs.github.com/spec/v1` 开头的文本指针。云端没有 Git LFS 时，可以从已完成 LFS 下载的本地 clone 传输这一个检测头到新建的版本目录。

在云端 ubuntu 桌面终端准备独立环境。下面命令仅在环境不存在时克隆，不修改原 `demo`：

```bash
if [ ! -x /home/ubuntu/.conda/envs/fm9g-train/bin/python ]; then /opt/conda/bin/conda create --prefix /home/ubuntu/.conda/envs/fm9g-train --clone /opt/conda/envs/demo --offline --yes; fi
/home/ubuntu/.conda/envs/fm9g-train/bin/python -m pip install --no-cache-dir peft==0.13.2
```

当前镜像的 `demo` 已有 PyTorch 2.3.1、torchvision 0.18.1、transformers 4.44.2；完整运行版本清单见 `tools/fm9g_grounding/requirements-runtime.txt`。若镜像版本不同，只在 `fm9g-train` 中按清单安装，完成 CUDA 与 ROS 导入检查。克隆失败时先确认模板环境路径，不能改装仿真用的 `demo`。

代码差异合并后在 `/home/ubuntu/catkin_ws` 中执行 `catkin_make`。确认入口执行权限：`chmod 755 sim_env_start.sh build.sh kill_ros_pid.sh src/object_det/scripts/det_color_conda.sh`。再运行：

```bash
source /home/ubuntu/catkin_ws/devel/setup.bash
/home/ubuntu/.conda/envs/fm9g-train/bin/python -c 'import torch,torchvision,transformers,rospy; from common_msgs.msg import Objects; print(torch.__version__,torchvision.__version__,transformers.__version__,torch.cuda.is_available())'
sha256sum tools/fm9g_grounding/deployment/fcos-recovered-20261008/epoch-04/head.safetensors
/home/ubuntu/.conda/envs/fm9g-train/bin/python tools/fm9g_grounding/fcos_grounding.py probe --head tools/fm9g_grounding/deployment/fcos-recovered-20261008/epoch-04
```

检测头 SHA-256 应为 `7a7cd85a5152a4b354efd134e7126c7e8b0732dbe2fe609f805ebe7aad2fa882`。`probe` 只做离线模型检查，不启动仿真或发飞行指令。之后按原流程启动 MAVROS/SITL，在另一个桌面终端执行 `PERCEPTION_BACKEND=fm9g ./sim_env_start.sh`；另开终端检查 `rostopic info /objects`，唯一发布者应为 `/fm9g_spatial_detector`。先看到实际图像和检测框，再输入任务。在线检测不需要训练数据盘挂载；采集和训练另按恢复手册准备数据盘。

## 同步命令（本地 → GPUFree）

在本地仓库根目录执行。先检查 Git 状态和远端目标，不要直接把文件写到已有路径：

```bash
git status --short
PORT=30624
HOST=root@120.209.70.195
TAG=$(date +%Y%m%d_%H%M%S)
REMOTE_STAGE="/tmp/uav-sync-$TAG"
ssh -p "$PORT" "$HOST" "mkdir -m 700 '$REMOTE_STAGE'"
SRC="云服务器上model文件夹里要改动的代码"

# mission_once.py
scp -P "$PORT" "$SRC/mission_once.py" "$HOST:$REMOTE_STAGE/mission_once.py"

# system_prompt.txt
scp -P "$PORT" "$SRC/system_prompt.txt" "$HOST:$REMOTE_STAGE/system_prompt.txt"

# mission_prompt.txt
scp -P "$PORT" "$SRC/mission_prompt.txt" "$HOST:$REMOTE_STAGE/mission_prompt.txt"
```

需要同步项目代码时，也先传到同一临时目录做审核，不要把训练实验目录复制覆盖到工作区：

```bash
scp -P "$PORT" sim_env_start.sh "$HOST:$REMOTE_STAGE/sim_env_start.sh"
scp -P "$PORT" -r tools/fm9g_grounding "$HOST:$REMOTE_STAGE/tools-fm9g_grounding"
ssh -p "$PORT" "$HOST" "find '$REMOTE_STAGE' -type f -printf '%p %s bytes\n'"
```

已有云端路径先比哈希和差异；只对审查过的单个文件进行人工合并，并保留可回退副本。不要直接 `scp` 覆盖目标，不要使用整目录镜像或 `rsync --delete`。新训练得到的检测头应放进新的版本目录，不能覆盖已记录哈希的 epoch-04。

对于已经是 Git clone、没有本地修改、且活动分支与所需发布分支一致的云端工作区，可以更新仓库版本和 LFS 检测头，再按项目要求编译。Git 只更新跟踪文件，保留未跟踪的服务器权重；不要使用 `git clean -fdx` 或 `git reset --hard`。普通旧工作区不要直接套用此段，仍使用前述暂存和差异合并流程：

```bash
cd /home/ubuntu/catkin_ws
git status --short
git pull --ff-only
git lfs pull
catkin_make
```

如果上传后文件所有者不是 `ubuntu`，可以在 GPUFree 上执行：

```bash
ssh -p 30624 root@120.209.70.195 \
  "chown ubuntu:ubuntu /home/ubuntu/model/mission_once.py \
   /home/ubuntu/model/prompts/system_prompt.txt \
   /home/ubuntu/model/prompts/mission_prompt.txt"
```

## 注意事项

1. **合并前确认最终版本**：在把分支合并到 `main` 之前，确保本地三个任务解析文件是预期版本。
2. **不要提交或覆盖外部模型**：`/home/ubuntu/model/FM9G4B-V` 与 `src/object_det/models/best.pt` 不在 Git 同步范围内；FM9G 头是独立的 FCOS 检测头，单独由 Git LFS 管理。
3. **禁止盲目覆盖**：云端现有文件先比对并保留备份；本地没有的服务器文件不能因镜像同步而被删除。

## 反向同步（GPUFree → 本地）

先查 SHA-256：本地已有且内容相同的文件跳过，不重复下载。内容不同的文件仅下载到全新临时目录用于比较；不要将下载目录设为本地源码目录。Linux/WSL/Git Bash 示例：

```bash
LOCAL_STAGE=$(mktemp -d)
scp -P 30624 root@120.209.70.195:/home/ubuntu/model/mission_once.py "$LOCAL_STAGE/mission_once.py"
diff -u "云服务器上model文件夹里要改动的代码/mission_once.py" "$LOCAL_STAGE/mission_once.py"
```

其余单文件按同样方式比较，确认云端新增的改动有必要保留后再人工合并。不要把整个云端 `model/` 下载进仓库。
