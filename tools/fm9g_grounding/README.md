# FM9G4B-V 图文定位实验

> 2026-10-09 成员部署：实时九格节点与 FCOS 检测头默认从仓库自身的 `tools/fm9g_grounding/` 加载。首次 clone 后执行 `git lfs pull`，按 [组员克隆与云端运行](../../docs/组员克隆与云端运行.md) 准备独立 Python 环境并核验头哈希。基础模型和 YOLO 权重不参加 Git 同步。采集/训练直接 source 工作区的 `tools/fm9g_grounding/cloud_env.sh`；当前活动云端启动脚本尚未同步此次本地调整。

> 2026-10-08：当前正式路线为冻结九格视觉编码器 + FCOS 检测头，新实例已恢复至 `root@120.209.70.195:30624`。完整的原图采集、人工标注、上传、检测头训练、评估与部署步骤见 [新实例恢复与九格采集训练](../../docs/新实例恢复与九格采集训练_20261008.md)。本文件下方的旧 LoRA 命令和旧实例阶段性记录只作历史参考。

分阶段计划、执行状态和下一步工作见 [九格全程感知替代 YOLO 任务清单](../../docs/九格全程感知替代YOLO任务清单.md)。

当前主路线是在冻结九格 `FM9G4B-V` 视觉编码器的空间 patch 特征上训练 `torchvision FCOS` 检测头，直接输出四类目标 bbox，不训练飞行动作或 VLN。实时节点不会调用九格语言生成器，也不会调用 YOLO；实验不会发布 ROS 控制命令。旧 LoRA 图文框生成仅作为历史基线保留。

赛事运行约束（用户于 2026-09-10 明确）：LoRA、视觉层微调和新增检测头都属于允许的“使用九格模型”方式；启用九格后，任务全程禁用 YOLO，包括搜索、穿框、气球识别、打击、返航及异常恢复。不得在九格超时、目标丢失或接近目标时切回 YOLO，也不能将重命名或包装后的 YOLO 作为九格检测头。当前数据方案采用人工标注，本次口径不推定离线 YOLO 预标注的许可范围。

当前 FCOS 检测头已经实现并完成独立测试。训练代码为 `fcos_grounding.py`，ROS 节点为 `ros_fcos_node.py`。后续跟踪须使用新的图像观测；观测无效时重新检测、悬停或退出，不能借助 YOLO 恢复。仍需用新增人工飞行数据验证定位精度、观测时效和全流程闭环表现。

云端路径：

- 基础模型：`/home/ubuntu/model/FM9G4B-V`
- 实验：`/home/ubuntu/model/experiments/fm9g4b_grounding`
- Python：`/home/ubuntu/.conda/envs/fm9g-train/bin/python`
- 远端脚本：实验目录的 `scripts/`
- 本地源码：`tools/fm9g_grounding/`
- 数据盘访问路径：`/home/ubuntu/fm9g-data`，实验的 `data/`、`checkpoints/` 指向该路径下同名目录。

## 历史 LoRA 验证范围（不用于当前在线检测）

`smoke` 使用程序生成的 640×480 空白图和空目标答案，进行两次优化更新。检查有限 loss、非零 LoRA 梯度、保存 adapter、清空其参数后恢复、恢复前后 loss 一致以及生成接口。输出目录含 `SMOKE-ONLY`，不能将该 adapter 用于比赛或认为已经训练出目标识别能力。

2026-09-22 已使用 459 张真实仿真图像完成首轮 LoRA：训练 207 张、验证 90 张、独立测试 162 张，按飞行场景隔离。最佳 `grounding-r16-v1/epoch-02` 验证 loss 为 0.55843，测试定位 F1@IoU0.5 为 19.81%，平均生成耗时 3.74 秒；此权重只作离线实验，尚不能代替 YOLO 接入飞行控制。这是历史 LoRA 基线记录，当前 FCOS 头的说明见 [部署检测头](deployment/README.md)。

示例 JSONL 仍只是格式示例，引用的图片不存在，不属于真实训练数据。

2026-09-09 云端 L40 已验证：310 个语言投影模块、21,109,760 个可训练参数；两步 smoke loss 为 2.7434 和 1.7505，保存前后验证 loss 均为 0.842862，峰值已分配显存约 9.85 GiB。独立进程重载成功。生成结果仍带解释文字，不符合严格 JSON 格式，不能接入控制。另用 3 张训练空白图和 1 张验证空白图跑通正式 CLI 的一个测试 epoch；这组 loss 没有真实识别性能含义。

本轮日志：`logs/smoke.log`、`logs/reload-infer.log`、`logs/pipeline-smoke.log`。结果 JSON 在 `eval/smoke-20260909-144634/` 和 `eval/pipeline-SMOKE-ONLY-20260909-145042/`。这些 adapter 只用于链路验证，不能部署到任务节点。

## 运行

```bash
conda activate /home/ubuntu/.conda/envs/fm9g-train
cd /home/ubuntu/model/experiments/fm9g4b_grounding
python scripts/fm9g_grounding.py probe
```

新的链路验证（输出目录必须不存在）：

```bash
python scripts/fm9g_grounding.py smoke --output eval/smoke-manual-001
```

实时查看本轮远程启动的验证：

```bash
tail -f logs/smoke.log
```

## 准备真实数据

每行一个 JSON，`image` 相对于 `--data-root`。bbox 是原图像素的 `[left,top,right,bottom]`，不是 YOLO 的中心宽高。类别只能为 `square_frame`、`red_balloon`、`blue_balloon`、`purple_balloon`。无目标用 `objects:[]`，不要标注完全不可见的物体。方框边界必须与三维定位使用的物理尺寸一致。

按整次采集的 `scene_id` 分开训练、验证、测试，避免相邻视频帧泄漏。可人工画框，也可按用户确认的实验方案直接用 YOLO 输出作为自动训练标签，无需逐张手动画框、选类别或确认。自动标签保留来源和未人工审核的标记；验证/测试集使用独立复核标签。只改变图像的颜色而不调整标签会破坏气球颜色监督。

预期目录：

```text
data/
  images/run_001/*.jpg
  images/run_002/*.jpg
  train.jsonl
  val.jsonl
  test.jsonl
```

优先先录制前视图像和里程计并进行标注。现有采集脚本只保存图片，没有 bbox 标注功能，默认话题也不是当前仿真话题。启动真实仿真后才执行录包；本次没有启动仿真或采集：

```bash
rosbag record -O run_001.bag /rflysim/sensor1/img_rgb /rflysim/uav1/local/odom
```

更方便的单独图片采集器已经放在 `scripts/capture_frames.py`。它只订阅图像，不启动仿真、不发布任务命令、不控制飞机；默认每秒保存一帧，最多 180 秒或 180 张，并严格检查图像编码、时间戳和数据盘挂载：

```bash
conda activate /home/ubuntu/.conda/envs/fm9g-train
source /home/ubuntu/catkin_ws/devel/setup.bash
cd /home/ubuntu/model/experiments/fm9g4b_grounding
python scripts/capture_frames.py --run run_001 --scene-id run_001 --seconds 180 --max-frames 180
```

开始采集前确认仿真传感器已启动，并核对话题确实存在。`capture_frames.py` 自身不依赖 YOLO；当前选择 `PERCEPTION_BACKEND=fm9g ./sim_env_start.sh` 可使用九格 FCOS 节点完成在线感知。默认 YOLO 模式不能用于全程禁用 YOLO 的比赛任务。

```bash
mountpoint /home/ubuntu/fm9g-data
rostopic hz /rflysim/sensor1/img_rgb
rostopic echo -n 1 /rflysim/sensor1/img_rgb/header
```

采集时让无人机分别处于起飞区、框前、框后、三个气球区域和不同高度；每次改变相机视角、距离或目标状态后使用新的 `--run`。相邻帧不能跨训练/验证/测试集合。建议先做 3～5 次独立运行，总计 300～500 张，确认标注规范后再扩充。

## 标注图片

### 从运行中的 YOLO 自动收集预标注（2026-09-20）

`capture_pseudo.py` 只订阅前视原图和 `/objects`，不发送飞行指令。按原始 ROS 时间戳及 frame_id 精确配对，可在检测消息先到或延迟到达时匹配；时间倒退则停止本批次，避免重启后把不同飞行混在一起。图像仍按 `encoding` 解码保存，不保存识别窗口的叠字截图。

在原来的仿真启动步骤完成后、输入任务之前，另开 ubuntu 终端执行：

```bash
source /home/ubuntu/catkin_ws/devel/setup.bash
cd /home/ubuntu/model/experiments/fm9g4b_grounding/scripts/yolo_auto_20260920
/home/ubuntu/.conda/envs/fm9g-train/bin/python capture_pseudo.py \
  --run yolo_run_002 --scene-id flight_002 --seconds 600 --max-frames 600
```

默认每秒最多保存一张；任务完成后在采集终端按 Ctrl+C 正常关闭文件，再重启仿真。每轮使用新 `--run`；同一飞行分段采集时保持同一 `--scene-id`。数据根目录默认 `/home/ubuntu/fm9g-data/data`，须已挂载数据盘。不要直接执行 `sim_env_start.sh` 来恢复挂载。

- 原图与采集元信息：`data/images/yolo_run_002/`。
- 自动框：`data/pseudo/yolo_run_002.jsonl`，同时写入图片目录的 `pseudo.jsonl`。带 `annotation_source=yolo_ros`、检测 score、时间戳及 `review_status=pending`。
- `/objects` 无同帧消息时保存 `detection_status=missing`，空框仅表示未知，不能当作已确认无目标。当前检测节点无检测时不会发布消息。
- 每张图都保留为待复核，包括高置信度框和没有匹配结果的图。`score` 不是颜色正确率，也不能衡量整张图是否漏检。
- 下载整个图片目录（包括 `pseudo.jsonl`），用最新版 `annotate.html` 选择此目录即可自动显示框、类别和置信度；旧版本只读取图片且需额外导入 JSONL，这是“目标框 0”的原因。
- 点击“导出 YOLO 标注”直接导出原始检测框，无需逐张确认；输出 `*_yolo_training.jsonl`，标记 `review_status=auto_accepted`、`human_reviewed=false`。新训练加载器允许此类匹配有效的自动标签，仍拒绝 missing 帧、无框帧及待处理的原始记录。此按钮使用原始 YOLO 结果，手动修改通过“导出已复核标注”输出。
- 自动训练标签放 `data/auto_annotations/`；独立复核标签放 `data/annotations/`。数据集准备工具和训练程序允许自动训练标签，但验证/测试集不使用自动标签，以免将 YOLO 的错误当成评估真值。
- `prepare_dataset.py` 是独立命令，不是 `fm9g_grounding.py prepare` 子命令，必须显式指定不同飞行批次作为验证集和测试集。

```bash
python prepare_dataset.py --data-root /home/ubuntu/fm9g-data/data \
  --annotations /home/ubuntu/fm9g-data/data/annotations \
  --output /home/ubuntu/fm9g-data/data/v1 \
  --val-scenes flight_002 --test-scenes flight_003
```

也可以直接在云端导出自动训练标签：

```bash
python export_yolo_labels.py --input /home/ubuntu/fm9g-data/data/pseudo/yolo_run_002.jsonl \
  --data-root /home/ubuntu/fm9g-data/data \
  --output /home/ubuntu/fm9g-data/data/auto_annotations/yolo_run_002_yolo_training.jsonl
```

云端新版工具目录 `scripts/yolo_auto_20260920` 包含采集、导出、数据准备和训练脚本；旧目录没有自动标签支持。准备正式数据集时将选定的自动训练文件与复核后的独立验证/测试文件放到同一输入标注目录，再执行上述 `prepare_dataset.py` 命令，不能重复放同一张图的两个版本。

YOLO 自动标签可直接监督训练，但其中的漏检、错色也会传给九格模型。正式九格任务仍全程禁用 YOLO。不要把离线预标注实验等同于赛事方对训练数据来源的最终确认。

本地检查：`python -m unittest discover -s tools/fm9g_grounding -p 'test_*.py' -v`。

### 浏览器复核

浏览器标注页面为 `scripts/annotate.html`，依赖同目录的 `lucide.min.js`。将 `data/images/run_001/` 从云端下载到本地后，用 Chrome 或 Edge 打开页面，点击“选择采集目录”，选择整个 `run_001` 文件夹。也可以直接在云端桌面浏览器打开同一文件；页面不会上传图片。

标注规则：

1. 方框标外框的四个像素边界，并始终使用同一种边界定义；它必须与 `TargetLocator` 使用的方框物理尺寸对应。
2. 气球框住可见气球主体，部分遮挡时框住可见部分并保留该图；完全看不到目标时点击“确认无目标”。
3. 目标类别只能是方框、红气球、蓝气球、紫气球。颜色不确定、严重模糊或框边界不清时点击“跳过”，不要猜。
4. 一张图有多个目标就分别画框。可切换选择工具移动/调整框，或删除误框；“确认并下一张”才会把这张图纳入导出。

页面会在浏览器本地保存进度。完成一批后点击“导出 JSONL”，把例如 `run_001.jsonl` 放入云端：

```text
data/annotations/run_001.jsonl
```

导出的每行包含原图相对路径、原图宽高、场景 ID、像素 bbox 和 `objects:[]` 空目标记录。不要手写归一化坐标，转换脚本会在训练时生成 0～1000 坐标。

Windows 下载示例（本地 PowerShell）：

```powershell
scp -P 30532 -r root@120.209.70.195:/root/gpufree-data/fm9g4b_grounding/data/images/run_001 .\run_001
```

标注完成后上传图片和 JSONL；不要只上传 JSONL：训练校验会检查图片尺寸和路径。

```powershell
scp -P 30532 -r .\run_001 root@120.209.70.195:/root/gpufree-data/fm9g4b_grounding/data/images/
scp -P 30532 .\run_001.jsonl root@120.209.70.195:/root/gpufree-data/fm9g4b_grounding/data/annotations/
```

上传后在云端运行数据整理。下面示例把 `run_001` 作为训练集、`run_002` 作为验证集、`run_003` 作为测试集；这三个场景必须真实存在且互不相同：

```bash
python scripts/prepare_dataset.py \
  --data-root data \
  --annotations data/annotations \
  --output data/v1 \
  --val-scenes run_002 \
  --test-scenes run_003
```

它会拒绝越界框、未知类别、重复图片、跨集合同一场景和缺失图片。然后先运行：

```bash
python scripts/fm9g_grounding.py validate \
  --data-root data \
  --train data/v1/train.jsonl \
  --val data/v1/val.jsonl
```

看到 `DATA_VALID` 后才开始真实 LoRA 训练。测试集 `data/v1/test.jsonl` 只用于最终评估，不参与训练或调参。

## FCOS 检测头校验和训练

先使用 `prepare_dataset.py` 按完整飞行场景生成 `data/dataset_<tag>/train.jsonl`、`val.jsonl` 和 `test.jsonl`，再运行检测头训练。验证集和测试集必须是人工复核标签；YOLO 结果只能作为训练集预标注候选。

```bash
python scripts/yolo_auto_20260920/fcos_grounding.py train --base /home/ubuntu/model/FM9G4B-V --data-root data --train data/dataset_<tag>/train.jsonl --val data/dataset_<tag>/val.jsonl --output checkpoints/fcos-<tag> --epochs 8 --learning-rate 0.0001 --image-size 448 --score-threshold 0.43 --seed 42
```

输出目录必须不存在。训练使用九格视觉 patch 特征和 FCOS 分类/回归头，冻结视觉编码器，逐图 batch=1、AdamW 和梯度裁剪。每轮保存 `epoch-XX/head.safetensors` 和 `val-metrics.json`，`best.json` 记录验证 F1 最佳 head，`training.jsonl` 保存训练和验证指标。当前没有旧 head 或优化器状态续训功能；不要添加旧 LoRA 参数 `--rank`、`--alpha`、`--dropout`、`--accumulation`、`--slices`。

完成标志是日志出现 `DETECTOR_TRAIN_FINISHED`。若需后台运行，使用 `nohup` 并将输出重定向到新日志；服务器关机仍会中断训练。

验证和测试：

```bash
HEAD=$(python -c 'import json; print(json.load(open("checkpoints/fcos-<tag>/best.json"))["head"])')
python scripts/yolo_auto_20260920/fcos_grounding.py eval --base /home/ubuntu/model/FM9G4B-V --data-root data --annotations data/dataset_<tag>/val.jsonl --head "$HEAD" --output eval/fcos-<tag>-val --image-size 448 --score-threshold 0.43
python scripts/yolo_auto_20260920/fcos_grounding.py eval --base /home/ubuntu/model/FM9G4B-V --data-root data --annotations data/dataset_<tag>/test.jsonl --head "$HEAD" --output eval/fcos-<tag>-test --image-size 448 --score-threshold 0.43
```

评估输出目录必须是新目录。记录整体及每类 Precision、Recall、F1、IoU、中心误差、空图误报、平均延迟和 p95 延迟。阈值只能根据验证集预测选择，再用测试集做一次最终报告：

```bash
python scripts/yolo_auto_20260920/select_fcos_threshold.py --predictions eval/fcos-<tag>-val/predictions.jsonl
python scripts/yolo_auto_20260920/diagnose_fcos_errors.py --predictions eval/fcos-<tag>-val/predictions.jsonl --threshold 0.43
```
## 批量离线评估（2026-09-22）

新增 `evaluate_grounding.py`，在独立复核集逐图保存原始输出、解析状态、框和预热后的生成耗时。采用同类别、按 IoU 降序的一对一匹配；记录 Precision/Recall/F1@IoU0.5、匹配框 IoU/中心误差、空图误报、每类统计及类别混淆，不把这些值称为 mAP。标签和输出均在 0～1000 网格上计算；无效输出算无有效检测，真实目标计入漏检。匹配框的平均误差仅覆盖 TP，必须和 Recall 一同阅读。

以下在云端实验根目录运行，输出目录必须全新；不带 `--adapter` 时评估基座模型：

```bash
/home/ubuntu/.conda/envs/fm9g-train/bin/python scripts/yolo_auto_20260920/evaluate_grounding.py --data-root data --annotations data/v1/val.jsonl --output eval/example-new-val --adapter checkpoints/grounding-r16-v1/epoch-02 --slices 2 --max-new-tokens 192
```

`--resume` 仅用于配置一致的已有评估，跳过完整落盘的图片。`--recompute-metrics` 只用完整预测文件重算指标，不加载模型、不占 GPU；不会修复半写入的 JSON 行。格式统计区分裸 JSON、可去除的代码围栏、夹带其他文字和无效输出；指标允许恢复外层文本，不能将恢复器视作已验证的飞控解析器。

`render_grounding_comparison.py` 可在有 Pillow 的环境下生成原图上的人工框／基座预测／adapter 预测对照图。`--image-root` 指向 `data/images`，不是数据根目录。

## 环境和存储

新环境由 `demo` 克隆，运行环境没有改动。训练依赖版本见 `requirements.txt`，实际完整快照见云端 `logs/pip-freeze-training.txt`。克隆的 decord wheel 带 Python 3.6 平台标签，单图训练不需要该视频库，因此仅在训练环境移除。

脚本、环境、日志和本轮 smoke 结果保存在系统盘。自动审批拒绝了增加 `ubuntu` 对 `/root` 穿越权限的操作，该操作没有执行。随后成功采用专用子目录 bind mount：仅将 `/root/gpufree-data/fm9g4b_grounding` 映射到 `/home/ubuntu/fm9g-data`，`/root` 仍为 `0700`。实验中原本为空的 `data/`、`checkpoints/` 已指向数据盘，因此正式图片和 adapter 将使用约 49 GiB 数据盘。

该 bind mount 没有写入 fstab，重启实例后须先检查。`mountpoint` 未成功时不要直接向挂载点写入，避免写到系统盘：

```bash
mountpoint /home/ubuntu/fm9g-data
df -h /home/ubuntu/fm9g-data
```

仅在尚未挂载时由 root 或有 sudo 权限的用户恢复已建立的专用目录挂载：

```bash
sudo mount --bind /root/gpufree-data/fm9g4b_grounding /home/ubuntu/fm9g-data
```

数据盘内容不会被保存到实例镜像，释放实例前需要另外导出。

根据用户给出的赛事口径，微调权重、LoRA、视觉层微调和新增检测头可以用于九格方案。正式比赛仍需证明所有 `/objects` 感知结果来自九格及其明确的非 YOLO 跟踪/后处理链路；真实数据训练、目标识别精度和 ROS 仿真闭环不在合成图验证范围内。
