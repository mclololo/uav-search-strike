# UAV Search Strike 开发者指南

本文件是给后续开发代理和维护者的仓库级上下文。内容基于截至 2026-09-09 对一方源码、启动脚本、ROS 接口、当前联调参数及 `docs/` 全部 Markdown 文档的核对；第三方依赖只核对集成入口，不把 vendored 源码当作本项目业务实现。修改前先阅读本文件。

信息冲突时按以下优先级判断：

1. 赛事方最终书面规则和正式答复。
2. 当前分支的一方源码及实际启用的 launch/XML/YAML。
3. 运行时话题、rosbag、日志和现场实验结果。
4. `docs/` 中的方案、调参记录、历史说明和仓库 README。

规划文档描述的是目标方案，不自动代表已经实现；launch 中存在参数也不代表源码一定使用了它。涉及合规但赛事方尚未明确的事项必须标记为“待确认”，不能自行下定论。

## 1. 项目定位与主链路

这是一个 ROS1/catkin 无人机仿真工作区，面向 RflySim/CopterSim + PX4 + MAVROS 的搜索、识别、穿框、气球打击和返航任务。仓库是强依赖外部仿真、CUDA、ROS 和模型文件的联调工程，不是可脱离环境运行的独立算法库。

```text
RflySim/CopterSim
  -> sensor_pkg（图像、里程计、深度点云）
  -> object_det / ArUco-QR
  -> /objects、/aruco/detected_id
  -> mission_pkg 行为树
  -> Ego-Planner / MAVROS
  -> 无人机执行
```

主要目录职责：

- `src/mission_pkg`：行为树入口、MAVROS 控制封装、动作/感知/条件节点、目标跟踪和视觉伺服。
- `src/object_det`：Ultralytics/YOLOE 检测、HSV 颜色分类、跨帧颜色投票和检测可视化。
- `src/sensor_pkg`：RflySim 传感器接入、比赛场景、点云坐标变换和独立视觉伺服工具。
- `src/common_msgs`：`Obj`、`Objects`、`Aruco`、`DetEnable` 等项目消息。
- `src/ego-planner`：EGO 局部规划器及 `quadrotor_msgs/PositionCommand`。
- `src/behaviortree_cpp`：随工作区集成的 BehaviorTree.CPP 第三方源码。
- `src/object_det/third_party/ultralytics`：Ultralytics/YOLOE 第三方源码。除非确有依赖问题，不要在这里做业务重构。
- `云服务器上model文件夹里要改动的代码/`：需要同步到 GPUFree 云端模型目录的三份文件。

## 2. 环境、构建和启动

完整运行依赖 ROS1、catkin、PX4/MAVROS、RflySim SDK、Xterm、CUDA、conda `demo` 环境和外部模型权重。启动脚本写死了若干 Linux/云端路径：

```text
/home/ubuntu/model
/mnt/c/PX4PSP/RflySimAPIs/RflySimSDK
```

典型构建流程：

```bash
cd /home/ubuntu/catkin_ws
catkin_make
source devel/setup.bash
```

根目录 `build.sh` 本质上执行 `catkin_make`。当前机器若缺少 ROS/RflySim/模型权重，只能做静态检查；不得把未执行的仿真或硬件测试写成已验证。

`sim_env_start.sh` 的完整启动顺序大致是：

1. 清理旧 ROS/Python 进程并 source 工作区。
2. 启动 `roscore`。
3. 启动 `/home/ubuntu/model/mission_once.py`。
4. 启动 `object_det detection_color.launch`。
5. 启动 `sensor_pkg/main.py`。
6. 启动 `sensor_pkg/AgentContestMap.py` 比赛场景。
7. 启动 `sensor_pkg cloud_transform.launch`。
8. 启动 `mission_pkg aruco_detect.launch show_debug:=true`。
9. 启动 `ego_planner FS-J310_ego-planner.launch`。
10. 启动 `mission_pkg llm_mission.launch`。
11. 启动 setpoint monitor 和资源监控窗口。

`kill_ros_pid.sh` 会强杀所有匹配 `ros` 的进程以及所有 `python3`，可能误杀其他开发任务。调试时优先停止明确的 ROS 节点和进程，不要随意运行该脚本。

## 3. ROS 接口、消息和坐标

常用接口如下。话题名有历史差异时，以当前 launch/YAML/源码和运行时 `rostopic list` 为准。

| 方向 | 接口 | 用途 |
| --- | --- | --- |
| 输入 | `/rflysim/sensor1/img_rgb` | 前视 RGB；`object_det` 当前 YAML 默认输入 |
| 输入 | `/rflysim/sensor2/img_rgb` | 下视 RGB；ArUco launch 默认输入 |
| 输入 | `/rflysim/uav1/local/odom` | 里程计 |
| 输入 | `/rflysim/sensor0/Depth_Cloud` | 深度点云 |
| 感知输出 | `/objects` | `common_msgs/Objects` 检测结果 |
| 感知输出 | `/aruco/detected_id` | ArUco/二维码识别 ID |
| 感知输出 | `/aruco/debug_image` | ArUco 调试图像 |
| 感知输出 | `/qr_result` | QR fallback 结果 |
| 规划输入 | `/move_base_simple/goal` | Ego-Planner 目标点 |
| 规划输出 | `/planning/pos_cmd` | `quadrotor_msgs/PositionCommand` |
| 任务输入 | `/mission_command` | `std_msgs/String` 高层任务命令 |
| 任务控制 | `/mission_abort` | 中止当前任务 |
| 任务输出 | `/mission_result` | `SUCCESS`、`FAILURE`、`ABORTED` 或 `ERROR:*` |
| 飞控输出 | `/mavros/setpoint_raw/local` | MAVROS 位置/速度 setpoint |

`common_msgs/Obj.msg` 当前字段是 header、类别、bbox 左上/右下坐标、中心、置信度，没有 `track_id`。`Objects.msg` 包含 header、`source_id` 和 `Obj[] objects`。不要在调用方假设消息中已有跟踪 ID。

项目任务参数大体使用 ENU 世界坐标（x 向东、y 向北、z 向上），但 `mavros_cnt.cpp` 内存在 ENU/NED 转换和 body-frame 控制。修改位置、速度、yaw、相机外参或点云转换时必须成链检查坐标和符号，不能只凭变量名判断。

`MavRosConnect` 是单例，订阅 `/mavros/state` 和默认 odom，发布 `/mavros/setpoint_raw/local`，并以 50 Hz 重发缓存命令以满足 OFFBOARD 频率要求。第一次 odom 会记录 home position；越过配置的安全边界时会锁定当前位置悬停。

### 已知话题命名差异

比赛手册部分位置写 `/rflysim/sensor1/image_rgb`，当前 `src/object_det/config/detection_params.yaml` 使用 `/rflysim/sensor1/img_rgb`。ArUco 当前 launch 使用 sensor2。不要仅因旧文档不同就批量改名；先核对运行系统的发布话题，再统一 launch、YAML 和文档。

## 4. 行为树和任务命令

入口 `src/mission_pkg/src/bt_ros.cpp` 会扫描 `devel/lib` 下插件 `.so` 并注册到 BehaviorTree.CPP：

- `mode=static`：从 XML 文件加载固定行为树。
- `mode=llm`：订阅 `/mission_command`，解析 JSON `{"commands":[...]}` 或逐行纯文本，动态生成 XML 并执行。
- 任务执行有互斥锁；忙时新命令返回 `BUSY`。
- 同一命令完成后 5 秒内重复发送会返回 `DUPLICATE_IGNORED`。
- `/mission_abort` 可中止任务，结果发布到 `/mission_result`。

当前主启动脚本运行 `llm_mission.launch`，它显式设置 `mode=llm`，所以日常整套任务以 `bt_ros.cpp` 动态拼出的行为树和该 launch 参数为准。`full_competition.launch`、`hit_balloon.launch` 没有设置 `mode`，会使用默认 `static` 并加载手写 XML；这些 XML 是独立测试/历史配置，参数可能落后于 LLM 主链路。当前 `full_competition.xml` 默认是紫色气球/B 区流程。静态和动态打击流程都用 `ForceSuccess` 包住获取/打击序列，因此打击返回 FAILURE 后整套任务仍可继续执行返航和降落；不能只根据最终任务 SUCCESS 判断气球已经命中。

支持的高层命令片段：

```text
takeoff
cross_frame
detect_aruco
return_cross_frame
hit_balloon:red|blue|purple
land
```

在已 source 工作区且 `roscore` 运行时，可先发送一条纯文本任务：

```bash
rostopic pub /mission_command std_msgs/String "data: 'takeoff'" --once
rostopic echo /mission_result
rostopic pub /mission_abort std_msgs/String "data: 'abort'" --once
```

也可以发布 JSON `{"commands":["takeoff","cross_frame","land"]}`；shell 引号需按当前 shell 正确转义。

关键节点职责：

- `Takeoff`：位置控制，必要时自动切 `OFFBOARD` 并解锁。
- `GoTo`：世界坐标或机体相对坐标移动，支持 yaw。
- `PlanningControl`：发布规划目标并订阅 `/planning/pos_cmd`。返航可用任务开始时记录的 home XY，并拒绝旧轨迹、错误目标和跳点过大的新轨迹。
- `Land`：默认切换 `AUTO.LAND`，也支持速度下降。
- `TargetLocator`：由 bbox、相机内外参和姿态估计三维世界坐标，滑动平均后写入黑板。
- `TargetTracker`：8 维恒速 Kalman 像素滤波 `[cx,vcx,cy,vcy,w,vw,h,vh]`，包含重捕获共识、关联门控、新息自适应和短时预测。
- `ScanForTarget`：旋转/侧移/后退的多阶段搜索，满足稳定条件后交给跟踪器。
- `VisualServo`：包含 `ALIGN`、`DESCENDING`、`APPROACHING`、`SPRINTING`、`BRAKING`、`ENTERING_WAIT`、`WAITING`、`SETTLING`、`LIFTING`、`RECOVERY` 等状态；结合 body-frame 速度/yaw rate、深度净空、边界保护、冲刺、制动和命中后抬升。

不要把 `src/sensor_pkg/visual_servo_balloon.py` 与行为树内的 `VisualServo` 混淆。前者是较旧的独立 `SEARCH/ALIGN/APPROACH/SPRINT/DONE` 调试工具。

### 当前 LLM 打击流程

`hit_balloon:<color>` 的当前实际链路如下：

1. 根据 `balloon_color_red/blue/purple` 映射 A/B/C 区，先用 `PlanningControl` 到接近点和 0.70 m 搜索点，再用 `GoTo` 确认搜索位置及走廊 yaw。未到位前不会启动 `ScanForTarget`。
2. `ScanForTarget` 代码支持旋转、侧移和后退。当前 launch 的 `yaw_rate=0.35 rad/s`，`lateral_velocity=0`、`backward_velocity=0`，因此主链路会在搜索点原地旋转扫描，但不会侧移或后退；`stable_frames=1`、`detection_hold_time=0`，检测一帧后即可快速交给跟踪/打击。
3. `TargetTracker` 与 `VisualServo` 并行运行。跟踪器用 8 状态 Kalman 滤波，并可在短时丢框时预测；当前跟踪器 lost grace 和冲刺丢框保持均为 1.5 s。
4. `VisualServo` 已拿到被 `ScanForTarget` 接受的目标。若高度高于 `strike_altitude + descent_tolerance`，先以 0.22 m/s 下降并做目标航向修正；到 0.40 m 附近后直接进入 `SPRINTING`。
5. 冲刺沿 body-frame 正 X 发送 0.85 m/s，走廊方向取区域配置 yaw，最大时间 6 s。当前 `sprint_lock_visual_correction=true`，冲刺阶段采用走廊航向保持，不用视觉 yaw 修正重新定义冲刺方向；高度控制仍会做有限垂直修正。
6. 冲刺阶段不再用目标丢失决定结束，无论检测框是否短暂消失，都会继续沿受保护的直线走廊推进。冲刺超时、为惯性预留的剩余制动距离不足或场地边界预测不安全仍有最终停止权；此阶段不读取 `VisualServo` 的深度点云，也不会由深度硬障碍触发命中前制动。
7. 冲刺结束先以 `MISS_TO_WAIT` 进入零速制动；制动完成、准备进入等待区时才开启 `VisualServo` 深度辅助。A/C 区按 `wait_entry_ratio` 低速前进一小段，B 区该比例为 0，会直接进入等待。等待区内结合深度接触、视觉框面积和近距离目标消失判断 HIT；没有形成接触证据且连续 0.5 s 看不到目标则按 ABORT 结束，再由外层任务衔接返航。
8. 命中后在制动阶段按区域配置的世界坐标方向离墙，停稳后抬升约 0.35 m，再把控制权交给后续返航命令。MISS/ABORT 返回 FAILURE，但外层 `ForceSuccess` 通常允许任务继续。

当前主参数：

| 项目 | 当前值 |
| --- | --- |
| 搜索高度 / 打击高度 | 0.70 m / 0.40 m |
| 下降速度 / 冲刺速度 | 0.22 m/s / 0.85 m/s |
| 冲刺最长时间 / 丢框保持 | 6.0 s / 1.5 s |
| 等待区无目标超时 | 0.5 s |
| 命中后抬升 | 约 0.35 m |
| EGO 最大速度 / 最大加速度 | 0.3 m/s / 0.5 m/s² |
| 搜索偏航速度 | 0.35 rad/s；侧移和后退搜索速度均为 0 |
| `forward_velocity` | 配置为 0.9 m/s；当前只被读取，未参与下降后直接冲刺的状态转移，也不是 EGO 巡航速度 |

区域参数以 `llm_mission.launch` 为准：A/红接近点为 `(5.3, 1.1, 1.0)`、搜索点为 `(5.3, 1.1, 0.70)`、yaw 约 π、`hit_offset_x=+35 px`、冲刺长度比例 0.86、等待区进入比例 0.35、离墙方向约 `-π/2`；B/紫接近点为 `(5.3, -2.60, 1.0)`、搜索点为 `(5.3, -2.60, 0.70)`、yaw 约 π、`hit_offset_x=-10 px`、冲刺比例 0.78、等待区进入比例 0、离墙方向约 `+π/2`，离墙距离 0.25 m、最长 1.50 s；C/蓝接近点为 `(5.4, 1.0, 1.0)`、搜索点为 `(5.4, 1.0, 0.70)`、yaw 约 `-π/2`、`hit_offset_x=-45 px`、冲刺比例 0.76、等待区进入比例 0.35、离墙方向约 π。这些坐标、yaw 和偏置是当前联调参数，不是由感知实时解算出的目标位置。

### 当前实现中的易误读点

- `targetInSprintZone()` 已实现深度/走廊/图像面积判断，但当前没有被状态转移调用。现版本并不是“只有气球位于冲刺区才允许冲刺”，而是检测已被接受且下降到打击高度后立即冲刺；`sprint_limit_ratio` 等参数主要限制无人机自身可冲刺距离和制动位置。
- `strike_stable_time`、冲刺前水平/垂直对齐容差、稳定帧数、冲刺高度/垂速容差和 `forward_velocity` 会被加载，但当前没有用于阻止 `DESCENDING/APPROACHING -> SPRINTING`。修改参数前先搜索成员变量的实际读写，不能假设调 launch 就会生效。
- LLM 主链路由 `llm_mission.launch` 传入 `sprint_lock_visual_correction=true`，冲刺时锁定区域走廊航向；当前四份 `full_competition*.xml` 静态配置仍写为 `false`。静态单项测试可能因此表现不同，排查时必须先确认 `bt_ros` 运行模式和实际加载的行为树。
- 深度相机在 `VisualServo` 中由 `depth_assist_active_` 门控：节点开始、下降、冲刺以及冲刺后的第一次制动阶段均不处理深度；只有该次制动以 `MISS_TO_WAIT` 结束、即将进入等待区时才开启。之后 `ENTERING_WAIT`/`WAITING` 使用深度判断硬障碍和目标接触，命中后的再次制动期间仍保持开启，节点完成或中止时关闭。传感器节点本身始终会发布共享点云，未物理停机；EGO-Planner 在飞往搜索点和返航时使用的公共点云链路不受这一局部门控影响。
- 冲刺阶段不再因丢框判定 HIT 或 MISS，而是持续到距离、时间或边界条件触发制动，再进入等待区统一收束。冲刺期间视觉框面积只锁存 `close_target_seen_`；等待区再结合该候选、深度接触、目标消失和无目标超时决定结果。必须结合视频、`/objects`、深度和 `/mission_debug` 判断真假命中。

## 5. 感知、传感器和规划

`object_det/scripts/det_color.py` 使用 YOLOE/Ultralytics 检测后做 HSV 颜色分类和 7 帧空间关联投票，输出 `red_balloon`、`blue_balloon`、`purple_balloon`，并可显示 OpenCV 检测窗口。当前 YAML 的原始检测类别是 `square_frame`、`red_balloon`，颜色分类会把含 `balloon` 的检测结果重新标为三色；`max_detections=2`、置信度阈值 0.5、FP16 开启、可视化窗口开启。YAML 当前把 `environment/is_simulation` 设为 `false`，尽管输入仍是 RflySim 的 `/rflysim/sensor1/img_rgb`；代码会据此选择不同通道转换。不要只按配置注释翻转该值，必须同时核对 ROS Image 的 `encoding`、原始像素通道和 HSV 结果。

当前仓库没有 `src/object_det/models/` 权重目录或可用权重，检测节点需要外部部署模型后才能运行。默认权重配置是 `$(find object_det)/models/best.pt`。`cv_bridge` 只作为兼容 fallback，不能把检测窗口或真实感知链路删掉。

ArUco 当前 `aruco_detect.launch` 实际启动 `src/mission_pkg/scripts/aruco_detect_node.py`，不是同目录的 C++ 节点。Python 节点支持 ArUco、多帧投票、CLAHE、OpenCV `QRCodeDetector` fallback、`/qr_result`、服务 `/aruco/enable`、`/aruco/detected_id` 和 `/aruco/debug_image`。部分旧文档只描述 ArUco，记录现状时应以这个 launch 和脚本为准。

`src/sensor_pkg/Config.json` 定义三个传感器：Seq 0 深度（640x640、FOV 80、UDP 9999、前向偏移约 `[0.2,0,0]`），Seq 1 前视 RGB（640x480、FOV 90、UDP 9998），Seq 2 下视 RGB（640x480、FOV 90、UDP 9996、pitch -90°）。`cloud_transform_node.py` 订阅深度点云和 odom，将 sensor/body 点云转换到 `world`，发布 `/cloud_registered`，供 Ego-Planner 避障。

`AgentContestMap.py` 创建比赛场景、立柱、方框、二维码、三个移动小车和三色气球；小车速度会随机变化，代码范围约为 0.05~1.0 m/s。不要用静态目标假设替代运行时感知。

Ego-Planner 启动文件是 `src/ego-planner/src/planner/plan_manage/launch/FS-J310/FS-J310_ego-planner.launch`。其关键输入为 `/rflysim/uav1/local/odom`、`/cloud_registered` 和 `/move_base_simple/goal`，规划命令经 remap 输出 `/planning/pos_cmd`。返航控制依赖轨迹 ID、目标位置、时间戳和命令新鲜度检查，修改时要保持这些安全条件。

## 6. 九格模型和云端同步

比赛规则要求自然语言解析必须使用九格模型，禁止更换模型。`mission_once.py` 从本地 `FM9G4B-V` 加载 `AutoModel/AutoTokenizer`，使用 `trust_remote_code=True`、`bfloat16` 和 CUDA，向 `/mission_command` 发布 `std_msgs/String`。

`MISSION_USE_IMAGE_CONTEXT=1` 时才订阅 `/rflysim/sensor1/img_rgb`，把图像缩放为 224x224 作为多模态输入；默认值为 `0`，默认解析仍是纯文本。模型输出只接受 fenced code，再由 `normalize_commands()` 根据用户原文做确定性修正。系统提示词要求只执行用户明确要求的动作，不能自动补全整套比赛流程。

CLI 传入任务后当前仍会进入交互式循环，这是现状；修改一次性调用时要明确是否需要退出，不能假设进程会自行结束。

云端路径（见 `docs/云服务器代码结构.md`）：

```text
/home/ubuntu/model/mission_once.py
/home/ubuntu/model/prompts/system_prompt.txt
/home/ubuntu/model/prompts/mission_prompt.txt
```

本地只同步以下三个被改动文件：

```text
云服务器上model文件夹里要改动的代码/mission_once.py
云服务器上model文件夹里要改动的代码/system_prompt.txt
云服务器上model文件夹里要改动的代码/mission_prompt.txt
```

不要覆盖云端已有的 `FM9G4B-V/`、`basic_cn.txt`、`test_rflysim.py` 等额外文件。当前可连接的新实例 SSH 端点为 `root@120.209.70.195:30532`；旧文档中的 `30287` 是上一实例端口，使用前仍应以用户给出的最新实例信息和实际连通性为准。ROS/RflySim 运行用户是 `ubuntu`，同步用户是 `root`。同步后需要 `chown -R ubuntu:ubuntu`，并在工作区代码变化后重新 `catkin_make`。凭据不能写入仓库。

截至 2026-09-09，`/home/ubuntu` 顶层累计的 19 个 `*.bak*`、`sync_backup_*` 和 `*invalid*` 历史备份已经清理，当前活动目录 `/home/ubuntu/catkin_ws`、`/home/ubuntu/model`、`/home/ubuntu/PX4PSP` 保持不变。不要假设云端仍有这些旧副本可供回滚；后续覆盖前只保留一个明确命名、位于工作区 `src/` 之外的时间戳备份，确认新版本可用后及时轮换，避免再次无限累积。

### 云端与本地标准同步流程

以下流程默认从本地仓库根目录执行，命令使用 Linux shell/WSL/Git Bash 语法。Windows PowerShell 也可以直接执行 `ssh`/`scp`，但变量和路径写法要按 PowerShell 调整。当前只允许同步明确列出的文件和目录，不使用整个 `model` 或 `src` 目录覆盖，也不使用带 `--delete` 的同步命令。

#### A. 同步前检查（双向都必须做）

1. 确认目标主机、端口和用户：当前实例为 `ssh -p 30532 root@120.209.70.195`。
2. 在本地执行 `git status --short`，确认即将发送或覆盖的文件没有未保存的意外修改。
3. 检查本地三份模型文件是否存在，并检查云端目标目录：

   ```bash
   test -f "云服务器上model文件夹里要改动的代码/mission_once.py"
   test -f "云服务器上model文件夹里要改动的代码/system_prompt.txt"
   test -f "云服务器上model文件夹里要改动的代码/mission_prompt.txt"
   ssh -p 30532 root@120.209.70.195 "ls -l /home/ubuntu/model/mission_once.py /home/ubuntu/model/prompts"
   ```

4. 对覆盖前的远端文件记录 `sha256sum`；若云端已有未合并修改，先下载、比较并取得确认，不能静默覆盖。

#### B. 本地 -> 云端：模型文件

只上传三份被修改文件，保持云端 `FM9G4B-V/`、`basic_cn.txt`、`test_rflysim.py` 等文件不变：

```bash
scp -P 30532 "云服务器上model文件夹里要改动的代码/mission_once.py" \
  root@120.209.70.195:/home/ubuntu/model/mission_once.py
scp -P 30532 "云服务器上model文件夹里要改动的代码/system_prompt.txt" \
  root@120.209.70.195:/home/ubuntu/model/prompts/system_prompt.txt
scp -P 30532 "云服务器上model文件夹里要改动的代码/mission_prompt.txt" \
  root@120.209.70.195:/home/ubuntu/model/prompts/mission_prompt.txt
```

上传完成后修复运行用户权限并核对远端哈希：

```bash
ssh -p 30532 root@120.209.70.195 \
  "chown ubuntu:ubuntu /home/ubuntu/model/mission_once.py \
   /home/ubuntu/model/prompts/system_prompt.txt \
   /home/ubuntu/model/prompts/mission_prompt.txt && \
   sha256sum /home/ubuntu/model/mission_once.py \
   /home/ubuntu/model/prompts/system_prompt.txt \
   /home/ubuntu/model/prompts/mission_prompt.txt"
```

将远端哈希与本地 `sha256sum`（PowerShell 可用 `Get-FileHash -Algorithm SHA256`）逐一比较。模型脚本或 prompt 更新后，重启正在运行的 `mission_once.py` 节点，避免旧进程继续使用旧内容；不要使用 `killall -9 python3` 代替针对性重启。

#### C. 云端 -> 本地：模型文件

需要以云端版本为准时，先只读检查并确认不会覆盖本地未提交修改，然后逐文件下载：

```bash
scp -P 30532 root@120.209.70.195:/home/ubuntu/model/mission_once.py \
  "云服务器上model文件夹里要改动的代码/mission_once.py"
scp -P 30532 root@120.209.70.195:/home/ubuntu/model/prompts/system_prompt.txt \
  "云服务器上model文件夹里要改动的代码/system_prompt.txt"
scp -P 30532 root@120.209.70.195:/home/ubuntu/model/prompts/mission_prompt.txt \
  "云服务器上model文件夹里要改动的代码/mission_prompt.txt"
```

云端历史盘点中曾有拼写错误的 `system_promt.txt`（少一个 `p`）。标准流程只读写规范名称 `system_prompt.txt`，不要把拼写错误文件重新传播；`basic_cn.txt`、测试脚本和模型权重也不要下载覆盖本地同步目录。

#### D. 本地 -> 云端：ROS 源码

当前云端差异主要位于 `src/mission_pkg` 和 `src/object_det`。从本地仓库根目录用管道打包，覆盖同名文件但不删除云端额外文件：

```bash
tar -czf - src/mission_pkg src/object_det | \
  ssh -p 30532 root@120.209.70.195 \
  "tar -xzf - -C /home/ubuntu/catkin_ws"
ssh -p 30532 root@120.209.70.195 \
  "chown -R ubuntu:ubuntu /home/ubuntu/catkin_ws/src/mission_pkg \
   /home/ubuntu/catkin_ws/src/object_det"
ssh -p 30532 root@120.209.70.195 \
  "cd /home/ubuntu/catkin_ws && catkin_make"
```

如果 `common_msgs`、`sensor_pkg` 或其他包也发生变更，先单独审查依赖，再把明确目录追加到 `tar` 列表；不要因为一次同步而打包整个 `src`。编译成功后再重启受影响的 ROS 节点，并用 `rostopic` 检查消息和话题版本。

#### E. 云端 -> 本地：ROS 源码

反向拉取前先确认本地工作树已提交或已备份，因为解包会覆盖本地同名文件：

```bash
ssh -p 30532 root@120.209.70.195 \
  "cd /home/ubuntu/catkin_ws && tar -czf - src/mission_pkg src/object_det" | \
  tar -xzf - -C .
git status --short
git diff -- src/mission_pkg src/object_det
```

确认差异后按本地开发流程重新构建。此流程不删除本地云端不存在的文件，也不会拉取 `/home/ubuntu/model/FM9G4B-V` 等大文件。

#### F. 同步后的统一验收

1. 模型文件：比较三份文件的 SHA-256，并确认远端所有者为 `ubuntu`。
2. ROS 源码：确认远端 `catkin_make` 成功，`devel/lib` 中的行为树插件已更新。
3. 运行服务：以 `ubuntu` 用户启动 ROS/RflySim；检查 `/mission_command`、`/mission_result`、`/objects` 和 `/planning/pos_cmd`。
4. 发现同步内容不对时，停止受影响节点、保留日志和哈希，先回滚到已确认版本；不要用全目录覆盖或强制删除来“重新同步”。

## 7. 比赛合规约束

`docs/比赛规则.md` 和赛事方正式答复优先于工程便利性：

- 场地约 6x5x3 m，限时 10 分钟。任务包括起飞、穿框、二维码识别、逆序穿框、指定颜色气球、返航降落和路径规划创新，另有多模态模型与时间排名加分。
- 必须使用九格模型解析自然语言，禁止更换九格大模型。
- 禁止遥控、比赛中触碰无人机、修改仿真速度或瞬移。
- 感知结果必须可视化并真实参与控制，不能用固定轨迹复现代替现场感知决策。
- 禁止新建一个复合控制接口，一键完成多个比赛阶段。
- 碰撞会扣分；同一阶段碰撞多个物体可能直接导致该阶段 0 分。
- 文档、源码和现场行为不一致时可能被审查并扣分。

2026-09-10 用户补充的赛事口径：LoRA、视觉层微调和新增检测头均属于允许的“使用九格模型”方式；启用九格时，任务全程禁用 YOLO，包括搜索、穿框、气球识别、打击、返航及异常恢复，不能切换到 YOLO 兜底。不得把仅重命名或包装后的 YOLO 当作九格检测头。后续修改 launch、启动脚本和感知节点时按此口径执行。

以上是目标方案的约束，不代表当前系统已完成替换：`sim_env_start.sh` 仍启动 `object_det detection_color.launch`。九格实验目前只通过合成图 LoRA 训练链路验证，尚无经过真实场景验证的替代检测节点，现有整套启动方式不能作为符合全程禁用 YOLO 约束的比赛配置。采集工具本身只订阅原始图像，标注方案采用人工画框；本次口径不用于推定离线 YOLO 预标注、其他辅助算法或区域先验的赛事许可范围。

当前已经明确的官方口径是：“不能使用固定的轨迹去跑，比如跑了一次最好的成绩，把最好成绩的轨迹录制下来，后面都按照这个轨迹去跑不符合比赛要求。”因此，录制最优飞行并开环复现、按时间/轨迹点盲飞，是明确禁止项。

当前 XML/launch 中仍有 A/B/C 搜索坐标、区域 yaw、走廊长度和桨叶像素偏置。赛事方尚未最终确认这类“区域先验/标定参数”是否等同于被禁止的预设关键点，所以本文件不把当前实现直接判定为违规，也不能宣称已经合规。它们必须作为合规审查项持续保留，并满足以下可辩护边界：

- 参数只作为到达大致任务区域、相机/桨叶标定和安全边界先验，不能是一整段录制轨迹。
- 目标出现、颜色、跟踪、冲刺时机、制动、障碍物和返航轨迹必须由比赛时的真实感知、里程计、深度点云和规划器实时闭环决定；启用九格时，目标检测不得依赖 YOLO，执行上述全程禁用要求。
- 现场应能通过检测窗口、ROS 话题、日志或 rosbag 证明感知结果实际改变了控制决策。
- 官方一旦给出最终书面解释，立即以正式解释为准，复审 `llm_mission.launch`、`full_competition*.xml`、`bt_ros.cpp` 及所有区域参数。

EGO launch 中列有五个示例全局 waypoint，但当前 `flight_type=1` 使用 `/move_base_simple/goal` 的单目标模式，这些 waypoint 不在当前主链路执行。不要把它们误报成正在复现的比赛轨迹，也不要在未确认模式时删除或启用。

## 8. 规划文档与当前代码的差异

`docs/attack_mission_plan.md` 建议的动态气球流程是：

```text
SEARCH -> CONFIRM_TARGET -> ESTIMATE_MOTION -> PLAN_ATTACK_CORRIDOR
-> INTERCEPT -> MATCH_VELOCITY -> CONTACT -> BRAKE -> EGRESS -> VERIFY_HIT
```

当前源码已有扫描节点、像素 Kalman 跟踪、深度净空、参数化走廊冲刺、制动和恢复状态；当前扫描偏航速度配置为 0.35 rad/s，但侧移和后退速度仍为 0。以下内容尚未完整实现，不能在文档或提交说明中宣称已完成：

- 目标三维速度估计、`track_id`、沿车速匹配和 next-best-view。
- 完整动态攻击走廊。
- 严格的多信号命中确认。
- 由实时场景理解生成搜索位置、走廊航向和命中偏置。

当前冲刺不以目标丢失提前结束，而是在等待区统一收束命中或无目标超时；等待区仍保留接触候选和近距消失判据。参数化直线冲刺和超时/距离制动仍是当前逻辑，后续结论必须以源码、有效参数和实验日志为准。

`docs/感知与大模型模块优化指导.md`、`docs/九格大模型调研报告.md`、`docs/visual_servo_pd_tuning.md` 中的优化建议和调参记录也属于规划/经验资料，不等于已经合入主链路。

## 9. 修改和排查规程

遇到任务不执行、检测不到或规划异常时，按链路逐层检查：

1. `roscore`、RflySim/CopterSim 和 `sensor_pkg/main.py` 是否运行。
2. 用 `rostopic list/echo` 确认图像、odom、点云、`/objects` 和 ArUco 输出，重点检查 `img_rgb` 与 `image_rgb` 命名。
3. 确认检测窗口可见、模型权重存在、`common_msgs` 消息已编译。
4. 确认 `/cloud_registered` 和 Ego-Planner 的 `/planning/pos_cmd` 有新鲜数据。
5. 确认 `bt_ros` 插件已生成到 `devel/lib`，且 `/mission_command` 是合法 JSON/文本格式。
6. 最后检查 MAVROS 状态、OFFBOARD/解锁、ENU/NED 转换、body-frame yaw 和安全边界。

修改原则：

- 优先复用现有 ROS、黑板、行为树和参数接口；不要为一个阶段新增“一键完成全流程”的控制 API。
- 改动消息字段、话题、坐标或 launch 参数时，同时更新所有发布者、订阅者和文档，并检查实际运行名称。
- 保留感知可视化和真实控制输入；明确禁止录制轨迹复现或按时间开环盲飞。区域先验是否允许尚待官方确认，不要擅自扩大或缩小规则口径。
- 第三方目录只做必要兼容修改，业务逻辑放在项目自己的包中。
- 静态改动后运行对应语言的语法/格式检查；涉及 ROS 接口时重新 `catkin_make`。
- 没有完整仿真环境时，在交付说明中明确未运行 ROS 构建/仿真，而不是推断成功。

## 10. 文档索引

`docs/` 是项目文档集中位置，重点文件：

- `docs/2026无人系统具身智能算法挑战赛--无人机搜索打击场景使用手册.md`：场景、部署和使用手册。
- `docs/比赛规则.md`：计分、禁用行为和审查约束。
- `docs/云服务器代码结构.md`：云端目录、同步和权限。
- `docs/attack_mission_plan.md`：动态目标打击方案。
- `docs/感知与大模型模块优化指导.md`：感知/LLM 优化建议。
- `docs/九格大模型调研报告.md`：九格模型调研。
- `docs/visual_servo_pd_tuning.md`：视觉伺服调参记录。
- 根目录及 `docs/GPUFree文件同步说明.md`：文件同步说明；两份内容若冲突，先核对云端现状，不要直接覆盖。
- `docs/自然语言命令测试.md`：自然语言命令测试记录。
- `docs/README.md`：文档索引和归档约定。

文档和实现冲突时，先记录冲突，再按本文件开头的信息优先级判断；涉及比赛合规时，以赛事方最终书面规则和正式答复为准，未明确事项保留为待确认项。
