# UAV Search Strike 开发者指南

本文件是给后续开发代理和维护者的仓库级上下文。基础内容来自截至 2026-09-09 对一方源码、启动脚本、ROS 接口及 `docs/` 的核对；九格感知与部署整理进展更新至 2026-10-09。第三方依赖只核对集成入口，不把 vendored 源码当作本项目业务实现。修改前先阅读本文件。

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
  -> 九格视觉编码器 + FCOS 或 object_det / ArUco-QR
  -> /objects、/aruco/detected_id
  -> mission_pkg 行为树
  -> Ego-Planner / MAVROS
  -> 无人机执行
```

主要目录职责：

- `src/mission_pkg`：行为树入口、MAVROS 控制封装、动作/感知/条件节点、目标跟踪和视觉伺服。
- `src/object_det`：Ultralytics/YOLOE 检测、HSV 颜色分类、跨帧颜色投票和检测可视化。
- `tools/fm9g_grounding`：九格图像采集、标注、数据划分、LoRA/FCOS 检测头训练与评估，以及九格 ROS 感知节点；`deployment/` 中的版本化 FCOS 头通过 Git LFS 分发。
- `tools/package_cloud_workspace.py`：从 Git 提交导出云端部署包，嵌入经过哈希验证的 LFS 检测头，保留 catkin 符号链接并补齐 ROS 脚本执行权限。
- `src/sensor_pkg`：RflySim 传感器接入、比赛场景、点云坐标变换和独立视觉伺服工具。
- `src/common_msgs`：`Obj`、`Objects`、`Aruco`、`DetEnable` 等项目消息。
- `src/ego-planner`：EGO 局部规划器及 `quadrotor_msgs/PositionCommand`。
- `src/behaviortree_cpp`：随工作区集成的 BehaviorTree.CPP 第三方源码。
- `src/object_det/third_party/ultralytics`：Ultralytics/YOLOE 第三方源码。除非确有依赖问题，不要在这里做业务重构。
- `云服务器上model文件夹里要改动的代码/`：任务解析脚本和两份提示词；当前启动脚本直接读取仓库版本，旧 `/home/ubuntu/model` 映射仅用于兼容手动部署。

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
3. 启动当前工作区 `云服务器上model文件夹里要改动的代码/mission_once.py`，使用同目录 `mission_prompt.txt` 和服务器预装基础模型。
4. 按 `PERCEPTION_BACKEND` 启动九格检测节点或 `object_det detection_color.launch`；未指定时默认 YOLO。
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
| 输入 | `/rflysim/sensor1/img_rgb` | 前视 RGB；YOLO 配置和九格检测节点均默认订阅 |
| 输入 | `/rflysim/sensor2/img_rgb` | 下视 RGB；ArUco launch 默认输入 |
| 输入 | `/rflysim/uav1/local/odom` | 里程计 |
| 输入 | `/rflysim/sensor0/Depth_Cloud` | 深度点云 |
| 感知输出 | `/objects` | `common_msgs/Objects` 检测结果；发布者应与本轮感知后端一致 |
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
- `PlanningControl`：发布规划目标并订阅 `/planning/pos_cmd`。返航使用连接器第一次收到 odom 时记录的 home XY，并拒绝旧轨迹、错误目标和跳点过大的新轨迹；当前不会在每轮任务起飞时重新记录 home。
- `Land`：默认切换 `AUTO.LAND`，也支持速度下降。
- `TargetLocator`：由 bbox、相机内外参和姿态估计三维世界坐标，滑动平均后写入黑板。
- `TargetTracker`：8 维恒速 Kalman 像素滤波 `[cx,vcx,cy,vcy,w,vw,h,vh]`，包含重捕获共识、关联门控、新息自适应和短时预测。
- `ScanForTarget`：旋转/侧移/后退的多阶段搜索，满足稳定条件后交给跟踪器。
- `VisualServo`：包含 `ALIGN`、`DESCENDING`、`APPROACHING`、`SPRINTING`、`BRAKING`、`ENTERING_WAIT`、`WAITING`、`SETTLING`、`LIFTING`、`RECOVERY` 等状态；结合 body-frame 速度/yaw rate、深度净空、边界保护、冲刺、制动和命中后抬升。

不要把 `src/sensor_pkg/visual_servo_balloon.py` 与行为树内的 `VisualServo` 混淆。前者是较旧的独立 `SEARCH/ALIGN/APPROACH/SPRINT/DONE` 调试工具。

2026-09-10 返航降落调整：LLM 的 `land`、四份 `full_competition*.xml` 及 `hit_balloon.xml` 依次执行 `PlanningControl(home XY, z=1.4, final_yaw=0, final_yaw_rate=0.25)`、`Land(AUTO.LAND)`。返航末段不再使用 GoTo，规划到位后先零速制动，低速持续 0.30 s 才允许原地调航向；位置仍须处于到达容差内。转向时只发零平移速度与限幅 yaw_rate，不直接吸附旧位置；漂移出到达范围时暂停转向、制动并请求新的 EGO 回位轨迹。航向误差不超过 0.10 rad、速度不超过 0.12 m/s 且位置/高度合格持续 0.50 s 后，才进入 AUTO.LAND。规划指令失效时制动重试，不追赶旧轨迹。已移除中间规划下降至 0.5 m 的步骤和主 launch 的 `land/return_low_pos` 参数。最后下降由飞控处理，不包含 EGO 避障规划；`Land` 以飞控撤防（armed=false）判断完成，不是解锁。

导航方式不能概括为全部使用路径规划：LLM 的跨区飞行、穿框/反向穿框和返航使用 `PlanningControl`，起飞使用 `Takeoff` 位置控制，二维码观测和打击搜索点到位确认仍使用带位置目标的 `GoTo`，原地转向使用只带 yaw 的 `GoTo`。静态 XML 还存在较长的 `GoTo` 位移，须按实际启动模式核对。

重复任务排查：每条 LLM 命令重新创建行为树及黑板，跟踪器和打击状态会重新初始化；区域搜索点为固定世界坐标，不随本轮落地点平移。2026-09-10 的连续红球/蓝球日志显示第二轮落地点约 `(0.09,0.09,0)`，仍到达蓝区搜索点约 `(5.40,0.99,0.73)`，但进入等待区约为 `(5.26,-2.57,0.40)`。原 body-X 冲刺只有航向反馈；同日修复将锁定走廊模式改为世界坐标速度，并对冲刺起点定义的平行走廊做限幅横向位置纠偏。`TargetTracker` 预测结果仍设置 `is_detected=true`，可以参与近距面积锁存，等待区的“目标消失”判据不能证明物理命中。详见 `docs/repeated_mission_diagnosis_20260910.md`；不能把代码修复或单元测试当成无碰撞、真命中的飞行验证。

2026-09-10 制动测速修复：当前云端 RflySim SDK 的 Odometry 只填充 pose，twist 全零。`MavRosConnect::getMeasuredSpeed()` 取至少 0.08 s 位移测速与已有 twist 模长的较大值，用于 VisualServo 制动/收束和返航 PlanningControl 停稳确认。无有效样本、时间倒退或样本过期时不允许判定停稳；原始 odom、SDK 和普通穿框控制不变。制动仍使用零速指令，不是根据虚假零速立即交给等待/抬升/返航。

上述修复已同步至 30532 实例，云端 catkin 编译和 7 项 C++ 回归测试通过，本地 7 项 Python 发布/中断测试通过。部署时整套 ROS 已停止，未自动启动或飞行，尚需仿真复测。备份为 `/home/ubuntu/sync_backup_strike_20260910_212221`；详见 `docs/strike_collision_and_command_fix_20260910.md`。

### 当前 LLM 打击流程

`hit_balloon:<color>` 的当前实际链路如下：

1. 根据 `balloon_color_red/blue/purple` 映射 A/B/C 区，先用 `PlanningControl` 到接近点和 0.70 m 搜索点，再用 `GoTo` 确认搜索位置及走廊 yaw。未到位前不会启动 `ScanForTarget`。
2. `ScanForTarget` 代码支持旋转、侧移和后退。当前 launch 的 `yaw_rate=0.35 rad/s`，`lateral_velocity=0`、`backward_velocity=0`，因此主链路会在搜索点原地旋转扫描，但不会侧移或后退；`stable_frames=1`、`detection_hold_time=0`，检测一帧后即可快速交给跟踪/打击。
3. `TargetTracker` 与 `VisualServo` 并行运行。跟踪器用 8 状态 Kalman 滤波，并可在短时丢框时预测；当前跟踪器 lost grace 和冲刺丢框保持均为 1.5 s。
4. `VisualServo` 已拿到被 `ScanForTarget` 接受的目标。若高度高于 `strike_altitude + descent_tolerance`，先以 0.22 m/s 下降并做目标航向修正；在搜索点 XY 和打击高度到位后，还须有未超时的跟踪目标持续满足 `strike_stable_time`，才进入 `SPRINTING`。
5. 当前主链路 `sprint_lock_visual_correction=true` 时，沿区域配置 yaw 的世界坐标切向发送 0.85 m/s，最大时间 6 s；横向参考线穿过实际冲刺起点，纠偏增益 1.2/s、速度上限 0.16 m/s，不使用视觉 yaw 改写走廊方向。速度经与规划控制相同的 `(-world_y, world_x, world_z)` 映射发送 MAVROS LOCAL_NED；离墙速度也使用此映射。机头继续保持走廊航向，高度继续有限修正。配置为 false 时仍使用原 body-X/视觉修正分支；世界速度闭环不等于能够保证实际轨迹绝无误差。
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

- `targetInSprintZone()` 已实现深度/走廊/图像面积判断，但当前没有被状态转移调用。现版本不是“只有气球位于冲刺区才允许冲刺”：搜索点和打击高度到位、跟踪目标持续有效后即可进入冲刺；`sprint_limit_ratio` 等参数主要限制无人机自身可冲刺距离和制动位置。
- `readyForSprint()` 现使用搜索点 XY、打击高度、有效目标时间戳和 `strike_stable_time` 门控冲刺；`targetInSprintZone()` 和冲刺前水平/垂直对齐容差、稳定帧数、垂速容差及 `forward_velocity` 仍未参与该状态转移。修改参数前先搜索成员变量的实际读写，不能假设调 launch 就会生效。
- LLM 主链路由 `llm_mission.launch` 传入 `sprint_lock_visual_correction=true`，冲刺时锁定区域走廊航向；当前四份 `full_competition*.xml` 静态配置仍写为 `false`。静态单项测试可能因此表现不同，排查时必须先确认 `bt_ros` 运行模式和实际加载的行为树。
- 深度相机在 `VisualServo` 中由 `depth_assist_active_` 门控：节点开始、下降、冲刺以及冲刺后的第一次制动阶段均不处理深度；只有该次制动以 `MISS_TO_WAIT` 结束、即将进入等待区时才开启。之后 `ENTERING_WAIT`/`WAITING` 使用深度判断硬障碍和目标接触，命中后的再次制动期间仍保持开启，节点完成或中止时关闭。传感器节点本身始终会发布共享点云，未物理停机；EGO-Planner 在飞往搜索点和返航时使用的公共点云链路不受这一局部门控影响。
- 冲刺阶段不再因丢框判定 HIT 或 MISS，而是持续到距离、时间或边界条件触发制动，再进入等待区统一收束。冲刺期间视觉框面积只锁存 `close_target_seen_`；等待区再结合该候选、深度接触、目标消失和无目标超时决定结果。必须结合视频、`/objects`、深度和 `/mission_debug` 判断真假命中。

## 5. 感知、传感器和规划

YOLO 模式下，`object_det/scripts/det_color.py` 使用 YOLOE/Ultralytics 检测后做 HSV 颜色分类和 7 帧空间关联投票，输出 `red_balloon`、`blue_balloon`、`purple_balloon`，并可显示 OpenCV 检测窗口。当前 YAML 的原始检测类别是 `square_frame`、`red_balloon`，颜色分类会把含 `balloon` 的检测结果重新标为三色；`max_detections=2`、置信度阈值 0.5、FP16 开启、可视化窗口开启。YAML 当前把 `environment/is_simulation` 设为 `false`，尽管输入仍是 RflySim 的 `/rflysim/sensor1/img_rgb`；代码会据此选择不同通道转换。不要只按配置注释翻转该值，必须同时核对 ROS Image 的 `encoding`、原始像素通道和 HSV 结果。

当前仓库没有 `src/object_det/models/` 权重目录或可用权重，检测节点需要外部部署模型后才能运行。默认权重配置是 `$(find object_det)/models/best.pt`。`cv_bridge` 只作为兼容 fallback，不能把检测窗口或真实感知链路删掉。

九格模式下，云端 `ros_fcos_node.py` 使用九格 `FM9G4B-V` 的冻结视觉编码器提取空间 patch 特征，接训练得到的 `torchvision` FCOS 分类/框回归头；推理时不运行九格语言生成器，也不调用 YOLO。该节点从前视原图直接输出 `square_frame`、`red_balloon`、`blue_balloon`、`purple_balloon` 四类及原图像素 bbox，默认分数阈值 0.43、最多 8 框，连同原图时间戳发布到 `/objects`，并显示 `FM9G Detection` 框图。类别由检测头直接给出，不走 YOLO 模式的 HSV 投票；无目标时也发布空 `Objects`。`source_id=1` 表示前视相机，不能据此区分检测模型。ROS 节点启动时会检查 `/objects` 是否已有发布者；切换后端应结束本轮任务并重启环境，再核对唯一发布者。

ArUco 当前 `aruco_detect.launch` 实际启动 `src/mission_pkg/scripts/aruco_detect_node.py`，不是同目录的 C++ 节点。Python 节点支持 ArUco、多帧投票、CLAHE、OpenCV `QRCodeDetector` fallback、`/qr_result`、服务 `/aruco/enable`、`/aruco/detected_id` 和 `/aruco/debug_image`。部分旧文档只描述 ArUco，记录现状时应以这个 launch 和脚本为准。

`src/sensor_pkg/Config.json` 定义三个传感器：Seq 0 深度（640x640、FOV 80、UDP 9999、前向偏移约 `[0.2,0,0]`），Seq 1 前视 RGB（640x480、FOV 90、UDP 9998），Seq 2 下视 RGB（640x480、FOV 90、UDP 9996、pitch -90°）。`cloud_transform_node.py` 订阅深度点云和 odom，将 sensor/body 点云转换到 `world`，发布 `/cloud_registered`，供 Ego-Planner 避障。

`AgentContestMap.py` 创建比赛场景、立柱、方框、二维码、三个移动小车和三色气球；小车速度会随机变化，代码范围约为 0.05~1.0 m/s。不要用静态目标假设替代运行时感知。

Ego-Planner 启动文件是 `src/ego-planner/src/planner/plan_manage/launch/FS-J310/FS-J310_ego-planner.launch`。其关键输入为 `/rflysim/uav1/local/odom`、`/cloud_registered` 和 `/move_base_simple/goal`，规划命令经 remap 输出 `/planning/pos_cmd`。返航控制依赖轨迹 ID、目标位置、时间戳和命令新鲜度检查，修改时要保持这些安全条件。

## 6. 九格模型和云端同步

比赛规则要求自然语言解析必须使用九格模型，禁止更换模型。`mission_once.py` 从本地 `FM9G4B-V` 加载 `AutoModel/AutoTokenizer`，使用 `trust_remote_code=True`、`bfloat16` 和 CUDA，向 `/mission_command` 发布 `std_msgs/String`。

`MISSION_USE_IMAGE_CONTEXT=1` 时才订阅 `/rflysim/sensor1/img_rgb`，把图像缩放为 224x224 作为多模态输入；默认值为 `0`，默认解析仍是纯文本。模型输出只接受 fenced code，再由 `normalize_commands()` 根据用户原文做确定性修正。系统提示词要求只执行用户明确要求的动作，不能自动补全整套比赛流程。

CLI 传入任务后当前仍会进入交互式循环，这是现状；修改一次性调用时要明确是否需要退出，不能假设进程会自行结束。

`mission_once.py` 于 2026-09-10 改为 `/mission_command` 使用 `latch=False`，等待订阅连接后只发送一次，不在重启任务节点时自动重放旧命令。等待超时使用单调时钟；模型加载后、推理流期间及发布前检查 ROS shutdown，中断时退出并提示重新运行脚本，不把发布者自身关闭误报为任务节点缺失。当天日志已证实一次 SIGINT 后仍继续推理、最终误报“没有订阅者”的情况。旧 latch=True 发布进程必须退出后新脚本才生效；重启任务节点前仍需核对撤防、任务空闲和发布者状态。消息发布不等于任务已接收/执行成功，需检查 `/mission_result`。

### 2026-10-09 Git 发布版本与成员部署（优先于下方历史路径）

当前云端 `sim_env_start.sh` 仍引用实验目录的 `scripts/speed_head_20260923/ros_fcos_node.py` 和 `deployment/current`，并非仓库节点路径。只读比对云端和本地：27 份九格工具、三份任务解析文件和原启动脚本均逐字节一致，无需重复下载。云端当前 FCOS 头及 `deployment.json` 已保存到本地 `tools/fm9g_grounding/deployment/fcos-recovered-20261008/epoch-04/`，头 SHA-256 为 `7a7cd85a5152a4b354efd134e7126c7e8b0732dbe2fe609f805ebe7aad2fa882`，通过 `.gitattributes` 配置 Git LFS。

发布版 `sim_env_start.sh` 默认使用 `${WS_DIR}/tools/fm9g_grounding/ros_fcos_node.py` 和仓库中的上述检测头版本，保留 `FM9G_NODE`、`FM9G_HEAD`、`FM9G_PYTHON` 覆盖入口。语言入口也改为当前仓库的 `mission_once.py`，通过 `MISSION_PROMPT_PATH` 读取仓库提示词，通过 `MISSION_MODEL_BASE` 使用服务器预装的 `/home/ubuntu/model/FM9G4B-V`；可用 `LLM_NODE` 显式指定入口。`cloud_env.sh` 的 `TOOL` 与所 source 的 `devel/setup.bash` 按自身所在工作区解析，不再依赖仅当前服务器存在的实验源码目录。

组员从 `feature/perception` 克隆并执行 `git lfs pull`，用 `tools/package_cloud_workspace.py` 导出提交版本，上传后解包到全新的 `/home/ubuntu/uav-releases/<版本>`，再在该目录编译和启动。打包器仅导出 Git 文件；不能直接打包 Windows 工作树，因为 `src/CMakeLists.txt` 必须保留 Linux 符号链接。成员完整流程见 `docs/组员克隆与云端运行.md`。YOLO 模式在新工作区创建指向服务器原 `best.pt` 的链接，九格模式无需此权重。

服务器预装的 `/home/ubuntu/model/FM9G4B-V`、`src/object_det/models/best.pt` 不下载、不提交、不覆盖。已有工作区升级先计算哈希：相同文件跳过；新增文件进入不存在的新路径；有差异的代码先暂存并审查/合并，禁止整目录镜像覆盖或删除服务器独有文件。本地保留晚于云端的打击制动和 EGO 参数改动，不能用云端旧文件回退本地。具体盘点见 `docs/九格云端文件核对_20261009.md`，旧工作区差异合并见 `docs/GPUFree文件同步说明.md`。用户已授权提交当前完整版本；该授权不等于允许自动合并 `main`、覆盖活动云端工作区或启动飞行。

本次一并发布既有控制修复：SDK pose 位移测速、世界坐标走廊冲刺、固定停车点及速度/位置稳定门控、命中后有限回退、搜索点位置保持、方框定位一致性检查与入口/中心/出口分段规划、蓝球任务复用本轮穿框出口、返航旧轨迹拒绝及停稳调向、EGO 新目标/过期轨迹重建，以及返航地图高度约束/障碍短期记忆/就绪确认。主 launch 当前紫球像素偏置为 -20 px、蓝球为 -60 px，紫区关闭侧向离墙并沿已通过走廊回退最多 0.10 m；`strike_stable_time=0.8 s`、搜索超时 20 s。上文较早记录中的旧偏置和侧移量属于历史值，以源码/launch 为准。

发布前已在 `/home/ubuntu/uav-releases/precommit-20261009-v2` 从暂存区部署包完成全新 catkin 构建；任务包 C++ 测试通过（catkin 报告 18 tests、0 failures），7 项任务发布/中断测试及 24 项九格数据/评估/ROS 解码测试全部通过，无跳过。仓库真实 head 的 GPU 加载与原图尺寸空图推理、冻结视觉参数检查、仓库语言模型/提示词路径覆盖及启动前默认路径检查通过。部署包未检出凭据/个人配置，保留 catkin 符号链接和 ROS 执行权限；本地 24 份工具 Python 源码和 50 段操作手册 Bash 命令语法通过。本次未启动 ROS 任务环境、未发布飞行指令、未重新测量识别精度或完整控制延迟；活动 `/home/ubuntu/catkin_ws` 保持此前版本。

### 新实例恢复现状（2026-10-08，优先于下方旧实例记录）

旧实例意外重置，原数据盘和旧 FCOS 权重已丢失。当前实例入口为 `root@120.209.70.195:30624`，恢复以本地工作树为准，包含尚未提交的任务控制和 EGO 修复；不是某个 Git 提交的部署。凭据不得写入仓库。

- 同步范围为 `mission_pkg`、`object_det`、`common_msgs`、`sensor_pkg`，以及 EGO 的 `ego_replan_fsm.cpp`、`planner_manager.cpp` 两份本地改进文件；根启动/构建脚本、三份模型解析文件、九格工具单独同步。没有覆盖基础 `FM9G4B-V`、PX4 或其他第三方源码。同步前备份在 `/home/ubuntu/recovery_backup_20261008`。
- 恢复独立环境 `/home/ubuntu/.conda/envs/fm9g-train`，由模板 `/opt/conda/envs/demo` 克隆，PyTorch 2.3.1、torchvision 0.18.1、transformers 4.44.2、peft 0.13.2。原 `demo` 保留作为仿真环境。当前 RflySim SDK 为 `/home/ubuntu/PX4PSP/RflySimAPIs/RflySimSDK`，启动脚本支持该路径和旧 WSL 路径。
- 从本地原始图像与八份 reviewed JSONL 恢复 1359 张图片，重建 `data/dataset_20260923_111402`，训练/验证/测试仍为 1004/193/162 张。`data/`、`checkpoints/`、`eval/`、`logs/` 指向专用数据盘 bind mount；保持人工 SSH 恢复挂载，未增加自动挂载服务或 fstab 项。
- 九格完整工具在实验目录 `scripts/speed_head_20260923`；旧 `scripts/yolo_auto_20260920` 是兼容链接。采集/训练终端可 `source .../scripts/speed_head_20260923/cloud_env.sh`，检查 mountpoint、数据路径和权限后初始化 `PY`、`TOOL`、`DATA`、`ANNO` 并 source 工作区。未挂载时不能写入。
- 原快速 head 未有本地备份，因此重新训练 8 轮，检查点为 `checkpoints/fcos-recovered-20261008`，按验证 F1 选 `epoch-04`。分数阈值 0.43、IoU 0.5 下独立测试 Precision 89.90%、Recall 84.36%、F1 87.04%；红球 Recall 87.27%、蓝球 Recall 60.87%；平均离线耗时 27.96 ms、p95 30.84 ms。该模型是重建模型，不等于旧权重逐字节恢复；蓝球仍是后续补数据重点。
- 系统盘部署副本为 `deployment/fcos-recovered-20261008/epoch-04/head.safetensors`，SHA-256 为 `7a7cd85a5152a4b354efd134e7126c7e8b0732dbe2fe609f805ebe7aad2fa882`；`deployment/current` 链接到该目录，启动脚本 `FM9G_HEAD` 默认使用此入口。普通关机后在线检测不依赖数据盘挂载；重置/释放仍需本地备份。
- 已完成云端 catkin 编译，16 项 C++ 测试、24 项 Python 测试通过；GPU detector probe、隔离 ROS 原图采集、正/空图检测消息和窗口检查通过，未发送飞行指令。YOLO 权重加载和 SDK 模块路径检查通过。未执行实际仿真飞行，旧任务成功记录不能替代新实例验收。

操作入口见 `docs/新实例恢复与九格采集训练_20261008.md`。下面的 2026-09-24 权重路径、SSH 端口和旧备份信息仅属于旧实例历史。

### 九格替代 YOLO 的历史实现（2026-09-24）

- `mission_once.py` 始终使用九格语言部分解析自然语言并发布任务命令；默认 `MISSION_USE_IMAGE_CONTEXT=0`，此时只处理文本。语言模型不逐帧输出飞控指令。
- 原先让九格完整图文模型逐 token 生成 bbox JSON 的 LoRA 路线已完成离线基线，但首轮独立测试 F1@IoU0.5 仅 19.81%，平均生成 3.74 s/图，不作为当前飞行感知源。LoRA adapter 与当前 FCOS 检测头是两套实验产物，不要混用。
- 当前快速检测路线在冻结的九格视觉编码器 `vpm` 的空间特征上训练非 YOLO 的 `torchvision` FCOS 头；不训练 VLN/飞行动作，也不在实时检测时调用语言生成部分。训练代码为 `tools/fm9g_grounding/fcos_grounding.py`，ROS 接口为 `tools/fm9g_grounding/ros_fcos_node.py`；云端部署脚本位于 `/home/ubuntu/model/experiments/fm9g4b_grounding/scripts/speed_head_20260923/`。
- 云端 `data/dataset_20260923_111402` 按飞行场景划分为训练 1004、验证 193、测试 162 张。`fcos-spatial-v2-20260923` 训练 8 轮，按验证集 F1 选取 `epoch-07`。在独立测试集、分数阈值 0.43、IoU 0.5 下：Precision 91.75%、Recall 84.36%、F1 87.90%；红球 Recall 85.45%、蓝球 Recall 63.04%。离线推理平均 26.7 ms、p95 28.7 ms/图；这不包含 ROS 传输、排队、跟踪及控制总延迟，也不是跨场景泛化保证。后续优先补蓝球和困难视角的独立飞行数据。
- 云端启动脚本通过 `PERCEPTION_BACKEND=fm9g` 选择九格检测；`PERCEPTION_BACKEND=yolo` 选择原 YOLO + HSV 检测。默认仍是 `yolo`，任务文本指令不会切换后端。两种模式都发布 `/objects` 供原行为树使用；二维码仍由独立 ArUco/QR 节点识别。九格模式下不得在打击、丢框或异常恢复时运行 YOLO 兜底。
- 2026-09-23 一次完整仿真任务中，启动进程的 `PERCEPTION_BACKEND=fm9g`，ROS master 从任务前到结束仅记录 `/fm9g_spatial_detector` 发布 `/objects`；任务节点订阅该话题，红球搜索、跟踪、冲刺及返航流程有日志，最终行为树返回 `SUCCESS`。这些记录支持本轮在线感知无 YOLO 参与，但没有全程进程审计；`SUCCESS` 和“近距目标消失”都不能单独证明气球被物理扎破。尚需多轮不同颜色、碰撞与真实命中验证，并留存 rosbag/视频。

云端运行方式（在 `/home/ubuntu/catkin_ws` 中选择一条启动）：

```bash
PERCEPTION_BACKEND=fm9g ./sim_env_start.sh
PERCEPTION_BACKEND=yolo ./sim_env_start.sh
```

启动后用 `rostopic info /objects` 核对发布者：九格应为 `/fm9g_spatial_detector`，YOLO 应为 `/object_detection_node`。九格检测头原件仍在数据盘 `/root/gpufree-data/fm9g4b_grounding/checkpoints/fcos-spatial-v2-20260923/epoch-07/head.safetensors`；2026-09-24 已将同哈希、约 29 MB 的部署副本保存到持久系统盘 `/home/ubuntu/model/experiments/fm9g4b_grounding/deployment/fcos-spatial-v2-20260923/epoch-07/head.safetensors`，并将启动脚本的 `FM9G_HEAD` 默认值指向副本。已在 `/home/ubuntu/fm9g-data` 未挂载的状态下核对副本可由 `ubuntu` 读取、脚本语法通过；未因此自动启动仿真或完成重启后的飞行测试。训练图片、标注及检查点仍通过实验目录中的 `data/`、`checkpoints/` 链接使用数据盘，采集和训练前仍需检查并恢复专用 bind mount；不要把未挂载目录当作数据盘写入。更新部署头时同时核对权重哈希、验证指标与 `FM9G_HEAD` 路径，不要直接覆盖原训练检查点。

云端路径（见 `docs/云服务器代码结构.md`）：

```text
/home/ubuntu/model/mission_once.py
/home/ubuntu/model/prompts/system_prompt.txt
/home/ubuntu/model/prompts/mission_prompt.txt
```

针对云端 `model/` 顶层任务解析脚本和提示词，本地只同步以下三个文件；九格检测训练与部署路径见上方专节：

```text
云服务器上model文件夹里要改动的代码/mission_once.py
云服务器上model文件夹里要改动的代码/system_prompt.txt
云服务器上model文件夹里要改动的代码/mission_prompt.txt
```

不要覆盖云端已有的 `FM9G4B-V/`、`basic_cn.txt`、`test_rflysim.py` 等额外文件。2026-09-24 实际连通的实例 SSH 端点为 `root@120.209.70.195:30698`；`30532`、`30287` 属于旧实例端口，使用前仍应以用户给出的最新实例信息和实际连通性为准。ROS/RflySim 运行用户是 `ubuntu`，同步用户是 `root`。同步后需要 `chown -R ubuntu:ubuntu`，并在工作区代码变化后重新 `catkin_make`。凭据不能写入仓库。

截至 2026-09-09，`/home/ubuntu` 顶层累计的 19 个 `*.bak*`、`sync_backup_*` 和 `*invalid*` 历史备份已经清理，当前活动目录 `/home/ubuntu/catkin_ws`、`/home/ubuntu/model`、`/home/ubuntu/PX4PSP` 保持不变。不要假设云端仍有这些旧副本可供回滚；后续覆盖前只保留一个明确命名、位于工作区 `src/` 之外的时间戳备份，确认新版本可用后及时轮换，避免再次无限累积。

### 云端与本地旧同步流程（历史命令，不再执行）

下方记录保留旧实例历史，仅供回溯。其直接 `scp`/解包覆盖同名文件的命令与 2026-10-09 用户要求冲突，已经停用；新成员采用 `docs/组员克隆与云端运行.md` 的新版本目录流程，已有目录升级采用 `docs/GPUFree文件同步说明.md` 的临时目录、差异审查和逐文件合并。

以下流程默认从本地仓库根目录执行，命令使用 Linux shell/WSL/Git Bash 语法。Windows PowerShell 也可以直接执行 `ssh`/`scp`，但变量和路径写法要按 PowerShell 调整。当前只允许同步明确列出的文件和目录，不使用整个 `model` 或 `src` 目录覆盖，也不使用带 `--delete` 的同步命令。

#### A. 同步前检查（双向都必须做）

1. 确认目标主机、端口和用户：最近一次连通的实例为 `ssh -p 30698 root@120.209.70.195`；云实例重建后重新核对。
2. 在本地执行 `git status --short`，确认即将发送或覆盖的文件没有未保存的意外修改。
3. 检查本地三份模型文件是否存在，并检查云端目标目录：

   ```bash
   test -f "云服务器上model文件夹里要改动的代码/mission_once.py"
   test -f "云服务器上model文件夹里要改动的代码/system_prompt.txt"
   test -f "云服务器上model文件夹里要改动的代码/mission_prompt.txt"
   ssh -p 30698 root@120.209.70.195 "ls -l /home/ubuntu/model/mission_once.py /home/ubuntu/model/prompts"
   ```

4. 对覆盖前的远端文件记录 `sha256sum`；若云端已有未合并修改，先下载、比较并取得确认，不能静默覆盖。

#### B. 本地 -> 云端：模型文件

只上传三份被修改文件，保持云端 `FM9G4B-V/`、`basic_cn.txt`、`test_rflysim.py` 等文件不变：

```bash
scp -P 30698 "云服务器上model文件夹里要改动的代码/mission_once.py" \
  root@120.209.70.195:/home/ubuntu/model/mission_once.py
scp -P 30698 "云服务器上model文件夹里要改动的代码/system_prompt.txt" \
  root@120.209.70.195:/home/ubuntu/model/prompts/system_prompt.txt
scp -P 30698 "云服务器上model文件夹里要改动的代码/mission_prompt.txt" \
  root@120.209.70.195:/home/ubuntu/model/prompts/mission_prompt.txt
```

上传完成后修复运行用户权限并核对远端哈希：

```bash
ssh -p 30698 root@120.209.70.195 \
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
scp -P 30698 root@120.209.70.195:/home/ubuntu/model/mission_once.py \
  "云服务器上model文件夹里要改动的代码/mission_once.py"
scp -P 30698 root@120.209.70.195:/home/ubuntu/model/prompts/system_prompt.txt \
  "云服务器上model文件夹里要改动的代码/system_prompt.txt"
scp -P 30698 root@120.209.70.195:/home/ubuntu/model/prompts/mission_prompt.txt \
  "云服务器上model文件夹里要改动的代码/mission_prompt.txt"
```

云端历史盘点中曾有拼写错误的 `system_promt.txt`（少一个 `p`）。标准流程只读写规范名称 `system_prompt.txt`，不要把拼写错误文件重新传播；`basic_cn.txt`、测试脚本和模型权重也不要下载覆盖本地同步目录。

#### D. 本地 -> 云端：ROS 源码

当前云端差异主要位于 `src/mission_pkg` 和 `src/object_det`。从本地仓库根目录用管道打包，覆盖同名文件但不删除云端额外文件：

```bash
tar -czf - src/mission_pkg src/object_det | \
  ssh -p 30698 root@120.209.70.195 \
  "tar -xzf - -C /home/ubuntu/catkin_ws"
ssh -p 30698 root@120.209.70.195 \
  "chown -R ubuntu:ubuntu /home/ubuntu/catkin_ws/src/mission_pkg \
   /home/ubuntu/catkin_ws/src/object_det"
ssh -p 30698 root@120.209.70.195 \
  "cd /home/ubuntu/catkin_ws && catkin_make"
```

如果 `common_msgs`、`sensor_pkg` 或其他包也发生变更，先单独审查依赖，再把明确目录追加到 `tar` 列表；不要因为一次同步而打包整个 `src`。根目录 `sim_env_start.sh`、`tools/fm9g_grounding/`、云端实验脚本及部署权重不在上述 ROS 源码打包范围内，九格相关更新须逐项比对、传输、校验。编译成功后再重启受影响的 ROS 节点，并用 `rostopic` 检查消息和话题版本。

#### E. 云端 -> 本地：ROS 源码

反向拉取前先确认本地工作树已提交或已备份，因为解包会覆盖本地同名文件：

```bash
ssh -p 30698 root@120.209.70.195 \
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
5. Windows 打包/传输可能丢失 Linux 执行权限，即使文件 SHA-256 完全一致仍会启动失败。2026-10-08 恢复遗漏了 `src/object_det/scripts/det_color_conda.sh` 的执行位，云端原权限 `666` 导致 `Cannot locate node`，现已修复为 `755`；换行符正常。随后单独重启 YOLO，确认 `/opt/conda/envs/demo/bin/python3`、CUDA 可用及 `/object_detection_node` 发布真实方框 bbox；未发送飞行指令。打包时显式保留 shell/Python 入口执行位，解包后以 `ubuntu` 检查 `test -x`、ROS 可执行节点解析及实际启动，不能只验证权重加载。

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

当前已有九格视觉编码器 + FCOS 头的替代节点，以及可选 `PERCEPTION_BACKEND=fm9g` 的整套启动方式；默认仍为 YOLO。2026-09-23 的一轮任务具备九格检测唯一 `/objects` 发布者的日志证据，但单轮流程 `SUCCESS` 不是扎破气球或比赛加分的证明。规则列有“使用九格多模态大模型目标识别”加 10 分；目前语言部分负责解析任务、视觉部分加 FCOS 头负责方框及三色气球识别，是否最终计分仍取决于赛事方现场评审。可展示模型结构、窗口、ROS 话题、控制链路及多轮真实结果；不得把未运行语言生成部分的视觉检测说成逐帧完整图文生成。YOLO 曾用于离线预标注，训练数据来源与其他辅助算法的赛事许可范围若无最终书面答复仍需标记待确认。

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
3. 确认 `PERCEPTION_BACKEND`、`/objects` 唯一发布者、检测窗口、对应模型权重及 `common_msgs` 消息；九格模式下还应确认部署权重可读，训练采集时另查数据盘挂载。
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
- `docs/九格全程感知替代YOLO任务清单.md`、`docs/fm9g_v1_training_results_20260922.md`：九格替代计划与首轮生成式 LoRA 基线；其中“检测头尚未实现”等状态是旧记录，以本文件的 2026-09-24 部署与测试记录为准。
- `tools/fm9g_grounding/README.md`：采集、训练和评估入口；部分阶段性状态同样早于当前 FCOS 部署。
- `docs/visual_servo_pd_tuning.md`：视觉伺服调参记录。
- `docs/组员克隆与云端运行.md`：克隆、LFS、打包、新工作区构建和任务执行的完整流程。
- `docs/GPUFree文件同步说明.md`：保留服务器模型及独有文件的差异同步说明；根目录旧同名文件已迁入 docs。
- `docs/自然语言命令测试.md`：自然语言命令测试记录。
- `docs/README.md`：文档索引和归档约定。

上述部分历史方案和本地测试记录未纳入 Git 发布；clone 后实际可用文档以 `docs/README.md` 为准。文档和实现冲突时，先记录冲突，再按本文件开头的信息优先级判断；涉及比赛合规时，以赛事方最终书面规则和正式答复为准，未明确事项保留为待确认项。
