# uav-search-strike

一个基于 **ROS1 + catkin** 的无人机仿真任务系统，面向“搜索-识别-规划-打击/穿越-返航”类任务场景。项目当前工作区集成了行为树任务编排、目标检测、点云转换、仿真传感器接入、避障规划以及大模型指令驱动任务执行能力。

> 从当前仓库结构来看，这不是单一功能包，而是一个 **catkin 工作区级项目**。核心业务集中在 `mission_pkg`、`object_det`、`sensor_pkg`、`ego-planner` 等包中。

---

## 当前版本：九格检测头与成员部署

`feature/perception` 分支提供九格语言任务解析、冻结九格视觉编码器 + FCOS 四类检测，以及采集、人工复核、训练和评估工具。九格在线检测不调用 YOLO 或逐 token 生成 bbox；二维码由独立 ArUco/QR 节点识别。已训练的约 29.6 MB 检测头通过 Git LFS 分发，克隆后须执行 `git lfs pull`。基础 `FM9G4B-V` 和 YOLO `best.pt` 使用赛事镜像预装文件，不包含在仓库中。

其他成员请按 [组员克隆与云端运行](docs/组员克隆与云端运行.md) 完成克隆、打包、在全新云端工作区编译及运行；[GPUFree 文件同步说明](docs/GPUFree文件同步说明.md) 说明已有工作区的差异合并。打包工具保留 Linux 链接/执行权限，不覆盖原工作区、模型或服务器独有文件。采集与训练入口见 [文档索引](docs/README.md)。

在已编译的新工作区选择 `PERCEPTION_BACKEND=fm9g ./sim_env_start.sh` 或 `PERCEPTION_BACKEND=yolo ./sim_env_start.sh`，默认仍为 YOLO。自然语言任务不会切换后端；九格模式的 `/objects` 唯一发布者应为 `/fm9g_spatial_detector`。组员须在各自实例进行飞行验收，构建和离线测试不代表实际命中。

## 1. 项目概览

该项目的目标是：

- 从仿真环境获取图像、IMU、里程计、深度点云等数据
- 通过目标检测识别方框、气球、ArUco 等目标
- 使用行为树编排任务流程（起飞、穿框、识别、击打、降落）
- 借助 Ego-Planner 执行局部规划/避障
- 支持通过 LLM/自然语言链路下发高层任务指令

从根目录脚本和各包配置判断，当前项目主要服务于 **RflySim / PX4 / ROS1** 仿真联调环境。

---

## 2. 工作区结构

根目录是标准 catkin 工作区：

```text
catkin_ws/
├── build.sh                  # 一键编译（本质上执行 catkin_make）
├── sim_env_start.sh          # 一键启动仿真任务环境
├── kill_ros_pid.sh           # 清理 ROS / python3 相关进程
├── src/
│   ├── mission_pkg/          # 任务控制与行为树执行核心包
│   ├── object_det/           # 目标检测与颜色分类
│   ├── sensor_pkg/           # 传感器接入、点云转换、视觉伺服工具
│   ├── common_msgs/          # 公共消息定义
│   ├── behaviortree_cpp/     # 行为树库（第三方/源码集成）
│   ├── cv_bridge/            # ROS 图像桥接包
│   └── ego-planner/          # 局部规划/避障
└── tools/                    # 辅助工具脚本
```

---

## 3. 架构分析

### 3.1 总体分层

项目可以分成 5 层：

1. **仿真与数据源层**
   - `sensor_pkg/main.py`
   - RflySim / VisionCapture / CopterSim
   - 输出图像、IMU、odom、深度点云等数据

2. **感知层**
   - `tools/fm9g_grounding/ros_fcos_node.py`：九格视觉编码器 + FCOS 检测
   - `object_det`：YOLO 目标检测、HSV 气球颜色分类
   - `mission_pkg/src/aruco_detect_node.cpp` / `scripts/aruco_detect_node.py`：ArUco 识别

3. **空间理解与规划层**
   - `sensor_pkg/cloud_transform_node.py`：点云转换到 world 坐标系
   - `ego-planner`：局部避障与轨迹规划

4. **任务决策与执行层**
   - `mission_pkg/src/bt_ros.cpp`
   - `mission_pkg/config/*.xml`
   - 基于 BehaviorTree.CPP 的任务编排

5. **交互与编排入口层**
   - `sim_env_start.sh`：一键拉起整套系统
   - `/mission_command`：任务指令输入
   - `/mission_result`：任务结果输出

### 3.2 核心数据流

```text
RflySim / CopterSim
   │
   ├── 图像/IMU/里程计/点云  -> sensor_pkg
   │
   ├── 图像 -> object_det -> /objects
   │
   ├── 图像 -> aruco_detect_node -> /aruco/detected_id
   │
   ├── 深度点云 + odom -> cloud_transform_node -> /cloud_registered
   │                                │
   │                                └-> ego-planner
   │
   └── mission_pkg(bt_ros)
         ├── 读取 /objects /aruco/detected_id
         ├── 生成 /move_base_simple/goal 或规划控制命令
         ├── 执行行为树任务
         └── 输出 /mission_result
```

---

## 4. 核心 ROS 包说明

## 4.1 `mission_pkg`：任务控制核心

这是当前项目最核心的业务包，负责：

- 任务行为树加载与执行
- 任务节点注册（起飞、降落、前往目标、穿框、打击气球等）
- 感知结果接入（`/objects`、`/aruco/detected_id`）
- LLM 模式下的动态任务构建

### 关键目录

```text
src/mission_pkg/
├── launch/
│   ├── llm_mission.launch
│   ├── cross_frame.launch
│   ├── hit_balloon.launch
│   └── aruco_detect.launch
├── config/
│   ├── basic_test.xml
│   ├── cross_frame.xml
│   ├── hit_balloon.xml
│   └── full_competition*.xml
├── src/
│   ├── bt_ros.cpp
│   ├── mavros_cnt.cpp
│   ├── aruco_detect_node.cpp
│   ├── nodes/action/
│   ├── nodes/perception/
│   └── nodes/condition/
└── include/
```

### 核心设计

#### 1) 行为树执行入口：`src/mission_pkg/src/bt_ros.cpp`

该文件承担以下职责：

- 从参数服务器读取行为树配置
- 扫描 `devel/lib` 下插件并注册到 `BehaviorTreeFactory`
- 支持两种模式：
  - `static`：直接加载 XML 行为树
  - `llm`：监听 `/mission_command` 动态生成行为树
- 发布 `/mission_result`

#### 2) LLM 模式

从代码中可确认，`bt_ros.cpp` 支持以下指令片段：

- `takeoff`
- `cross_frame`
- `detect_aruco`
- `return_cross_frame`
- `hit_balloon[:区域:颜色]`
- `land`

也就是说，项目已具备“高层语义指令 -> 行为树 XML -> 执行”的链路。

#### 3) 任务行为节点

`src/mission_pkg/src/nodes/` 下已经按职责拆分：

- `action/`：动作节点，如 `takeoff.cpp`、`land.cpp`、`visual_servo.cpp`
- `perception/`：感知相关节点，如 `aruco_detector.cpp`、`target_locator.cpp`
- `condition/`：条件判断节点

这说明任务层采用了比较清晰的模块化设计，而不是把所有逻辑堆在单个 ROS 节点里。

### 典型话题

- 输入：
  - `/mission_command`
  - `/objects`
  - `/aruco/detected_id`
- 输出：
  - `/mission_result`
  - `/move_base_simple/goal`
  - `/planning/pos_cmd`

---

## 4.2 `object_det`：目标检测与颜色分类

这是 Python 实现的感知包，负责：

- 基于 YOLO 模型做目标检测
- 对 `balloon` 检测框进一步做 HSV 颜色分类
- 输出统一检测结果到 `/objects`

### 关键文件

- `src/object_det/scripts/det_color.py`
- `src/object_det/scripts/det.py`
- `src/object_det/launch/detection_color.launch`
- `src/object_det/models/`：模型权重目录
- `src/object_det/third_party/`：本地第三方依赖

### 当前实现特点

#### 1) ROS + Python 标准结构

包内已明确采用标准 ROS Python 结构：

- `scripts/` 放 ROS 可执行节点
- `src/object_det/` 放可复用 Python 模块

#### 2) 颜色分类增强

`det_color.py` 在 YOLO 检测后，针对气球目标做 HSV 分类，输出如：

- `red_balloon`
- `blue_balloon`
- `purple_balloon`

这部分是任务层击打不同颜色气球的重要前置能力。

#### 3) 兼容性处理较多

从代码可见，该包为实际运行做了不少工程化兼容：

- 手动控制 `sys.path` 以加载本地 `third_party`
- 对 `cv_bridge` 不可用场景做降级
- 控制 PyTorch CPU 线程数，避免占满资源

说明它已经不是“纯 demo”，而是朝可落地运行方向做过适配。

### 主要话题

- 订阅：`/rflysim/sensor1/img_rgb`（默认配置）
- 发布：`/objects`

---

## 4.3 `sensor_pkg`：仿真传感器接入与工具节点

该包承担仿真环境的底层接入与辅助能力。

### 关键脚本

- `main.py`
- `AgentContestMap.py`
- `cloud_transform_node.py`
- `visual_servo_balloon.py`

### 角色拆解

#### 1) `main.py`

负责接入 RflySim/VisionCapture，主要作用：

- 获取仿真器 IP
- 加载 `Config.json` 中的传感器配置
- 开启图像采集
- 请求 IMU/仿真数据回传
- 启用 ROS 数据转发模式

它更像是 **仿真传感器总入口**。

#### 2) `cloud_transform_node.py`

功能很明确：

- 订阅 `/rflysim/uav1/local/odom`
- 订阅 `/rflysim/sensor0/Depth_Cloud`
- 将点云从传感器/机体系变换到 `world`
- 发布 `/cloud_registered`

这一步是 Ego-Planner 能使用深度点云避障的关键桥接层。

#### 3) `AgentContestMap.py`

从命名和类定义 `MapRandomizer` 看，主要用于比赛场景地图/元素加载与随机化。

#### 4) `visual_servo_balloon.py`

提供独立的视觉伺服能力：

- 订阅 `/objects`
- 订阅 odom
- 输出基于目标偏差的控制命令

当前项目中既有行为树内的视觉伺服节点，也保留了这个独立工具节点，适合调参与分模块测试。

---

## 4.4 `common_msgs`：公共消息定义

该包提供项目内共享消息类型，`object_det` 和 `mission_pkg` 都依赖它。

从 `package.xml` 可见其依赖：

- `std_msgs`
- `geometry_msgs`
- `message_generation`
- `message_runtime`

这是项目内部各模块解耦的重要基础。

---

## 4.5 `behaviortree_cpp`：行为树基础库

项目直接将 `behaviortree_cpp` 源码纳入工作区，而不是简单依赖系统安装版本。

这样做的好处：

- 方便自定义/调试行为树能力
- 便于和 `mission_pkg` 的插件式节点集成
- 版本可控

`mission_pkg` 的任务系统就是建立在它之上的。

---

## 4.6 `ego-planner`：局部规划/避障

虽然当前没有展开其内部代码，但从 `sim_env_start.sh` 可以确认：

- 系统启动时会拉起 `ego_planner FS-J310_ego-planner.launch`
- 点云输入由 `sensor_pkg/cloud_transform.launch` 提供

因此它在整体架构中的定位很清晰：

> 为无人机运动提供局部路径规划与避障能力。

---

## 5. 启动链路分析

根目录 `sim_env_start.sh` 已经给出了当前项目最完整的启动编排。

### 启动顺序

脚本大致会启动：

1. `roscore`
2. 大模型进程 `mission_once.py`
3. 九格 `ros_fcos_node.py` 或 `object_det` 的 `detection_color.launch`，由 `PERCEPTION_BACKEND` 选择
4. `sensor_pkg/main.py`
5. `sensor_pkg/AgentContestMap.py`
6. `sensor_pkg/cloud_transform.launch`
7. `mission_pkg/aruco_detect.launch`
8. `ego_planner` 启动文件
9. `mission_pkg/llm_mission.launch`
10. Setpoint 监控
11. ROS 节点资源监控

### 说明

这意味着当前系统并非“多个独立工具包”，而是已经形成了一个 **完整联调工作流**：

- 先让感知与大模型准备完成
- 再启动传感器和地图
- 再挂接规划与任务节点
- 最终通过 `/mission_command` 触发任务

---

## 6. 构建与运行

## 6.1 编译

根目录的 `build.sh` 非常简单：

```bash
./build.sh
```

本质上等价于：

```bash
catkin_make
```

推荐标准流程：

```bash
cd /home/ubuntu/catkin_ws
catkin_make
source devel/setup.bash
```

## 6.2 一键启动仿真环境

```bash
cd /home/ubuntu/catkin_ws
bash sim_env_start.sh
```

该脚本会自动：

- 清理旧 ROS 相关进程
- source 工作区环境
- 补充 RflySim SDK 路径
- 启动感知、点云转换、ArUco、规划、任务控制、监控窗口

## 6.3 发送任务指令

根据脚本中的示例，可以发送：

```bash
rostopic pub /mission_command std_msgs/String "data: '{"commands":["takeoff","cross_frame","land"]}'" --once
```

或：

```bash
rostopic pub /mission_command std_msgs/String "data: '{"commands":["takeoff","hit_balloon:red","land"]}'" --once
```

查看结果：

```bash
rostopic echo /mission_result
```

中止任务：

```bash
rostopic pub /mission_abort std_msgs/String "data: 'abort'" --once
```

> 注意：如果直接复制命令到 shell，JSON 和引号需要按你当前 shell 环境正确转义。

---

## 7. 关键配置入口

### 行为树配置

- `src/mission_pkg/config/basic_test.xml`
- `src/mission_pkg/config/cross_frame.xml`
- `src/mission_pkg/config/hit_balloon.xml`
- `src/mission_pkg/config/full_competition.xml`

### 任务参数

- `src/mission_pkg/launch/llm_mission.launch`

这里集中定义了：

- 起飞目标点
- 穿框观察点 / 预期位置
- ArUco 悬停点
- 回穿方框参数
- 不同颜色气球与区域映射
- Visual Servo 参数
- 安全边界参数

### 目标检测

- `src/object_det/launch/detection_color.launch`
- `src/object_det/scripts/det_color.py`
- `src/object_det/models/`

### 点云转换

- `src/sensor_pkg/launch/cloud_transform.launch`
- `src/sensor_pkg/cloud_transform_node.py`

---

## 8. 当前项目的优点与特点

基于现有代码结构，我认为当前架构有几个明显优点：

### 1) 模块职责比较清晰

- `sensor_pkg` 负责仿真与传感器
- `object_det` 负责视觉检测
- `mission_pkg` 负责任务编排
- `ego-planner` 负责规划

### 2) 行为树适合复杂任务编排

相比把任务逻辑写死在单个状态机中，BehaviorTree.CPP 更适合：

- 拆分动作/条件/感知节点
- 方便复用与调参
- 容易与 LLM 指令生成链路结合

### 3) 已具备“自然语言/LLM 驱动任务”的基础

`bt_ros.cpp` 中的 `llm` 模式，加上根目录启动脚本对 `mission_once.py` 和 `llm_mission.launch` 的编排，说明项目已经在朝“自然语言任务下发”方向演进。

### 4) 工程化痕迹明显

比如：

- 统一启动脚本
- 进程清理脚本
- 监控窗口
- 资源监控
- rosbag 可选录制
- 兼容 `cv_bridge` 问题的 Python 降级实现

这些都说明项目更偏向“比赛/联调系统”，而不只是算法实验代码。

---

## 9. 当前存在的风险/改进建议

在分析过程中，也看到一些值得后续优化的地方：

### 1) 根目录缺少统一 README

这是本次补充的重点。此前文档更多散落在子包内部，工作区级的整体说明不足。

### 2) 启动脚本对环境路径耦合较强

如：

- `/home/ubuntu/model`
- `/mnt/c/PX4PSP/RflySimAPIs/RflySimSDK`

这意味着跨机器迁移时需要先调整路径。

### 3) `kill_ros_pid.sh` 清理策略偏激进

它会：

- `kill -9` 所有包含 `ros` 的进程
- `killall -9 python3`

如果开发机上还有其他 Python/ROS 任务，会被一并杀掉。建议后续按工作区或节点名精细化清理。

### 4) 外部依赖较多

项目依赖：

- ROS1 / catkin
- RflySim SDK
- PX4/MAVROS 相关环境
- 本地模型文件
- Python 第三方库

后续可以补一个更完整的环境准备文档或 `requirements`/依赖清单。

---

## 10. 推荐的开发阅读顺序

如果你是第一次接手这个项目，建议按下面顺序阅读：

1. `sim_env_start.sh`
   - 先理解系统如何被整体拉起
2. `src/mission_pkg/launch/llm_mission.launch`
   - 看清系统运行参数和任务配置
3. `src/mission_pkg/src/bt_ros.cpp`
   - 理解任务入口、LLM 指令和行为树构建
4. `src/mission_pkg/config/*.xml`
   - 理解具体任务流程
5. `src/object_det/scripts/det_color.py`
   - 理解检测输出格式
6. `src/sensor_pkg/cloud_transform_node.py`
   - 理解规划所需点云数据如何生成

---

## 11. 最小使用流程

### 编译

```bash
cd /home/ubuntu/catkin_ws
catkin_make
source devel/setup.bash
```

### 启动整套系统

```bash
bash sim_env_start.sh
```

### 发送任务

```bash
rostopic pub /mission_command std_msgs/String "data: '{"commands":["takeoff","cross_frame","land"]}'" --once
```

### 查看结果

```bash
rostopic echo /mission_result
```

---

## 12. 总结

当前项目是一个典型的 **ROS1 无人机任务工作区**，具备以下主线能力：

- **传感器与仿真接入**：`sensor_pkg`
- **视觉感知**：`object_det` + ArUco 检测
- **行为树任务编排**：`mission_pkg`
- **规划避障**：`ego-planner`
- **高层指令驱动**：LLM / `/mission_command`

如果只用一句话概括它：

> 这是一个面向无人机仿真比赛/任务执行场景的、集成感知、规划、行为树控制与自然语言指令接口的 ROS1 catkin 工作区。
