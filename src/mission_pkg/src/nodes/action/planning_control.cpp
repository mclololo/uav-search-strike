/******************************************************************************
 * @file       planning_control.cpp
 * @brief      Planning Control 行为树节点 - 规划器控制
 *
 * @details    与外部路径规划器交互，执行复杂导航任务
 *
 * @author     郭定<1604289397@qq.com>
 * @date       2026-01-06
 ******************************************************************************/
#include "nodes/action/planning_control.h"
#include "utils/pose_utils.h"
#include <algorithm>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace {
double normalizeYaw(double yaw) {
    while (yaw > M_PI) yaw -= 2.0 * M_PI;
    while (yaw < -M_PI) yaw += 2.0 * M_PI;
    return yaw;
}
}

PlanningControl::PlanningControl(const std::string& name, const NodeConfig& config)
    : DroneActionNode(name, config), has_planner_cmd(false) {
    
    auto nh = mct->getROSHandle();
    std::string goal_topic, cmd_topic;
    nh->param<std::string>("goal_topic", goal_topic, "/move_base_simple/goal");
    nh->param<std::string>("planner_cmd_topic", cmd_topic, "/planning/pos_cmd");

    planner_goal_pub = nh->advertise<geometry_msgs::PoseStamped>(goal_topic, 10);
    planner_cmd_sub = nh->subscribe(cmd_topic, 1, &PlanningControl::plannerCmdCallback, this);
}

PortsList PlanningControl::providedPorts() {
    return { InputPort<Position3D>("goal", "目标点(x;y;z)"),
             InputPort<double>("offset_z", 0.0, "相对于目标的Z轴偏移(随飞机航向)"),
             InputPort<double>("tolerance", "到达容差"),
             InputPort<bool>("control_yaw", false, "是否控制偏航角"),
             InputPort<double>("yaw_rate", 0.35, "控制偏航时的最大偏航角速度(rad/s)"),
             InputPort<double>("republish_interval", 5.0, "卡死恢复：规划器无新指令超过该时长重发目标 (s)，0=禁用"),
             InputPort<double>("stuck_timeout", 8.0, "卡死恢复：距离不收敛超过该时长重发目标 (s)，0=禁用"),
             OutputPort<Position3D>("final_position", "到达的最终位置(x;y;z)") };
}

void PlanningControl::plannerCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
    last_planner_cmd = *msg;
    has_planner_cmd = true;
    last_cmd_recv_time_ = ros::Time::now();
}

void PlanningControl::initializeTarget(const Position3D& pos) {
    target_pos_ = pos;

    // 计算带有偏移的目标点 (沿着飞机当前位姿的机头方向投射)
    tf2::Quaternion q_curr;
    tf2::fromMsg(fcu_odom_ptr->pose.pose.orientation, q_curr);
    double r, p, yaw;
    tf2::Matrix3x3(q_curr).getRPY(r, p, yaw);

    double target_x = target_pos_.x + offset_z * cos(yaw);
    double target_y = target_pos_.y + offset_z * sin(yaw);
    double target_z = target_pos_.z;

    // 向规划器发送目标（都使用 ENU，直接发送）
    geometry_msgs::PoseStamped ps;
    ps.header.stamp = ros::Time::now();
    ps.header.frame_id = "world";
    ps.pose.position.x = target_x;
    ps.pose.position.y = target_y;
    ps.pose.position.z = target_z;
    
    tf2::Quaternion q_tgt;
    q_tgt.setRPY(0, 0, 0); 
    ps.pose.orientation = tf2::toMsg(q_tgt);

    planner_goal_pub.publish(ps);
    has_planner_cmd = false;

    // 每次（重）发目标都重置看门狗计时基线
    last_cmd_recv_time_ = ros::Time::now();
    last_progress_time_ = last_cmd_recv_time_;
    last_dist_ = 1e9;

    ROS_ERROR("[%s]: >>> NED goal to ego=(%.2f, %.2f, %.2f), yaw=%.2f, offset=%.2f <<<",
             name().c_str(), ps.pose.position.x, ps.pose.position.y, ps.pose.position.z, yaw, offset_z);

    // 为了 checkArrival 判断，更新目标点
    target_pos_.x = target_x;
    target_pos_.y = target_y;

    target_initialized = true;
    ROS_INFO("[%s]: Target acquired! Planning navigation start -> (%.2f, %.2f, %.2f), offset: %.2f", 
             name().c_str(), target_x, target_y, target_z, offset_z);
    publishDebugEvent("plan_start", "goal=(" + std::to_string(target_x) + "," + std::to_string(target_y) + "," + std::to_string(target_z) + ") offset_z=" + std::to_string(offset_z));
}

NodeStatus PlanningControl::onStart() {
    auto off_res = getInput<double>("offset_z");
    offset_z = off_res.value();

    auto tol_res = getInput<double>("tolerance");
    if (tol_res) {
        tolerance = tol_res.value();
    } else {
        ros::NodeHandle nh;
        nh.param("tolerance", tolerance, 0.2);
    }

    auto yaw_res = getInput<bool>("control_yaw");
    control_yaw = yaw_res.value();
    yaw_rate_limit_ = getInput<double>("yaw_rate").value();

    auto repub_res = getInput<double>("republish_interval");
    republish_interval_ = repub_res.value();
    auto stuck_res = getInput<double>("stuck_timeout");
    stuck_timeout_ = stuck_res.value();
    republish_count_ = 0;

    // 捕获初始偏航角 (供 !control_yaw 使用)
    initial_yaw_ = PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation);
    current_yaw_sp_ = initial_yaw_;
    last_loop_time_ = ros::Time::now();

    ROS_INFO("[%s]: Initialized with offset_z=%.2f, tolerance=%.2f, control_yaw=%d, yaw_rate=%.2f, republish=%.1f/%.1fs",
             name().c_str(), offset_z, tolerance, control_yaw, yaw_rate_limit_,
             republish_interval_, stuck_timeout_);

    // 尝试获取目标点
    auto goal_res = getInput<Position3D>("goal");
    if (!goal_res) {
        ROS_INFO("[%s]: Waiting for goal...", name().c_str());
        target_initialized = false;
        sendHoverCommand();
        return NodeStatus::RUNNING;
    }

    // 初始化目标（保留原始目标供看门狗重发）
    raw_goal_ = goal_res.value();
    initializeTarget(goal_res.value());
    return NodeStatus::RUNNING;
}

NodeStatus PlanningControl::onRunning() {
    // 如果目标还未初始化，尝试获取
    if (!target_initialized) {
        auto goal_res = getInput<Position3D>("goal");
        if (!goal_res) {
            sendHoverCommand();
            return NodeStatus::RUNNING;
        }
        
        raw_goal_ = goal_res.value();
        initializeTarget(goal_res.value());
        ROS_INFO("[%s]: Target acquired!", name().c_str());
    }

    // 检查是否到达 (3D 距离)
    geometry_msgs::Point target;
    target.x = target_pos_.x;
    target.y = target_pos_.y;
    target.z = target_pos_.z;
    if (checkArrival(target, tolerance)) {
        ROS_INFO("[%s]: Planning navigation completed.", name().c_str());
        publishDebugEvent("plan_success", "planner target reached");
        
        // 输出最终到达的位置
        Position3D final_pos;
        final_pos.x = fcu_odom_ptr->pose.pose.position.x;
        final_pos.y = fcu_odom_ptr->pose.pose.position.y;
        final_pos.z = fcu_odom_ptr->pose.pose.position.z;
        setOutput("final_position", final_pos);
        
        ROS_INFO("[%s]: Final position: (%.2f, %.2f, %.2f)", 
                 name().c_str(), final_pos.x, final_pos.y, final_pos.z);
        
        sendHoverCommand();
        return NodeStatus::SUCCESS;
    }

    // 卡死恢复看门狗（设计背景见 planning_control.h）：
    // 规划器静默超 republish_interval_ 或 距离超 stuck_timeout_ 不收敛 -> 重发目标。
    // 重发触发 ego waypointCallback 从当前位置重新规划全局轨迹，打破 EXEC_TRAJ 楔死。
    const ros::Time wd_now = ros::Time::now();
    double wdx = target_pos_.x - fcu_odom_ptr->pose.pose.position.x;
    double wdy = target_pos_.y - fcu_odom_ptr->pose.pose.position.y;
    double wdz = target_pos_.z - fcu_odom_ptr->pose.pose.position.z;
    double wd_dist = std::sqrt(wdx * wdx + wdy * wdy + wdz * wdz);
    if (wd_dist < last_dist_ - 0.1) {
        last_dist_ = wd_dist;
        last_progress_time_ = wd_now;
    }
    bool silent = republish_interval_ > 0.0 && (wd_now - last_cmd_recv_time_).toSec() > republish_interval_;
    bool stuck  = stuck_timeout_ > 0.0 && (wd_now - last_progress_time_).toSec() > stuck_timeout_;
    if (silent || stuck) {
        republish_count_++;
        ROS_WARN("[%s]: Watchdog republish goal #%d (reason=%s, dist=%.2f)",
                 name().c_str(), republish_count_, silent ? "silent" : "stuck", wd_dist);
        publishDebugEvent("plan_republish",
                          "reason=" + std::string(silent ? "silent" : "stuck") + " dist=" + std::to_string(wd_dist));
        initializeTarget(raw_goal_);  // 重发原始目标（未二次投影），并重置看门狗
        return NodeStatus::RUNNING;
    }

    if (!has_planner_cmd) return NodeStatus::RUNNING;

    // Ego-planner 输出经过 ENU→NED 转换后直接给飞控。
    // Planner 自带 yaw 在当前仿真坐标转换下可能跳变，control_yaw=true 时
    // 改为机头平滑朝向最终目标点，保证返航阶段深度相机大致看向飞行方向。
    geometry_msgs::Point ego_pos = last_planner_cmd.position;
    double ego_yaw = initial_yaw_;
    if (control_yaw) {
        ros::Time now = ros::Time::now();
        double dt = (now - last_loop_time_).toSec();
        last_loop_time_ = now;
        if (dt < 0.001) dt = 0.001;

        double dx = target_pos_.x - fcu_odom_ptr->pose.pose.position.x;
        double dy = target_pos_.y - fcu_odom_ptr->pose.pose.position.y;
        if (std::hypot(dx, dy) > 0.2) {
            double target_yaw = std::atan2(dy, dx);
            double yaw_error = normalizeYaw(target_yaw - current_yaw_sp_);
            double max_step = std::max(0.05, yaw_rate_limit_) * dt;
            if (std::abs(yaw_error) <= max_step) {
                current_yaw_sp_ = target_yaw;
            } else {
                current_yaw_sp_ = normalizeYaw(current_yaw_sp_ + std::copysign(max_step, yaw_error));
            }
        }
        ego_yaw = current_yaw_sp_;
    }

    mavros_msgs::PositionTarget cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = name();
    cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    cmd.type_mask = mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    cmd.position.x = -ego_pos.y;
    cmd.position.y = ego_pos.x;
    cmd.position.z = ego_pos.z;
    cmd.velocity.x = -last_planner_cmd.velocity.y;
    cmd.velocity.y = last_planner_cmd.velocity.x;
    cmd.velocity.z = last_planner_cmd.velocity.z;
    cmd.acceleration_or_force.x = -last_planner_cmd.acceleration.y;
    cmd.acceleration_or_force.y = last_planner_cmd.acceleration.x;
    cmd.acceleration_or_force.z = last_planner_cmd.acceleration.z;
    cmd.yaw = ego_yaw + M_PI / 2.0;

    mct->updateCommand(cmd);
    
    return NodeStatus::RUNNING;
}

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<PlanningControl>("PlanningControl");
}
