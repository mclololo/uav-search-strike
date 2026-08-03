/******************************************************************************
 * @file       planning_control.cpp
 * @brief      Planning control with an isolated safe-return mode
 ******************************************************************************/
#include "nodes/action/planning_control.h"

#include "utils/pose_utils.h"

#include <algorithm>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace {
double normalizeYaw(double yaw) {
    while (yaw > M_PI) yaw -= 2.0 * M_PI;
    while (yaw < -M_PI) yaw += 2.0 * M_PI;
    return yaw;
}

double distance3d(const geometry_msgs::Point& lhs, const geometry_msgs::Point& rhs) {
    return std::sqrt((lhs.x - rhs.x) * (lhs.x - rhs.x) +
                     (lhs.y - rhs.y) * (lhs.y - rhs.y) +
                     (lhs.z - rhs.z) * (lhs.z - rhs.z));
}
}  // namespace

PlanningControl::PlanningControl(const std::string& name, const NodeConfig& config)
    : DroneActionNode(name, config) {
    auto nh = mct->getROSHandle();
    std::string goal_topic;
    std::string command_topic;
    nh->param<std::string>("goal_topic", goal_topic, "/move_base_simple/goal");
    nh->param<std::string>("planner_cmd_topic", command_topic, "/planning/pos_cmd");
    planner_goal_pub = nh->advertise<geometry_msgs::PoseStamped>(goal_topic, 10);
    planner_cmd_sub = nh->subscribe(command_topic, 1,
                                    &PlanningControl::plannerCmdCallback, this);
}

PortsList PlanningControl::providedPorts() {
    return {
        InputPort<Position3D>("goal", "目标点(x;y;z)"),
        InputPort<double>("offset_z", 0.0, "相对于目标的Z轴偏移(随飞机航向)"),
        InputPort<double>("tolerance", "到达容差"),
        InputPort<bool>("control_yaw", false, "是否控制偏航角"),
        InputPort<double>("yaw_rate", 0.35, "控制偏航时的最大偏航角速度(rad/s)"),
        InputPort<bool>("use_home_xy", false, "返航模式：使用任务启动时记录的home XY"),
        InputPort<double>("stop_speed_threshold", 0.12, "返航规划接管前最大速度"),
        InputPort<double>("stop_hold_time", 0.30, "返航规划接管前低速保持时间"),
        InputPort<double>("planner_wait_timeout", 6.0, "等待返航新轨迹超时"),
        InputPort<double>("planner_cmd_timeout", 0.50, "返航规划指令新鲜度上限"),
        InputPort<double>("goal_match_tolerance", 0.30, "返航轨迹目标匹配容差"),
        InputPort<double>("max_command_distance", 1.00, "返航轨迹首端最大跳点距离"),
        OutputPort<Position3D>("final_position", "到达的最终位置(x;y;z)")
    };
}

void PlanningControl::plannerCmdCallback(
    const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
    // Preserve the 80635ef behavior for frame crossing and every normal plan.
    if (!use_home_xy_) {
        last_planner_cmd = *msg;
        has_planner_cmd = true;
        return;
    }

    std::lock_guard<std::mutex> lock(planner_cmd_mutex_);
    latest_seen_trajectory_id_ = std::max(latest_seen_trajectory_id_, msg->trajectory_id);
    if (!return_goal_published_) return;
    if (msg->trajectory_flag != quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY) {
        return;
    }
    if (msg->trajectory_id <= minimum_trajectory_id_) {
        ROS_WARN_THROTTLE(1.0, "[%s]: Reject stale return trajectory id=%u",
                          name().c_str(), msg->trajectory_id);
        return;
    }

    geometry_msgs::Point expected;
    expected.x = target_pos_.x;
    expected.y = target_pos_.y;
    expected.z = target_pos_.z;
    if (distance3d(msg->goal_pos, expected) > goal_match_tolerance_) {
        ROS_WARN_THROTTLE(1.0, "[%s]: Reject trajectory for another goal",
                          name().c_str());
        return;
    }
    if (msg->header.stamp + ros::Duration(0.05) < goal_publish_time_) {
        return;
    }

    const bool first_for_trajectory = accepted_trajectory_id_ != msg->trajectory_id;
    const geometry_msgs::Point current = fcu_odom_ptr->pose.pose.position;
    if (first_for_trajectory &&
        distance3d(msg->position, current) > max_command_distance_) {
        ROS_ERROR_THROTTLE(1.0, "[%s]: Reject unsafe return trajectory first point",
                           name().c_str());
        return;
    }

    last_planner_cmd = *msg;
    accepted_trajectory_id_ = msg->trajectory_id;
    last_valid_cmd_time_ = ros::Time::now();
    has_planner_cmd = true;
}

void PlanningControl::initializeTarget(const Position3D& pos) {
    target_pos_ = pos;

    tf2::Quaternion quaternion;
    tf2::fromMsg(fcu_odom_ptr->pose.pose.orientation, quaternion);
    double roll;
    double pitch;
    double yaw;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

    target_pos_.x += offset_z * std::cos(yaw);
    target_pos_.y += offset_z * std::sin(yaw);
    target_initialized = true;

    if (use_home_xy_) {
        ROS_INFO("[%s]: Return target resolved to (%.2f, %.2f, %.2f)",
                 name().c_str(), target_pos_.x, target_pos_.y, target_pos_.z);
        return;
    }

    // Baseline mode publishes the goal immediately, exactly as 80635ef did.
    geometry_msgs::PoseStamped goal;
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = "world";
    goal.pose.position.x = target_pos_.x;
    goal.pose.position.y = target_pos_.y;
    goal.pose.position.z = target_pos_.z;
    tf2::Quaternion target_quaternion;
    target_quaternion.setRPY(0, 0, 0);
    goal.pose.orientation = tf2::toMsg(target_quaternion);
    planner_goal_pub.publish(goal);
    has_planner_cmd = false;

    ROS_ERROR("[%s]: >>> NED goal to ego=(%.2f, %.2f, %.2f), yaw=%.2f, offset=%.2f <<<",
              name().c_str(), target_pos_.x, target_pos_.y, target_pos_.z, yaw, offset_z);
    publishDebugEvent("plan_start", "baseline planning goal published");
}

bool PlanningControl::tryInitializeTarget() {
    auto goal_result = getInput<Position3D>("goal");
    if (!goal_result) return false;

    Position3D goal = goal_result.value();
    if (use_home_xy_) {
        if (!mct->hasHomePosition()) {
            ROS_WARN_THROTTLE(1.0, "[%s]: Waiting for recorded home position",
                              name().c_str());
            return false;
        }
        const geometry_msgs::Point home = mct->getHomePosition();
        goal.x = home.x;
        goal.y = home.y;
    }
    initializeTarget(goal);
    return true;
}

void PlanningControl::publishReturnGoal() {
    geometry_msgs::PoseStamped goal;
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = "world";
    goal.pose.position.x = target_pos_.x;
    goal.pose.position.y = target_pos_.y;
    goal.pose.position.z = target_pos_.z;
    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, 0);
    goal.pose.orientation = tf2::toMsg(quaternion);

    {
        std::lock_guard<std::mutex> lock(planner_cmd_mutex_);
        minimum_trajectory_id_ = latest_seen_trajectory_id_;
        accepted_trajectory_id_ = 0;
        has_planner_cmd = false;
        goal_publish_time_ = goal.header.stamp;
        return_goal_published_ = true;
    }
    planner_goal_pub.publish(goal);
    publishDebugEvent("return_goal", "waiting for fresh matched EGO trajectory");
}

bool PlanningControl::isStopped() const {
    if (!fcu_odom_ptr || fcu_odom_ptr->header.stamp == ros::Time(0)) return false;
    const auto& velocity = fcu_odom_ptr->twist.twist.linear;
    return std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y +
                     velocity.z * velocity.z) <= stop_speed_threshold_;
}

void PlanningControl::deactivateReturnPlanner() {
    std::lock_guard<std::mutex> lock(planner_cmd_mutex_);
    return_goal_published_ = false;
    has_planner_cmd = false;
    accepted_trajectory_id_ = 0;
}

NodeStatus PlanningControl::onStart() {
    offset_z = getInput<double>("offset_z").value();
    auto tolerance_result = getInput<double>("tolerance");
    if (tolerance_result) {
        tolerance = tolerance_result.value();
    } else {
        ros::NodeHandle nh;
        nh.param("tolerance", tolerance, 0.2);
    }
    control_yaw = getInput<bool>("control_yaw").value();
    yaw_rate_limit_ = getInput<double>("yaw_rate").value();
    use_home_xy_ = getInput<bool>("use_home_xy").value();
    stop_speed_threshold_ = getInput<double>("stop_speed_threshold").value();
    stop_hold_time_ = getInput<double>("stop_hold_time").value();
    planner_wait_timeout_ = getInput<double>("planner_wait_timeout").value();
    planner_cmd_timeout_ = getInput<double>("planner_cmd_timeout").value();
    goal_match_tolerance_ = getInput<double>("goal_match_tolerance").value();
    max_command_distance_ = getInput<double>("max_command_distance").value();

    initial_yaw_ = PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation);
    current_yaw_sp_ = initial_yaw_;
    last_loop_time_ = ros::Time::now();
    target_initialized = false;
    stop_hold_start_ = ros::Time(0);
    return_goal_published_ = false;
    accepted_trajectory_id_ = 0;
    has_planner_cmd = false;

    if (!tryInitializeTarget()) sendHoverCommand();
    if (use_home_xy_) {
        ROS_INFO("[%s]: Safe return mode waiting for vehicle stop", name().c_str());
    }
    return NodeStatus::RUNNING;
}

NodeStatus PlanningControl::onRunning() {
    if (!target_initialized && !tryInitializeTarget()) {
        sendHoverCommand();
        return NodeStatus::RUNNING;
    }
    return use_home_xy_ ? runSafeReturnPlanner() : runBaselinePlanner();
}

NodeStatus PlanningControl::runBaselinePlanner() {
    geometry_msgs::Point target;
    target.x = target_pos_.x;
    target.y = target_pos_.y;
    target.z = target_pos_.z;
    if (checkArrival(target, tolerance)) {
        Position3D final_position;
        final_position.x = fcu_odom_ptr->pose.pose.position.x;
        final_position.y = fcu_odom_ptr->pose.pose.position.y;
        final_position.z = fcu_odom_ptr->pose.pose.position.z;
        setOutput("final_position", final_position);
        sendHoverCommand();
        publishDebugEvent("plan_success", "planner target reached");
        return NodeStatus::SUCCESS;
    }
    if (!has_planner_cmd) return NodeStatus::RUNNING;
    applyPlannerCommand(last_planner_cmd);
    return NodeStatus::RUNNING;
}

NodeStatus PlanningControl::runSafeReturnPlanner() {
    geometry_msgs::Point target;
    target.x = target_pos_.x;
    target.y = target_pos_.y;
    target.z = target_pos_.z;
    if (checkArrival(target, tolerance) && isStopped()) {
        Position3D final_position;
        final_position.x = fcu_odom_ptr->pose.pose.position.x;
        final_position.y = fcu_odom_ptr->pose.pose.position.y;
        final_position.z = fcu_odom_ptr->pose.pose.position.z;
        setOutput("final_position", final_position);
        sendHoverCommand();
        publishDebugEvent("return_success", "home target reached");
        deactivateReturnPlanner();
        return NodeStatus::SUCCESS;
    }

    bool goal_published;
    {
        std::lock_guard<std::mutex> lock(planner_cmd_mutex_);
        goal_published = return_goal_published_;
    }
    if (!goal_published) {
        sendHoverCommand();
        if (!isStopped()) {
            stop_hold_start_ = ros::Time(0);
            return NodeStatus::RUNNING;
        }
        if (stop_hold_start_.isZero()) stop_hold_start_ = ros::Time::now();
        if ((ros::Time::now() - stop_hold_start_).toSec() >= stop_hold_time_) {
            publishReturnGoal();
        }
        return NodeStatus::RUNNING;
    }

    quadrotor_msgs::PositionCommand command;
    bool has_command;
    ros::Time command_time;
    ros::Time publish_time;
    {
        std::lock_guard<std::mutex> lock(planner_cmd_mutex_);
        has_command = has_planner_cmd;
        command = last_planner_cmd;
        command_time = last_valid_cmd_time_;
        publish_time = goal_publish_time_;
    }
    if (!has_command) {
        sendHoverCommand();
        if ((ros::Time::now() - publish_time).toSec() > planner_wait_timeout_) {
            ROS_ERROR("[%s]: Timed out waiting for fresh return trajectory", name().c_str());
            deactivateReturnPlanner();
            return NodeStatus::FAILURE;
        }
        return NodeStatus::RUNNING;
    }
    if ((ros::Time::now() - command_time).toSec() > planner_cmd_timeout_) {
        sendHoverCommand();
        ROS_ERROR("[%s]: Return planner command stream stale", name().c_str());
        deactivateReturnPlanner();
        return NodeStatus::FAILURE;
    }

    applyPlannerCommand(command);
    return NodeStatus::RUNNING;
}

void PlanningControl::applyPlannerCommand(
    const quadrotor_msgs::PositionCommand& planner_command) {
    double ego_yaw = initial_yaw_;
    if (control_yaw) {
        const ros::Time now = ros::Time::now();
        const double dt = std::max((now - last_loop_time_).toSec(), 0.001);
        last_loop_time_ = now;
        const double dx = target_pos_.x - fcu_odom_ptr->pose.pose.position.x;
        const double dy = target_pos_.y - fcu_odom_ptr->pose.pose.position.y;
        if (std::hypot(dx, dy) > 0.2) {
            const double target_yaw = std::atan2(dy, dx);
            const double yaw_error = normalizeYaw(target_yaw - current_yaw_sp_);
            const double max_step = std::max(0.05, yaw_rate_limit_) * dt;
            current_yaw_sp_ = std::abs(yaw_error) <= max_step
                ? target_yaw
                : normalizeYaw(current_yaw_sp_ + std::copysign(max_step, yaw_error));
        }
        ego_yaw = current_yaw_sp_;
    }

    mavros_msgs::PositionTarget command;
    command.header.stamp = ros::Time::now();
    command.header.frame_id = name();
    command.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    command.type_mask = mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    command.position.x = -planner_command.position.y;
    command.position.y = planner_command.position.x;
    command.position.z = planner_command.position.z;
    command.velocity.x = -planner_command.velocity.y;
    command.velocity.y = planner_command.velocity.x;
    command.velocity.z = planner_command.velocity.z;
    command.acceleration_or_force.x = -planner_command.acceleration.y;
    command.acceleration_or_force.y = planner_command.acceleration.x;
    command.acceleration_or_force.z = planner_command.acceleration.z;
    command.yaw = ego_yaw + M_PI / 2.0;
    mct->updateCommand(command);
}

void PlanningControl::onHalted() {
    if (use_home_xy_) deactivateReturnPlanner();
    DroneActionNode::onHalted();
}

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<PlanningControl>("PlanningControl");
}
