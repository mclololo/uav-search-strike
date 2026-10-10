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
    cloud_sub_ = nh->subscribe("/cloud_registered", 1,
                               &PlanningControl::cloudCallback, this);
    return_map_sub_ = nh->subscribe("/planning/return_map_stamp", 1,
                                    &PlanningControl::returnMapCallback, this);
    return_map_pub_ = nh->advertise<geometry_msgs::PointStamped>(
        "/planning/return_clearance", 1);
}

PortsList PlanningControl::providedPorts() {
    return {
        InputPort<Position3D>("goal", "目标点(x;y;z)"),
        InputPort<double>("offset_z", 0.0, "相对于目标的Z轴偏移(随飞机航向)"),
        InputPort<double>("offset_x", 0.0, "目标的世界坐标X偏移"),
        InputPort<double>("offset_y", 0.0, "目标的世界坐标Y偏移"),
        InputPort<double>("tolerance", "到达容差"),
        InputPort<bool>("control_yaw", false, "是否控制偏航角"),
        InputPort<double>("yaw_rate", 0.35, "控制偏航时的最大偏航角速度(rad/s)"),
        InputPort<bool>("use_home_xy", false, "返航模式：使用任务启动时记录的home XY"),
        InputPort<double>("planner_wait_timeout", 6.0, "等待返航新轨迹超时"),
        InputPort<double>("planner_cmd_timeout", 0.50, "返航规划指令新鲜度上限"),
        InputPort<double>("cloud_timeout", 0.80, "返航障碍点云新鲜度上限"),
        InputPort<double>("stop_speed_threshold", 0.12, "返航接管前最大实测速度"),
        InputPort<double>("stop_hold_time", 0.30, "返航接管前低速保持时间"),
        InputPort<double>("goal_match_tolerance", 0.30, "返航轨迹目标匹配容差"),
        InputPort<double>("max_command_distance", 1.00, "返航轨迹首端最大跳点距离"),
        InputPort<double>("final_yaw", "降落前目标航向(rad)"),
        InputPort<double>("final_yaw_rate", 0.25, "返航末段偏航速度(rad/s)"),
        OutputPort<Position3D>("final_position", "到达的最终位置(x;y;z)")
    };
}

void PlanningControl::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    if (msg->width == 0 || msg->height == 0 || msg->header.frame_id != "world") return;
    const ros::Time now = ros::Time::now();
    if (msg->header.stamp.isZero() || msg->header.stamp > now) return;
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    last_cloud_time_ = msg->header.stamp;
}

bool PlanningControl::hasFreshCloud() const {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    const ros::Time now = ros::Time::now();
    const ros::Time stamp = use_home_xy_ ? return_map_time_ : last_cloud_time_;
    return !stamp.isZero() && now >= stamp &&
           (now - stamp).toSec() <= cloud_timeout_ &&
           (!use_home_xy_ || stamp >= return_mode_start_);
}

bool PlanningControl::hasCloudSince(const ros::Time& time) const {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    const ros::Time now = ros::Time::now();
    const ros::Time stamp = use_home_xy_ ? return_map_time_ : last_cloud_time_;
    return !time.isZero() && stamp >= time + ros::Duration(0.20) &&
           now >= stamp && (now - stamp).toSec() <= cloud_timeout_;
}

void PlanningControl::returnMapCallback(const std_msgs::Time::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    return_map_time_ = msg->data;
}

void PlanningControl::publishReturnMapMode(bool active) {
    geometry_msgs::PointStamped mode;
    mode.header.stamp = ros::Time::now();
    mode.header.frame_id = active ? "world" : "";
    mode.point = fcu_odom_ptr->pose.pose.position;
    mode.point.z = return_min_altitude_;
    return_map_pub_.publish(mode);
}

void PlanningControl::plannerCmdCallback(
    const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
    // Preserve the 80635ef behavior for frame crossing and every normal plan.
    if (!use_home_xy_) {
        std::lock_guard<std::mutex> lock(planner_cmd_mutex_);
        latest_seen_trajectory_id_ = std::max(latest_seen_trajectory_id_, msg->trajectory_id);
        if (baseline_replan_pending_ &&
            (msg->trajectory_id <= minimum_trajectory_id_ ||
             msg->header.stamp + ros::Duration(0.05) < goal_publish_time_)) {
            return;
        }
        if (baseline_replan_pending_ &&
            distance3d(msg->position, fcu_odom_ptr->pose.pose.position) > max_command_distance_) {
            return;
        }
        if (accepted_trajectory_id_ != 0 && msg->trajectory_id < accepted_trajectory_id_) return;
        accepted_trajectory_id_ = msg->trajectory_id;
        baseline_replan_pending_ = false;
        last_planner_cmd = *msg;
        last_valid_cmd_time_ = ros::Time::now();
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
    target_pos_.x += offset_x_;
    target_pos_.y += offset_y_;
    target_initialized = true;

    if (use_home_xy_) {
        return_min_altitude_ = std::min(return_min_altitude_, target_pos_.z - 0.05);
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
    {
        std::lock_guard<std::mutex> lock(planner_cmd_mutex_);
        has_planner_cmd = false;
        goal_publish_time_ = goal.header.stamp;
        last_valid_cmd_time_ = ros::Time(0);
    }
    planner_goal_pub.publish(goal);
    baseline_stall_start_ = ros::Time(0);
    baseline_cloud_lost_ = false;

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
    stop_hold_start_ = ros::Time(0);
    publishDebugEvent("return_goal", "waiting for fresh matched EGO trajectory");
}

void PlanningControl::deactivateReturnPlanner() {
    std::lock_guard<std::mutex> lock(planner_cmd_mutex_);
    return_goal_published_ = false;
    has_planner_cmd = false;
    baseline_stall_start_ = ros::Time(0);
    accepted_trajectory_id_ = 0;
    return_scan_start_ = ros::Time(0);
}

NodeStatus PlanningControl::onStart() {
    offset_z = getInput<double>("offset_z").value();
    offset_x_ = getInput<double>("offset_x").value();
    offset_y_ = getInput<double>("offset_y").value();
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
    planner_wait_timeout_ = getInput<double>("planner_wait_timeout").value();
    planner_cmd_timeout_ = getInput<double>("planner_cmd_timeout").value();
    cloud_timeout_ = std::max(0.1, getInput<double>("cloud_timeout").value());
    stop_speed_threshold_ = std::max(0.0, getInput<double>("stop_speed_threshold").value());
    stop_hold_time_ = std::max(0.0, getInput<double>("stop_hold_time").value());
    goal_match_tolerance_ = getInput<double>("goal_match_tolerance").value();
    max_command_distance_ = getInput<double>("max_command_distance").value();
    const auto final_yaw = getInput<double>("final_yaw");
    final_yaw_enabled_ = use_home_xy_ && static_cast<bool>(final_yaw);
    final_yaw_active_ = false;
    return_heading_pending_ = false;
    return_heading_aligned_ = false;
    return_hold_active_ = false;
    if (final_yaw_enabled_) final_yaw_ = normalizeYaw(final_yaw.value());
    final_yaw_rate_ = std::max(0.05, std::abs(getInput<double>("final_yaw_rate").value()));

    initial_yaw_ = PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation);
    current_yaw_sp_ = initial_yaw_;
    last_loop_time_ = ros::Time::now();
    target_initialized = false;
    baseline_cloud_lost_ = false;
    {
        std::lock_guard<std::mutex> lock(planner_cmd_mutex_);
        return_goal_published_ = false;
        baseline_replan_pending_ = false;
        accepted_trajectory_id_ = 0;
        has_planner_cmd = false;
    }
    stop_hold_start_ = ros::Time(0);
    return_scan_start_ = ros::Time(0);
    arrival_stop_hold_start_ = ros::Time(0);
    arrival_hold_start_ = ros::Time(0);

    if (use_home_xy_) {
        return_min_altitude_ = fcu_odom_ptr->pose.pose.position.z - 0.05;
        return_mode_start_ = ros::Time::now();
        {
            std::lock_guard<std::mutex> lock(cloud_mutex_);
            return_map_time_ = ros::Time(0);
        }
        publishReturnMapMode(true);
    }

    if (!hasFreshCloud() || !tryInitializeTarget()) sendHoverCommand();
    if (use_home_xy_) {
        publishReturnBrake();
    }
    return NodeStatus::RUNNING;
}

NodeStatus PlanningControl::onRunning() {
    if (use_home_xy_) {
        if (target_initialized) {
            return_min_altitude_ = std::max(return_min_altitude_,
                std::min(fcu_odom_ptr->pose.pose.position.z, target_pos_.z) - 0.05);
        }
        publishReturnMapMode(true);
    }
    if (!target_initialized && (!hasFreshCloud() || !tryInitializeTarget())) {
        sendHoverCommand();
        return NodeStatus::RUNNING;
    }
    return use_home_xy_ ? runSafeReturnPlanner() : runBaselinePlanner();
}

NodeStatus PlanningControl::runBaselinePlanner() {
    if (!hasFreshCloud()) {
        baseline_cloud_lost_ = true;
        sendHoverCommand();
        ROS_WARN_THROTTLE(1.0, "[%s]: Planning cloud unavailable or stale; holding",
                          name().c_str());
        return NodeStatus::RUNNING;
    }
    if (baseline_cloud_lost_) {
        // The old trajectory was generated without the current obstacle map.
        // Publish a new goal before allowing planner commands through again.
        auto goal = getInput<Position3D>("goal");
        if (!goal) return NodeStatus::FAILURE;
        {
            std::lock_guard<std::mutex> lock(planner_cmd_mutex_);
            minimum_trajectory_id_ = latest_seen_trajectory_id_;
            baseline_replan_pending_ = true;
        }
        initializeTarget(goal.value());
        baseline_cloud_lost_ = false;
        sendHoverCommand();
        return NodeStatus::RUNNING;
    }
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
    const ros::Time now = ros::Time::now();
    quadrotor_msgs::PositionCommand command;
    bool has_command;
    ros::Time last_command_time;
    {
        std::lock_guard<std::mutex> lock(planner_cmd_mutex_);
        has_command = has_planner_cmd;
        command = last_planner_cmd;
        last_command_time = has_command ? last_valid_cmd_time_ : goal_publish_time_;
    }
    if (last_command_time.isZero() ||
        (now - last_command_time).toSec() > planner_cmd_timeout_) {
        sendHoverCommand();
        if (!last_command_time.isZero() &&
            (now - last_command_time).toSec() > planner_wait_timeout_) {
            ROS_ERROR("[%s]: Planner command timed out during normal flight", name().c_str());
            return NodeStatus::FAILURE;
        }
        return NodeStatus::RUNNING;
    }
    if (!has_command) return NodeStatus::RUNNING;

    const bool terminal_away_from_goal =
        distance3d(command.position, target) > tolerance &&
        std::hypot(command.velocity.x, command.velocity.y,
                   command.velocity.z) < 0.05;
    if (terminal_away_from_goal) {
        if (baseline_stall_start_.isZero()) baseline_stall_start_ = now;
        if ((now - baseline_stall_start_).toSec() > planner_wait_timeout_) {
            sendHoverCommand();
            ROS_ERROR("[%s]: Planner stopped at an intermediate trajectory endpoint", name().c_str());
            return NodeStatus::FAILURE;
        }
    } else {
        baseline_stall_start_ = ros::Time(0);
    }
    applyPlannerCommand(command);
    return NodeStatus::RUNNING;
}

NodeStatus PlanningControl::runSafeReturnPlanner() {
    geometry_msgs::Point target;
    target.x = target_pos_.x;
    target.y = target_pos_.y;
    target.z = target_pos_.z;
    const bool at_home = checkArrival(target, tolerance);
    const ros::Time now = ros::Time::now();
    const double measured_speed = mct->getMeasuredSpeed();
    const bool stopped = std::isfinite(measured_speed) && measured_speed <= stop_speed_threshold_;
    if (!at_home || !stopped) arrival_hold_start_ = ros::Time(0);
    // Once the return point is reached, latch the yaw phase.  Do not leave it
    // just because the position briefly moves outside the tolerance while the
    // vehicle is braking or yawing; that used to cause repeated brake/turn
    // cycles near the landing point.
    if (final_yaw_enabled_ && at_home && !final_yaw_active_) {
        final_yaw_active_ = true;
        arrival_stop_hold_start_ = now;
        ROS_INFO("[%s]: Enter continuous final-yaw phase", name().c_str());
    }
    const bool yaw_ready = !final_yaw_enabled_ || std::abs(normalizeYaw(final_yaw_ -
        PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation))) <= 0.15;
    if (at_home && stopped && yaw_ready) {
        if (arrival_hold_start_.isZero()) arrival_hold_start_ = now;
        if ((now - arrival_hold_start_).toSec() >= stop_hold_time_) return finishReturn();
    } else {
        arrival_hold_start_ = ros::Time(0);
    }

    if (final_yaw_active_) {
        // Keep all position axes held while yawing, without restarting the
        // turn on small odometry fluctuations.
        publishReturnYawHold(final_yaw_, final_yaw_rate_, true);
        if (at_home && yaw_ready && stopped) {
            if (arrival_hold_start_.isZero()) arrival_hold_start_ = now;
            if ((now - arrival_hold_start_).toSec() >= stop_hold_time_) return finishReturn();
        } else {
            arrival_hold_start_ = ros::Time(0);
        }
        return NodeStatus::RUNNING;
    }

    if (at_home) {
        publishReturnBrake();
        return NodeStatus::RUNNING;
    }

    if (!hasFreshCloud()) {
        publishReturnBrake();
        ROS_WARN_THROTTLE(1.0, "[%s]: Return cloud unavailable or stale; holding",
                          name().c_str());
        deactivateReturnPlanner();
        stop_hold_start_ = ros::Time(0);
        return NodeStatus::RUNNING;
    }

    bool goal_published;
    {
        std::lock_guard<std::mutex> lock(planner_cmd_mutex_);
        goal_published = return_goal_published_;
    }
    if (!goal_published) {
        // A pending heading change is a single continuous yaw-only phase.  Do
        // not emit a brake command before every yaw command in this branch.
        if (return_heading_pending_) {
            const double error = normalizeYaw(return_heading_target_ -
                PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation));
            if (std::abs(error) > 0.20) {
                stop_hold_start_ = ros::Time(0);
                return_scan_start_ = ros::Time(0);
                publishReturnYawHold(return_heading_target_, yaw_rate_limit_, false);
                return NodeStatus::RUNNING;
            }
            return_heading_pending_ = false;
            return_heading_aligned_ = true;
            return_scan_start_ = ros::Time(0);
        }
        publishReturnBrake();
        if (!stopped) {
            stop_hold_start_ = ros::Time(0);
            return_scan_start_ = ros::Time(0);
            return NodeStatus::RUNNING;
        }
        if (stop_hold_start_.isZero()) stop_hold_start_ = now;
        if (return_scan_start_.isZero()) return_scan_start_ = now;
        if ((now - stop_hold_start_).toSec() >= stop_hold_time_ &&
            hasCloudSince(return_scan_start_)) {
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
        publishReturnBrake();
        if ((ros::Time::now() - publish_time).toSec() > planner_wait_timeout_) {
            ROS_ERROR("[%s]: Timed out waiting for fresh return trajectory", name().c_str());
            publishReturnGoal();
        }
        return NodeStatus::RUNNING;
    }
    if ((ros::Time::now() - command_time).toSec() > planner_cmd_timeout_ ||
        distance3d(command.position, fcu_odom_ptr->pose.pose.position) > max_command_distance_ ||
        command.position.z < return_min_altitude_ - 0.02) {
        publishReturnBrake();
        ROS_ERROR("[%s]: Return command stale, too far away or below altitude floor; replan", name().c_str());
        deactivateReturnPlanner();
        return NodeStatus::RUNNING;
    }

    if (!return_heading_aligned_) {
        if (std::hypot(command.velocity.x, command.velocity.y) <= 0.08) {
            // A valid return trajectory may climb before it starts translating.
            // Keep that vertical segment running; align when its horizontal leg starts.
            applyPlannerCommand(command);
            return NodeStatus::RUNNING;
        }
        const double bearing = std::atan2(command.velocity.y, command.velocity.x);
        if (std::abs(normalizeYaw(bearing -
            PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation))) > 0.35) {
            return_heading_target_ = bearing;
            return_heading_pending_ = true;
            publishReturnBrake();
            deactivateReturnPlanner();
            ROS_WARN("[%s]: Aligning once with the first return trajectory segment",
                              name().c_str());
            return NodeStatus::RUNNING;
        }
        return_heading_aligned_ = true;
    }

    applyPlannerCommand(command);
    return NodeStatus::RUNNING;
}

void PlanningControl::publishReturnYawHold(double target_yaw, double yaw_rate,
                                           bool hold_return_position) {
    resetHoverCapture();
    const ros::Time now = ros::Time::now();
    if (!hold_return_position && !return_hold_active_) {
        return_hold_position_ = fcu_odom_ptr->pose.pose.position;
        return_hold_active_ = true;
        current_yaw_sp_ = PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation);
        last_loop_time_ = now;
    }
    const double dt = std::clamp((now - last_loop_time_).toSec(), 0.0, 0.1);
    last_loop_time_ = now;
    const double step = std::max(0.05, yaw_rate) * dt;
    const double error = normalizeYaw(target_yaw - current_yaw_sp_);
    current_yaw_sp_ = std::abs(error) <= step ? target_yaw :
        normalizeYaw(current_yaw_sp_ + std::copysign(step, error));

    geometry_msgs::Point hold_position;
    if (hold_return_position) {
        hold_position.x = target_pos_.x;
        hold_position.y = target_pos_.y;
        hold_position.z = target_pos_.z;
    } else {
        // Turn at the stopped pose, not at a moving or previously planned point.
        // Zero velocity alone allowed drift toward the wall during this phase.
        hold_position = return_hold_position_;
        hold_position.z = std::max(hold_position.z, return_min_altitude_ + 0.05);
    }
    publishPosition(hold_position, current_yaw_sp_);
}

void PlanningControl::publishReturnBrake() {
    resetHoverCapture();
    const double speed = mct->getMeasuredSpeed();
    if (!return_hold_active_ && std::isfinite(speed) && speed <= stop_speed_threshold_) {
        return_hold_position_ = fcu_odom_ptr->pose.pose.position;
        return_hold_active_ = true;
        current_yaw_sp_ = PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation);
        last_loop_time_ = ros::Time::now();
    }
    if (return_hold_active_) {
        return_hold_position_.z = std::max(return_hold_position_.z,
                                            return_min_altitude_ + 0.05);
        publishPosition(return_hold_position_, current_yaw_sp_);
        return;
    }
    // While still moving, brake without attracting the vehicle to an old position.
    mavros_msgs::PositionTarget cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = "return_brake";
    cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    cmd.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                    mavros_msgs::PositionTarget::IGNORE_PY |
                    mavros_msgs::PositionTarget::IGNORE_PZ |
                    mavros_msgs::PositionTarget::IGNORE_YAW |
                    mavros_msgs::PositionTarget::IGNORE_AFX |
                    mavros_msgs::PositionTarget::IGNORE_AFY |
                    mavros_msgs::PositionTarget::IGNORE_AFZ;
    cmd.velocity.x = cmd.velocity.y = cmd.velocity.z = 0.0;
    cmd.yaw_rate = 0.0;
    mct->updateCommand(cmd);
}

NodeStatus PlanningControl::finishReturn() {
    const auto& p = fcu_odom_ptr->pose.pose.position;
    Position3D final_position;
    final_position.x = p.x;
    final_position.y = p.y;
    final_position.z = p.z;
    setOutput("final_position", final_position);
    publishReturnBrake();
    deactivateReturnPlanner();
    publishReturnMapMode(false);
    publishDebugEvent("return_success", "home target reached");
    return NodeStatus::SUCCESS;
}

void PlanningControl::applyPlannerCommand(
    const quadrotor_msgs::PositionCommand& planner_command) {
    if (use_home_xy_) return_hold_active_ = false;
    double ego_yaw = initial_yaw_;
    if (control_yaw || final_yaw_active_) {
        const ros::Time now = ros::Time::now();
        const double dt = std::max((now - last_loop_time_).toSec(), 0.001);
        last_loop_time_ = now;
        const double dx = target_pos_.x - fcu_odom_ptr->pose.pose.position.x;
        const double dy = target_pos_.y - fcu_odom_ptr->pose.pose.position.y;
        if (final_yaw_active_ || std::hypot(dx, dy) > 0.2) {
            const double vx = planner_command.velocity.x;
            const double vy = planner_command.velocity.y;
            const double target_yaw = final_yaw_active_ ? final_yaw_ :
                (use_home_xy_ ? (std::hypot(vx, vy) > 0.08
                    ? std::atan2(vy, vx) : current_yaw_sp_) : std::atan2(dy, dx));
            const double yaw_error = normalizeYaw(target_yaw - current_yaw_sp_);
            const double rate = final_yaw_active_ ? final_yaw_rate_ : std::max(0.05, yaw_rate_limit_);
            const double max_step = rate * (use_home_xy_ ? std::min(dt, 0.1) : dt);
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
    if (use_home_xy_) {
        deactivateReturnPlanner();
        publishReturnMapMode(false);
    }
    DroneActionNode::onHalted();
}

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<PlanningControl>("PlanningControl");
}
