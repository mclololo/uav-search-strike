/****************************************************************************
 * @file       scan_for_target.cpp
 * @brief      ScanForTarget 节点实现
 ******************************************************************************/
#include "nodes/perception/scan_for_target.h"

#include <algorithm>
#include <cmath>

ScanForTarget::ScanForTarget(const std::string& name, const NodeConfig& config)
    : TargetDetectorBase(name, config) {}

PortsList ScanForTarget::providedPorts() {
    auto ports = baseProvidedPorts();
    ports.insert(InputPort<Position3D>("staging_position", "World search point for stationary balloon scans"));
    ports.insert(InputPort<double>("yaw_rate", 0.35, "扫描偏航角速度(rad/s)"));
    ports.insert(InputPort<double>("timeout", 8.0, "扫描超时时间(s)"));
    ports.insert(InputPort<double>("center_gate_x_ratio", 0.22, "水平居中窗口占图像宽度比例"));
    ports.insert(InputPort<double>("center_gate_y_ratio", 0.35, "垂直居中窗口占图像高度比例"));
    ports.insert(InputPort<int>("stable_frames", 3, "连续稳定检测帧数"));
    ports.insert(InputPort<double>("detection_hold_time", 0.6, "稳定检测保持时间(s)"));
    ports.insert(InputPort<double>("phase_duration", 2.5, "单个搜索阶段持续时间(s)"));
    ports.insert(InputPort<double>("lateral_velocity", 0.18, "侧移搜索速度(m/s)"));
    ports.insert(InputPort<double>("backward_velocity", 0.12, "后退搜索速度(m/s)"));
    return ports;
}

NodeStatus ScanForTarget::onStart() {
    NodeStatus base_status = TargetDetectorBase::onStart();

    yaw_rate_ = std::abs(getInput<double>("yaw_rate").value());
    timeout_ = getInput<double>("timeout").value();
    center_gate_x_ratio_ = getInput<double>("center_gate_x_ratio").value();
    center_gate_y_ratio_ = getInput<double>("center_gate_y_ratio").value();
    stable_frames_required_ = std::max(1, getInput<int>("stable_frames").value());
    detection_hold_time_ = getInput<double>("detection_hold_time").value();
    phase_duration_ = getInput<double>("phase_duration").value();
    lateral_velocity_ = getInput<double>("lateral_velocity").value();
    backward_velocity_ = getInput<double>("backward_velocity").value();

    hold_staging_position_ = false;
    const auto staging = getInput<Position3D>("staging_position");
    if (staging) {
        staging_position_.x = staging.value().x;
        staging_position_.y = staging.value().y;
        staging_position_.z = staging.value().z;
        if (!std::isfinite(staging_position_.x) || !std::isfinite(staging_position_.y) ||
            !std::isfinite(staging_position_.z)) {
            return NodeStatus::FAILURE;
        }
        hold_staging_position_ = true;
    }

    stable_frames_ = 0;
    has_stable_detection_ = false;
    start_time_ = ros::Time::now();
    phase_start_time_ = start_time_;
    first_stable_time_ = ros::Time(0);
    current_phase_ = SearchPhase::RotateLeft;

    ROS_INFO("[%s]: Scan started for %s, yaw_rate=%.2f timeout=%.1f",
             name().c_str(), target_class.c_str(), yaw_rate_, timeout_);
    publishDebugEvent("scan_start", std::string("target=") + target_class + " phase=" + phaseName());

    return base_status;
}


void ScanForTarget::advanceSearchPhase() {
    switch (current_phase_) {
        case SearchPhase::RotateLeft:
            current_phase_ = SearchPhase::StrafeLeft;
            break;
        case SearchPhase::StrafeLeft:
            current_phase_ = SearchPhase::RotateRight;
            break;
        case SearchPhase::RotateRight:
            current_phase_ = SearchPhase::StrafeRight;
            break;
        case SearchPhase::StrafeRight:
            current_phase_ = SearchPhase::BackOff;
            break;
        case SearchPhase::BackOff:
            current_phase_ = SearchPhase::RotateLeft;
            break;
    }
    phase_start_time_ = ros::Time::now();
    publishDebugEvent("scan_phase", std::string("phase=") + phaseName());
}

const char* ScanForTarget::phaseName() const {
    switch (current_phase_) {
        case SearchPhase::RotateLeft:
            return "rotate_left";
        case SearchPhase::StrafeLeft:
            return "strafe_left";
        case SearchPhase::RotateRight:
            return "rotate_right";
        case SearchPhase::StrafeRight:
            return "strafe_right";
        case SearchPhase::BackOff:
            return "back_off";
    }
    return "unknown";
}

NodeStatus ScanForTarget::onRunning() {
    if ((ros::Time::now() - start_time_).toSec() > timeout_) {
        sendHoverCommand();
        ROS_WARN("[%s]: Scan timeout, target %s not centered.", name().c_str(), target_class.c_str());
        publishDebugEvent("scan_timeout", std::string("target=") + target_class + " not acquired");
        return NodeStatus::FAILURE;
    }
    return TargetDetectorBase::onRunning();
}

void ScanForTarget::publishSearchCommand(double vx, double vy, double yaw_rate) {
    if (hold_staging_position_) {
        const double vz = std::clamp(
            1.5 * (staging_position_.z - fcu_odom_ptr->pose.pose.position.z), -0.15, 0.15);
        publishXYHoldWithYawRate(staging_position_, vz, yaw_rate);
    } else {
        publishVelocityWithYawRate(vx, vy, 0.0, yaw_rate);
    }
}

NodeStatus ScanForTarget::handleDetectedObject(const common_msgs::Obj& obj) {
    const double img_cx = 320.0;
    const double img_cy = 240.0;
    const double img_w = 640.0;
    const double img_h = 480.0;

    double err_x = static_cast<double>(obj.center_x) - img_cx;
    double err_y = static_cast<double>(obj.center_y) - img_cy;
    double gate_x = img_w * center_gate_x_ratio_;
    double gate_y = img_h * center_gate_y_ratio_;
    bool centered = std::abs(err_x) <= gate_x && std::abs(err_y) <= gate_y;

    // 只要稳定看到目标，就把后续精确对准交给 TargetTracker + VisualServo，
    // 不在扫描阶段卡死等待“完全居中”。
    stable_frames_++;

    double steer = 0.0;
    if (!centered) {
        steer = (std::abs(err_x) > 8.0) ? -std::copysign(yaw_rate_, err_x) : 0.0;
    }
    publishSearchCommand(0.0, 0.0, steer);

    if (stable_frames_ >= stable_frames_required_) {
        if (!has_stable_detection_) {
            has_stable_detection_ = true;
            first_stable_time_ = ros::Time::now();
        }

        // 一旦目标已经进入可跟踪的中心窗口，就立刻交棒给后续节点。
        // 否则在检测抖动时容易错过短暂但有效的观测机会。
        if (centered) {
            if (!hold_staging_position_) sendHoverCommand();
            ROS_INFO("[%s]: Target %s centered, handing off to tracker immediately.",
                     name().c_str(), target_class.c_str());
            publishDebugEvent("scan_success", std::string("target=") + target_class + " centered_immediate");
            return NodeStatus::SUCCESS;
        }

        double held = (ros::Time::now() - first_stable_time_).toSec();
        if (held >= detection_hold_time_) {
            if (!hold_staging_position_) sendHoverCommand();
            ROS_INFO("[%s]: Target %s detected%s, handing off to tracker.",
                     name().c_str(), target_class.c_str(), centered ? " and centered" : "");
            publishDebugEvent("scan_success", std::string("target=") + target_class + (centered ? " centered" : " detected"));
            return NodeStatus::SUCCESS;
        }
    }

    ROS_INFO_THROTTLE(0.5,
                      "[%s]: scanning %s center=(%d,%d) err=(%.1f,%.1f) centered=%s stable=%d/%d",
                      name().c_str(), target_class.c_str(), obj.center_x, obj.center_y,
                      err_x, err_y, centered ? "yes" : "no", stable_frames_, stable_frames_required_);
    return NodeStatus::RUNNING;
}

NodeStatus ScanForTarget::handleNoDetection() {
    stable_frames_ = 0;
    has_stable_detection_ = false;

    if ((ros::Time::now() - phase_start_time_).toSec() >= phase_duration_) {
        advanceSearchPhase();
    }

    double vx = 0.0;
    double vy = 0.0;
    double yaw = 0.0;
    switch (current_phase_) {
        case SearchPhase::RotateLeft:
            yaw = yaw_rate_;
            break;
        case SearchPhase::StrafeLeft:
            vy = lateral_velocity_;
            yaw = yaw_rate_ * 0.5;
            break;
        case SearchPhase::RotateRight:
            yaw = -yaw_rate_;
            break;
        case SearchPhase::StrafeRight:
            vy = -lateral_velocity_;
            yaw = -yaw_rate_ * 0.5;
            break;
        case SearchPhase::BackOff:
            vx = -backward_velocity_;
            yaw = yaw_rate_ * 0.4;
            break;
    }

    publishSearchCommand(vx, vy, yaw);
    ROS_WARN_THROTTLE(1.0, "[%s]: No %s detected, search phase=%s.", name().c_str(), target_class.c_str(), phaseName());
    publishDebugEvent("scan_search", std::string("phase=") + phaseName());
    return NodeStatus::RUNNING;
}

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<ScanForTarget>("ScanForTarget");
}
