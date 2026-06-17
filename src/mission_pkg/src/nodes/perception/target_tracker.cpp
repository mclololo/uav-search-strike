/****************************************************************************
 * @file       target_tracker.cpp
 * @brief      TargetTracker 节点实现
 *
 * @details    简单像素跟踪：直接提取像素坐标+简单平滑
 *****************************************************************************/
#include "nodes/perception/target_tracker.h"

TargetTracker::TargetTracker(const std::string& name, const NodeConfig& config)
    : TargetDetectorBase(name, config) {}

PortsList TargetTracker::providedPorts() {
    auto ports = baseProvidedPorts();
    ports.insert(InputPort<int>("filter_window_size", 10, "像素坐标平滑窗口大小"));
    ports.insert(InputPort<double>("lost_target_grace_time", 0.8, "目标短暂丢失时保留上次目标的时间(s)"));
    ports.insert(OutputPort<PixelTarget>("pixel_target", "像素目标（中心+尺寸）"));
    return ports;
}

NodeStatus TargetTracker::onStart() {
    NodeStatus base_status = TargetDetectorBase::onStart();
    auto window_res = getInput<int>("filter_window_size");
    filter_window_size = window_res.value();
    auto grace_res = getInput<double>("lost_target_grace_time");
    lost_target_grace_time_ = grace_res.value();
    pixel_history.clear();
    has_last_valid_target_ = false;
    ROS_INFO("[%s]: Pixel tracking filter window: %d, grace_time: %.2fs",
             name().c_str(), filter_window_size, lost_target_grace_time_);
    return base_status;
}

NodeStatus TargetTracker::handleDetectedObject(const common_msgs::Obj& obj) {
    double pixel_width = obj.right_bottom_x - obj.left_top_x;
    double pixel_height = obj.right_bottom_y - obj.left_top_y;

    if (pixel_width <= 0 || pixel_height <= 0) {
        return handleNoDetection();
    }

    PixelTarget current_pixel(
        obj.center_x,
        obj.center_y,
        pixel_width,
        pixel_height,
        ros::Time::now().toSec(),
        true
    );

    pixel_history.push_back(current_pixel);
    while (static_cast<int>(pixel_history.size()) > filter_window_size) {
        pixel_history.pop_front();
    }

    PixelTarget smoothed = getSmoothedPixel();
    smoothed.timestamp = ros::Time::now().toSec();
    smoothed.is_detected = true;
    last_valid_target_ = smoothed;
    has_last_valid_target_ = true;
    setOutput("pixel_target", smoothed);

    ROS_INFO_THROTTLE(0.5, "[%s]: Tracking pixel target: center=(%.1f, %.1f), size=(%.1f x %.1f)",
                      name().c_str(), smoothed.center_x, smoothed.center_y,
                      smoothed.width, smoothed.height);
    publishDebugEvent("track_update", "center=(" + std::to_string(smoothed.center_x) + "," + std::to_string(smoothed.center_y) + ") area=" + std::to_string(smoothed.area()));

    return NodeStatus::RUNNING;
}

NodeStatus TargetTracker::handleNoDetection() {
    const double now = ros::Time::now().toSec();
    if (has_last_valid_target_ && (now - last_valid_target_.timestamp) <= lost_target_grace_time_) {
        PixelTarget held_target = last_valid_target_;
        held_target.timestamp = now;
        held_target.is_detected = true;
        setOutput("pixel_target", held_target);
        ROS_WARN_THROTTLE(1.0, "[%s]: Target temporarily lost, holding last valid target", name().c_str());
        publishDebugEvent("track_hold", "holding last valid target");
        return NodeStatus::RUNNING;
    }

    PixelTarget empty_target;
    empty_target.timestamp = now;
    empty_target.is_detected = false;
    setOutput("pixel_target", empty_target);
    pixel_history.clear();
    has_last_valid_target_ = false;
    ROS_WARN_THROTTLE(1.0, "[%s]: No target detected, publishing INVALID target", name().c_str());
    publishDebugEvent("track_lost", "no valid target");
    return NodeStatus::RUNNING;
}

PixelTarget TargetTracker::getSmoothedPixel() {
    if (pixel_history.empty()) {
        return PixelTarget(0, 0, 0, 0, ros::Time::now().toSec());
    }

    double sum_x = 0, sum_y = 0, sum_w = 0, sum_h = 0;
    for (const auto& px : pixel_history) {
        sum_x += px.center_x;
        sum_y += px.center_y;
        sum_w += px.width;
        sum_h += px.height;
    }

    int n = pixel_history.size();
    return PixelTarget(
        sum_x / n,
        sum_y / n,
        sum_w / n,
        sum_h / n,
        ros::Time::now().toSec()
    );
}

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<TargetTracker>("TargetTracker");
}
