/****************************************************************************
 * @file       visual_servo.cpp
 * @brief      Corridor-constrained balloon strike controller
 ******************************************************************************/
#include "nodes/action/visual_servo.h"

#include "utils/pose_utils.h"

#include <algorithm>
#include <cmath>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <stdexcept>
#include <vector>

namespace {
struct CloudPoint {
    float x;
    float y;
    float z;
    float image_x;
    float image_y;
    bool in_target;
};

double normalizeYaw(double yaw) {
    while (yaw > M_PI) yaw -= 2.0 * M_PI;
    while (yaw < -M_PI) yaw += 2.0 * M_PI;
    return yaw;
}

double positiveOr(double value, double fallback) {
    return std::isfinite(value) && value > 0.0 ? value : fallback;
}
}  // namespace

VisualServo::VisualServo(const std::string& name, const NodeConfig& config)
    : DroneActionNode(name, config) {
    auto nh = mct->getROSHandle();
    nh->param<double>("cx", img_cx_, 320.0);
    nh->param<double>("cy", img_cy_, 240.0);
    nh->param<int>("img_w", img_w_, 640);
    nh->param<int>("img_h", img_h_, 480);
    nh->param<double>("visual_servo/rgb_hfov_deg", rgb_hfov_deg_, 90.0);
    nh->param<double>("visual_servo/depth_image_y_sign", depth_image_y_sign_, 1.0);
    nh->param<double>("visual_servo/depth_image_z_sign", depth_image_z_sign_, 1.0);
    nh->param<double>("safety/x_min", boundary_x_min_, -10.0);
    nh->param<double>("safety/x_max", boundary_x_max_, 10.0);
    nh->param<double>("safety/y_min", boundary_y_min_, -10.0);
    nh->param<double>("safety/y_max", boundary_y_max_, 10.0);

    std::string depth_topic;
    nh->param<std::string>("visual_servo/depth_topic", depth_topic,
                           "/rflysim/sensor0/Depth_Cloud");
    depth_cloud_sub_ = nh->subscribe(depth_topic, 1, &VisualServo::depthCloudCallback, this);
}

PortsList VisualServo::providedPorts() {
    return {
        InputPort<PixelTarget>("pixel_target", "像素目标"),
        InputPort<double>("k_p_yaw_rate", 0.004, "偏航角速度P增益"),
        InputPort<double>("k_d_yaw_rate", 0.008, "偏航角速度D增益"),
        InputPort<double>("max_yaw_rate", 0.7, "最大偏航角速度"),
        InputPort<double>("deadzone_yaw", 8.0, "偏航死区(pixel)"),
        InputPort<double>("k_p_vz", 0.002, "垂直速度P增益"),
        InputPort<double>("k_d_vz", 0.006, "垂直速度D增益"),
        InputPort<double>("max_vz", 0.3, "最大垂直速度"),
        InputPort<double>("deadzone_vz", 10.0, "垂直死区(pixel)"),
        InputPort<double>("forward_velocity", 0.9, "常态接近速度"),
        InputPort<double>("sprint_velocity", 0.85, "直线冲刺速度"),
        InputPort<double>("sprint_time", 6.0, "冲刺最长时间"),
        InputPort<double>("hit_offset_x", 45.0, "侧桨打击像素偏置"),
        InputPort<double>("hit_offset_y", 0.0, "垂直像素偏置"),
        InputPort<double>("hit_offset_reference_depth", 0.80, "像素偏置标定深度"),
        InputPort<double>("max_hit_offset_ratio", 0.35, "动态偏置图像宽度上限"),
        InputPort<double>("approach_area_threshold", 0.035, "进入冲刺的目标面积占比"),
        InputPort<double>("post_hit_success_area_ratio", 0.025, "近距离丢失命中判据"),
        InputPort<double>("center_tolerance", 0.12, "目标对齐容差比例"),
        InputPort<double>("strike_stable_time", 0.15, "冲刺前稳定跟踪时间"),
        InputPort<double>("sprint_alignment_tolerance_px", 24.0, "侧桨冲刺前水平对准容差"),
        InputPort<double>("sprint_vertical_alignment_tolerance_px", 160.0, "冲刺前垂直可见范围"),
        InputPort<int>("sprint_alignment_stable_frames", 2, "侧桨冲刺前连续对准帧数"),
        InputPort<double>("sprint_altitude_tolerance", 0.04, "冲刺前高度容差"),
        InputPort<double>("sprint_vertical_speed_tolerance", 0.06, "冲刺前垂直速度容差"),
        InputPort<double>("sprint_altitude_hold_gain", 1.5, "冲刺高度保持增益"),
        InputPort<double>("sprint_max_vertical_correction", 0.08, "冲刺最大垂直修正速度"),
        InputPort<double>("sprint_target_loss_grace", 1.50, "冲刺中目标丢失后直线续航时间"),
        InputPort<double>("data_timeout", 0.30, "目标数据超时"),
        InputPort<double>("startup_grace_time", 1.0, "首帧目标等待宽限"),

        InputPort<double>("strike_altitude", 0.40, "冲刺高度"),
        InputPort<double>("descent_velocity", 0.22, "发现目标后的下降速度"),
        InputPort<double>("descent_tolerance", 0.05, "冲刺高度容差"),
        InputPort<double>("descent_timeout", 4.0, "下降阶段超时"),
        InputPort<double>("corridor_reference_length", 5.20, "搜索点至外边界参考长度"),
        InputPort<double>("corridor_yaw", "打击区无障碍长轴航向"),
        InputPort<double>("adaptive_corridor_min_length", 1.50, "兼容旧XML，不参与冲刺准入"),
        InputPort<double>("sprint_limit_ratio", 0.78, "冲刺区占参考长度比例"),
        InputPort<double>("wait_entry_ratio", 0.35, "进入等待区的剩余区间比例"),
        InputPort<double>("corridor_lateral_limit", 0.40, "兼容旧XML，不触发横向中止"),
        InputPort<double>("max_alignment_yaw_deviation", 0.15, "对准阶段相对走廊航向的最大偏差"),
        InputPort<double>("heading_hold_gain", 1.5, "直线航向保持增益"),
        InputPort<double>("heading_hold_max_rate", 0.30, "直线航向最大修正角速度"),
        InputPort<double>("wait_velocity", 0.16, "进入等待区速度"),
        InputPort<double>("wait_timeout", 12.0, "等待小车折返超时"),
        InputPort<double>("wait_no_target_timeout", 0.5,
                          "等待区连续未检测到气球时的返航超时"),
        InputPort<double>("wait_contact_distance", 0.42, "等待碰撞接触距离"),

        InputPort<double>("depth_timeout", 0.20, "深度点云新鲜度上限"),
        InputPort<double>("depth_min_valid_range", 0.12, "最小有效深度"),
        InputPort<int>("depth_min_points", 8, "有效点数下限"),
        InputPort<double>("target_depth_band", 0.22, "目标气球深度带"),
        InputPort<double>("target_depth_quantile", 0.30, "目标前景深度分位数"),
        InputPort<double>("target_bbox_margin_px", 5.0, "目标深度掩膜像素余量"),
        InputPort<double>("target_zone_near_distance", 0.15, "气球冲刺区近端越界容差"),
        InputPort<double>("target_zone_far_ratio", 0.80, "气球冲刺区远端占走廊比例"),
        InputPort<double>("target_zone_lateral_limit", 0.55, "气球冲刺区横向半宽"),
        InputPort<double>("target_zone_fallback_area_ratio", 0.002, "无深度时目标区面积后备阈值"),
        InputPort<double>("target_closing_speed_threshold", 0.15, "快速接近判据"),
        InputPort<double>("target_ttc_threshold", 2.0, "紧急打击预计接触时间"),
        InputPort<double>("target_intercept_margin_px", 12.0, "左桨打击带像素余量"),
        InputPort<double>("target_carrier_depth_margin", 0.55, "目标载车深度关联范围"),
        InputPort<double>("target_carrier_below_margin_px", 120.0, "目标载车向下关联范围"),
        InputPort<double>("corridor_half_width", 0.38, "深度走廊半宽"),
        InputPort<double>("corridor_half_height", 0.32, "深度走廊半高"),
        InputPort<double>("approach_clearance", 0.65, "兼容旧XML，不参与冲刺准入"),
        InputPort<double>("emergency_clearance", 0.48, "冲刺紧急净空"),
        InputPort<double>("contact_distance", 0.35, "气球接触深度"),
        InputPort<double>("braking_acceleration", 0.8, "实测制动减速度"),
        InputPort<double>("control_latency", 0.15, "控制链延迟"),
        InputPort<double>("min_attack_altitude", 0.30, "打击阶段最低高度"),
        InputPort<double>("altitude_guard_margin", 0.05, "最低高度保护余量"),
        InputPort<double>("altitude_recovery_speed", 0.18, "低高度恢复速度"),
        InputPort<double>("safety_predict_time", 0.45, "边界预测时间"),
        InputPort<double>("boundary_margin", 0.40, "墙面边界余量"),
        InputPort<double>("brake_speed_threshold", 0.12, "制动完成速度"),
        InputPort<double>("brake_hold_time", 0.30, "低速稳定时间"),
        InputPort<double>("brake_timeout", 1.80, "制动告警时间"),
        InputPort<double>("post_hit_settle_time", 0.45, "停稳交接时间"),
        InputPort<double>("post_hit_escape_yaw", 0.0, "命中制动时的世界坐标离墙方向"),
        InputPort<double>("post_hit_escape_velocity", 0.18, "命中制动时的离墙速度"),
        InputPort<double>("post_hit_escape_distance", 0.10, "命中制动时的离墙距离"),
        InputPort<double>("post_hit_escape_time", 0.60, "命中制动时的离墙时间上限"),
        InputPort<double>("post_hit_escape_yaw_rate", 0.12, "命中制动时的最大偏航速度"),
        InputPort<double>("recovery_delay", 0.15, "丢失后恢复延迟"),
        InputPort<double>("recovery_yaw_rate", 0.45, "恢复扫描角速度"),
        InputPort<double>("recovery_total_timeout", 3.0, "恢复总超时"),

        // Compatibility ports retained for existing hand-written XML files.
        InputPort<double>("side_hit_offset_px", 0.0, "兼容旧XML"),
        InputPort<double>("hit_area_ratio", 0.35, "兼容旧XML"),
        InputPort<double>("center_exit_factor", 1.5, "兼容旧XML"),
        InputPort<double>("post_hit_climb_height", 0.0, "兼容旧XML"),
        InputPort<double>("post_hit_climb_velocity", 0.0, "兼容旧XML"),
        InputPort<double>("post_hit_climb_timeout", 0.0, "兼容旧XML"),
        InputPort<double>("safety_margin", 0.40, "兼容旧XML"),
        InputPort<double>("sprint_lift_velocity", 0.0, "兼容旧XML"),
        InputPort<bool>("sprint_lock_visual_correction", false, "冲刺阶段是否锁定视觉航向修正"),
        InputPort<double>("post_hit_brake_velocity", 0.0, "兼容旧XML"),
        InputPort<double>("post_hit_brake_time", 1.80, "兼容旧XML"),
        InputPort<double>("post_hit_lift_height", 0.35, "post-hit relative lift height"),
        InputPort<double>("post_hit_lift_time", 0.45, "post-hit lift ramp time"),
        InputPort<double>("post_hit_lift_timeout", 1.0, "post-hit lift timeout"),
        InputPort<double>("post_hit_lift_tolerance", 0.12, "post-hit lift tolerance"),
        InputPort<double>("recovery_backoff_velocity", 0.0, "兼容旧XML"),
        InputPort<double>("recovery_backoff_time", 0.0, "兼容旧XML"),
        InputPort<double>("egress_timeout", 0.0, "兼容旧XML"),
        InputPort<double>("egress_tolerance", 0.0, "兼容旧XML"),
        InputPort<double>("egress_velocity", 0.0, "兼容旧XML"),
        InputPort<double>("egress_position_gain", 0.0, "兼容旧XML"),
        InputPort<double>("anchor_min_target_distance", 0.0, "兼容旧XML")
    };
}

NodeStatus VisualServo::onStart() {
    k_p_yaw_rate_ = getInput<double>("k_p_yaw_rate").value();
    k_d_yaw_rate_ = getInput<double>("k_d_yaw_rate").value();
    max_yaw_rate_ = getInput<double>("max_yaw_rate").value();
    deadzone_yaw_ = getInput<double>("deadzone_yaw").value();
    k_p_vz_ = getInput<double>("k_p_vz").value();
    k_d_vz_ = getInput<double>("k_d_vz").value();
    max_vz_ = getInput<double>("max_vz").value();
    deadzone_vz_ = getInput<double>("deadzone_vz").value();
    forward_velocity_ = getInput<double>("forward_velocity").value();
    sprint_velocity_ = getInput<double>("sprint_velocity").value();
    sprint_time_ = getInput<double>("sprint_time").value();
    sprint_lock_visual_correction_ = getInput<bool>("sprint_lock_visual_correction").value();
    hit_offset_x_ = getInput<double>("hit_offset_x").value();
    hit_offset_y_ = getInput<double>("hit_offset_y").value();
    hit_offset_reference_depth_ = getInput<double>("hit_offset_reference_depth").value();
    max_hit_offset_ratio_ = getInput<double>("max_hit_offset_ratio").value();
    approach_area_threshold_ = getInput<double>("approach_area_threshold").value();
    post_hit_success_area_ratio_ = getInput<double>("post_hit_success_area_ratio").value();
    center_tolerance_ = getInput<double>("center_tolerance").value();
    strike_stable_time_ = getInput<double>("strike_stable_time").value();
    sprint_alignment_tolerance_px_ = std::max(
        1.0, getInput<double>("sprint_alignment_tolerance_px").value());
    sprint_vertical_alignment_tolerance_px_ = std::max(
        1.0, getInput<double>("sprint_vertical_alignment_tolerance_px").value());
    sprint_alignment_stable_frames_ = std::max(
        1, getInput<int>("sprint_alignment_stable_frames").value());
    sprint_altitude_tolerance_ = std::max(
        0.01, getInput<double>("sprint_altitude_tolerance").value());
    sprint_vertical_speed_tolerance_ = std::max(
        0.01, getInput<double>("sprint_vertical_speed_tolerance").value());
    sprint_altitude_hold_gain_ = std::max(
        0.0, getInput<double>("sprint_altitude_hold_gain").value());
    sprint_max_vertical_correction_ = std::max(
        0.0, getInput<double>("sprint_max_vertical_correction").value());
    sprint_target_loss_grace_ = getInput<double>("sprint_target_loss_grace").value();
    data_timeout_ = getInput<double>("data_timeout").value();
    startup_grace_time_ = getInput<double>("startup_grace_time").value();
    max_alignment_yaw_deviation_ = std::max(
        0.0, getInput<double>("max_alignment_yaw_deviation").value());

    strike_altitude_ = getInput<double>("strike_altitude").value();
    descent_velocity_ = getInput<double>("descent_velocity").value();
    descent_tolerance_ = getInput<double>("descent_tolerance").value();
    descent_timeout_ = getInput<double>("descent_timeout").value();
    corridor_reference_length_ = getInput<double>("corridor_reference_length").value();
    const auto corridor_yaw = getInput<double>("corridor_yaw");
    has_configured_corridor_yaw_ = corridor_yaw && std::isfinite(corridor_yaw.value());
    if (has_configured_corridor_yaw_) {
        configured_corridor_yaw_ = normalizeYaw(corridor_yaw.value());
    }
    sprint_limit_ratio_ = std::clamp(getInput<double>("sprint_limit_ratio").value(), 0.20, 0.95);
    wait_entry_ratio_ = std::clamp(getInput<double>("wait_entry_ratio").value(), 0.0, 0.90);
    heading_hold_gain_ = getInput<double>("heading_hold_gain").value();
    heading_hold_max_rate_ = getInput<double>("heading_hold_max_rate").value();
    wait_velocity_ = getInput<double>("wait_velocity").value();
    wait_timeout_ = getInput<double>("wait_timeout").value();
    wait_no_target_timeout_ = std::max(
        0.0, getInput<double>("wait_no_target_timeout").value());
    wait_contact_distance_ = getInput<double>("wait_contact_distance").value();

    depth_timeout_ = getInput<double>("depth_timeout").value();
    depth_min_valid_range_ = getInput<double>("depth_min_valid_range").value();
    depth_min_points_ = getInput<int>("depth_min_points").value();
    target_depth_band_ = getInput<double>("target_depth_band").value();
    target_depth_quantile_ = std::clamp(getInput<double>("target_depth_quantile").value(), 0.05, 0.50);
    target_bbox_margin_px_ = getInput<double>("target_bbox_margin_px").value();
    target_zone_near_distance_ = std::max(
        0.0, getInput<double>("target_zone_near_distance").value());
    target_zone_far_ratio_ = std::clamp(
        getInput<double>("target_zone_far_ratio").value(), 0.20, 1.0);
    target_zone_lateral_limit_ = std::max(
        0.10, getInput<double>("target_zone_lateral_limit").value());
    target_zone_fallback_area_ratio_ = std::max(
        0.0, getInput<double>("target_zone_fallback_area_ratio").value());
    target_closing_speed_threshold_ = std::max(
        0.0, getInput<double>("target_closing_speed_threshold").value());
    target_ttc_threshold_ = std::max(
        0.1, getInput<double>("target_ttc_threshold").value());
    target_intercept_margin_px_ = std::max(
        0.0, getInput<double>("target_intercept_margin_px").value());
    target_carrier_depth_margin_ = std::max(
        0.0, getInput<double>("target_carrier_depth_margin").value());
    target_carrier_below_margin_px_ = std::max(
        0.0, getInput<double>("target_carrier_below_margin_px").value());
    corridor_half_width_ = getInput<double>("corridor_half_width").value();
    corridor_half_height_ = getInput<double>("corridor_half_height").value();
    emergency_clearance_ = getInput<double>("emergency_clearance").value();
    contact_distance_ = getInput<double>("contact_distance").value();
    braking_acceleration_ = getInput<double>("braking_acceleration").value();
    control_latency_ = getInput<double>("control_latency").value();
    min_attack_altitude_ = getInput<double>("min_attack_altitude").value();
    altitude_guard_margin_ = getInput<double>("altitude_guard_margin").value();
    altitude_recovery_speed_ = getInput<double>("altitude_recovery_speed").value();
    safety_predict_time_ = getInput<double>("safety_predict_time").value();
    boundary_margin_ = getInput<double>("boundary_margin").value();
    brake_speed_threshold_ = getInput<double>("brake_speed_threshold").value();
    brake_hold_time_ = getInput<double>("brake_hold_time").value();
    brake_timeout_ = getInput<double>("brake_timeout").value();
    post_hit_lift_height_ = std::max(
        0.0, getInput<double>("post_hit_lift_height").value());
    post_hit_lift_time_ = std::max(
        0.05, getInput<double>("post_hit_lift_time").value());
    post_hit_lift_timeout_ = std::max(
        post_hit_lift_time_, getInput<double>("post_hit_lift_timeout").value());
    post_hit_lift_tolerance_ = std::max(
        0.01, getInput<double>("post_hit_lift_tolerance").value());
    post_hit_settle_time_ = getInput<double>("post_hit_settle_time").value();
    post_hit_escape_yaw_ = normalizeYaw(getInput<double>("post_hit_escape_yaw").value());
    post_hit_escape_velocity_ = std::max(
        0.0, getInput<double>("post_hit_escape_velocity").value());
    post_hit_escape_distance_ = std::max(
        0.0, getInput<double>("post_hit_escape_distance").value());
    post_hit_escape_time_ = std::max(
        0.0, getInput<double>("post_hit_escape_time").value());
    post_hit_escape_yaw_rate_ = std::max(
        0.0, getInput<double>("post_hit_escape_yaw_rate").value());
    recovery_delay_ = getInput<double>("recovery_delay").value();
    recovery_yaw_rate_ = getInput<double>("recovery_yaw_rate").value();
    recovery_total_timeout_ = getInput<double>("recovery_total_timeout").value();

    strike_altitude_ = std::max(strike_altitude_, min_attack_altitude_ + altitude_guard_margin_);
    corridor_reference_length_ = positiveOr(corridor_reference_length_, 5.20);
    corridor_limit_distance_ = corridor_reference_length_ * sprint_limit_ratio_;
    wait_entry_distance_ = corridor_reference_length_ * (1.0 - sprint_limit_ratio_) * wait_entry_ratio_;
    const double hfov_rad = std::clamp(rgb_hfov_deg_, 1.0, 179.0) * M_PI / 180.0;
    rgb_focal_px_ = 0.5 * img_w_ / std::tan(0.5 * hfov_rad);

    state_ = ServoState::APPROACHING;
    brake_outcome_ = BrakeOutcome::ABORT;
    has_ever_received_data_ = false;
    close_target_seen_ = false;
    // VisualServo is started only after PlanningControl/GoTo reaches the search point.
    // Keep the depth switch local to the strike phase; it does not disable EGO-Planner's
    // /cloud_registered obstacle input while navigating to the search point.
    setDepthAssistActive(false);
    max_area_ratio_ = 0.0;
    align_stable_frames_ = 0;
    prev_err_x_ = 0.0;
    prev_err_y_ = 0.0;
    node_start_time_ = state_start_time_ = prev_time_ = ros::Time::now();
    stable_track_start_ = ros::Time(0);
    brake_hold_start_ = ros::Time(0);
    lift_start_time_ = ros::Time(0);
    wait_last_target_time_ = ros::Time(0);
    post_hit_lift_completed_ = false;
    {
        std::lock_guard<std::mutex> lock(depth_mutex_);
        target_closing_speed_ = 0.0;
        previous_target_depth_ = std::numeric_limits<double>::infinity();
        previous_target_depth_time_ = ros::Time(0);
    }

    geometry_valid_ = fcu_odom_ptr && fcu_odom_ptr->header.stamp != ros::Time(0);
    if (geometry_valid_) {
        corridor_origin_ = fcu_odom_ptr->pose.pose.position;
        route_yaw_ = has_configured_corridor_yaw_
            ? configured_corridor_yaw_
            : PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation);
        if (fcu_odom_ptr->pose.pose.position.z > strike_altitude_ + descent_tolerance_) {
            state_ = ServoState::DESCENDING;
        }
    }
    updateTargetData();

    const double start_altitude = geometry_valid_
        ? fcu_odom_ptr->pose.pose.position.z : std::numeric_limits<double>::quiet_NaN();
    ROS_INFO("[%s]: Strike corridor started: yaw=%.2f length=%.2f limit=%.2f wait_entry=%.2f z=%.2f target_z=%.2f",
             name().c_str(), route_yaw_, corridor_reference_length_, corridor_limit_distance_,
             wait_entry_distance_, start_altitude, strike_altitude_);
    publishDebugEvent("strike_start", "target acquired -> descend -> side align -> straight sprint");
    return NodeStatus::RUNNING;
}

NodeStatus VisualServo::onRunning() {
    updateTargetData();

    if (!geometry_valid_ && fcu_odom_ptr && fcu_odom_ptr->header.stamp != ros::Time(0)) {
        corridor_origin_ = fcu_odom_ptr->pose.pose.position;
        route_yaw_ = has_configured_corridor_yaw_
            ? configured_corridor_yaw_
            : PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation);
        geometry_valid_ = true;
        if (fcu_odom_ptr->pose.pose.position.z > strike_altitude_ + descent_tolerance_) {
            state_ = ServoState::DESCENDING;
            state_start_time_ = ros::Time::now();
        }
    }
    if (!geometry_valid_) {
        sendHoverCommand();
        return NodeStatus::RUNNING;
    }

    switch (state_) {
        case ServoState::BRAKING:
            return executeBrake();
        case ServoState::ENTERING_WAIT:
            return executeWaitEntry();
        case ServoState::WAITING:
            return executeWaiting();
        case ServoState::SETTLING:
            return finishAfterSettle();
        case ServoState::LIFTING:
            return executePostHitLift();
        case ServoState::RECOVERY: {
            PixelTarget recovered_target;
            double recovered_age;
            if (getTargetSnapshot(recovered_target, recovered_age)) {
                state_ = ServoState::APPROACHING;
                state_start_time_ = ros::Time::now();
                align_stable_frames_ = 0;
                stable_track_start_ = ros::Time(0);
                publishDebugEvent("strike_state", "RECOVERY -> SIDE ALIGN");
            } else {
                return executeRecovery();
            }
            break;
        }
        default:
            break;
    }

    const ros::Time now = ros::Time::now();
    double dt = std::max((now - prev_time_).toSec(), 0.001);
    prev_time_ = now;

    PixelTarget target;
    double target_age = std::numeric_limits<double>::infinity();
    if (!getTargetSnapshot(target, target_age)) {
        if (state_ == ServoState::SPRINTING) {
            return executeSprint(false, target_age, 0.0, headingHoldYawRate());
        }
        publishVelocityWithYawRate(0, 0, 0, headingHoldYawRate());
        const double elapsed = (now - node_start_time_).toSec();
        if (elapsed < startup_grace_time_ || target_age < recovery_delay_) {
            return NodeStatus::RUNNING;
        }
        return startRecovery();
    }

    const double image_area = static_cast<double>(img_w_) * img_h_;
    const double area_ratio = target.area() / image_area;
    max_area_ratio_ = std::max(max_area_ratio_, area_ratio);

    const double desired_center_x = desiredHitCenterX();
    const double err_x = target.center_x - desired_center_x;
    const double err_y = target.center_y - (img_cy_ + hit_offset_y_);
    const double yaw_rate = computeYawRate(err_x, dt);
    const double alignment_yaw_rate = alignmentYawRate(yaw_rate);
    const double vz = computeVz(err_y, dt);

    switch (state_) {
        case ServoState::ALIGN: {
            publishGuardedVelocity(0, 0, vz, alignment_yaw_rate, 0.0, false);
            if (isCentered(target, desired_center_x)) {
                ++align_stable_frames_;
                if (align_stable_frames_ >= kAlignStableFrames) {
                    state_ = ServoState::APPROACHING;
                    state_start_time_ = now;
                    align_stable_frames_ = 0;
                    publishDebugEvent("strike_state", "ALIGN -> SIDE ALIGN");
                }
            } else {
                align_stable_frames_ = 0;
            }
            break;
        }

        case ServoState::DESCENDING: {
            const double altitude = fcu_odom_ptr->pose.pose.position.z;
            if (altitude <= strike_altitude_ + descent_tolerance_) {
                enterSprint(now);
                return executeSprint(true, target_age, area_ratio, yaw_rate);
            }
            if ((now - state_start_time_).toSec() >= descent_timeout_) {
                return startBrake(BrakeOutcome::ABORT, "descent timeout");
            }
            publishGuardedVelocity(0, 0, -std::abs(descent_velocity_),
                                   alignment_yaw_rate, 0.0, false);
            break;
        }

        case ServoState::APPROACHING: {
            const double altitude = fcu_odom_ptr->pose.pose.position.z;
            if (altitude > strike_altitude_ + descent_tolerance_) {
                state_ = ServoState::DESCENDING;
                state_start_time_ = now;
                publishGuardedVelocity(0, 0, -std::abs(descent_velocity_),
                                       alignment_yaw_rate, 0.0, false);
                publishDebugEvent("strike_state", "APPROACHING -> DESCENDING");
                break;
            }

            // Detection has already been accepted by ScanForTarget. Descent provides the
            // only strike-entry delay; once at propeller height, launch immediately.
            enterSprint(now);
            return executeSprint(true, target_age, area_ratio, yaw_rate);
        }

        case ServoState::SPRINTING: {
            return executeSprint(true, target_age, area_ratio, yaw_rate);
        }

        default:
            break;
    }
    return NodeStatus::RUNNING;
}

void VisualServo::onHalted() {
    setDepthAssistActive(false);
    DroneActionNode::onHalted();
}

void VisualServo::updateTargetData() {
    auto result = getInput<PixelTarget>("pixel_target");
    if (!result || !result.value().is_detected) return;
    std::lock_guard<std::mutex> lock(target_mutex_);
    last_target_ = result.value();
    last_valid_detection_time_ = ros::Time::now();
    has_ever_received_data_ = true;
}

bool VisualServo::getTargetSnapshot(PixelTarget& target, double& age) const {
    std::lock_guard<std::mutex> lock(target_mutex_);
    if (!has_ever_received_data_) {
        age = std::numeric_limits<double>::infinity();
        return false;
    }
    target = last_target_;
    age = (ros::Time::now() - last_valid_detection_time_).toSec();
    return age <= data_timeout_;
}

double VisualServo::desiredHitCenterX() const {
    double hard_clearance;
    double target_depth;
    double target_lateral;
    double target_closing_speed;
    double depth_scale = 1.0;
    if (getDepthSnapshot(hard_clearance, target_depth, target_lateral,
                         target_closing_speed) &&
        std::isfinite(target_depth) && target_depth > depth_min_valid_range_) {
        depth_scale = std::clamp(
            positiveOr(hit_offset_reference_depth_, 0.80) / target_depth,
            0.25, 2.0);
    }
    const double limit = std::clamp(max_hit_offset_ratio_, 0.0, 0.49) * img_w_;
    return img_cx_ + std::clamp(hit_offset_x_ * depth_scale, -limit, limit);
}

bool VisualServo::isCentered(const PixelTarget& target, double desired_center_x) const {
    return std::abs(target.center_x - desired_center_x) < img_w_ * center_tolerance_ &&
           std::abs(target.center_y - (img_cy_ + hit_offset_y_)) < img_h_ * center_tolerance_;
}

double VisualServo::computeYawRate(double err_x, double dt) {
    const double error = std::abs(err_x) > deadzone_yaw_
        ? err_x - std::copysign(deadzone_yaw_, err_x) : 0.0;
    const double derivative = (error - prev_err_x_) / std::max(dt, 1e-6);
    prev_err_x_ = error;
    return std::clamp(-(k_p_yaw_rate_ * error + k_d_yaw_rate_ * derivative),
                      -max_yaw_rate_, max_yaw_rate_);
}

double VisualServo::computeVz(double err_y, double dt) {
    const double error = std::abs(err_y) > deadzone_vz_
        ? err_y - std::copysign(deadzone_vz_, err_y) : 0.0;
    const double derivative = (error - prev_err_y_) / std::max(dt, 1e-6);
    prev_err_y_ = error;
    return std::clamp(-(k_p_vz_ * error + k_d_vz_ * derivative), -max_vz_, max_vz_);
}

double VisualServo::alignmentYawRate(double visual_yaw_rate) const {
    if (!geometry_valid_ || !fcu_odom_ptr) return visual_yaw_rate;
    if (max_alignment_yaw_deviation_ <= 0.0) return headingHoldYawRate();

    const double yaw = PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation);
    const double deviation = normalizeYaw(yaw - route_yaw_);
    const bool turning_farther_from_route =
        (deviation >= max_alignment_yaw_deviation_ && visual_yaw_rate > 0.0) ||
        (deviation <= -max_alignment_yaw_deviation_ && visual_yaw_rate < 0.0);
    return turning_farther_from_route ? headingHoldYawRate() : visual_yaw_rate;
}

double VisualServo::headingHoldYawRate() const {
    const double yaw = PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation);
    const double error = normalizeYaw(route_yaw_ - yaw);
    return std::clamp(heading_hold_gain_ * error,
                      -std::abs(heading_hold_max_rate_), std::abs(heading_hold_max_rate_));
}

double VisualServo::currentSpeed() const {
    const auto& velocity = fcu_odom_ptr->twist.twist.linear;
    return std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z);
}

double VisualServo::boundaryClearance() const {
    const auto& position = fcu_odom_ptr->pose.pose.position;
    return std::min({position.x - boundary_x_min_, boundary_x_max_ - position.x,
                     position.y - boundary_y_min_, boundary_y_max_ - position.y});
}

double VisualServo::stoppingDistance(double speed) const {
    const double bounded_speed = std::max(0.0, speed);
    const double acceleration = positiveOr(braking_acceleration_, 0.8);
    return bounded_speed * bounded_speed / (2.0 * acceleration) +
           bounded_speed * std::max(0.0, control_latency_);
}

double VisualServo::corridorProgress(const geometry_msgs::Point& origin) const {
    const auto& position = fcu_odom_ptr->pose.pose.position;
    const double dx = position.x - origin.x;
    const double dy = position.y - origin.y;
    return std::cos(route_yaw_) * dx + std::sin(route_yaw_) * dy;
}

double VisualServo::remainingSprintDistance() const {
    return corridor_limit_distance_ - corridorProgress(corridor_origin_);
}

void VisualServo::depthCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    if (!depth_assist_active_.load() || !msg || msg->data.empty()) return;

    PixelTarget target;
    ros::Time target_time;
    bool target_valid = false;
    {
        std::lock_guard<std::mutex> lock(target_mutex_);
        target = last_target_;
        target_time = last_valid_detection_time_;
        target_valid = has_ever_received_data_ &&
                       (ros::Time::now() - target_time).toSec() <= data_timeout_;
    }

    std::vector<CloudPoint> points;
    std::vector<float> target_depths;
    points.reserve(static_cast<size_t>(msg->width) * msg->height);

    try {
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            const float x = *iter_x;
            const float y = *iter_y;
            const float z = *iter_z;
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
                x < depth_min_valid_range_) {
                continue;
            }
            const double image_x = img_cx_ + depth_image_y_sign_ *
                rgb_focal_px_ * static_cast<double>(y) / x;
            const double image_y = img_cy_ + depth_image_z_sign_ *
                rgb_focal_px_ * static_cast<double>(z) / x;
            bool in_target = false;
            if (target_valid) {
                const double half_w = 0.5 * target.width + target_bbox_margin_px_;
                const double half_h = 0.5 * target.height + target_bbox_margin_px_;
                in_target = image_x >= 0.0 && image_x < img_w_ &&
                            image_y >= 0.0 && image_y < img_h_ &&
                            std::abs(image_x - target.center_x) <= half_w &&
                            std::abs(image_y - target.center_y) <= half_h;
            }
            points.push_back({x, y, z, static_cast<float>(image_x),
                              static_cast<float>(image_y), in_target});
            if (in_target) target_depths.push_back(x);
        }
    } catch (const std::runtime_error& error) {
        ROS_ERROR_THROTTLE(1.0, "[%s]: Invalid depth cloud: %s", name().c_str(), error.what());
        return;
    }

    double balloon_depth = std::numeric_limits<double>::infinity();
    if (static_cast<int>(target_depths.size()) >= depth_min_points_) {
        const size_t quantile_index = std::min(
            target_depths.size() - 1,
            static_cast<size_t>(target_depth_quantile_ * (target_depths.size() - 1)));
        std::nth_element(target_depths.begin(), target_depths.begin() + quantile_index,
                         target_depths.end());
        const double seed = target_depths[quantile_index];
        std::vector<float> foreground;
        foreground.reserve(target_depths.size());
        for (const float depth : target_depths) {
            if (std::abs(depth - seed) <= target_depth_band_) foreground.push_back(depth);
        }
        if (static_cast<int>(foreground.size()) >= depth_min_points_) {
            const size_t middle = foreground.size() / 2;
            std::nth_element(foreground.begin(), foreground.begin() + middle, foreground.end());
            balloon_depth = foreground[middle];
        }
    }

    double hard_clearance = std::numeric_limits<double>::infinity();
    int hard_points = 0;
    for (const CloudPoint& point : points) {
        const double carrier_half_width = std::max(
            0.75 * std::abs(target.width) + target_bbox_margin_px_, 24.0);
        const bool target_associated_carrier =
            target_valid && std::isfinite(balloon_depth) &&
            std::abs(point.x - balloon_depth) <= target_carrier_depth_margin_ &&
            std::abs(point.image_x - target.center_x) <= carrier_half_width &&
            point.image_y >= target.center_y - 0.5 * std::abs(target.height) -
                                 target_bbox_margin_px_ &&
            point.image_y <= target.center_y + 0.5 * std::abs(target.height) +
                                 target_carrier_below_margin_px_;
        // The balloon and its low carrier are the intended contact assembly. Static
        // obstacles outside this narrow RGB/depth association remain hard obstacles.
        if (point.in_target || target_associated_carrier) continue;
        if (std::abs(point.y) <= corridor_half_width_ &&
            std::abs(point.z) <= corridor_half_height_) {
            hard_clearance = std::min(hard_clearance, static_cast<double>(point.x));
            if (point.x <= emergency_clearance_) ++hard_points;
        }
    }

    const ros::Time now = ros::Time::now();
    double balloon_lateral = std::numeric_limits<double>::infinity();
    if (std::isfinite(balloon_depth) && target_valid) {
        const double lateral_sign = std::abs(depth_image_y_sign_) > 0.5
            ? depth_image_y_sign_ : 1.0;
        balloon_lateral = (target.center_x - img_cx_) * balloon_depth /
            (lateral_sign * positiveOr(rgb_focal_px_, 320.0));
    }

    if (!depth_assist_active_.load()) return;
    std::lock_guard<std::mutex> lock(depth_mutex_);
    if (std::isfinite(balloon_depth)) {
        if (std::isfinite(previous_target_depth_) &&
            !previous_target_depth_time_.isZero()) {
            const double dt = (now - previous_target_depth_time_).toSec();
            if (dt >= 0.01 && dt <= 0.50) {
                const double raw_closing_speed = std::clamp(
                    (previous_target_depth_ - balloon_depth) / dt, -3.0, 3.0);
                target_closing_speed_ = 0.65 * target_closing_speed_ +
                                        0.35 * raw_closing_speed;
            }
        }
        previous_target_depth_ = balloon_depth;
        previous_target_depth_time_ = now;
    } else if (!previous_target_depth_time_.isZero() &&
               (now - previous_target_depth_time_).toSec() > depth_timeout_) {
        previous_target_depth_ = std::numeric_limits<double>::infinity();
        previous_target_depth_time_ = ros::Time(0);
        target_closing_speed_ = 0.0;
    }

    depth_cloud_time_ = now;
    depth_frame_valid_ = static_cast<int>(points.size()) >= depth_min_points_;
    hard_front_clearance_ = hard_clearance;
    target_depth_ = balloon_depth;
    target_lateral_ = balloon_lateral;
    hard_corridor_points_ = hard_points;
}

void VisualServo::setDepthAssistActive(bool active) {
    depth_assist_active_.store(false);
    {
        std::lock_guard<std::mutex> lock(depth_mutex_);
        depth_frame_valid_ = false;
        depth_cloud_time_ = ros::Time(0);
        hard_front_clearance_ = std::numeric_limits<double>::infinity();
        target_depth_ = std::numeric_limits<double>::infinity();
        target_lateral_ = std::numeric_limits<double>::infinity();
        target_closing_speed_ = 0.0;
        previous_target_depth_ = std::numeric_limits<double>::infinity();
        previous_target_depth_time_ = ros::Time(0);
        hard_corridor_points_ = 0;
    }
    depth_assist_active_.store(active);
    publishDebugEvent("strike_depth", active ? "wait-zone assist enabled"
                                               : "strike assist disabled");
}

bool VisualServo::getDepthSnapshot(double& hard_clearance, double& target_depth) const {
    double target_lateral;
    double target_closing_speed;
    return getDepthSnapshot(hard_clearance, target_depth, target_lateral,
                            target_closing_speed);
}

bool VisualServo::getDepthSnapshot(double& hard_clearance, double& target_depth,
                                   double& target_lateral,
                                   double& target_closing_speed) const {
    if (!depth_assist_active_.load()) {
        hard_clearance = std::numeric_limits<double>::infinity();
        target_depth = std::numeric_limits<double>::infinity();
        target_lateral = std::numeric_limits<double>::infinity();
        target_closing_speed = 0.0;
        return false;
    }
    std::lock_guard<std::mutex> lock(depth_mutex_);
    hard_clearance = hard_front_clearance_;
    target_depth = target_depth_;
    target_lateral = target_lateral_;
    target_closing_speed = target_closing_speed_;
    return depth_frame_valid_ &&
           (ros::Time::now() - depth_cloud_time_).toSec() <= depth_timeout_;
}

bool VisualServo::targetInSprintZone(const PixelTarget& target, double area_ratio,
                                     double& target_depth,
                                     double& target_longitudinal,
                                     double& target_lateral,
                                     double& target_closing_speed) const {
    double hard_clearance;
    double body_lateral;
    target_depth = std::numeric_limits<double>::infinity();
    target_longitudinal = std::numeric_limits<double>::infinity();
    target_lateral = std::numeric_limits<double>::infinity();
    target_closing_speed = 0.0;

    const bool depth_fresh = getDepthSnapshot(
        hard_clearance, target_depth, body_lateral, target_closing_speed);
    if (depth_fresh && std::isfinite(target_depth) &&
        std::isfinite(body_lateral) && geometry_valid_ && fcu_odom_ptr) {
        const auto& position = fcu_odom_ptr->pose.pose.position;
        const double yaw = PoseUtils::getYawFromQuat(
            fcu_odom_ptr->pose.pose.orientation);
        const double target_world_x = position.x +
            std::cos(yaw) * target_depth + std::sin(yaw) * body_lateral;
        const double target_world_y = position.y +
            std::sin(yaw) * target_depth - std::cos(yaw) * body_lateral;
        const double dx = target_world_x - corridor_origin_.x;
        const double dy = target_world_y - corridor_origin_.y;
        target_longitudinal = std::cos(route_yaw_) * dx +
                              std::sin(route_yaw_) * dy;
        target_lateral = std::sin(route_yaw_) * dx -
                         std::cos(route_yaw_) * dy;

        const double far_limit = corridor_reference_length_ * target_zone_far_ratio_;
        return target_longitudinal >= -target_zone_near_distance_ &&
               target_longitudinal <= far_limit &&
               std::abs(target_lateral) <= target_zone_lateral_limit_;
    }

    // A stale/missing depth frame must not make a close, valid YOLO target inert.
    // The fallback is deliberately only used when no target depth can be recovered.
    return target.is_detected && area_ratio >= target_zone_fallback_area_ratio_;
}

bool VisualServo::emergencyObstacleDetected() const {
    if (!depth_assist_active_.load()) return false;
    std::lock_guard<std::mutex> lock(depth_mutex_);
    return depth_frame_valid_ &&
           (ros::Time::now() - depth_cloud_time_).toSec() <= depth_timeout_ &&
           std::isfinite(hard_front_clearance_) &&
           hard_front_clearance_ <= emergency_clearance_ &&
           hard_corridor_points_ >= depth_min_points_;
}

bool VisualServo::boundaryCommandSafe(double vx_body, double vy_body) const {
    if (std::hypot(vx_body, vy_body) < 1e-3) {
        return true;
    }

    const auto& position = fcu_odom_ptr->pose.pose.position;
    const double yaw = PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation);
    const double vx_world = std::cos(yaw) * vx_body + std::sin(yaw) * vy_body;
    const double vy_world = std::sin(yaw) * vx_body - std::cos(yaw) * vy_body;
    const double predicted_x = position.x + vx_world * safety_predict_time_;
    const double predicted_y = position.y + vy_world * safety_predict_time_;
    const auto motionSafeOnAxis = [](double current, double predicted,
                                     double minimum, double maximum) {
        if (current < minimum) return predicted > current;
        if (current > maximum) return predicted < current;
        return predicted >= minimum && predicted <= maximum;
    };
    return motionSafeOnAxis(position.x, predicted_x,
                            boundary_x_min_ + boundary_margin_,
                            boundary_x_max_ - boundary_margin_) &&
           motionSafeOnAxis(position.y, predicted_y,
                            boundary_y_min_ + boundary_margin_,
                            boundary_y_max_ - boundary_margin_);
}

bool VisualServo::publishGuardedVelocity(double vx, double vy, double vz, double yaw_rate,
                                         double required_clearance, bool require_depth) {
    // This RflySim/MAVROS chain was verified experimentally: positive body vz climbs.
    const double altitude = fcu_odom_ptr->pose.pose.position.z;
    if (altitude < min_attack_altitude_) {
        publishVelocityWithYawRate(0, 0, std::abs(altitude_recovery_speed_), 0);
        ROS_ERROR_THROTTLE(0.5, "[%s]: Low-altitude guard at z=%.2f", name().c_str(), altitude);
        return false;
    }
    if (altitude <= min_attack_altitude_ + altitude_guard_margin_ && vz < 0.0) {
        vz = 0.0;
    }
    if (!boundaryCommandSafe(vx, vy)) {
        publishVelocityWithYawRate(0, 0, std::max(vz, 0.0), headingHoldYawRate());
        ROS_ERROR_THROTTLE(0.5, "[%s]: Wall boundary guard rejected motion", name().c_str());
        return false;
    }
    if (require_depth) {
        double hard_clearance;
        double target_depth;
        if (!getDepthSnapshot(hard_clearance, target_depth)) {
            publishVelocityWithYawRate(0, 0, std::max(vz, 0.0), headingHoldYawRate());
            ROS_ERROR_THROTTLE(0.5, "[%s]: Depth unavailable; horizontal motion inhibited", name().c_str());
            return false;
        }
        if (vx > 0.0 && hard_clearance < required_clearance) {
            publishVelocityWithYawRate(0, 0, std::max(vz, 0.0), headingHoldYawRate());
            return false;
        }
    }
    publishVelocityWithYawRate(vx, vy, vz, yaw_rate);
    return true;
}

void VisualServo::publishWorldVelocityWithYawRate(double vx_world, double vy_world,
                                                  double vz, double yaw_rate) {
    const double current_yaw = PoseUtils::getYawFromQuat(
        fcu_odom_ptr->pose.pose.orientation);
    // Convert world ENU velocity to the BODY_NED convention used by the yaw-rate API.
    const double body_vx = std::cos(current_yaw) * vx_world +
                           std::sin(current_yaw) * vy_world;
    const double body_vy = std::sin(current_yaw) * vx_world -
                           std::cos(current_yaw) * vy_world;
    publishVelocityWithYawRate(body_vx, body_vy, vz, yaw_rate);
}

NodeStatus VisualServo::startBrake(BrakeOutcome outcome, const std::string& reason) {
    brake_outcome_ = outcome;
    finish_reason_ = reason;
    state_ = ServoState::BRAKING;
    state_start_time_ = ros::Time::now();
    brake_hold_start_ = ros::Time(0);
    brake_origin_ = fcu_odom_ptr->pose.pose.position;
    if (outcome == BrakeOutcome::HIT && post_hit_escape_velocity_ > 0.0 &&
        post_hit_escape_distance_ > 0.0 && post_hit_escape_time_ > 0.0) {
        const double current_yaw = PoseUtils::getYawFromQuat(
            fcu_odom_ptr->pose.pose.orientation);
        const double escape_yaw_rate = std::clamp(
            normalizeYaw(post_hit_escape_yaw_ - current_yaw),
            -post_hit_escape_yaw_rate_, post_hit_escape_yaw_rate_);
        publishWorldVelocityWithYawRate(
            post_hit_escape_velocity_ * std::cos(post_hit_escape_yaw_),
            post_hit_escape_velocity_ * std::sin(post_hit_escape_yaw_),
            0.0, escape_yaw_rate);
    } else {
        publishVelocityWithYawRate(0, 0, 0, headingHoldYawRate());
    }
    publishDebugEvent("strike_brake", reason);
    ROS_WARN("[%s]: BRAKE outcome=%d reason=%s progress=%.2f remaining=%.2f",
             name().c_str(), static_cast<int>(outcome), reason.c_str(),
             corridorProgress(corridor_origin_), remainingSprintDistance());
    return NodeStatus::RUNNING;
}

void VisualServo::enterSprint(const ros::Time& now) {
    // route_yaw_ is captured at the staged search point and defines the obstacle-free
    // corridor. Visual alignment may turn the aircraft briefly, but must not redefine
    // that corridor or a long sprint can drift into a frame post or wall.
    const double current_yaw = PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation);
    corridor_origin_ = fcu_odom_ptr->pose.pose.position;
    corridor_limit_distance_ = corridor_reference_length_ * sprint_limit_ratio_;
    wait_entry_distance_ = corridor_reference_length_ *
        (1.0 - sprint_limit_ratio_) * wait_entry_ratio_;
    state_ = ServoState::SPRINTING;
    state_start_time_ = now;
    close_target_seen_ = false;
    setDepthAssistActive(false);
    publishDebugEvent("strike_state", "APPROACHING -> SPRINTING");
    ROS_INFO("[%s]: Straight sprint started: corridor_yaw=%.2f current_yaw=%.2f max_distance=%.2fm z=%.2f vz=%.2f",
             name().c_str(), route_yaw_, current_yaw, corridor_limit_distance_,
             fcu_odom_ptr->pose.pose.position.z,
             fcu_odom_ptr->twist.twist.linear.z);
}

NodeStatus VisualServo::executeSprint(bool target_visible, double target_age,
                                      double area_ratio, double visual_yaw_rate) {
    if (target_visible) {
        const double close_area = std::max(approach_area_threshold_,
                                           post_hit_success_area_ratio_);
        close_target_seen_ = close_target_seen_ || area_ratio >= close_area;
    } else if (target_age > sprint_target_loss_grace_) {
        ROS_WARN_THROTTLE(0.5,
                          "[%s]: Target lost during sprint; continuing to wait zone (close=%d)",
                          name().c_str(), close_target_seen_);
    }

    if ((ros::Time::now() - state_start_time_).toSec() >= sprint_time_) {
        return startBrake(BrakeOutcome::MISS_TO_WAIT, "sprint time limit reached");
    }

    // Body velocity feedback can lag the commanded sprint in RflySim. Reserve braking
    // distance for at least the commanded speed so inertia cannot carry the aircraft into
    // the end wall after a miss.
    const double stop_distance = stoppingDistance(std::max(currentSpeed(), sprint_velocity_));
    if (remainingSprintDistance() <= stop_distance + 0.08) {
        return startBrake(BrakeOutcome::MISS_TO_WAIT, "sprint boundary braking point reached");
    }
    if (!boundaryCommandSafe(sprint_velocity_, 0.0)) {
        return startBrake(BrakeOutcome::MISS_TO_WAIT, "field boundary braking point reached");
    }
    const double altitude = fcu_odom_ptr->pose.pose.position.z;
    if (altitude < min_attack_altitude_) {
        publishVelocityWithYawRate(0, 0, std::abs(altitude_recovery_speed_), 0);
        return NodeStatus::RUNNING;
    }

    const double sprint_yaw_rate = target_visible && !sprint_lock_visual_correction_
        ? visual_yaw_rate : headingHoldYawRate();
    const double sprint_vz = std::clamp(
        sprint_altitude_hold_gain_ * (strike_altitude_ - altitude),
        -sprint_max_vertical_correction_, sprint_max_vertical_correction_);
    // Match the proven initial controller: body-frame positive VX is sent directly on
    // every sprint tick. Heading and altitude are held for a horizontal straight line.
    publishVelocityWithYawRate(sprint_velocity_, 0, sprint_vz, sprint_yaw_rate);
    ROS_INFO_THROTTLE(0.5,
                      "[%s]: SPRINT command vx=%.2f vz=%.2f z=%.2f yaw_rate=%.2f progress=%.2f/%.2fm visual=%d",
                      name().c_str(), sprint_velocity_, sprint_vz, altitude, sprint_yaw_rate,
                      corridorProgress(corridor_origin_), corridor_limit_distance_,
                      target_visible && !sprint_lock_visual_correction_);
    return NodeStatus::RUNNING;
}

NodeStatus VisualServo::executeBrake() {
    const double altitude = fcu_odom_ptr->pose.pose.position.z;
    const double vz = altitude < min_attack_altitude_
        ? std::abs(altitude_recovery_speed_) : 0.0;

    const auto& position = fcu_odom_ptr->pose.pose.position;
    const double escape_progress =
        std::cos(post_hit_escape_yaw_) * (position.x - brake_origin_.x) +
        std::sin(post_hit_escape_yaw_) * (position.y - brake_origin_.y);
    const double brake_elapsed = (ros::Time::now() - state_start_time_).toSec();
    const bool escape_active = brake_outcome_ == BrakeOutcome::HIT &&
        post_hit_escape_velocity_ > 0.0 && post_hit_escape_distance_ > 0.0 &&
        post_hit_escape_time_ > 0.0 &&
        escape_progress < post_hit_escape_distance_ &&
        brake_elapsed < post_hit_escape_time_;
    if (escape_active) {
        const double current_yaw = PoseUtils::getYawFromQuat(
            fcu_odom_ptr->pose.pose.orientation);
        const double escape_yaw_rate = std::clamp(
            normalizeYaw(post_hit_escape_yaw_ - current_yaw),
            -post_hit_escape_yaw_rate_, post_hit_escape_yaw_rate_);
        publishWorldVelocityWithYawRate(
            post_hit_escape_velocity_ * std::cos(post_hit_escape_yaw_),
            post_hit_escape_velocity_ * std::sin(post_hit_escape_yaw_),
            vz, escape_yaw_rate);
        brake_hold_start_ = ros::Time(0);
        ROS_INFO_THROTTLE(0.25,
                          "[%s]: HIT brake escaping wall: progress=%.2f/%.2fm yaw=%.2f",
                          name().c_str(), escape_progress,
                          post_hit_escape_distance_, post_hit_escape_yaw_);
        return NodeStatus::RUNNING;
    }

    double brake_yaw_rate = headingHoldYawRate();
    if (brake_outcome_ == BrakeOutcome::HIT) {
        const double current_yaw = PoseUtils::getYawFromQuat(
            fcu_odom_ptr->pose.pose.orientation);
        brake_yaw_rate = std::clamp(
            normalizeYaw(route_yaw_ - current_yaw),
            -post_hit_escape_yaw_rate_, post_hit_escape_yaw_rate_);
    }
    publishVelocityWithYawRate(0, 0, vz, brake_yaw_rate);

    const double speed = currentSpeed();
    if (speed <= brake_speed_threshold_) {
        if (brake_hold_start_.isZero()) brake_hold_start_ = ros::Time::now();
        if ((ros::Time::now() - brake_hold_start_).toSec() >= brake_hold_time_) {
            if (brake_outcome_ == BrakeOutcome::MISS_TO_WAIT) {
                wait_entry_origin_ = fcu_odom_ptr->pose.pose.position;
                setDepthAssistActive(true);
                state_ = ServoState::ENTERING_WAIT;
                state_start_time_ = ros::Time::now();
                publishDebugEvent("strike_state", "BRAKING -> ENTERING_WAIT");
            } else {
                state_ = ServoState::SETTLING;
                settle_start_time_ = ros::Time::now();
                publishDebugEvent("strike_state", "BRAKING -> SETTLING");
            }
        }
    } else {
        brake_hold_start_ = ros::Time(0);
    }

    if ((ros::Time::now() - state_start_time_).toSec() >= brake_timeout_) {
        ROS_ERROR_THROTTLE(0.5, "[%s]: Braking slower than expected, speed=%.2f",
                           name().c_str(), speed);
    }
    return NodeStatus::RUNNING;
}

NodeStatus VisualServo::executeWaitEntry() {
    PixelTarget target;
    double target_age;
    double hard_clearance;
    double target_depth;
    const bool target_valid = getTargetSnapshot(target, target_age);
    const bool depth_fresh = getDepthSnapshot(hard_clearance, target_depth);
    if (target_valid && depth_fresh && std::isfinite(target_depth) &&
        target_depth <= wait_contact_distance_) {
        return startBrake(BrakeOutcome::HIT, "balloon contacted while entering wait zone");
    }

    const double progress = corridorProgress(wait_entry_origin_);
    const bool entry_complete = progress >= wait_entry_distance_;
    const bool blocked = emergencyObstacleDetected();
    const bool at_boundary = !boundaryCommandSafe(wait_velocity_, 0.0);
    const bool timeout = (ros::Time::now() - state_start_time_).toSec() >= 4.0;
    if (entry_complete || blocked || at_boundary || timeout) {
        sendHoverCommand();
        state_ = ServoState::WAITING;
        state_start_time_ = ros::Time::now();
        wait_last_target_time_ = state_start_time_;
        publishDebugEvent("strike_state", "ENTERING_WAIT -> WAITING");
        return NodeStatus::RUNNING;
    }

    if (!publishGuardedVelocity(wait_velocity_, 0, 0, headingHoldYawRate(), 0.0, false)) {
        sendHoverCommand();
        state_ = ServoState::WAITING;
        state_start_time_ = ros::Time::now();
        wait_last_target_time_ = state_start_time_;
        publishDebugEvent("strike_state", "ENTERING_WAIT blocked -> WAITING");
    }
    return NodeStatus::RUNNING;
}

NodeStatus VisualServo::executeWaiting() {
    publishVelocityWithYawRate(0, 0, 0, headingHoldYawRate());

    PixelTarget target;
    double target_age;
    const bool target_valid = getTargetSnapshot(target, target_age);
    double hard_clearance;
    double target_depth;
    const bool depth_fresh = getDepthSnapshot(hard_clearance, target_depth);
    if (target_valid) {
        wait_last_target_time_ = ros::Time::now();
        const double area_ratio = target.area() / (static_cast<double>(img_w_) * img_h_);
        const bool close_now = (depth_fresh && std::isfinite(target_depth) &&
                                target_depth <= wait_contact_distance_) ||
                               area_ratio >= std::max(approach_area_threshold_ * 1.5,
                                                      post_hit_success_area_ratio_);
        close_target_seen_ = close_target_seen_ || close_now;
        if (depth_fresh && std::isfinite(target_depth) &&
            target_depth <= contact_distance_) {
            return startBrake(BrakeOutcome::HIT, "returning cart reached propeller");
        }
    } else if (close_target_seen_) {
        return startBrake(BrakeOutcome::HIT, "close target disappeared in wait zone");
    } else if (!wait_last_target_time_.isZero() &&
               (ros::Time::now() - wait_last_target_time_).toSec() >=
                   wait_no_target_timeout_) {
        return startBrake(BrakeOutcome::ABORT,
                          "no balloon detected in wait zone for 0.5s");
    }

    if ((ros::Time::now() - state_start_time_).toSec() >= wait_timeout_) {
        return startBrake(BrakeOutcome::ABORT, "wait-zone intercept timeout");
    }
    return NodeStatus::RUNNING;
}

NodeStatus VisualServo::startRecovery() {
    state_ = ServoState::RECOVERY;
    state_start_time_ = ros::Time::now();
    align_stable_frames_ = 0;
    sendHoverCommand();
    publishDebugEvent("strike_state", "TARGET LOST -> RECOVERY");
    return NodeStatus::RUNNING;
}

NodeStatus VisualServo::executeRecovery() {
    const double elapsed = (ros::Time::now() - state_start_time_).toSec();
    if (elapsed >= recovery_total_timeout_ || boundaryClearance() < boundary_margin_) {
        return startBrake(BrakeOutcome::ABORT, "stationary target recovery failed");
    }
    const double phase = std::fmod(elapsed, 2.4);
    const double scan_rate = phase < 1.2 ? recovery_yaw_rate_ : -recovery_yaw_rate_;
    publishVelocityWithYawRate(0, 0, 0,
                               std::clamp(scan_rate, -max_yaw_rate_, max_yaw_rate_));
    return NodeStatus::RUNNING;
}

NodeStatus VisualServo::startPostHitLift() {
    post_hit_lift_start_pos_ = fcu_odom_ptr->pose.pose.position;
    post_hit_lift_target_z_ = post_hit_lift_start_pos_.z + post_hit_lift_height_;
    post_hit_lift_yaw_ = PoseUtils::getYawFromQuat(
        fcu_odom_ptr->pose.pose.orientation);
    lift_start_time_ = ros::Time::now();
    state_ = ServoState::LIFTING;
    publishDebugEvent("strike_state", "SETTLING -> LIFTING");
    ROS_INFO("[%s]: Post-hit lift started: z=%.2f target_z=%.2f height=%.2f",
             name().c_str(), post_hit_lift_start_pos_.z,
             post_hit_lift_target_z_, post_hit_lift_height_);
    return NodeStatus::RUNNING;
}

NodeStatus VisualServo::executePostHitLift() {
    const double elapsed = (ros::Time::now() - lift_start_time_).toSec();
    const double ratio = std::clamp(elapsed / post_hit_lift_time_, 0.0, 1.0);
    const double smooth_ratio = ratio * ratio * (3.0 - 2.0 * ratio);
    geometry_msgs::Point lift_target = post_hit_lift_start_pos_;
    lift_target.z += post_hit_lift_height_ * smooth_ratio;
    publishPosition(lift_target, post_hit_lift_yaw_);

    const double current_z = fcu_odom_ptr->pose.pose.position.z;
    const bool height_reached =
        std::abs(current_z - post_hit_lift_target_z_) <= post_hit_lift_tolerance_;
    const bool timed_out = elapsed >= post_hit_lift_timeout_;
    if (height_reached || timed_out) {
        post_hit_lift_completed_ = true;
        state_ = ServoState::SETTLING;
        settle_start_time_ = ros::Time::now();
        publishDebugEvent("strike_state", "LIFTING -> SETTLING");
        ROS_INFO("[%s]: Post-hit lift finished: reached=%d z=%.2f target_z=%.2f elapsed=%.2f",
                 name().c_str(), height_reached, current_z,
                 post_hit_lift_target_z_, elapsed);
    }
    return NodeStatus::RUNNING;
}

NodeStatus VisualServo::finishAfterSettle() {
    sendHoverCommand();
    if ((ros::Time::now() - settle_start_time_).toSec() < post_hit_settle_time_ ||
        currentSpeed() > brake_speed_threshold_) {
        return NodeStatus::RUNNING;
    }
    const bool success = brake_outcome_ == BrakeOutcome::HIT;
    if (success && !post_hit_lift_completed_ && post_hit_lift_height_ > 0.0) {
        return startPostHitLift();
    }
    setDepthAssistActive(false);
    publishDebugEvent(success ? "strike_success" : "strike_failure", finish_reason_);
    return success ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<VisualServo>("VisualServo");
}
