/****************************************************************************
 * @file       visual_servo.cpp
 * @brief      Visual Servo 行为树节点 - 三阶段控制（body系 VX+VZ+yaw_rate）
 ******************************************************************************/
#include "nodes/action/visual_servo.h"
#include "utils/pose_utils.h"
#include <algorithm>
#include <cmath>
#include <limits>

VisualServo::VisualServo(const std::string& name, const NodeConfig& config)
    : DroneActionNode(name, config),
      current_state_(ALIGN),
      has_ever_received_data_(false),
      max_area_ratio_(0.0) {
    auto nh = mct->getROSHandle();
    nh->param<double>("cx", img_cx_, 320.0);
    nh->param<double>("cy", img_cy_, 240.0);
    nh->param<int>("img_w", img_w_, 640);
    nh->param<int>("img_h", img_h_, 480);
}

PortsList VisualServo::providedPorts() {
    return { InputPort<PixelTarget>("pixel_target", "像素目标"),
             InputPort<double>("k_p_yaw_rate", 0.004, "偏航角速度 P 增益 (rad/s per pixel)"),
             InputPort<double>("k_d_yaw_rate", 0.008, "偏航角速度 D 增益"),
             InputPort<double>("max_yaw_rate", 0.7, "最大偏航角速度 (rad/s)"),
             InputPort<double>("deadzone_yaw", 8.0, "偏航死区 (pixel)"),
             InputPort<double>("k_p_vz", 0.002, "垂直速度 P 增益"),
             InputPort<double>("k_d_vz", 0.006, "垂直速度 D 增益"),
             InputPort<double>("max_vz", 0.3, "最大垂直速度 (m/s)"),
             InputPort<double>("deadzone_vz", 10.0, "垂直死区 (pixel)"),
             InputPort<double>("forward_velocity", 0.9, "逼近阶段前进速度 (m/s)"),
             InputPort<double>("sprint_velocity", 0.85, "冲刺速度 (m/s)"),
             InputPort<double>("sprint_time", 0.55, "冲刺持续时间 (s)"),
             InputPort<double>("hit_offset_x", 45.0, "侧桨打击水平像素偏置 (pixel)"),
             InputPort<double>("hit_offset_y", 0.0, "侧桨打击垂直像素偏置 (pixel)"),
             InputPort<double>("side_hit_offset_px", 0.0, "兼容当前 XML，499e9f8 逻辑使用 hit_offset_x"),
             InputPort<double>("hit_area_ratio", 0.35, "兼容当前 XML，499e9f8 逻辑不使用"),
             InputPort<double>("center_exit_factor", 1.5, "兼容当前 XML，499e9f8 逻辑不使用"),
             InputPort<double>("post_hit_climb_height", 0.7, "兼容当前 XML，499e9f8 逻辑使用 post_hit_lift_height"),
             InputPort<double>("post_hit_climb_velocity", 0.9, "兼容当前 XML，499e9f8 逻辑不使用"),
             InputPort<double>("post_hit_climb_timeout", 1.2, "兼容当前 XML，499e9f8 逻辑使用 post_hit_lift_timeout"),
             InputPort<double>("safety_predict_time", 0.35, "兼容当前 XML，499e9f8 逻辑不使用"),
             InputPort<double>("safety_margin", 0.2, "兼容当前 XML，499e9f8 逻辑不使用"),
             InputPort<double>("sprint_lift_velocity", 0.12, "冲刺阶段轻微上扬速度 (m/s)"),
             InputPort<bool>("sprint_lock_visual_correction", true, "冲刺阶段锁定视觉修正"),
             InputPort<double>("post_hit_brake_velocity", 0.35, "撞击后反向刹车速度 (m/s)"),
             InputPort<double>("post_hit_brake_time", 0.35, "撞击后反向刹车时间 (s)"),
             InputPort<double>("post_hit_lift_height", 0.35, "撞击后抬升高度 (m)"),
             InputPort<double>("post_hit_lift_time", 0.45, "撞击后抬升目标斜坡时间 (s)"),
             InputPort<double>("post_hit_lift_timeout", 1.0, "撞击后抬升最大等待时间 (s)"),
             InputPort<double>("post_hit_lift_tolerance", 0.12, "撞击后抬升到位高度容差 (m)"),
             InputPort<double>("post_hit_settle_time", 0.45, "撞击后悬停稳定时间 (s)"),
             InputPort<double>("data_timeout", 0.3, "数据超时 (s)"),
             InputPort<double>("approach_area_threshold", 0.06, "触发冲刺的面积占比"),
             InputPort<double>("post_hit_success_area_ratio", 0.025, "打击后目标丢失判为成功的面积占比"),
             InputPort<double>("center_tolerance", 0.15, "对齐判定区域比例"),
             // 目标丢失后的主动恢复参数
             InputPort<double>("recovery_delay", 0.15, "丢失多久后进入恢复 (s)"),
             InputPort<double>("recovery_backoff_velocity", -0.25, "恢复阶段后退速度 (m/s, 负值)"),
             InputPort<double>("recovery_backoff_time", 0.8, "恢复阶段后退持续时间 (s)"),
             InputPort<double>("recovery_yaw_rate", 0.5, "恢复阶段扫描角速度 (rad/s)"),
             InputPort<double>("recovery_total_timeout", 3.0, "恢复阶段总超时 (s)") };
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
    hit_offset_x_ = getInput<double>("hit_offset_x").value();
    hit_offset_y_ = getInput<double>("hit_offset_y").value();
    sprint_lift_velocity_ = getInput<double>("sprint_lift_velocity").value();
    sprint_lock_visual_correction_ = getInput<bool>("sprint_lock_visual_correction").value();
    post_hit_brake_velocity_ = getInput<double>("post_hit_brake_velocity").value();
    post_hit_brake_time_ = getInput<double>("post_hit_brake_time").value();
    post_hit_lift_height_ = getInput<double>("post_hit_lift_height").value();
    post_hit_lift_time_ = getInput<double>("post_hit_lift_time").value();
    post_hit_lift_timeout_ = getInput<double>("post_hit_lift_timeout").value();
    post_hit_lift_tolerance_ = getInput<double>("post_hit_lift_tolerance").value();
    post_hit_settle_time_ = getInput<double>("post_hit_settle_time").value();

    data_timeout_ = getInput<double>("data_timeout").value();
    approach_area_threshold_ = getInput<double>("approach_area_threshold").value();
    post_hit_success_area_ratio_ = getInput<double>("post_hit_success_area_ratio").value();
    center_tolerance_ = getInput<double>("center_tolerance").value();

    recovery_delay_ = getInput<double>("recovery_delay").value();
    recovery_backoff_velocity_ = getInput<double>("recovery_backoff_velocity").value();
    recovery_backoff_time_ = getInput<double>("recovery_backoff_time").value();
    recovery_yaw_rate_ = getInput<double>("recovery_yaw_rate").value();
    recovery_total_timeout_ = getInput<double>("recovery_total_timeout").value();

    current_state_ = APPROACHING;
    has_ever_received_data_ = false;
    max_area_ratio_ = 0.0;
    align_stable_frames_ = 0;
    prev_err_x_ = 0.0;
    prev_err_y_ = 0.0;
    prev_time_ = ros::Time::now();

    updateTargetData();

    ROS_INFO("[%s]: Visual Servo Started (VX+VZ+yaw_rate)", name().c_str());
    ROS_INFO("[%s]: forward_vel=%.2f sprint_vel=%.2f sprint_time=%.2f offset=(%.1f,%.1f) sprint_lift=%.2f brake=%.2fm/s %.2fs lift=%.2fm %.2fs settle=%.2fs area_thresh=%.3f",
             name().c_str(), forward_velocity_, sprint_velocity_, sprint_time_,
             hit_offset_x_, hit_offset_y_, sprint_lift_velocity_, post_hit_brake_velocity_, post_hit_brake_time_, post_hit_lift_height_, post_hit_lift_time_, post_hit_settle_time_, approach_area_threshold_);
    ROS_INFO("[%s]: recovery delay=%.2fs backoff=%.2fm/s %.2fs yaw=%.2f timeout=%.2fs",
             name().c_str(), recovery_delay_, recovery_backoff_velocity_, recovery_backoff_time_,
             recovery_yaw_rate_, recovery_total_timeout_);
    publishDebugEvent("vs_start", "visual servo active");

    return NodeStatus::RUNNING;
}

NodeStatus VisualServo::onRunning() {
    updateTargetData();

    ros::Time now = ros::Time::now();
    double dt = (now - prev_time_).toSec();
    if (dt < 0.001) dt = 0.001;

    if (current_state_ == POST_HIT_LIFTING) {
        double lift_dt = (ros::Time::now() - lift_start_time_).toSec();
        double lift_time = std::max(0.05, post_hit_lift_time_);
        double ratio = std::clamp(lift_dt / lift_time, 0.0, 1.0);
        geometry_msgs::Point lift_target = post_hit_lift_start_pos_;
        lift_target.z = post_hit_lift_start_pos_.z + post_hit_lift_height_ * ratio;
        double current_yaw = PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation);
        publishPosition(lift_target, current_yaw);

        double current_z = fcu_odom_ptr->pose.pose.position.z;
        bool height_reached = std::abs(current_z - post_hit_lift_target_z_) <= post_hit_lift_tolerance_;
        bool lift_timeout = lift_dt >= std::max(lift_time, post_hit_lift_timeout_);
        if (height_reached || lift_timeout) {
            current_state_ = SETTLING;
            settle_start_time_ = ros::Time::now();
            publishDebugEvent("vs_state", std::string("POST_HIT_LIFTING -> SETTLING reached=") +
                              (height_reached ? "1" : "0") + " z=" + std::to_string(current_z) +
                              " target_z=" + std::to_string(post_hit_lift_target_z_));
        }
        return NodeStatus::RUNNING;
    }

    if (current_state_ == BRAKING) {
        double brake_dt = (ros::Time::now() - brake_start_time_).toSec();
        if (brake_dt < post_hit_brake_time_) {
            publishVelocityWithYawRate(-post_hit_brake_velocity_, 0, 0, 0);
            return NodeStatus::RUNNING;
        }
        current_state_ = SETTLING;
        settle_start_time_ = ros::Time::now();
        sendHoverCommand();
        publishDebugEvent("vs_state", "BRAKING -> SETTLING");
        return NodeStatus::RUNNING;
    }

    if (current_state_ == SETTLING) {
        sendHoverCommand();
        double settle_dt = (ros::Time::now() - settle_start_time_).toSec();
        if (settle_dt >= post_hit_settle_time_) {
            publishDebugEvent("vs_success", "visual servo completed after post-hit settle");
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::RUNNING;
    }

    bool target_now_valid = !isTargetLost();

    // 如果正在恢复中，优先处理恢复状态
    if (current_state_ == RECOVERY) {
        if (target_now_valid) {
            current_state_ = ALIGN;
            align_stable_frames_ = 0;
            ROS_INFO("[%s]: Target reacquired during recovery, resuming tracking.", name().c_str());
            publishDebugEvent("vs_state", "RECOVERY -> ALIGN (reacquired)");
            // 继续下面的正常跟踪逻辑
        } else {
            return executeRecovery();
        }
    }

    if (!target_now_valid) {
        double age = targetAge();
        if (current_state_ == SPRINTING || max_area_ratio_ >= post_hit_success_area_ratio_) {
            ROS_INFO("[%s]: Target lost after close approach, treating as hit success (max_area=%.3f).",
                     name().c_str(), max_area_ratio_);
            return startPostHitBrake("target lost after close approach");
        }
        if (age >= recovery_delay_) {
            return startRecovery();
        }
        // 短暂丢失：立即刹车悬停，避免沿旧目标继续前飞撞墙
        publishVelocityWithYawRate(0, 0, 0, 0);
        ROS_WARN_THROTTLE(1.0, "[%s]: Target lost, waiting... (age=%.2fs)", name().c_str(), age);
        publishDebugEvent("vs_wait", "target lost, waiting");
        return NodeStatus::RUNNING;
    }

    double img_area = static_cast<double>(img_w_) * img_h_;
    double current_ratio = last_target_.area() / img_area;
    double desired_cx = img_cx_ + hit_offset_x_;
    double desired_cy = img_cy_ + hit_offset_y_;
    double err_x = last_target_.center_x - desired_cx;
    double err_y = last_target_.center_y - desired_cy;

    if (current_ratio > max_area_ratio_) {
        max_area_ratio_ = current_ratio;
    }

    double yaw_rate = computeYawRate(err_x, dt);
    double vz = computeVz(err_y, dt);
    prev_time_ = now;

    switch (current_state_) {
        case ALIGN: {
            publishVelocityWithYawRate(0, 0, vz, yaw_rate);
            if (isCentered()) {
                align_stable_frames_++;
                if (align_stable_frames_ >= ALIGN_STABLE_FRAMES_REQUIRED) {
                    current_state_ = APPROACHING;
                    align_stable_frames_ = 0;
                    publishDebugEvent("vs_state", "ALIGN -> APPROACHING");
                }
            } else {
                align_stable_frames_ = 0;
            }
            break;
        }

        case APPROACHING: {
            publishVelocityWithYawRate(forward_velocity_, 0, vz, yaw_rate);
            if (!isCentered()) {
                current_state_ = ALIGN;
                align_stable_frames_ = 0;
                publishDebugEvent("vs_state", "APPROACHING -> ALIGN");
                break;
            }
            if (current_ratio >= approach_area_threshold_) {
                current_state_ = SPRINTING;
                sprint_start_time_ = ros::Time::now();
                ROS_INFO("[%s]: Area %.1f%% >= %.1f%% -> SPRINT!", name().c_str(),
                         current_ratio * 100, approach_area_threshold_ * 100);
                publishDebugEvent("vs_state", "APPROACHING -> SPRINTING");
            }
            break;
        }

        case SPRINTING: {
            double sprint_dt = (ros::Time::now() - sprint_start_time_).toSec();
            if (sprint_dt >= sprint_time_) {
                return startPostHitBrake("sprint finished");
            }
            double sprint_yaw_rate = sprint_lock_visual_correction_ ? 0.0 : yaw_rate;
            double sprint_vz = sprint_lock_visual_correction_ ? -sprint_lift_velocity_ : (vz - sprint_lift_velocity_);
            publishVelocityWithYawRate(sprint_velocity_, 0, sprint_vz, sprint_yaw_rate);
            break;
        }

        case POST_HIT_LIFTING:
        case BRAKING:
        case SETTLING:
            break;

        case RECOVERY:
            // 正常逻辑中不应到达这里（上面已处理），加上兜底
            break;
    }

    return NodeStatus::RUNNING;
}

double VisualServo::computeYawRate(double err_x, double dt) {
    double err_after_dz = 0.0;
    if (std::abs(err_x) > deadzone_yaw_) {
        err_after_dz = err_x - std::copysign(deadzone_yaw_, err_x);
    }
    double d_err = (dt > 1e-6) ? (err_after_dz - prev_err_x_) / dt : 0.0;
    double yaw_rate = -(k_p_yaw_rate_ * err_after_dz + k_d_yaw_rate_ * d_err);
    yaw_rate = std::clamp(yaw_rate, -max_yaw_rate_, max_yaw_rate_);
    prev_err_x_ = err_after_dz;
    return yaw_rate;
}

double VisualServo::computeVz(double err_y, double dt) {
    double err_after_dz = 0.0;
    if (std::abs(err_y) > deadzone_vz_) {
        err_after_dz = err_y - std::copysign(deadzone_vz_, err_y);
    }
    double d_err = (dt > 1e-6) ? (err_after_dz - prev_err_y_) / dt : 0.0;
    double vz = -(k_p_vz_ * err_after_dz + k_d_vz_ * d_err);
    vz = std::clamp(vz, -max_vz_, max_vz_);
    prev_err_y_ = err_after_dz;
    return vz;
}

void VisualServo::updateTargetData() {
    auto target_res = getInput<PixelTarget>("pixel_target");
    if (!target_res) return;
    PixelTarget current_target = target_res.value();
    if (current_target.is_detected) {
        last_target_ = current_target;
        last_valid_detection_time_ = ros::Time::now();
        has_ever_received_data_ = true;
        double img_area = static_cast<double>(img_w_) * img_h_;
        double ratio = last_target_.area() / img_area;
        if (ratio > max_area_ratio_) {
            max_area_ratio_ = ratio;
        }
    }
}

bool VisualServo::isTargetLost() {
    if (!has_ever_received_data_) return true;
    return targetAge() > data_timeout_;
}

double VisualServo::targetAge() const {
    if (!has_ever_received_data_) {
        return std::numeric_limits<double>::infinity();
    }
    return (ros::Time::now() - last_valid_detection_time_).toSec();
}

bool VisualServo::isCentered() const {
    double desired_cx = img_cx_ + hit_offset_x_;
    double desired_cy = img_cy_ + hit_offset_y_;
    double err_x = last_target_.center_x - desired_cx;
    double err_y = last_target_.center_y - desired_cy;
    double tol_w = img_w_ * center_tolerance_;
    double tol_h = img_h_ * center_tolerance_;
    return std::abs(err_x) < tol_w && std::abs(err_y) < tol_h;
}

NodeStatus VisualServo::startPostHitBrake(const std::string& reason) {
    lift_start_time_ = ros::Time::now();
    post_hit_lift_start_pos_ = fcu_odom_ptr->pose.pose.position;
    post_hit_lift_target_z_ = post_hit_lift_start_pos_.z + post_hit_lift_height_;
    current_state_ = POST_HIT_LIFTING;
    publishDebugEvent("vs_state", std::string("SPRINTING -> POST_HIT_LIFTING | ") + reason);
    return NodeStatus::RUNNING;
}

NodeStatus VisualServo::startRecovery() {
    recovery_start_time_ = ros::Time::now();
    current_state_ = RECOVERY;
    align_stable_frames_ = 0;
    ROS_WARN("[%s]: Target lost confirmed -> enter RECOVERY (backoff + scan)", name().c_str());
    publishDebugEvent("vs_state", "TARGET LOST -> RECOVERY");
    return NodeStatus::RUNNING;
}

NodeStatus VisualServo::executeRecovery() {
    ros::Time now = ros::Time::now();
    double elapsed = (now - recovery_start_time_).toSec();

    if (elapsed >= recovery_total_timeout_) {
        sendHoverCommand();
        ROS_WARN("[%s]: Recovery timeout, target not reacquired. Aborting visual servo.", name().c_str());
        publishDebugEvent("vs_recovery_timeout", "target not reacquired");
        return NodeStatus::FAILURE;
    }

    double vx = 0.0;
    double yaw_rate = 0.0;

    if (elapsed < recovery_backoff_time_) {
        // 阶段1：边后退边左右扫视，尽快脱离前方墙壁/障碍物
        vx = recovery_backoff_velocity_;
        if (elapsed < recovery_backoff_time_ / 2.0) {
            yaw_rate = recovery_yaw_rate_;
        } else {
            yaw_rate = -recovery_yaw_rate_;
        }
    } else {
        // 阶段2：原地往复旋转扫描，尝试重新捕获目标
        constexpr double scan_period = 1.2;
        double phase = std::fmod(elapsed - recovery_backoff_time_, 2.0 * scan_period);
        yaw_rate = (phase < scan_period) ? recovery_yaw_rate_ : -recovery_yaw_rate_;
    }

    publishVelocityWithYawRate(vx, 0.0, 0.0, yaw_rate);
    ROS_INFO_THROTTLE(0.5, "[%s]: RECOVERY elapsed=%.2f/%.2f vx=%.2f yaw_rate=%.2f",
                      name().c_str(), elapsed, recovery_total_timeout_, vx, yaw_rate);
    publishDebugEvent("vs_recovery", "elapsed=" + std::to_string(elapsed));
    return NodeStatus::RUNNING;
}


BT_REGISTER_NODES(factory) {
    factory.registerNodeType<VisualServo>("VisualServo");
}
