/****************************************************************************
 * @file       visual_servo.cpp
 * @brief      Visual Servo 行为树节点 - 三阶段控制（body系 VX+VZ+yaw_rate）
 ******************************************************************************/
#include "nodes/action/visual_servo.h"
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
             InputPort<double>("post_hit_brake_velocity", 0.45, "撞击后反向刹车速度 (m/s)"),
             InputPort<double>("post_hit_brake_time", 0.45, "撞击后反向刹车时间 (s)"),
             InputPort<double>("data_timeout", 0.3, "数据超时 (s)"),
             InputPort<double>("approach_area_threshold", 0.06, "触发冲刺的面积占比"),
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
    post_hit_brake_velocity_ = getInput<double>("post_hit_brake_velocity").value();
    post_hit_brake_time_ = getInput<double>("post_hit_brake_time").value();

    data_timeout_ = getInput<double>("data_timeout").value();
    approach_area_threshold_ = getInput<double>("approach_area_threshold").value();
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
    ROS_INFO("[%s]: forward_vel=%.2f sprint_vel=%.2f sprint_time=%.2f brake=%.2fm/s %.2fs area_thresh=%.3f",
             name().c_str(), forward_velocity_, sprint_velocity_, sprint_time_,
             post_hit_brake_velocity_, post_hit_brake_time_, approach_area_threshold_);
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

    if (current_state_ == BRAKING) {
        double brake_dt = (ros::Time::now() - brake_start_time_).toSec();
        if (brake_dt < post_hit_brake_time_) {
            publishVelocityWithYawRate(-post_hit_brake_velocity_, 0, 0, 0);
            return NodeStatus::RUNNING;
        }
        sendHoverCommand();
        publishDebugEvent("vs_success", "visual servo completed");
        return NodeStatus::SUCCESS;
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
    double err_x = last_target_.center_x - img_cx_;
    double err_y = last_target_.center_y - img_cy_;

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
            publishVelocityWithYawRate(sprint_velocity_, 0, vz, yaw_rate);
            break;
        }

        case BRAKING:
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
    double err_x = last_target_.center_x - img_cx_;
    double err_y = last_target_.center_y - img_cy_;
    double tol_w = img_w_ * center_tolerance_;
    double tol_h = img_h_ * center_tolerance_;
    return std::abs(err_x) < tol_w && std::abs(err_y) < tol_h;
}

NodeStatus VisualServo::startPostHitBrake(const std::string& reason) {
    brake_start_time_ = ros::Time::now();
    current_state_ = BRAKING;
    publishDebugEvent("vs_state", std::string("SPRINTING -> BRAKING | ") + reason);
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
