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
    // 与 mavros_cnt 同一组 safety/* 边界参数（ENU），供开火前剩余空间守卫使用
    nh->param<double>("safety/x_min", boundary_x_min_, -10.0);
    nh->param<double>("safety/x_max", boundary_x_max_, 10.0);
    nh->param<double>("safety/y_min", boundary_y_min_, -10.0);
    nh->param<double>("safety/y_max", boundary_y_max_, 10.0);
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
             InputPort<double>("sprint_time", 0.55, "冲刺持续时间上限 (s)，实际时长按距离自适应"),
             InputPort<double>("sprint_range_margin", 1.15, "自适应冲刺时长系数：时长 = 估计距离/冲刺速度 × 该值"),
             InputPort<double>("sprint_min_time", 0.35, "自适应冲刺时长下限 (s)"),
             InputPort<double>("k_p_vy", 0.004, "冲刺段横向视觉修正 P 增益 (m/s per pixel)"),
             InputPort<double>("max_vy", 0.5, "冲刺段横向速度限幅 (m/s)"),
             InputPort<double>("startup_grace_time", 1.0, "启动宽限：未收到过目标数据时该时长内不进 RECOVERY (s)"),
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
             InputPort<bool>("sprint_lock_visual_correction", true, "冲刺锁定视觉修正：true=全锁(旧行为,盲冲)，false=保持 vy/vz 修正仅锁 yaw"),
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
             InputPort<double>("strike_stable_time", 0.8, "稳定跟踪(持续有效且居中)达到该时长才允许冲刺 (s)"),
             InputPort<double>("sprint_room_margin", 0.5, "开火空间余量：机头方向剩余空间 < 冲刺行程+该值时不冲刺 (m)"),
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
    sprint_range_margin_ = getInput<double>("sprint_range_margin").value();
    sprint_min_time_ = getInput<double>("sprint_min_time").value();
    k_p_vy_ = getInput<double>("k_p_vy").value();
    max_vy_ = getInput<double>("max_vy").value();
    startup_grace_time_ = getInput<double>("startup_grace_time").value();
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
    strike_stable_time_ = getInput<double>("strike_stable_time").value();
    sprint_room_margin_ = getInput<double>("sprint_room_margin").value();

    recovery_delay_ = getInput<double>("recovery_delay").value();
    recovery_backoff_velocity_ = getInput<double>("recovery_backoff_velocity").value();
    recovery_backoff_time_ = getInput<double>("recovery_backoff_time").value();
    recovery_yaw_rate_ = getInput<double>("recovery_yaw_rate").value();
    recovery_total_timeout_ = getInput<double>("recovery_total_timeout").value();

    current_state_ = APPROACHING;
    has_ever_received_data_ = false;
    max_area_ratio_ = 0.0;
    align_stable_frames_ = 0;
    stable_tracking_ = false;
    sprint_duration_ = sprint_time_;
    prev_err_x_ = 0.0;
    prev_err_y_ = 0.0;
    prev_time_ = ros::Time::now();
    node_start_time_ = prev_time_;

    updateTargetData();

    ROS_INFO("[%s]: Visual Servo Started (VX+VZ+yaw_rate)", name().c_str());
    ROS_INFO("[%s]: forward_vel=%.2f sprint_vel=%.2f sprint_time<=%.2f (adaptive x%.2f, min %.2f) offset=(%.1f,%.1f) sprint_lift=%.2f lock=%d vy(kp=%.3f,max=%.2f) area_thresh=%.3f",
             name().c_str(), forward_velocity_, sprint_velocity_, sprint_time_,
             sprint_range_margin_, sprint_min_time_,
             hit_offset_x_, hit_offset_y_, sprint_lift_velocity_,
             (int)sprint_lock_visual_correction_, k_p_vy_, max_vy_, approach_area_threshold_);
    ROS_INFO("[%s]: recovery delay=%.2fs backoff=%.2fm/s %.2fs yaw=%.2f timeout=%.2fs",
             name().c_str(), recovery_delay_, recovery_backoff_velocity_, recovery_backoff_time_,
             recovery_yaw_rate_, recovery_total_timeout_);
    ROS_INFO("[%s]: strike_stable_time=%.2fs sprint_room_margin=%.2fm (boundary x[%.1f,%.1f] y[%.1f,%.1f])",
             name().c_str(), strike_stable_time_, sprint_room_margin_,
             boundary_x_min_, boundary_x_max_, boundary_y_min_, boundary_y_max_);
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
        double smooth_ratio = ratio * ratio * (3.0 - 2.0 * ratio);
        geometry_msgs::Point lift_target = post_hit_lift_start_pos_;
        lift_target.z = post_hit_lift_start_pos_.z + post_hit_lift_height_ * smooth_ratio;
        publishPosition(lift_target, post_hit_lift_yaw_);

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

    // 稳定跟踪计时：目标在视野中持续有效即累计（从首次可见起算，丢失即清零）。
    // 开火门要求该计时达到 strike_stable_time_ —— 用户语义："出现在视野 0.8s 后即打击"。
    bool stable_now = target_now_valid;
    if (stable_now) {
        if (!stable_tracking_) {
            stable_tracking_ = true;
            stable_track_start_ = now;
        }
    } else {
        stable_tracking_ = false;
    }
    double stable_dur = stable_tracking_ ? (now - stable_track_start_).toSec() : 0.0;

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
        // 启动宽限：跟踪器首帧数据还没到时原地悬停等待，
        // 避免 onStart 后立即误判丢失进 RECOVERY（后退+扫描反而扰动交接）
        if (!has_ever_received_data_ && (now - node_start_time_).toSec() < startup_grace_time_) {
            publishVelocityWithYawRate(0, 0, 0, 0);
            ROS_INFO_THROTTLE(0.5, "[%s]: Startup grace: waiting for first target data (%.2f/%.2fs)",
                              name().c_str(), (now - node_start_time_).toSec(), startup_grace_time_);
            return NodeStatus::RUNNING;
        }
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
            if (!isCentered()) {
                current_state_ = ALIGN;
                align_stable_frames_ = 0;
                publishDebugEvent("vs_state", "APPROACHING -> ALIGN");
                break;
            }
            if (current_ratio >= approach_area_threshold_) {
                // 已到开火距离，先过两道门再冲刺：
                // 1) 稳定跟踪门：目标在视野中持续有效满 strike_stable_time_ 秒
                // 2) 剩余空间门：机头方向剩余空间 >= 冲刺行程 + sprint_room_margin_
                // 未过门时原地悬停继续修正（小车目标会移动，几何会自愈）
                // 冲刺时长自适应：按面积实时解算距离 d_est=sqrt(0.03/ratio)
                // (0.3m 气球、f≈320px 标定)，时长 = d_est/v×余量，夹在 [min, sprint_time_]
                double d_est = std::sqrt(0.03 / std::max(current_ratio, 1e-6));
                sprint_duration_ = std::clamp(d_est / sprint_velocity_ * sprint_range_margin_,
                                              sprint_min_time_, sprint_time_);
                double room_need = sprint_velocity_ * sprint_duration_ + sprint_room_margin_;
                double room = roomAhead();
                if (stable_dur < strike_stable_time_) {
                    publishVelocityWithYawRate(0, 0, vz, yaw_rate);
                    ROS_INFO_THROTTLE(0.5, "[%s]: Fire hold: stable %.2f/%.2fs",
                                      name().c_str(), stable_dur, strike_stable_time_);
                    publishDebugEvent("vs_fire_hold", "stable=" + std::to_string(stable_dur));
                    break;
                }
                if (room < room_need) {
                    publishVelocityWithYawRate(0, 0, vz, yaw_rate);
                    ROS_WARN_THROTTLE(1.0, "[%s]: Fire hold: room %.2fm < %.2fm (wall ahead), keep tracking",
                                      name().c_str(), room, room_need);
                    publishDebugEvent("vs_fire_hold", "room=" + std::to_string(room) + " need=" + std::to_string(room_need));
                    break;
                }
                current_state_ = SPRINTING;
                sprint_start_time_ = ros::Time::now();
                ROS_INFO("[%s]: Area %.1f%% >= %.1f%% -> SPRINT! (stable %.2fs, d_est %.2fm, dur %.2fs, room %.2fm)",
                         name().c_str(), current_ratio * 100, approach_area_threshold_ * 100,
                         stable_dur, d_est, sprint_duration_, room);
                publishDebugEvent("vs_state", "APPROACHING -> SPRINTING");
            } else {
                publishVelocityWithYawRate(forward_velocity_, 0, vz, yaw_rate);
            }
            break;
        }

        case SPRINTING: {
            double sprint_dt = (ros::Time::now() - sprint_start_time_).toSec();
            if (sprint_dt >= sprint_duration_) {
                return startPostHitBrake("sprint finished");
            }
            if (sprint_lock_visual_correction_) {
                // 旧行为：全锁盲冲
                publishVelocityWithYawRate(sprint_velocity_, 0, -sprint_lift_velocity_, 0.0);
            } else {
                // 只锁 yaw（防止甩 FOV），vy/vz 保持视觉修正：
                // 横逃目标在冲刺段也能被 vy 咬住。err_x<0(目标偏左) -> vy>0(向左飞)
                double sprint_vy = std::clamp(-k_p_vy_ * err_x, -max_vy_, max_vy_);
                publishVelocityWithYawRate(sprint_velocity_, sprint_vy,
                                           vz - sprint_lift_velocity_, 0.0);
            }
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

double VisualServo::roomAhead() const {
    // 机头水平朝向射线到安全边界盒的最近距离（2D ENU）。
    // 已在界外时结果为负 -> 必然拦停冲刺，等悬停锁投影把飞机带回界内。
    const auto& p = fcu_odom_ptr->pose.pose.position;
    double yaw = PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation);
    double dx = std::cos(yaw), dy = std::sin(yaw);
    double room = std::numeric_limits<double>::infinity();
    if (dx > 1e-6) room = std::min(room, (boundary_x_max_ - p.x) / dx);
    else if (dx < -1e-6) room = std::min(room, (boundary_x_min_ - p.x) / dx);
    if (dy > 1e-6) room = std::min(room, (boundary_y_max_ - p.y) / dy);
    else if (dy < -1e-6) room = std::min(room, (boundary_y_min_ - p.y) / dy);
    return room;
}

NodeStatus VisualServo::startPostHitBrake(const std::string& reason) {
    lift_start_time_ = ros::Time::now();
    post_hit_lift_start_pos_ = fcu_odom_ptr->pose.pose.position;
    post_hit_lift_target_z_ = post_hit_lift_start_pos_.z + post_hit_lift_height_;
    post_hit_lift_yaw_ = PoseUtils::getYawFromQuat(fcu_odom_ptr->pose.pose.orientation);
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
