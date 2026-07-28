/****************************************************************************
 * @file       visual_servo.h
 * @brief      Visual Servo - 三阶段控制（body系 VX+VZ+yaw_rate 控制版）
 * 
 * @details    ALIGN → APPROACH → SPRINT
 *             全程使用 publishVelocityWithYawRate (body 坐标系)
 ******************************************************************************/
#ifndef VISUAL_SERVO_H
#define VISUAL_SERVO_H

#include "base/drone_action_node.h"
#include <ros/time.h>

class VisualServo : public DroneActionNode {
public:
    VisualServo(const std::string& name, const NodeConfig& config);
    static PortsList providedPorts();
    virtual NodeStatus onStart() override;
    virtual NodeStatus onRunning() override;

private:
    enum ServoState {
        ALIGN,
        APPROACHING,
        SPRINTING,
        POST_HIT_LIFTING,
        BRAKING,
        SETTLING,
        RECOVERY
    };

    ServoState current_state_;
    PixelTarget last_target_;
    ros::Time last_valid_detection_time_;
    bool has_ever_received_data_;
    double max_area_ratio_;
    ros::Time sprint_start_time_;
    ros::Time brake_start_time_;
    ros::Time lift_start_time_;
    ros::Time settle_start_time_;
    geometry_msgs::Point post_hit_lift_start_pos_;
    double post_hit_lift_target_z_;
    double post_hit_lift_yaw_;
    int align_stable_frames_;

    double img_cx_, img_cy_;
    int img_w_, img_h_;

    double k_p_yaw_rate_;
    double k_d_yaw_rate_;
    double max_yaw_rate_;
    double deadzone_yaw_;

    double k_p_vz_;
    double k_d_vz_;
    double max_vz_;
    double deadzone_vz_;

    double forward_velocity_;
    double sprint_velocity_;
    double sprint_time_;        // 自适应冲刺时长上限 (s)
    double sprint_duration_;    // 开火瞬间按 d_est 算好的本次冲刺时长 (s)
    double sprint_range_margin_;// 冲刺时长 = d_est/v × 该系数（余量）
    double sprint_min_time_;    // 自适应冲刺时长下限 (s)
    double k_p_vy_;             // 冲刺段横向视觉修正 P 增益 (m/s per pixel)
    double max_vy_;             // 冲刺段横向速度限幅 (m/s)
    double startup_grace_time_; // 启动宽限：未收到过目标数据时该时长内不进 RECOVERY (s)
    double hit_offset_x_;
    double hit_offset_y_;
    double sprint_lift_velocity_;
    bool sprint_lock_visual_correction_;
    double post_hit_brake_velocity_;
    double post_hit_brake_time_;
    double post_hit_lift_height_;
    double post_hit_lift_time_;
    double post_hit_lift_timeout_;
    double post_hit_lift_tolerance_;
    double post_hit_settle_time_;

    double data_timeout_;
    double approach_area_threshold_;
    double post_hit_success_area_ratio_;
    double center_tolerance_;

    // 稳定跟踪开火门：目标在视野中持续有效达到该时长才允许进入冲刺
    // （语义：从首次可见起算，用户要求"出现在 yolo 视野 0.8s 后即打击"；
    //  开火当刻仍需居中+面积达标，近距离交接的居中时间天然提供 PD/KF 收敛）。
    double strike_stable_time_;
    bool stable_tracking_;
    ros::Time stable_track_start_;
    ros::Time node_start_time_;

    // 开火前剩余空间守卫：机头水平朝向到安全边界盒的距离
    // < 冲刺行程+余量 时不冲刺（保持跟踪等目标离开墙角），从几何上杜绝贴墙开火。
    double sprint_room_margin_;
    double boundary_x_min_, boundary_x_max_, boundary_y_min_, boundary_y_max_;

    // 目标丢失后的主动恢复参数
    double recovery_delay_;            // 丢失多久后进入恢复 (s)
    double recovery_backoff_velocity_; // 后退速度 (m/s, 负值)
    double recovery_backoff_time_;     // 后退持续时间 (s)
    double recovery_yaw_rate_;         // 恢复阶段扫描角速度 (rad/s)
    double recovery_total_timeout_;    // 恢复阶段总超时 (s)
    ros::Time recovery_start_time_;

    double prev_err_x_, prev_err_y_;
    ros::Time prev_time_;

    static constexpr int ALIGN_STABLE_FRAMES_REQUIRED = 5;

    void updateTargetData();
    bool isTargetLost();
    double targetAge() const;
    bool isCentered() const;
    NodeStatus startPostHitBrake(const std::string& reason);

    // 目标丢失恢复
    NodeStatus startRecovery();
    NodeStatus executeRecovery();

    double computeYawRate(double err_x, double dt);
    double computeVz(double err_y, double dt);

    // 机头水平朝向到安全边界盒的剩余距离（ENU，odom 与 safety/* 同系）
    double roomAhead() const;
};

#endif
