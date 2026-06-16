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
        BRAKING
    };

    ServoState current_state_;
    PixelTarget last_target_;
    ros::Time last_valid_detection_time_;
    bool has_ever_received_data_;
    double max_area_ratio_;
    ros::Time sprint_start_time_;
    ros::Time brake_start_time_;
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
    double sprint_time_;
    double post_hit_brake_velocity_;
    double post_hit_brake_time_;

    double data_timeout_;
    double approach_area_threshold_;
    double center_tolerance_;

    double prev_err_x_, prev_err_y_;
    ros::Time prev_time_;

    static constexpr int ALIGN_STABLE_FRAMES_REQUIRED = 5;

    void updateTargetData();
    bool isTargetLost();
    double targetAge() const;
    bool isCentered() const;
    NodeStatus startPostHitBrake(const std::string& reason);

    double computeYawRate(double err_x, double dt);
    double computeVz(double err_y, double dt);
};

#endif
