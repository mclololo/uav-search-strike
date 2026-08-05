/****************************************************************************
 * @file       visual_servo.h
 * @brief      Corridor-constrained balloon strike controller
 ******************************************************************************/
#ifndef VISUAL_SERVO_H
#define VISUAL_SERVO_H

#include "base/drone_action_node.h"

#include <limits>
#include <mutex>
#include <ros/time.h>
#include <sensor_msgs/PointCloud2.h>

class VisualServo : public DroneActionNode {
public:
    VisualServo(const std::string& name, const NodeConfig& config);

    static PortsList providedPorts();
    NodeStatus onStart() override;
    NodeStatus onRunning() override;

private:
    enum class ServoState {
        ALIGN,
        DESCENDING,
        APPROACHING,
        SPRINTING,
        BRAKING,
        ENTERING_WAIT,
        WAITING,
        SETTLING,
        RECOVERY
    };

    enum class BrakeOutcome {
        HIT,
        MISS_TO_WAIT,
        ABORT
    };

    ServoState state_{ServoState::ALIGN};
    BrakeOutcome brake_outcome_{BrakeOutcome::ABORT};

    PixelTarget last_target_;
    ros::Time last_valid_detection_time_;
    ros::Time node_start_time_;
    ros::Time state_start_time_;
    ros::Time stable_track_start_;
    ros::Time brake_hold_start_;
    ros::Time settle_start_time_;
    ros::Time wait_last_target_time_;
    bool has_ever_received_data_{false};
    bool close_target_seen_{false};
    double max_area_ratio_{0.0};
    int align_stable_frames_{0};
    mutable std::mutex target_mutex_;

    geometry_msgs::Point corridor_origin_;
    geometry_msgs::Point wait_entry_origin_;
    double route_yaw_{0.0};
    double corridor_limit_distance_{0.0};
    double wait_entry_distance_{0.0};
    bool geometry_valid_{false};
    std::string finish_reason_;

    double img_cx_{320.0};
    double img_cy_{240.0};
    int img_w_{640};
    int img_h_{480};
    double rgb_hfov_deg_{90.0};
    double rgb_focal_px_{320.0};
    double depth_image_y_sign_{1.0};
    double depth_image_z_sign_{1.0};

    double k_p_yaw_rate_{0.004};
    double k_d_yaw_rate_{0.008};
    double max_yaw_rate_{0.7};
    double deadzone_yaw_{8.0};
    double k_p_vz_{0.002};
    double k_d_vz_{0.006};
    double max_vz_{0.3};
    double deadzone_vz_{10.0};
    double prev_err_x_{0.0};
    double prev_err_y_{0.0};
    ros::Time prev_time_;

    double forward_velocity_{0.35};
    double sprint_velocity_{0.85};
    double sprint_time_{6.0};
    bool sprint_lock_visual_correction_{false};
    double hit_offset_x_{45.0};
    double hit_offset_y_{0.0};
    double hit_offset_reference_depth_{0.80};
    double max_hit_offset_ratio_{0.35};
    double approach_area_threshold_{0.035};
    double post_hit_success_area_ratio_{0.025};
    double center_tolerance_{0.12};
    double strike_stable_time_{0.15};
    double sprint_alignment_tolerance_px_{24.0};
    double sprint_vertical_alignment_tolerance_px_{160.0};
    int sprint_alignment_stable_frames_{2};
    double sprint_altitude_tolerance_{0.04};
    double sprint_vertical_speed_tolerance_{0.06};
    double sprint_altitude_hold_gain_{1.5};
    double sprint_max_vertical_correction_{0.08};
    double sprint_target_loss_grace_{0.60};
    double data_timeout_{0.30};
    double startup_grace_time_{1.0};
    double max_alignment_yaw_deviation_{0.15};

    double strike_altitude_{0.40};
    double descent_velocity_{0.22};
    double descent_tolerance_{0.05};
    double descent_timeout_{4.0};
    double corridor_reference_length_{5.20};
    double configured_corridor_yaw_{0.0};
    bool has_configured_corridor_yaw_{false};
    double sprint_limit_ratio_{0.78};
    double wait_entry_ratio_{0.35};
    double heading_hold_gain_{1.5};
    double heading_hold_max_rate_{0.30};
    double wait_velocity_{0.16};
    double wait_timeout_{12.0};
    double wait_no_target_timeout_{0.5};
    double wait_contact_distance_{0.42};

    double brake_speed_threshold_{0.12};
    double brake_hold_time_{0.30};
    double brake_timeout_{1.80};
    double post_hit_settle_time_{0.45};

    double recovery_delay_{0.15};
    double recovery_yaw_rate_{0.45};
    double recovery_total_timeout_{3.0};

    ros::Subscriber depth_cloud_sub_;
    mutable std::mutex depth_mutex_;
    ros::Time depth_cloud_time_;
    bool depth_frame_valid_{false};
    double hard_front_clearance_{std::numeric_limits<double>::infinity()};
    double target_depth_{std::numeric_limits<double>::infinity()};
    double target_lateral_{std::numeric_limits<double>::infinity()};
    double target_closing_speed_{0.0};
    double previous_target_depth_{std::numeric_limits<double>::infinity()};
    ros::Time previous_target_depth_time_;
    int hard_corridor_points_{0};
    double depth_timeout_{0.20};
    double depth_min_valid_range_{0.12};
    int depth_min_points_{8};
    double target_depth_band_{0.22};
    double target_depth_quantile_{0.30};
    double target_bbox_margin_px_{5.0};
    double target_zone_near_distance_{0.15};
    double target_zone_far_ratio_{0.80};
    double target_zone_lateral_limit_{0.55};
    double target_zone_fallback_area_ratio_{0.002};
    double target_closing_speed_threshold_{0.15};
    double target_ttc_threshold_{2.0};
    double target_intercept_margin_px_{12.0};
    double target_carrier_depth_margin_{0.55};
    double target_carrier_below_margin_px_{120.0};
    double corridor_half_width_{0.38};
    double corridor_half_height_{0.32};
    double emergency_clearance_{0.48};
    double contact_distance_{0.35};
    double braking_acceleration_{0.8};
    double control_latency_{0.15};

    double min_attack_altitude_{0.30};
    double altitude_guard_margin_{0.05};
    double altitude_recovery_speed_{0.18};
    double safety_predict_time_{0.45};
    double boundary_margin_{0.40};
    double boundary_x_min_{-10.0};
    double boundary_x_max_{10.0};
    double boundary_y_min_{-10.0};
    double boundary_y_max_{10.0};

    static constexpr int kAlignStableFrames = 5;

    void updateTargetData();
    bool getTargetSnapshot(PixelTarget& target, double& age) const;
    double desiredHitCenterX() const;
    bool isCentered(const PixelTarget& target, double desired_center_x) const;
    double computeYawRate(double err_x, double dt);
    double computeVz(double err_y, double dt);
    double alignmentYawRate(double visual_yaw_rate) const;
    double headingHoldYawRate() const;

    double currentSpeed() const;
    double boundaryClearance() const;
    double stoppingDistance(double speed) const;
    double corridorProgress(const geometry_msgs::Point& origin) const;
    double remainingSprintDistance() const;

    void depthCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    bool getDepthSnapshot(double& hard_clearance, double& target_depth) const;
    bool getDepthSnapshot(double& hard_clearance, double& target_depth,
                          double& target_lateral, double& target_closing_speed) const;
    bool targetInSprintZone(const PixelTarget& target, double area_ratio,
                            double& target_depth, double& target_longitudinal,
                            double& target_lateral, double& target_closing_speed) const;
    bool emergencyObstacleDetected() const;
    bool boundaryCommandSafe(double vx_body, double vy_body) const;
    bool publishGuardedVelocity(double vx, double vy, double vz, double yaw_rate,
                                double required_clearance, bool require_depth);

    NodeStatus startBrake(BrakeOutcome outcome, const std::string& reason);
    void enterSprint(const ros::Time& now);
    NodeStatus executeSprint(bool target_visible, double target_age, double area_ratio,
                             double visual_yaw_rate);
    NodeStatus executeBrake();
    NodeStatus executeWaitEntry();
    NodeStatus executeWaiting();
    NodeStatus startRecovery();
    NodeStatus executeRecovery();
    NodeStatus finishAfterSettle();
};

#endif
