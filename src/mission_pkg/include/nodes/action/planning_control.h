/******************************************************************************
 * @file       planning_control.h
 * @brief      Planning control with an isolated safe-return mode
 ******************************************************************************/
#ifndef PLANNING_CONTROL_H
#define PLANNING_CONTROL_H

#include "base/drone_action_node.h"

#include <cstdint>
#include <geometry_msgs/PointStamped.h>
#include <mutex>
#include <quadrotor_msgs/PositionCommand.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Time.h>

class PlanningControl : public DroneActionNode {
public:
    PlanningControl(const std::string& name, const NodeConfig& config);

    static PortsList providedPorts();
    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;

private:
    void plannerCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg);
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void returnMapCallback(const std_msgs::Time::ConstPtr& msg);
    void publishReturnMapMode(bool active);
    bool hasFreshCloud() const;
    bool hasCloudSince(const ros::Time& time) const;
    void initializeTarget(const Position3D& pos);
    bool tryInitializeTarget();
    void publishReturnGoal();
    void deactivateReturnPlanner();
    NodeStatus runBaselinePlanner();
    NodeStatus runSafeReturnPlanner();
    NodeStatus finishReturn();
    void publishReturnBrake();
    void publishReturnYawHold(double target_yaw, double yaw_rate, bool hold_return_position);
    void applyPlannerCommand(const quadrotor_msgs::PositionCommand& planner_cmd);

    Position3D target_pos_;
    double tolerance{0.2};
    double offset_z{0.0};
    double offset_x_{0.0};
    double offset_y_{0.0};
    bool control_yaw{false};
    bool use_home_xy_{false};
    bool final_yaw_enabled_{false};
    bool final_yaw_active_{false};
    bool return_heading_pending_{false};
    bool return_heading_aligned_{false};
    double return_heading_target_{0.0};
    geometry_msgs::Point return_hold_position_;
    bool return_hold_active_{false};
    double return_min_altitude_{0.0};
    ros::Time return_mode_start_;
    ros::Time return_map_time_;
    double final_yaw_{0.0};
    double final_yaw_rate_{0.25};
    bool target_initialized{false};
    double initial_yaw_{0.0};
    double yaw_rate_limit_{0.35};
    double current_yaw_sp_{0.0};
    ros::Time last_loop_time_;

    ros::Publisher planner_goal_pub;
    ros::Subscriber planner_cmd_sub;
    ros::Subscriber cloud_sub_;
    ros::Subscriber return_map_sub_;
    ros::Publisher return_map_pub_;
    quadrotor_msgs::PositionCommand last_planner_cmd;
    bool has_planner_cmd{false};

    double planner_wait_timeout_{6.0};
    double planner_cmd_timeout_{0.50};
    double cloud_timeout_{0.80};
    double stop_speed_threshold_{0.12};
    double stop_hold_time_{0.30};
    ros::Time stop_hold_start_;
    ros::Time return_scan_start_;
    ros::Time arrival_stop_hold_start_;
    ros::Time arrival_hold_start_;
    double goal_match_tolerance_{0.30};
    double max_command_distance_{1.00};
    ros::Time goal_publish_time_;
    ros::Time last_valid_cmd_time_;
    ros::Time baseline_stall_start_;
    bool baseline_cloud_lost_{false};
    bool baseline_replan_pending_{false};
    bool return_goal_published_{false};
    uint32_t latest_seen_trajectory_id_{0};
    uint32_t minimum_trajectory_id_{0};
    uint32_t accepted_trajectory_id_{0};
    mutable std::mutex planner_cmd_mutex_;
    mutable std::mutex cloud_mutex_;
    ros::Time last_cloud_time_;
};

#endif
