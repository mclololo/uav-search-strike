/******************************************************************************
 * @file       planning_control.h
 * @brief      Planning control with an isolated safe-return mode
 ******************************************************************************/
#ifndef PLANNING_CONTROL_H
#define PLANNING_CONTROL_H

#include "base/drone_action_node.h"

#include <cstdint>
#include <mutex>
#include <quadrotor_msgs/PositionCommand.h>

class PlanningControl : public DroneActionNode {
public:
    PlanningControl(const std::string& name, const NodeConfig& config);

    static PortsList providedPorts();
    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;

private:
    void plannerCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg);
    void initializeTarget(const Position3D& pos);
    bool tryInitializeTarget();
    void publishReturnGoal();
    bool isStopped() const;
    void deactivateReturnPlanner();
    NodeStatus runBaselinePlanner();
    NodeStatus runSafeReturnPlanner();
    void applyPlannerCommand(const quadrotor_msgs::PositionCommand& planner_cmd);

    Position3D target_pos_;
    double tolerance{0.2};
    double offset_z{0.0};
    bool control_yaw{false};
    bool use_home_xy_{false};
    bool target_initialized{false};
    double initial_yaw_{0.0};
    double yaw_rate_limit_{0.35};
    double current_yaw_sp_{0.0};
    ros::Time last_loop_time_;

    ros::Publisher planner_goal_pub;
    ros::Subscriber planner_cmd_sub;
    quadrotor_msgs::PositionCommand last_planner_cmd;
    bool has_planner_cmd{false};

    double stop_speed_threshold_{0.12};
    double stop_hold_time_{0.30};
    double planner_wait_timeout_{6.0};
    double planner_cmd_timeout_{0.50};
    double goal_match_tolerance_{0.30};
    double max_command_distance_{1.00};
    ros::Time stop_hold_start_;
    ros::Time goal_publish_time_;
    ros::Time last_valid_cmd_time_;
    bool return_goal_published_{false};
    uint32_t latest_seen_trajectory_id_{0};
    uint32_t minimum_trajectory_id_{0};
    uint32_t accepted_trajectory_id_{0};
    mutable std::mutex planner_cmd_mutex_;
};

#endif
