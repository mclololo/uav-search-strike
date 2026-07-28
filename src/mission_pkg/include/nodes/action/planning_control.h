/******************************************************************************
 * @file       planning_control.h
 * @brief      Planning Control 行为树节点声明
 *
 * @author     郭定<1604289397@qq.com>
 * @date       2026-01-06
 ******************************************************************************/
#ifndef PLANNING_CONTROL_H
#define PLANNING_CONTROL_H

#include "base/drone_action_node.h"
#include <quadrotor_msgs/PositionCommand.h>

class PlanningControl : public DroneActionNode {
public:
    PlanningControl(const std::string& name, const NodeConfig& config);
    
    static PortsList providedPorts();

    virtual NodeStatus onStart() override;
    virtual NodeStatus onRunning() override;

private:
   void plannerCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg);
   void initializeTarget(const Position3D& pos);

    Position3D target_pos_;
    Position3D raw_goal_;       // 原始目标（未做 offset 投影），看门狗重发目标时用，避免二次投影
    double tolerance;
    double offset_z;
    bool control_yaw;
    double initial_yaw_{0.0};  // Starting yaw when control_yaw is false
    double yaw_rate_limit_{0.35};
    double current_yaw_sp_{0.0};
    ros::Time last_loop_time_;
    bool target_initialized;

    ros::Publisher planner_goal_pub;
    ros::Subscriber planner_cmd_sub;
    quadrotor_msgs::PositionCommand last_planner_cmd;
    bool has_planner_cmd;

    // 卡死恢复看门狗：规划器长时间无新指令 或 距离长期不收敛 时重发目标。
    // 背景：ego-planner 的 EXEC_TRAJ 以"轨迹时间走完"即收工（与实际是否到达脱钩），
    // 而本节点原先目标只发一次、双方均无重试 -> 一旦规划器收工/楔死即永久悬停
    // （15:20 任务实测：打击后返航目标一次性下发，ego 未执行，悬停 16 分钟未返航）。
    ros::Time last_cmd_recv_time_;      // 最近一次收到规划器 pos_cmd 的时刻
    ros::Time last_progress_time_;      // 最近一次到目标距离明显缩短的时刻
    double last_dist_{1e9};
    double republish_interval_{5.0};    // 无新指令超过该时长重发目标 (s)，0=禁用
    double stuck_timeout_{8.0};         // 距离不收敛超过该时长重发目标 (s)，0=禁用
    int republish_count_{0};
};

#endif // PLANNING_CONTROL_H
