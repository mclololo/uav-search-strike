/****************************************************************************
 * @file       scan_for_target.h
 * @brief      ScanForTarget 节点 - 原地偏航搜索并确认目标进入视野
 ******************************************************************************/
#ifndef SCAN_FOR_TARGET_H
#define SCAN_FOR_TARGET_H

#include "base/target_detector_base.h"

class ScanForTarget : public TargetDetectorBase {
public:
    ScanForTarget(const std::string& name, const NodeConfig& config);

    static PortsList providedPorts();

    NodeStatus onStart() override;
    NodeStatus onRunning() override;

protected:
    NodeStatus handleDetectedObject(const common_msgs::Obj& obj) override;
    NodeStatus handleNoDetection() override;

private:
    enum class SearchPhase {
        RotateLeft,
        StrafeLeft,
        RotateRight,
        StrafeRight,
        BackOff
    };

    void advanceSearchPhase();
    const char* phaseName() const;
    void publishSearchCommand(double vx, double vy, double yaw_rate);

    geometry_msgs::Point staging_position_;
    bool hold_staging_position_{false};

    double yaw_rate_{0.35};
    double timeout_{8.0};
    double center_gate_x_ratio_{0.22};
    double center_gate_y_ratio_{0.35};
    double detection_hold_time_{0.6};
    double phase_duration_{2.5};
    double lateral_velocity_{0.18};
    double backward_velocity_{0.12};
    int stable_frames_required_{3};
    int stable_frames_{0};
    ros::Time start_time_;
    ros::Time phase_start_time_;
    ros::Time first_stable_time_;
    bool has_stable_detection_{false};
    SearchPhase current_phase_{SearchPhase::RotateLeft};
};

#endif // SCAN_FOR_TARGET_H
