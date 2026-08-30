/****************************************************************************
 * @file       target_tracker.h
 * @brief      TargetTracker 节点 - 卡尔曼滤波像素跟踪
 *
 * @details    持续输出 pixel_target 用于视觉伺服
 *             设计：对 (center_x, center_y, width, height) 四个通道分别建立
 *             恒速(Constant Velocity)卡尔曼滤波模型，平滑 YOLO 检测抖动，
 *             并在短暂漏检时按速度外推预测目标位置（替代原有的滑动平均+
 *             冻结保持策略）。
 *             另带量测关联门控：检测中心与滤波器预测中心距离过远时视为
 *             误检/他目标（如远处红紫气球互相误分类导致的框跳变），按漏检
 *             处理，避免滤波器被拉飞。
 *             新息自适应：新息（量测-预测偏差）持续偏大说明目标在机动
 *             （如直线追击逼近），自动放大过程噪声提升响应；平稳时回落，
 *             兼顾"稳态平滑"与"机动响应"。
 ******************************************************************************/
#ifndef TARGET_TRACKER_H
#define TARGET_TRACKER_H

#include "base/target_detector_base.h"
#include <opencv2/video/tracking.hpp>  // cv::KalmanFilter

class TargetTracker : public TargetDetectorBase {
public:
    TargetTracker(const std::string& name, const NodeConfig& config);

    static PortsList providedPorts();

    NodeStatus onStart() override;

protected:
    NodeStatus handleDetectedObject(const common_msgs::Obj& obj) override;
    NodeStatus handleNoDetection() override;

private:
    /**
     * @brief 卡尔曼滤波器，状态向量 8 维:
     *        [cx, vcx, cy, vcy, w, vw, h, vh]
     *        量测向量 4 维: [cx, cy, w, h]
     */
    cv::KalmanFilter kf_;
    bool kf_initialized_{false};

    double last_detection_time_{0.0};  // 上一次真实检测到目标的时间（用于丢失宽限判断）
    double last_update_time_{0.0};     // 上一次滤波器 predict/correct 的时间（用于 dt 计算）

    double lost_target_grace_time_{1.5};
    double kf_process_noise_{5000.0};      // 过程噪声：加速度谱密度 (px^2/s^3)，越大响应越快
    double kf_measurement_noise_{36.0};    // 量测噪声方差 (px^2)，越大输出越平滑
    double kf_gate_distance_{120.0};       // 量测关联门限 (px)，0 = 关闭门控
    double kf_adapt_innov_ref_{8.0};       // 新息自适应参考值 (px)，0 = 关闭自适应
    double kf_adapt_max_scale_{8.0};       // 过程噪声自适应最大放大倍数
    double innov_ema_{0.0};                // 新息幅值的指数滑动平均

    // 重捕获共识：滤波器未初始化时，要求连续两帧位置一致的检测才初始化，
    // 防止颜色误分类的瞬时错误检测把滤波器引到错误气球上
    bool has_pending_init_{false};
    double pending_cx_{0.0}, pending_cy_{0.0};
    double pending_time_{0.0};

    // 用首个有效检测初始化滤波器
    void initKalman(double cx, double cy, double w, double h, double now);

    // 根据实测 dt 更新状态转移矩阵 A 与过程噪声 Q（连续白噪声加速度模型）
    void updateKalmanTransition(double dt, double q);

    // 从滤波器状态构造输出（对宽高做下限保护）
    PixelTarget stateToPixelTarget(const cv::Mat& state, double now, bool detected) const;
};

#endif // TARGET_TRACKER_H
