/****************************************************************************
 * @file       target_tracker.cpp
 * @brief      TargetTracker 节点实现
 *
 * @details    卡尔曼滤波像素跟踪：
 *             - 检测到目标：按实测 dt predict + 量测 correct，输出滤波后状态
 *             - 短暂漏检（宽限时间内）：仅 predict，输出速度外推的预测位置
 *             - 丢失超过宽限时间：输出无效目标并重置滤波器
 *             - 量测关联门控：检测框中心跳变超过门限时按漏检处理，
 *             避免多目标误分类（红/紫气球混淆）把滤波器拉飞
 *****************************************************************************/
#include "nodes/perception/target_tracker.h"

#include <cmath>

namespace {
// dt 限幅：BT tick 偶尔卡顿/抖动时避免外推爆炸或除零
constexpr double kMinDt = 1e-3;
constexpr double kMaxDt = 0.25;

inline double clampDt(double dt) {
    return std::min(std::max(dt, kMinDt), kMaxDt);
}
}  // namespace

TargetTracker::TargetTracker(const std::string& name, const NodeConfig& config)
    : TargetDetectorBase(name, config) {}

PortsList TargetTracker::providedPorts() {
    auto ports = baseProvidedPorts();
    // 已弃用：原为滑动平均窗口，现由卡尔曼滤波替代。保留端口以兼容旧的行为树 XML。
    ports.insert(InputPort<int>("filter_window_size", 10, "[已弃用] 现使用卡尔曼滤波，此参数无效"));
    ports.insert(InputPort<double>("lost_target_grace_time", 1.5, "目标短暂丢失时按预测保留目标的时间(s)"));
    ports.insert(InputPort<double>("kf_process_noise", 5000.0,
                                   "卡尔曼过程噪声(加速度谱密度 px^2/s^3)，越大跟踪响应越快"));
    ports.insert(InputPort<double>("kf_measurement_noise", 36.0,
                                   "卡尔曼量测噪声方差(px^2)，越大输出越平滑"));
    ports.insert(InputPort<double>("kf_gate_distance", 120.0,
                                   "量测关联门限(px)：检测中心偏离预测中心超过此值时视为误检/目标切换，0=关闭"));
    ports.insert(InputPort<double>("kf_adapt_innov_ref", 8.0,
                                   "新息自适应参考值(px)：新息均值超过此值时自动放大过程噪声提升机动响应，0=关闭自适应"));
    ports.insert(InputPort<double>("kf_adapt_max_scale", 8.0,
                                   "过程噪声自适应最大放大倍数"));
    ports.insert(OutputPort<PixelTarget>("pixel_target", "像素目标（中心+尺寸）"));
    return ports;
}

NodeStatus TargetTracker::onStart() {
    NodeStatus base_status = TargetDetectorBase::onStart();

    auto grace_res = getInput<double>("lost_target_grace_time");
    lost_target_grace_time_ = grace_res.value();
    auto proc_res = getInput<double>("kf_process_noise");
    kf_process_noise_ = proc_res.value();
    auto meas_res = getInput<double>("kf_measurement_noise");
    kf_measurement_noise_ = meas_res.value();
    auto gate_res = getInput<double>("kf_gate_distance");
    kf_gate_distance_ = gate_res.value();
    auto adapt_ref_res = getInput<double>("kf_adapt_innov_ref");
    kf_adapt_innov_ref_ = adapt_ref_res.value();
    auto adapt_scale_res = getInput<double>("kf_adapt_max_scale");
    kf_adapt_max_scale_ = adapt_scale_res.value();

    // 状态 8 维 [cx,vcx,cy,vcy,w,vw,h,vh]，量测 4 维 [cx,cy,w,h]，无控制输入
    kf_.init(8, 4, 0, CV_64F);

    // 量测矩阵 H：从状态中取出 4 个位置分量
    kf_.measurementMatrix = cv::Mat::zeros(4, 8, CV_64F);
    kf_.measurementMatrix.at<double>(0, 0) = 1.0;  // cx
    kf_.measurementMatrix.at<double>(1, 2) = 1.0;  // cy
    kf_.measurementMatrix.at<double>(2, 4) = 1.0;  // w
    kf_.measurementMatrix.at<double>(3, 6) = 1.0;  // h

    // 量测噪声 R
    cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar::all(kf_measurement_noise_));

    kf_initialized_ = false;
    has_pending_init_ = false;

    ROS_INFO("[%s]: Kalman pixel tracking, grace_time: %.2fs, Q: %.1f, R: %.1f, gate: %.1fpx, adapt_ref: %.1fpx(max x%.1f)",
             name().c_str(), lost_target_grace_time_, kf_process_noise_,
             kf_measurement_noise_, kf_gate_distance_,
             kf_adapt_innov_ref_, kf_adapt_max_scale_);
    return base_status;
}

void TargetTracker::initKalman(double cx, double cy, double w, double h, double now) {
    kf_.statePost = cv::Mat::zeros(8, 1, CV_64F);
    kf_.statePost.at<double>(0) = cx;
    kf_.statePost.at<double>(2) = cy;
    kf_.statePost.at<double>(4) = w;
    kf_.statePost.at<double>(6) = h;

    // 初始协方差：位置较确定，速度不确定性大
    cv::setIdentity(kf_.errorCovPost, cv::Scalar::all(1e4));
    kf_.errorCovPost.at<double>(0, 0) = 100.0;
    kf_.errorCovPost.at<double>(2, 2) = 100.0;
    kf_.errorCovPost.at<double>(4, 4) = 100.0;
    kf_.errorCovPost.at<double>(6, 6) = 100.0;
    kf_.errorCovPre = kf_.errorCovPost.clone();

    kf_initialized_ = true;
    last_detection_time_ = now;
    last_update_time_ = now;
    innov_ema_ = 0.0;
    has_pending_init_ = false;
}

void TargetTracker::updateKalmanTransition(double dt, double q) {
    // 状态转移矩阵 A：每个通道 [pos, vel] 满足 pos' = pos + vel*dt
    kf_.transitionMatrix = cv::Mat::eye(8, 8, CV_64F);
    kf_.transitionMatrix.at<double>(0, 1) = dt;
    kf_.transitionMatrix.at<double>(2, 3) = dt;
    kf_.transitionMatrix.at<double>(4, 5) = dt;
    kf_.transitionMatrix.at<double>(6, 7) = dt;

    // 过程噪声 Q：连续白噪声加速度模型(CWNA)，每个通道一个 2x2 块
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt3 * dt;
    const double q_pos = q * dt4 / 4.0;
    const double q_pos_vel = q * dt3 / 2.0;
    const double q_vel = q * dt2;

    kf_.processNoiseCov = cv::Mat::zeros(8, 8, CV_64F);
    const int channels[4] = {0, 2, 4, 6};
    for (int i : channels) {
        kf_.processNoiseCov.at<double>(i, i) = q_pos;
        kf_.processNoiseCov.at<double>(i, i + 1) = q_pos_vel;
        kf_.processNoiseCov.at<double>(i + 1, i) = q_pos_vel;
        kf_.processNoiseCov.at<double>(i + 1, i + 1) = q_vel;
    }
}

PixelTarget TargetTracker::stateToPixelTarget(const cv::Mat& state, double now, bool detected) const {
    return PixelTarget(
        state.at<double>(0),                    // cx
        state.at<double>(2),                    // cy
        std::max(1.0, state.at<double>(4)),     // w（下限保护）
        std::max(1.0, state.at<double>(6)),     // h（下限保护）
        now,
        detected
    );
}

NodeStatus TargetTracker::handleDetectedObject(const common_msgs::Obj& obj) {
    double pixel_width = obj.right_bottom_x - obj.left_top_x;
    double pixel_height = obj.right_bottom_y - obj.left_top_y;

    if (pixel_width <= 0 || pixel_height <= 0) {
        return handleNoDetection();
    }

    const double now = ros::Time::now().toSec();

    if (!kf_initialized_) {
        // 重捕获共识：要求连续两帧位置一致的检测才初始化滤波器。
        // 颜色误分类会让其他气球顶着目标类名瞬时出现一两帧，
        // 若见检即初始化，滤波器会锁到错误目标上（曾导致无人机追错气球）。
        constexpr double kPendingTimeout = 1.0;  // 首帧等待的有效期(s)
        // 共识距离：跟随门控门限；门控关闭时用默认 120px
        const double consensus_dist = (kf_gate_distance_ > 0.0) ? kf_gate_distance_ : 120.0;
        bool consensus = false;
        if (has_pending_init_ && (now - pending_time_) <= kPendingTimeout) {
            consensus = (std::hypot(obj.center_x - pending_cx_,
                                    obj.center_y - pending_cy_) <= consensus_dist);
        }
        if (consensus) {
            initKalman(obj.center_x, obj.center_y, pixel_width, pixel_height, now);
        } else {
            pending_cx_ = obj.center_x;
            pending_cy_ = obj.center_y;
            pending_time_ = now;
            has_pending_init_ = true;
            // 共识未达成：本拍按无有效目标处理（不初始化、不采纳该检测）
            return handleNoDetection();
        }
    } else {
        // 检测中心与滤波器当前位置的偏差（近似新息；v*dt 误差相对像素级偏差可忽略）
        const double dx = obj.center_x - kf_.statePost.at<double>(0);
        const double dy = obj.center_y - kf_.statePost.at<double>(2);
        const double jump = std::hypot(dx, dy);

        // 量测关联门控：偏差过远判定为误检/其他目标
        // （典型场景：远处红紫气球互相误分类导致识别框在两者之间跳变），
        // 本次量测不送入滤波器，按漏检处理（惯性外推，超时后自然重置重捕获）。
        // 门限随跟踪目标的尺寸自适应：远处目标小（多目标混淆场景）保持基础门限；
        // 近距离目标图像大、帧间位移大，门限放宽，避免末端逼近时把真实检测误杀。
        if (kf_gate_distance_ > 0.0) {
            constexpr double kGateSizeScale = 1.5;  // 门限 = max(基础值, 1.5 * 目标边长)
            const double tracked_size = std::max(
                std::max(kf_.statePost.at<double>(4), kf_.statePost.at<double>(6)), 1.0);
            const double gate = std::max(kf_gate_distance_, kGateSizeScale * tracked_size);
            if (jump > gate) {
                ROS_WARN_THROTTLE(0.5, "[%s]: Detection gated out (jump=%.1fpx > %.1fpx), coasting",
                                  name().c_str(), jump, gate);
                publishDebugEvent("track_gated", "jump=" + std::to_string(jump));
                return handleNoDetection();
            }
        }

        // 新息自适应：偏差持续偏大说明目标在机动（如直线追击逼近），
        // 临时放大过程噪声 Q 提升响应；平稳时自动回落保持平滑。
        innov_ema_ = 0.6 * innov_ema_ + 0.4 * jump;
        double q_eff = kf_process_noise_;
        if (kf_adapt_innov_ref_ > 0.0) {
            const double scale = std::min(innov_ema_ / kf_adapt_innov_ref_, kf_adapt_max_scale_);
            q_eff *= std::max(1.0, scale);
        }

        const double dt = clampDt(now - last_update_time_);
        updateKalmanTransition(dt, q_eff);
        kf_.predict();

        cv::Mat measurement = (cv::Mat_<double>(4, 1) <<
                               obj.center_x, obj.center_y, pixel_width, pixel_height);
        kf_.correct(measurement);

        last_detection_time_ = now;
        last_update_time_ = now;
    }

    PixelTarget filtered = stateToPixelTarget(kf_.statePost, now, true);
    setOutput("pixel_target", filtered);

    ROS_INFO_THROTTLE(0.5, "[%s]: KF tracking pixel target: center=(%.1f, %.1f), size=(%.1f x %.1f)",
                      name().c_str(), filtered.center_x, filtered.center_y,
                      filtered.width, filtered.height);
    publishDebugEvent("track_update", "center=(" + std::to_string(filtered.center_x) + "," +
                      std::to_string(filtered.center_y) + ") area=" + std::to_string(filtered.area()));

    return NodeStatus::RUNNING;
}

NodeStatus TargetTracker::handleNoDetection() {
    const double now = ros::Time::now().toSec();

    // 宽限时间内：仅做 predict，输出速度外推的预测位置（替代原来的冻结保持）
    if (kf_initialized_ && (now - last_detection_time_) <= lost_target_grace_time_) {
        const double dt = clampDt(now - last_update_time_);
        updateKalmanTransition(dt, kf_process_noise_);
        const cv::Mat& predicted = kf_.predict();
        last_update_time_ = now;

        PixelTarget predicted_target = stateToPixelTarget(predicted, now, true);
        // 标记为外推帧：下游（VisualServo）据此不把它当作真实检测，
        // 命中（气球爆裂）后的丢失判定不再被外推宽限掩盖
        predicted_target.extrapolated = true;
        setOutput("pixel_target", predicted_target);
        ROS_WARN_THROTTLE(1.0, "[%s]: Target temporarily lost, KF predicting: center=(%.1f, %.1f)",
                          name().c_str(), predicted_target.center_x, predicted_target.center_y);
        publishDebugEvent("track_predict", "kf prediction, center=(" +
                          std::to_string(predicted_target.center_x) + "," +
                          std::to_string(predicted_target.center_y) + ")");
        return NodeStatus::RUNNING;
    }

    // 丢失超时：输出无效目标并重置滤波器
    PixelTarget empty_target;
    empty_target.timestamp = now;
    empty_target.is_detected = false;
    setOutput("pixel_target", empty_target);
    kf_initialized_ = false;
    ROS_WARN_THROTTLE(1.0, "[%s]: No target detected, publishing INVALID target", name().c_str());
    publishDebugEvent("track_lost", "no valid target");
    return NodeStatus::RUNNING;
}

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<TargetTracker>("TargetTracker");
}
