#pragma once

#include <cmath>
#include <limits>

// RflySim's pose-only Odometry leaves twist zero even while the vehicle moves.
class PoseSpeedEstimator {
public:
    void update(double stamp, double x, double y, double z) {
        if (!std::isfinite(stamp) || !std::isfinite(x) ||
            !std::isfinite(y) || !std::isfinite(z) || stamp <= 0.0) {
            initialized_ = valid_ = false;
            return;
        }
        if (!initialized_ || stamp < latest_stamp_ || stamp - latest_stamp_ > 0.5) {
            initialized_ = true;
            valid_ = false;
            anchor_stamp_ = latest_stamp_ = stamp;
            x_ = x; y_ = y; z_ = z;
            return;
        }
        if (stamp == latest_stamp_) return;
        latest_stamp_ = stamp;
        const double dt = stamp - anchor_stamp_;
        if (dt < 0.08) return;
        speed_ = std::sqrt((x - x_) * (x - x_) + (y - y_) * (y - y_) +
                           (z - z_) * (z - z_)) / dt;
        valid_ = true;
        anchor_stamp_ = stamp;
        x_ = x; y_ = y; z_ = z;
    }

    double speed(double now) const {
        if (!valid_ || !std::isfinite(now) || now < latest_stamp_ ||
            now - latest_stamp_ > 0.3) {
            return std::numeric_limits<double>::infinity();
        }
        return speed_;
    }

private:
    bool initialized_{false};
    bool valid_{false};
    double latest_stamp_{0.0}, anchor_stamp_{0.0};
    double x_{0.0}, y_{0.0}, z_{0.0}, speed_{0.0};
};
