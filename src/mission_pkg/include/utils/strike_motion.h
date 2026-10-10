#pragma once

#include <algorithm>
#include <cmath>

namespace StrikeMotion {
struct VelocityXY { double x; double y; };
struct PositionXY { double x; double y; };

inline PositionXY projectToCorridor(double x, double y, double reference_x,
                                    double reference_y, double yaw) {
    const double along = (x - reference_x) * std::cos(yaw) +
                         (y - reference_y) * std::sin(yaw);
    return {reference_x + along * std::cos(yaw),
            reference_y + along * std::sin(yaw)};
}

inline double boundedBackoffDistance(double x, double y, double route_yaw,
                                     double sprint_progress, double requested,
                                     double x_min, double x_max,
                                     double y_min, double y_max) {
    double distance = std::clamp(requested, 0.0, std::max(0.0, sprint_progress - 0.10));
    const double dx = -std::cos(route_yaw);
    const double dy = -std::sin(route_yaw);
    if (dx > 1e-6) distance = std::min(distance, (x_max - x) / dx);
    if (dx < -1e-6) distance = std::min(distance, (x_min - x) / dx);
    if (dy > 1e-6) distance = std::min(distance, (y_max - y) / dy);
    if (dy < -1e-6) distance = std::min(distance, (y_min - y) / dy);
    return std::max(0.0, distance);
}

inline VelocityXY corridorVelocity(double yaw, double forward, double cross_error,
                                   double gain, double max_correction) {
    const double lateral = std::clamp(-gain * cross_error, -max_correction, max_correction);
    return {forward * std::cos(yaw) - lateral * std::sin(yaw),
            forward * std::sin(yaw) + lateral * std::cos(yaw)};
}

// Use the same map-to-MAVROS rotation as position control and EGO trajectories.
inline VelocityXY toMavrosLocal(VelocityXY world) {
    return {-world.y, world.x};
}
}  // namespace StrikeMotion
