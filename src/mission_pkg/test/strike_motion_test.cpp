#include <gtest/gtest.h>
#include "utils/pose_speed_estimator.h"
#include "utils/strike_motion.h"

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

TEST(PoseSpeed, MovingPoseIsNotStopped) {
    PoseSpeedEstimator speed;
    speed.update(10.0, 0, 0, 0.4);
    EXPECT_FALSE(std::isfinite(speed.speed(10.0)));
    speed.update(10.1, 0.1, 0, 0.4);
    EXPECT_NEAR(speed.speed(10.1), 1.0, 1e-9);
    EXPECT_GT(speed.speed(10.1), 0.12);
}

TEST(PoseSpeed, StationarySamplesEventuallyStop) {
    PoseSpeedEstimator speed;
    speed.update(10.0, 0, 0, 0.4);
    speed.update(10.1, 0.1, 0, 0.4);
    speed.update(10.2, 0.1, 0, 0.4);
    EXPECT_DOUBLE_EQ(speed.speed(10.2), 0.0);
}

TEST(PoseSpeed, VerticalMotionCountsAsMoving) {
    PoseSpeedEstimator speed;
    speed.update(10.0, 0, 0, 0.4);
    speed.update(10.1, 0, 0, 0.435);
    EXPECT_NEAR(speed.speed(10.1), 0.35, 1e-9);
}

TEST(PoseSpeed, StaleDuplicateAndRewoundSamplesCannotReportStopped) {
    PoseSpeedEstimator speed;
    speed.update(10.0, 0, 0, 0);
    speed.update(10.1, 0, 0, 0);
    speed.update(10.1, 5, 0, 0);
    EXPECT_DOUBLE_EQ(speed.speed(10.2), 0.0);
    EXPECT_FALSE(std::isfinite(speed.speed(10.5)));
    speed.update(9.0, 0, 0, 0);
    EXPECT_FALSE(std::isfinite(speed.speed(9.0)));
    speed.update(9.1, 0, 0, 0);
    EXPECT_DOUBLE_EQ(speed.speed(9.1), 0.0);
    speed.update(11.0, 0, 0, 0);
    EXPECT_FALSE(std::isfinite(speed.speed(11.0)));
}

TEST(PoseSpeed, HighRateAndInvalidInput) {
    PoseSpeedEstimator speed;
    for (int i = 0; i <= 20; ++i) speed.update(10.0 + i * 0.01, i * 0.01, 0, 0);
    EXPECT_NEAR(speed.speed(10.2), 1.0, 1e-9);
    speed.update(10.21, NAN, 0, 0);
    EXPECT_FALSE(std::isfinite(speed.speed(10.21)));
    speed.update(10.22, 0, 0, 0);
    EXPECT_FALSE(std::isfinite(speed.speed(10.22)));
}

TEST(StrikeMotion, RegionHeadingsAndMavrosTransform) {
    const double pi = std::acos(-1.0);
    for (double yaw : {pi, pi, -pi / 2}) {
        const auto v = StrikeMotion::corridorVelocity(yaw, 0.85, 0, 1.2, 0.16);
        EXPECT_NEAR(v.x, 0.85 * std::cos(yaw), 1e-9);
        EXPECT_NEAR(v.y, 0.85 * std::sin(yaw), 1e-9);
        const auto local = StrikeMotion::toMavrosLocal(v);
        EXPECT_DOUBLE_EQ(local.x, -v.y);
        EXPECT_DOUBLE_EQ(local.y, v.x);
    }
    const auto red = StrikeMotion::toMavrosLocal({-0.85, 0});
    const auto blue = StrikeMotion::toMavrosLocal({0, -0.85});
    EXPECT_DOUBLE_EQ(red.y, -0.85);
    EXPECT_DOUBLE_EQ(blue.x, 0.85);
}

TEST(StrikeMotion, BrakeTargetProjectionIsFixedOnCorridor) {
    const double yaw = std::acos(-1.0) / 4.0;
    const auto target = StrikeMotion::projectToCorridor(3.0, 2.0, 1.0, 1.0, yaw);
    const double cross_track = -std::sin(yaw) * (target.x - 1.0) +
                               std::cos(yaw) * (target.y - 1.0);
    EXPECT_NEAR(cross_track, 0.0, 1e-9);
    EXPECT_NEAR(target.x, 2.5, 1e-9);
    EXPECT_NEAR(target.y, 2.5, 1e-9);
}

TEST(StrikeMotion, CorrectionIsBoundedAndNeverReducesForwardSpeed) {
    for (double yaw : {0.0, 3.14, -1.57}) {
        for (double error : {-2.0, -0.05, 0.0, 0.05, 2.0}) {
            const auto v = StrikeMotion::corridorVelocity(yaw, 0.85, error, 1.2, 0.16);
            const double forward = v.x * std::cos(yaw) + v.y * std::sin(yaw);
            const double lateral = -v.x * std::sin(yaw) + v.y * std::cos(yaw);
            EXPECT_NEAR(forward, 0.85, 1e-9);
            EXPECT_LE(std::abs(lateral), 0.16 + 1e-9);
            EXPECT_LE(lateral * error, 1e-9);
        }
    }
}

TEST(StrikeMotion, PostHitBackoffStaysOnTraversedCorridorAndInsideBoundary) {
    const double pi = std::acos(-1.0);
    EXPECT_NEAR(StrikeMotion::boundedBackoffDistance(
        2.0, -2.3, pi, 3.0, 0.25, -0.6, 5.6, -2.6, 2.6), 0.25, 1e-9);
    EXPECT_NEAR(StrikeMotion::boundedBackoffDistance(
        5.5, -2.3, pi, 3.0, 0.25, -0.6, 5.6, -2.6, 2.6), 0.10, 1e-9);
    EXPECT_NEAR(StrikeMotion::boundedBackoffDistance(
        2.0, -2.3, pi, 0.12, 0.25, -0.6, 5.6, -2.6, 2.6), 0.02, 1e-9);
    EXPECT_NEAR(StrikeMotion::boundedBackoffDistance(
        2.0, 2.55, -pi / 2, 3.0, 0.25, -0.6, 5.6, -2.6, 2.6), 0.05, 1e-9);
}
