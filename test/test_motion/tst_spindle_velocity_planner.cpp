#include <gtest/gtest.h>
#include "spindle_velocity_planner.h"

namespace
{
constexpr double kControlPeriodSeconds = 0.001;
constexpr double kTolerance = 1e-9;

spindle_velocity_planner_t makePlanner(double current,
                                       double target,
                                       double acceleration,
                                       bool enabled)
{
    spindle_velocity_planner_t planner{};
    planner.current_vel = current;
    planner.target_vel = target;
    planner.max_acc = acceleration;
    planner.enable = enabled;
    return planner;
}
} // namespace

TEST(SpindleVelocityPlanner, AcceleratesByConfiguredStepWithoutOvershoot)
{
    spindle_velocity_planner_t planner =
        makePlanner(0.0, 1000.0, 2000.0, true);

    EXPECT_FALSE(spindle_velocity_planner_update(&planner,
                                                 kControlPeriodSeconds));
    EXPECT_NEAR(planner.current_vel, 2.0, kTolerance);
    EXPECT_TRUE(planner.active);
}

TEST(SpindleVelocityPlanner, SnapsToTargetInsideOneCycleStep)
{
    spindle_velocity_planner_t planner =
        makePlanner(998.5, 1000.0, 2000.0, true);

    EXPECT_TRUE(spindle_velocity_planner_update(&planner,
                                                kControlPeriodSeconds));
    EXPECT_NEAR(planner.current_vel, 1000.0, kTolerance);
    EXPECT_TRUE(planner.active);
}

TEST(SpindleVelocityPlanner, DisabledPlannerDeceleratesTowardZero)
{
    spindle_velocity_planner_t planner =
        makePlanner(-5.0, -1000.0, 2000.0, false);

    EXPECT_FALSE(spindle_velocity_planner_update(&planner,
                                                 kControlPeriodSeconds));
    EXPECT_NEAR(planner.current_vel, -3.0, kTolerance);
    EXPECT_TRUE(planner.active);

    EXPECT_FALSE(spindle_velocity_planner_update(&planner,
                                                 kControlPeriodSeconds));
    EXPECT_NEAR(planner.current_vel, -1.0, kTolerance);

    EXPECT_TRUE(spindle_velocity_planner_update(&planner,
                                                kControlPeriodSeconds));
    EXPECT_DOUBLE_EQ(planner.current_vel, 0.0);
    EXPECT_FALSE(planner.active);
}

TEST(SpindleVelocityPlanner, ReversesDirectionThroughZeroWithoutOvershoot)
{
    spindle_velocity_planner_t planner =
        makePlanner(1.0, -10.0, 2000.0, true);

    EXPECT_FALSE(spindle_velocity_planner_update(&planner,
                                                 kControlPeriodSeconds));
    EXPECT_NEAR(planner.current_vel, -1.0, kTolerance);
    EXPECT_TRUE(planner.active);
}

TEST(SpindleVelocityPlanner, RejectsInvalidArgumentsWithoutChangingState)
{
    spindle_velocity_planner_t planner =
        makePlanner(25.0, 100.0, 2000.0, true);

    EXPECT_FALSE(spindle_velocity_planner_update(nullptr,
                                                 kControlPeriodSeconds));
    EXPECT_FALSE(spindle_velocity_planner_update(&planner, 0.0));
    EXPECT_DOUBLE_EQ(planner.current_vel, 25.0);
}
