#include <gtest/gtest.h>

#include <adas_common/lateral_controllers.hpp>

using namespace adas_common;

namespace
{
const double lookahead = 1.0;
const double axleDistance = 0.3;
const double dt = 0.033;
const tf::Pose goal(tf::Quaternion(0, 0, 0, 1), tf::Vector3(2.5, 1.5, 0.0));
}

TEST(Intersection, TrivialAngle)
{
    auto result = findIntersection({0, 0, 0}, {0, 1, 0}, 0);
    ASSERT_DOUBLE_EQ(result.getX(), 0);
    ASSERT_DOUBLE_EQ(result.getY(), 1);
    ASSERT_DOUBLE_EQ(result.getZ(), 0);

    result = findIntersection({0, 0, 0}, {1, 0, 0}, 0);
    ASSERT_DOUBLE_EQ(result.getX(), 0);
    ASSERT_DOUBLE_EQ(result.getY(), 0);
    ASSERT_DOUBLE_EQ(result.getZ(), 0);

    result = findIntersection({0, 0, 0}, {1, 0, 0}, M_PI / 2);
    ASSERT_NEAR(result.getX(), 1, 1e-6);
    ASSERT_NEAR(result.getY(), 0, 1e-6);
    ASSERT_DOUBLE_EQ(result.getZ(), 0);
}

TEST(Intersection, TrivialSlope)
{
    auto result = findIntersection({0, 0, 0}, {0, 1, 0}, {1, 0, 0});
    ASSERT_DOUBLE_EQ(result.getX(), 0);
    ASSERT_DOUBLE_EQ(result.getY(), 1);
    ASSERT_DOUBLE_EQ(result.getZ(), 0);

    result = findIntersection({0, 0, 0}, {1, 0, 0}, {1, 0, 0});
    ASSERT_DOUBLE_EQ(result.getX(), 0);
    ASSERT_DOUBLE_EQ(result.getY(), 0);
    ASSERT_DOUBLE_EQ(result.getZ(), 0);

    result = findIntersection({0, 0, 0}, {1, 0, 0}, {0, 1, 0});
    ASSERT_DOUBLE_EQ(result.getX(), 1);
    ASSERT_DOUBLE_EQ(result.getY(), 0);
    ASSERT_DOUBLE_EQ(result.getZ(), 0);
}

TEST(Intersection, Angle)
{
    auto result = findIntersection({1, 2, 0}, {3, 2, 0}, -M_PI / 4);
    ASSERT_DOUBLE_EQ(result.getX(), 2);
    ASSERT_DOUBLE_EQ(result.getY(), 3);
    ASSERT_DOUBLE_EQ(result.getZ(), 0);
}

TEST(Intersection, Slope)
{
    auto result = findIntersection({1, 2, 0}, {3, 2, 0}, {-1, 1, 0});
    ASSERT_DOUBLE_EQ(result.getX(), 2);
    ASSERT_DOUBLE_EQ(result.getY(), 3);
    ASSERT_DOUBLE_EQ(result.getZ(), 0);
}

// No goal set means zero command value.
TEST(PurePursuitController, NoGoal)
{
    PurePursuitController c(lookahead, axleDistance);

    double command = c.commandStep(
        tf::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1, 2, 0)));
    ASSERT_DOUBLE_EQ(command, 0.0);
    command = c.commandStep(
        tf::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(2, 1, 0)));
    ASSERT_DOUBLE_EQ(command, 0.0);
    command = c.commandStep(
        tf::Pose(tf::Quaternion(0, 0, 0, 1),
        tf::Vector3(3, 0, 0)));
    ASSERT_DOUBLE_EQ(command, 0.0);
}

// Goal set, not on goal -> nonzero command value
TEST(PurePursuitController, Basic)
{
    PurePursuitController c(lookahead, axleDistance);
    c.setGoal(goal);

    double command = c.commandStep(
        tf::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1, 2, 0)));
    ASSERT_NE(command, 0.0);
    command = c.commandStep(
        tf::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(2, 1, 0)));
    ASSERT_NE(command, 0.0);
    command = c.commandStep(
        tf::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(3, 0, 0)));
    ASSERT_NE(command, 0.0);
}

// Steering commands' sign is as expected
TEST(PurePursuitController, SteeringSign)
{
    PurePursuitController c(lookahead, axleDistance);
    c.setGoal(goal);

    // To the right of the goal trajectory, command should be positive
    double command = c.commandStep(
        tf::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1, 1, 0)));
    ASSERT_TRUE(command > 0.0);

    // To the left of the goal trajectory, command should be negative
    command = c.commandStep(
        tf::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1, 2, 0)));
    ASSERT_TRUE(command < 0.0);
}

// Given same input, the value generated is consistent
TEST(PurePursuitController, Consistency)
{
    PurePursuitController c(lookahead, axleDistance);
    c.setGoal(goal);

    double command1 = c.commandStep(
        tf::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1, 2, 0)));
    double command2 = c.commandStep(
        tf::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1, 2, 0)));
    double command3 = c.commandStep(
        tf::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1, 2, 0)));

    ASSERT_DOUBLE_EQ(command1, command2);
    ASSERT_DOUBLE_EQ(command2, command3);
}

// Steering commands' magnitude is as expected
TEST(PurePursuitController, SteeringMagnitude)
{
    PurePursuitController c(lookahead, axleDistance);
    c.setGoal(goal);

    double command1 = c.commandStep(
        tf::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1, 2, 0)));

    double command2 = c.commandStep(
        tf::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1, 4, 0)));

    // Signs match
    ASSERT_TRUE(command1 * command2 > 0);
    // Being further away means the command magnitude is higher
    ASSERT_TRUE(std::fabs(command1) < std::fabs(command2));
}

// Axle distance changes steering values
TEST(PurePursuitController, AxleDistance)
{
    PurePursuitController c1(lookahead, axleDistance);
    PurePursuitController c2(lookahead, axleDistance * 2);
    PurePursuitController c3(lookahead, axleDistance * 3);
    c1.setGoal(goal);
    c2.setGoal(goal);
    c3.setGoal(goal);

    double command1 = c1.commandStep(
        tf::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1, 2, 0)));
    double command2 = c2.commandStep(
        tf::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1, 2, 0)));
    double command3 = c3.commandStep(
        tf::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1, 2, 0)));

    // Signs match
    ASSERT_TRUE(command1 * command2 > 0);
    ASSERT_TRUE(command2 * command3 > 0);

    // Larger axle distance increases steering command
    ASSERT_TRUE(std::fabs(command1) < std::fabs(command2));
    ASSERT_TRUE(std::fabs(command2) < std::fabs(command3));
}

// Changing goal point affects the command
TEST(PurePursuitController, GoalChange)
{
    PurePursuitController c(lookahead, axleDistance);
    tf::Vector3 goalPos(2.5, 2.5, 0);
    c.setGoal(tf::Pose(tf::Quaternion(0, 0, 0, 1), goalPos));

    double command1 = c.commandStep(
        tf::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1, 2, 0)));

    goalPos = tf::Vector3(2.5, 1.5, 0);
    c.setGoal(tf::Pose(tf::Quaternion(0, 0, 0, 1), goalPos));
    double command2 = c.commandStep(
        tf::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1, 2, 0)));

    // Signs are opposite
    ASSERT_TRUE(command1 * command2 < 0);

    // Magnitude are the same, as the goal is simply mirrored around the current position.
    ASSERT_DOUBLE_EQ(std::fabs(command1), std::fabs(command2));
}

TEST(StanleyController, HeadingOnlyTrivial)
{
    StanleyController c{1.0, 0.0, 0.0, 0.0, axleDistance};

    c.setGoal(tf::Pose{tf::createQuaternionFromYaw(0), {0, 0, 0}});

    // On the trajectory - expect zero.
    auto command = c.commandStep(
        tf::Pose{tf::createQuaternionFromYaw(0), {1, 0, 0}},
        {1.0, 0.0, 0.0},
        0.0,
        dt);
    ASSERT_DOUBLE_EQ(command, 0.0);

    // Away from the trajectory, but still heading the same direction - still zero.
    command = c.commandStep(
        tf::Pose{tf::createQuaternionFromYaw(0), {1, 5, 0}},
        {1.0, 0.0, 0.0},
        0.0,
        dt);
    ASSERT_DOUBLE_EQ(command, 0.0);
}

TEST(StanleyController, HeadingOnly)
{
    StanleyController c{1.0, 0.0, 0.0, 0.0, axleDistance};

    c.setGoal(tf::Pose{tf::createQuaternionFromYaw(0), {0, 0, 0}});

    const double vehicleYaw = M_PI / 6;

    // On the goal but not following the path
    auto command = c.commandStep(
        tf::Pose{tf::createQuaternionFromYaw(vehicleYaw), {0, 0, 0}},
        {1.0, 0.0, 0.0},
        0.0,
        dt);
    ASSERT_DOUBLE_EQ(command, -vehicleYaw);

    c.setGoal(tf::Pose{tf::createQuaternionFromYaw(vehicleYaw), {0, 0, 0}});
    // Away from the goal, but the yaw coincides - no steering needed.
    command = c.commandStep(
        tf::Pose{tf::createQuaternionFromYaw(vehicleYaw), {0, 3, 0}},
        {1.0, 0.0, 0.0},
        0.0,
        dt);
    ASSERT_DOUBLE_EQ(command, 0);
}

TEST(StanleyController, CrosstrackTrivial)
{
    StanleyController c{1.0, 0.0, 1.0, 1.0, axleDistance};

    c.setGoal(tf::Pose{tf::createQuaternionFromYaw(0), {0, 0, 0}});
    // On the goal
    auto command = c.commandStep(
        tf::Pose{tf::createQuaternionFromYaw(0), {1, 0, 0}},
        {1.0, 0.0, 0.0},
        0.0,
        dt);
    ASSERT_DOUBLE_EQ(command, 0);
}

TEST(StanleyController, CrosstrackMagnitude)
{
    StanleyController c{1.0, 0.0, 1.0, 1.0, axleDistance};

    c.setGoal(tf::Pose{tf::createQuaternionFromYaw(0), {0, 0, 0}});
    auto commandCloser = c.commandStep(
        tf::Pose{tf::createQuaternionFromYaw(0), {1, 1, 0}},
        {1.0, 0.0, 0.0},
        0.0,
        dt);
    auto commandFurther = c.commandStep(
        tf::Pose{tf::createQuaternionFromYaw(0), {1, 5, 0}},
        {1.0, 0.0, 0.0},
        0.0,
        dt);
    auto commandOpposite = c.commandStep(
        tf::Pose{tf::createQuaternionFromYaw(0), {1, -1, 0}},
        {1.0, 0.0, 0.0},
        0.0,
        dt);

    ASSERT_TRUE(commandCloser < 0);
    ASSERT_DOUBLE_EQ(commandCloser, -commandOpposite);
    ASSERT_TRUE(std::fabs(commandCloser) < std::fabs(commandFurther));
}

TEST(StanleyController, CrosstrackVelocity)
{
    StanleyController c{1.0, 0.0, 1.0, 1.0, axleDistance};

    c.setGoal(tf::Pose{tf::createQuaternionFromYaw(0), {0, 0, 0}});
    auto commandSlow = c.commandStep(
        tf::Pose{tf::createQuaternionFromYaw(0), {1, 1, 0}},
        {0.5, 0.0, 0.0},
        0.0,
        dt);
    auto commandMedium = c.commandStep(
        tf::Pose{tf::createQuaternionFromYaw(0), {1, 1, 0}},
        {1.0, 0.0, 0.0},
        0.0,
        dt);
    auto commandFast = c.commandStep(
        tf::Pose{tf::createQuaternionFromYaw(0), {1, 1, 0}},
        {1.5, 0.0, 0.0},
        0.0,
        dt);

    ASSERT_TRUE(commandSlow < commandMedium);
    ASSERT_TRUE(commandMedium < commandFast);
}

int main(int argc, char *argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
