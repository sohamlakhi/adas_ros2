#include <gtest/gtest.h>

#include <adas_common/pid_controller.hpp>

using namespace adas_common;

namespace
{
const double saturationMin = 0.0;
const double saturationMax = 100.0;
const double controlValue = 1.0;
const double feedforward = 0.0;
const double dt = 0.1;
const double goal = 3.0;
}

// No goal set -> zero control value
TEST(PIDController, NoGoal)
{
    PIDController c(
        controlValue,
        controlValue,
        controlValue,
        saturationMin,
        saturationMax);

    double command = c.commandStep(feedforward, 1.0, dt);
    ASSERT_DOUBLE_EQ(command, 0.0);
    command = c.commandStep(feedforward, 2.0, dt);
    ASSERT_DOUBLE_EQ(command, 0.0);
    command = c.commandStep(feedforward, 3.0, dt);
    ASSERT_DOUBLE_EQ(command, 0.0);
    command = c.commandStep(feedforward, 4.0, dt);
    ASSERT_DOUBLE_EQ(command, 0.0);
    command = c.commandStep(feedforward, -1.0, dt);
    ASSERT_DOUBLE_EQ(command, 0.0);
}

// Proportional control
TEST(PIDController, Proportional)
{
    PIDController c(
        controlValue,
        0.0,
        0.0,
        -saturationMax,
        saturationMax);
    c.setGoal(goal);

    double command = c.commandStep(feedforward, 1.0, dt);
    ASSERT_DOUBLE_EQ(command, 2.0);

    c = PIDController(
        controlValue,
        0.0,
        0.0,
        -saturationMax,
        saturationMax);
    c.setGoal(goal);
    command = c.commandStep(feedforward, 4.0, dt);
    ASSERT_DOUBLE_EQ(command, -1.0);
}

// Proportional constant scales output value
TEST(PIDController, PConstant)
{
    PIDController c1(
        controlValue,
        0.0,
        0.0,
        -saturationMax,
        saturationMax);
    c1.setGoal(goal);
    PIDController c2(
        controlValue * 2,
        0.0,
        0.0,
        -saturationMax,
        saturationMax);
    c2.setGoal(goal);

    double command1 = c1.commandStep(feedforward, 1.0, dt);
    double command2 = c2.commandStep(feedforward, 1.0, dt);

    ASSERT_DOUBLE_EQ(command1 * 2, command2);
}

// Value clamped to min/max saturation
TEST(PIDController, Saturation)
{
    PIDController c(
        1000.0,
        1000.0,
        1000.0,
        saturationMin,
        saturationMax);
    c.setGoal(goal);
    double command = c.commandStep(feedforward, 1.0, dt);
    ASSERT_DOUBLE_EQ(command, saturationMax);

    command = c.commandStep(feedforward, 5.0, dt);
    ASSERT_DOUBLE_EQ(command, saturationMin);
}

// Value feedforward carryover
TEST(PIDController, Feedforward)
{
    PIDController c1(
        controlValue,
        controlValue,
        controlValue,
        -saturationMax,
        saturationMax);
    c1.setGoal(goal);
    PIDController c2(
        controlValue,
        controlValue,
        controlValue,
        -saturationMax,
        saturationMax);
    c2.setGoal(goal);

    double command1 = c1.commandStep(feedforward, 1.0, dt);
    double command2 = c2.commandStep(20, 1.0, dt);
    ASSERT_DOUBLE_EQ(command1 + 20, command2);

    command1 = c1.commandStep(feedforward, 2.0, dt);
    command2 = c2.commandStep(20, 2.0, dt);
    ASSERT_DOUBLE_EQ(command1 + 20, command2);
}

// First invocation of commandStep returns P value only
TEST(PIDController, FirstStepPOnly)
{
    PIDController c(
        controlValue,
        controlValue,
        controlValue,
        -saturationMax,
        saturationMax);
    c.setGoal(goal);
    double command1 = c.commandStep(feedforward, 1.0, dt);
    ASSERT_DOUBLE_EQ(command1, 2.0);
    double command2 = c.commandStep(feedforward, 1.0, dt);
    // Signs match
    ASSERT_TRUE(command1 * command2 > 0);
    // Integral action increases the value
    ASSERT_TRUE(command1 < command2);
}

// Integral term calculation
TEST(PIDController, Integral)
{
    PIDController c(
        0,
        controlValue,
        0,
        -saturationMax,
        saturationMax);
    c.setGoal(goal);

    double command = c.commandStep(feedforward, goal + 1.0, dt);
    ASSERT_DOUBLE_EQ(command, 0.0);

    command = c.commandStep(feedforward, goal + 1.0, dt);
    ASSERT_DOUBLE_EQ(command, -0.1);

    command = c.commandStep(feedforward, goal - 1.5, dt);
    ASSERT_DOUBLE_EQ(command, -0.075);
}

// Integral constant scales output value
TEST(PIDController, IConstant)
{
    PIDController c1(
        0.0,
        controlValue,
        0.0,
        -saturationMax,
        saturationMax);
    c1.setGoal(goal);
    PIDController c2(
        0.0,
        controlValue * 2,
        0.0,
        -saturationMax,
        saturationMax);
    c2.setGoal(goal);

    double command1 = c1.commandStep(feedforward, goal, dt);
    double command2 = c2.commandStep(feedforward, goal, dt);

    ASSERT_DOUBLE_EQ(command1, 0.0);
    ASSERT_DOUBLE_EQ(command2, 0.0);

    command1 = c1.commandStep(feedforward, goal + 1.0, dt);
    command2 = c2.commandStep(feedforward, goal + 1.0, dt);
    ASSERT_DOUBLE_EQ(command1 * 2.0, command2);
}

// Override accumulation of integrator
TEST(PIDController, IOverride)
{
    PIDController c1(
        0.0,
        controlValue,
        0.0,
        -saturationMax,
        saturationMax);
    c1.setGoal(goal);
    PIDController c2(
        0.0,
        controlValue * 2,
        0.0,
        -saturationMax,
        saturationMax);
    c2.setGoal(goal);

    double command1 = c1.commandStep(feedforward, goal, dt);
    double command2 = c2.commandStep(feedforward, goal, dt);

    ASSERT_DOUBLE_EQ(command1, 0.0);
    ASSERT_DOUBLE_EQ(command2, 0.0);

    command1 = c1.commandStep(feedforward, goal + 1.0, dt);
    command2 = c2.commandStep(feedforward, goal + 1.0, dt);
    ASSERT_DOUBLE_EQ(command1 * 2.0, command2);
}

// Integral term reset
TEST(PIDController, IReset)
{
    PIDController c(
        0,
        controlValue,
        0,
        -saturationMax,
        saturationMax);
    c.setGoal(goal);

    double command = c.commandStep(feedforward, goal + 1.0, dt);
    ASSERT_DOUBLE_EQ(command, 0.0);

    command = c.commandStep(feedforward, goal + 1.0, dt);
    ASSERT_DOUBLE_EQ(command, -0.1);

    c.reset();

    command = c.commandStep(feedforward, goal - 1.5, dt);
    ASSERT_DOUBLE_EQ(command, 0.0);

    command = c.commandStep(feedforward, goal, dt);
    ASSERT_DOUBLE_EQ(command, 0.075);
}

// Windup prevention
TEST(PIDController, Windup)
{
    PIDController c(
        0,
        controlValue,
        0,
        -1,
        1);
    c.setGoal(goal);

    double command = c.commandStep(feedforward, goal + 10.0, dt);
    ASSERT_DOUBLE_EQ(command, 0.0);

    // Reach saturation, accumulator overflowed by -1.0
    for (unsigned int i = 0; i < 5; i++)
    {
        command = c.commandStep(feedforward, goal + 10.0, dt);
        ASSERT_DOUBLE_EQ(command, -1.0);
    }

    // Adds to zero
    command = c.commandStep(feedforward, goal - 10.0, dt);
    ASSERT_DOUBLE_EQ(command, -1.0);

    // Gets rid of overflow
    command = c.commandStep(feedforward, goal - 10.0, dt);
    ASSERT_DOUBLE_EQ(command, -1.0);

    // Back to normal
    command = c.commandStep(feedforward, goal - 10.0, dt);
    ASSERT_NEAR(command, 0.0, 1e-12);
}

// Derivative term calculation
TEST(PIDController, Derivative)
{
    PIDController c(
        0,
        0,
        controlValue,
        -saturationMax,
        saturationMax);
    c.setGoal(goal);
    double command = c.commandStep(feedforward, goal, dt);
    ASSERT_DOUBLE_EQ(command, 0.0);

    command = c.commandStep(feedforward, goal + 1.0, dt);
    ASSERT_DOUBLE_EQ(command, -10.0);

    command = c.commandStep(feedforward, goal - 1.0, dt);
    ASSERT_DOUBLE_EQ(command, 20.0);
}

// Derivative constant scales output value
TEST(PIDController, DConstant)
{
    PIDController c1(
        0.0,
        0.0,
        controlValue,
        -saturationMax,
        saturationMax);
    c1.setGoal(goal);
    PIDController c2(
        0.0,
        0.0,
        controlValue * 2,
        -saturationMax,
        saturationMax);
    c2.setGoal(goal);

    double command1 = c1.commandStep(feedforward, goal, dt);
    double command2 = c2.commandStep(feedforward, goal, dt);

    ASSERT_DOUBLE_EQ(command1, 0.0);
    ASSERT_DOUBLE_EQ(command2, 0.0);

    command1 = c1.commandStep(feedforward, 1.0, dt);
    command2 = c2.commandStep(feedforward, 1.0, dt);
    ASSERT_DOUBLE_EQ(command1 * 2.0, command2);
}

// Derivative term reset
TEST(PIDController, DReset)
{
    PIDController c(
        0,
        0,
        controlValue,
        -saturationMax,
        saturationMax);
    c.setGoal(goal);

    double command = c.commandStep(feedforward, goal, dt);
    ASSERT_DOUBLE_EQ(command, 0.0);

    command = c.commandStep(feedforward, goal + 1.0, dt);
    ASSERT_DOUBLE_EQ(command, -10.0);

    c.reset();

    command = c.commandStep(feedforward, goal - 1.0, dt);
    ASSERT_DOUBLE_EQ(command, 0.0);

    command = c.commandStep(feedforward, goal, dt);
    ASSERT_DOUBLE_EQ(command, -10.0);
}

int main(int argc, char *argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
