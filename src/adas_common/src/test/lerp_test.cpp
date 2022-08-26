#include <gtest/gtest.h>

#include <adas_common/lerp.hpp>

using namespace adas_common;

namespace
{
constexpr double a = -3.0;
constexpr double b = 2.0;
}

TEST(LERP, Zero)
{
    const double result = lerp(a, b, 0.0);
    EXPECT_DOUBLE_EQ(result, a);
}

TEST(LERP, One)
{
    const double result = lerp(a, b, 1.0);
    EXPECT_DOUBLE_EQ(result, b);
}

TEST(LERP, UnderZero)
{
    const double result = lerp(a, b, -0.5);
    EXPECT_DOUBLE_EQ(result, a);
}

TEST(LERP, OverOne)
{
    const double result = lerp(a, b, 1.2);
    EXPECT_DOUBLE_EQ(result, b);
}

TEST(LERP, Scaling)
{
    constexpr unsigned iterations = 10;

    for (unsigned i = 0; i < iterations; i++)
    {
        const double t = static_cast<double>(i) / iterations;
        const double result = lerp(-1.0, 1.0, t);
        EXPECT_DOUBLE_EQ(result, 2.0 * t - 1.0);
    }
}

TEST(LERP, ScalingReverse)
{
    constexpr unsigned iterations = 10;

    for (unsigned i = 0; i < iterations; i++)
    {
        const double t = static_cast<double>(i) / iterations;
        const double result = lerp(1.0, -1.0, t);
        EXPECT_DOUBLE_EQ(result, 1.0 - 2.0 * t);
    }
}

int main(int argc, char *argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
