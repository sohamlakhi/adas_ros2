#include <gtest/gtest.h>

#include "scaffold.hpp"

#include <iostream>

TEST(CalculateShift, Test)
{
    constexpr size_t N = 50;
    constexpr size_t lag = 12;

    ASSERT_TRUE(N >= lag * 2);

    std::vector<double> orig(N, 0.0);
    std::iota(orig.begin(), orig.end(), 0.0);

    std::vector<double> lagged(N, 0.0);
    std::iota(lagged.begin() + lag, lagged.end(), 0.0);

    auto result = calcShift(orig, lagged);

    EXPECT_EQ(lag, result);
}

int main(int argc, char *argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
