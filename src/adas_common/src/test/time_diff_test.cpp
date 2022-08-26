#include <gtest/gtest.h>

#include <random>

#include <adas_common/time_diff.hpp>

using namespace adas_common;

TEST(TimeDiff, Initial)
{
    TimeDiff<uint32_t> td;
    uint32_t diff;

    bool result = td.update(0, diff);

    EXPECT_EQ(result, false);
}

TEST(TimeDiff, Basic)
{
    TimeDiff<uint32_t> td;
    uint32_t diff;

    bool result = td.update(0, diff);
    EXPECT_EQ(result, false);

    result = td.update(5, diff);
    EXPECT_EQ(result, true);
    EXPECT_EQ(diff, 5);
}

TEST(TimeDiff, NonZero)
{
    TimeDiff<uint32_t> td;
    uint32_t diff;

    bool result = td.update(5, diff);
    EXPECT_EQ(result, false);

    result = td.update(8, diff);
    EXPECT_EQ(result, true);
    EXPECT_EQ(diff, 3);
}

TEST(TimeDiff, ZeroDiff)
{
    TimeDiff<uint32_t> td;
    uint32_t diff;

    bool result = td.update(5, diff);
    EXPECT_EQ(result, false);

    result = td.update(5, diff);
    EXPECT_EQ(result, true);
    EXPECT_EQ(diff, 0);
}

TEST(TimeDiff, Consecutive)
{
    TimeDiff<uint32_t> td;
    uint32_t diff;
    uint32_t initialTime = 3;

    std::random_device randDev;
    std::mt19937 gen(randDev());
    std::uniform_int_distribution<uint32_t> dist(0, 20);

    bool result = td.update(initialTime, diff);
    EXPECT_EQ(result, false);
    uint32_t prevTime = initialTime;

    for (unsigned int i = 0; i < 10; i++)
    {
        uint32_t increment = dist(gen);
        uint32_t currTime = prevTime + increment;

        // Not expecting rollover here.
        ASSERT_TRUE(currTime >= prevTime);

        result = td.update(currTime, diff);
        EXPECT_EQ(result, true);
        EXPECT_EQ(diff, increment);

        prevTime = currTime;
    }
}

TEST(TimeDiff, Rollover)
{
    TimeDiff<uint32_t> td;
    uint32_t diff;
    uint32_t initialTime = std::numeric_limits<uint32_t>::max() - 5;
    uint32_t increment = 10;
    uint32_t afterTime = initialTime + increment;

    ASSERT_TRUE(initialTime > afterTime);

    bool result = td.update(initialTime, diff);
    EXPECT_EQ(result, false);

    result = td.update(afterTime, diff);
    EXPECT_EQ(result, true);
    EXPECT_EQ(diff, increment);
}

TEST(TimeDiff, Reset)
{
    TimeDiff<uint32_t> td;
    uint32_t diff;

    bool result = td.update(0, diff);
    EXPECT_EQ(result, false);

    result = td.update(5, diff);
    EXPECT_EQ(result, true);
    EXPECT_EQ(diff, 5);

    td.reset();
    result = td.update(10, diff);
    EXPECT_EQ(result, false);

    result = td.update(15, diff);
    EXPECT_EQ(result, true);
    EXPECT_EQ(diff, 5);
}

int main(int argc, char *argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
