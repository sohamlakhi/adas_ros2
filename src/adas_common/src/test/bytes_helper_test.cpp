#include <gtest/gtest.h>

#include "adas_common/bytes_helper.hpp"

using namespace bytes;

TEST(SerialProt, Conversions)
{
    EXPECT_EQ(lower8(0xdead), 0xad);
    EXPECT_EQ(upper8(0xdead), 0xde);
    EXPECT_EQ(to16(0xde, 0xad), 0xdead);
    uint8_t upper, lower;
    std::tie(upper, lower) = from16(0xdead);
    EXPECT_EQ(upper, 0xde);
    EXPECT_EQ(lower, 0xad);
}

int main(int argc, char *argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
