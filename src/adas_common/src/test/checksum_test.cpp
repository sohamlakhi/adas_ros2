#include <gtest/gtest.h>

#include <adas_common/checksum.hpp>

#include <array>

using namespace adas_common;

namespace
{
const std::array<uint8_t, 15> arr = {
    0xaa,
    0xf7,
    0x2e,
    0x0f,
    0xcf,
    0x5d,
    0x55,
    0x8a,
    0xf8,
    0xd8,
    0x7b,
    0x8d,
    0xf9,
    0x04,
    0xea};

constexpr uint8_t xBeeChecksumCalc = 0x57;
constexpr uint8_t crc8ccittCalc = 0xb2;
}

TEST(ChecksumXBee, Generate)
{
    auto data = arr;
    auto checksum = xBeeChecksum();
    uint8_t calculated = checksum.generate({data});
    EXPECT_EQ(calculated, xBeeChecksumCalc);
}

TEST(ChecksumXBee, Verify)
{
    auto data = arr;
    auto checksum = xBeeChecksum();
    EXPECT_TRUE(checksum.verify({data}, xBeeChecksumCalc));
}

TEST(ChecksumCRC8CCITT, Generate)
{
    auto data = arr;
    auto checksum = crc8Ccitt();
    uint8_t calculated = checksum.generate({data});
    EXPECT_EQ(calculated, crc8ccittCalc);
}

TEST(ChecksumCRC8CCITT, Verify)
{
    auto data = arr;
    auto checksum = crc8Ccitt();
    EXPECT_TRUE(checksum.verify({data}, crc8ccittCalc));
}

int main(int argc, char *argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
