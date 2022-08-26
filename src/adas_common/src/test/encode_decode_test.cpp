#include <gtest/gtest.h>

#include <array>

#include <adas_common/encode_decode.hpp>

using namespace adas_common;

namespace
{

const std::array<uint8_t, 6> data{0xde, 0xad, 0xbe, 0xef, 0xad, 0xa5};

}

TEST(Encode, Empty)
{
    Encoder encoder{0};

    auto result = encoder.get();
    EXPECT_TRUE(result.empty());
}

TEST(Encode, Bytes)
{
    Encoder encoder{data.size()};

    encoder.add<uint8_t>(0xde);
    encoder.add<uint8_t>(0xad);
    encoder.add<uint8_t>(0xbe);
    encoder.add<uint8_t>(0xef);
    encoder.add<uint8_t>(0xad);
    encoder.add<uint8_t>(0xa5);

    auto result = encoder.get();

    ASSERT_EQ(result.size(), data.size());

    for (unsigned i = 0; i < result.size(); i++)
    {
        EXPECT_EQ(result[i], data[i]);
    }
}

TEST(Encode, Wide)
{
    Encoder encoder{data.size()};

    encoder.add<uint16_t>(0xdead);
    encoder.add<uint32_t>(0xbeefada5);

    auto result = encoder.get();

    ASSERT_EQ(result.size(), data.size());

    for (unsigned i = 0; i < result.size(); i++)
    {
        EXPECT_EQ(result[i], data[i]);
    }
}

TEST(Decode, Empty)
{
    auto toDecode = data;
    Decoder decoder{{toDecode.data(), 0}};

    EXPECT_TRUE(decoder.end());
}

TEST(Decode, Bytes)
{
    auto toDecode = data;
    Decoder decoder{{toDecode}};

    for (unsigned i = 0; i < toDecode.size(); i++)
    { EXPECT_EQ(decoder.parseAs<uint8_t>(), toDecode[i]); }

    EXPECT_TRUE(decoder.end());
}

TEST(Decode, Wide)
{
    auto toDecode = data;
    Decoder decoder{{toDecode}};

    EXPECT_EQ(decoder.parseAs<uint32_t>(), 0xdeadbeef);
    EXPECT_EQ(decoder.parseAs<uint16_t>(), 0xada5);

    EXPECT_TRUE(decoder.end());
}

TEST(Decode, Overrun)
{
    auto toDecode = data;
    Decoder decoder{{toDecode}};

    for (unsigned i = 0; i < toDecode.size(); i++)
    { ASSERT_NO_THROW(decoder.parseAs<uint8_t>()); }

    ASSERT_THROW(decoder.parseAs<uint8_t>(), std::runtime_error);
    ASSERT_THROW(decoder.parseAs<uint8_t>(), std::runtime_error);
}

int main(int argc, char *argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
