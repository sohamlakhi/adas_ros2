#include <gtest/gtest.h>

#include <adas_common/serial_protocol.hpp>

#include <deque>
#include <algorithm>
#include <stdexcept>

using namespace adas_common;
using namespace serial;

namespace
{

class TestDevice : public CharDevice
{
public:
    using CharDevice::TimeoutMicro;

    TestDevice() : isValid{true}, readError{false} {}
    virtual ~TestDevice() = default;

    void reset()
    {
        isValid = true;
        readData.clear();
        written.clear();
    }

    virtual bool valid() const override
    { return isValid; }

    virtual void write(const Span<uint8_t> data) override
    { written.insert(written.end(), data.begin(), data.end()); }

    virtual Byte readByte(TimeoutMicro interByteTimeoutMicro = boost::none) override
    {
        if (readError)
        { return {}; }
        if (readData.empty())
        { throw std::runtime_error{"Attempted read while empty."}; }
        auto ret = readData.front();
        readData.pop_front();

        return ret;
    }

    bool isValid;
    bool readError;
    std::deque<boost::optional<uint8_t>> readData;
    std::deque<uint8_t> written;
};

constexpr uint8_t frameStart = 0x7e;
constexpr uint8_t testChecksum = 0xad;
uint8_t generateTestChecksum(const Span<uint8_t> data)
{ return testChecksum; }

bool verifyTestChecksumPass(const Span<uint8_t>, const uint8_t)
{ return true; }

bool verifyTestChecksumFail(const Span<uint8_t>, const uint8_t)
{ return false; }

Checksum getChecksumPass()
{ return {.generate = generateTestChecksum, .verify = verifyTestChecksumPass}; }

Checksum getChecksumFail()
{ return {.generate = generateTestChecksum, .verify = verifyTestChecksumFail}; }

constexpr size_t headerSize = sizeof(uint8_t) + sizeof(uint16_t);
constexpr size_t checksumSize = sizeof(uint8_t);
constexpr size_t overheadSize = headerSize + checksumSize;

}

TEST(FrameRW, Construction)
{
    auto dev = std::shared_ptr<CharDevice>{new TestDevice{}};
    FrameRW frameRW = {dev, getChecksumPass()};

    EXPECT_TRUE(frameRW.valid());
}

TEST(FrameRW, ConstructionError)
{
    auto dev = std::shared_ptr<CharDevice>{new TestDevice{}};
    ASSERT_THROW(
        FrameRW(dev, {.generate = nullptr, .verify = nullptr}),
        std::runtime_error);
    ASSERT_THROW(
        FrameRW(dev, {.generate = generateTestChecksum, .verify = nullptr}),
        std::runtime_error);
    ASSERT_THROW(
        FrameRW(dev, {.generate = nullptr, .verify = verifyTestChecksumPass}),
        std::runtime_error);
}

TEST(FrameRW, MinimumValid)
{
    auto dev = std::shared_ptr<TestDevice>{new TestDevice{}};
    FrameRW frameRW = {dev, getChecksumPass()};

    std::array<uint8_t, overheadSize + 1> data{frameStart, 0x00, 0x01, 0xde, 0x00};
    dev->readData = {data.begin(), data.end()};

    auto frame = frameRW.read();
    ASSERT_EQ(frame.size(), 1);
    EXPECT_EQ(frame[0], 0xde);
}

TEST(FrameRW, MinimumValidDelayed)
{
    auto dev = std::shared_ptr<TestDevice>{new TestDevice{}};
    FrameRW frameRW = {dev, getChecksumPass()};

    std::array<uint8_t, overheadSize + 2> data{0x00, frameStart, 0x00, 0x01, 0xde, 0x00};
    dev->readData = {data.begin(), data.end()};

    auto frame = frameRW.read();
    ASSERT_EQ(frame.size(), 0);

    frame = frameRW.read();
    ASSERT_EQ(frame.size(), 1);
    EXPECT_EQ(frame[0], 0xde);
}

TEST(FrameRW, InvalidHeader)
{
    auto dev = std::shared_ptr<TestDevice>{new TestDevice{}};
    FrameRW frameRW = {dev, getChecksumPass()};

    std::array<uint8_t, overheadSize + 1> data{0x7f, 0x00, 0x01, 0xde, 0x00};
    dev->readData = {data.begin(), data.end()};

    auto frame = frameRW.read();
    ASSERT_EQ(frame.size(), 0);
}

TEST(FrameRW, InvalidChecksum)
{
    auto dev = std::shared_ptr<TestDevice>{new TestDevice{}};
    FrameRW frameRW = {dev, getChecksumFail()};

    std::array<uint8_t, overheadSize + 1> data{frameStart, 0x00, 0x01, 0xde, 0x00};
    dev->readData = {data.begin(), data.end()};

    auto frame = frameRW.read();
    ASSERT_EQ(frame.size(), 0);
}

TEST(FrameRW, ReadTimeout)
{
    auto dev = std::shared_ptr<TestDevice>{new TestDevice{}};
    FrameRW frameRW = {dev, getChecksumPass()};

    std::array<uint8_t, overheadSize + 1> data{frameStart, 0x00, 0x01, 0xde, 0x00};
    dev->readData = {data.begin(), data.end()};
    dev->readError = true;

    auto frame = frameRW.read();
    ASSERT_EQ(frame.size(), 0);
}

TEST(FrameRW, FrameParsed)
{
    auto dev = std::shared_ptr<TestDevice>{new TestDevice{}};
    FrameRW frameRW = {dev, getChecksumPass()};

    std::array<uint8_t, overheadSize + 4> data{frameStart, 0x00, 0x04, 0xde, 0xad, 0xbe, 0xef, 0x00};
    dev->readData = {data.begin(), data.end()};

    auto frame = frameRW.read();
    ASSERT_EQ(frame.size(), 4);
    EXPECT_EQ(frame[0], 0xde);
    EXPECT_EQ(frame[1], 0xad);
    EXPECT_EQ(frame[2], 0xbe);
    EXPECT_EQ(frame[3], 0xef);
}

TEST(FrameRW, Write)
{
    auto dev = std::shared_ptr<TestDevice>{new TestDevice{}};
    FrameRW frameRW = {dev, getChecksumPass()};
    std::vector<uint8_t> data = {0xde, 0xad, 0xbe, 0xef};

    frameRW.write({data});

    ASSERT_EQ(dev->written.size(), data.size() + overheadSize);
    EXPECT_EQ(dev->written[0], frameStart);
    EXPECT_EQ(dev->written[1], 0x00);
    EXPECT_EQ(dev->written[2], 0x04);
    for (unsigned i = 0; i < data.size(); i++)
    { EXPECT_EQ(dev->written[headerSize + i], data[i]); }
    EXPECT_EQ(dev->written[headerSize + data.size()], testChecksum);
}

TEST(FrameRW, WriteTooBig)
{
    auto dev = std::shared_ptr<TestDevice>{new TestDevice{}};
    FrameRW frameRW = {dev, getChecksumPass()};
    std::vector<uint8_t> data(std::numeric_limits<uint16_t>::max() + 1, 0);

    EXPECT_THROW(frameRW.write({data}), std::runtime_error);
}

int main(int argc, char *argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
