#ifndef SERIAL_PROTOCOL_HPP_
#define SERIAL_PROTOCOL_HPP_

#include <cstdint>
#include <vector>
#include <cstdio>
#include <string>
#include <functional>
#include <memory>

#include <boost/optional.hpp>

#include "adas_common/span.hpp"
#include "adas_common/checksum.hpp"

namespace adas_common
{
namespace serial
{

class CharDevice
{
public:
    using TimeoutMicro = boost::optional<unsigned>;
    using Byte = boost::optional<uint8_t>;

    CharDevice() = default;
    virtual ~CharDevice() = default;

    virtual bool valid() const = 0;
    virtual void write(const Span<uint8_t> data) = 0;
    virtual Byte readByte(TimeoutMicro timeout = boost::none) = 0;
    Span<uint8_t> read(
        Span<uint8_t> buffer,
        TimeoutMicro interByteTimeout = boost::none);
};

class SerialDevice : public CharDevice
{
public:
    using CharDevice::TimeoutMicro;

    SerialDevice(const std::string &port, const size_t baud);
    virtual ~SerialDevice() = default;

    virtual bool valid() const override;
    virtual void write(const Span<uint8_t> data) override;
    virtual Byte readByte(TimeoutMicro interByteTimeoutMicro = boost::none) override;

private:
    struct FD
    {
        FD(int fd);
        ~FD();
        int operator*() const;
        int fd;
    };

    FD fd;
};

class FrameRW
{
public:
    using TimeoutMicro = CharDevice::TimeoutMicro;

    FrameRW(
        std::shared_ptr<CharDevice> dev,
        Checksum checksum);

    bool valid() const;
    std::vector<uint8_t> read(TimeoutMicro interByteTimeout = boost::none);
    void write(const Span<uint8_t> data);

private:
    bool read(Span<uint8_t> buf, TimeoutMicro interByteTimeout);

    static constexpr uint8_t frameStart = 0x7e;
    static constexpr size_t sizeSize = sizeof(uint16_t);
    static constexpr size_t headerSize = sizeof(frameStart) + sizeSize;
    static constexpr size_t checksumSize = sizeof(uint8_t);

    std::shared_ptr<CharDevice> dev;
    Checksum checksum;
};

}
}

#endif
