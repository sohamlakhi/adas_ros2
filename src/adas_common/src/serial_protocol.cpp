#include "adas_common/serial_protocol.hpp"
#include "adas_common/bytes_helper.hpp"

#include <stdexcept>
#include <array>
#include <vector>
#include <cstdio>
#include <cstring>

//See: https://stackoverflow.com/questions/1041866/what-is-the-effect-of-extern-c-in-c
extern "C"
{
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
}

namespace adas_common
{
namespace serial
{

Span<uint8_t> CharDevice::read(
    Span<uint8_t> buffer,
    TimeoutMicro interByteTimeout)
{
    size_t read = 0;

    for (unsigned i = 0; i < buffer.size(); i++)
    {
        Byte byte = readByte(interByteTimeout);
        if (!byte) { break; }
        buffer[read] = *byte;
        ++read;
    }

    return buffer.makeSubspan(0, read);
}

SerialDevice::SerialDevice(const std::string &port, const size_t baud) :
    fd{open(port.c_str(), O_RDWR | O_NONBLOCK)}
{
    termios io;
    std::memset(&io, 0, sizeof(io));
    io.c_iflag = 0;
    io.c_oflag = 0;
    /*
     * 8-bit data, enable receiver, ignore modem control lines.
     * Implicitly no parity and 1 stop bit.
     */
    io.c_cflag = CS8 | CREAD | CLOCAL;

    io.c_lflag = 0;
    // Minimum 1 character, no timeout
    io.c_cc[VMIN] = 1;
    io.c_cc[VTIME] = 0;

    speed_t setBaud;
    switch (baud)
    {
    case 115200:
        setBaud = B115200;
        break;
    case 57600:
        setBaud = B57600;
        break;
    default:
        setBaud = B9600;
        break;
    }

    if (cfsetospeed(&io, setBaud) != 0)
    { throw std::runtime_error("Could not set outgoing baud rate."); }
    if (cfsetispeed(&io, setBaud) != 0)
    { throw std::runtime_error("Could not set incoming baud rate."); }

    if (tcflush(*fd, TCIOFLUSH) != 0)
    { throw std::runtime_error("Failed to flush."); }
    if (tcsetattr(*fd, TCSANOW, &io) != 0)
    { throw std::runtime_error("Failed to set attributes."); }
}

bool SerialDevice::valid() const
{ return *fd >= 0; }

void SerialDevice::write(const Span<uint8_t> data)
{
    size_t written = 0;
    while (written < data.size())
    {
        ssize_t currWritten = ::write(*fd, &data[written], data.size() - written);
        if (currWritten < 0) { throw std::runtime_error{"Write error."}; }
        written += static_cast<size_t>(currWritten);
    }
}

SerialDevice::Byte SerialDevice::readByte(TimeoutMicro timeout)
{
    timeval tv;
    timeval *tvPtr = nullptr;

    if (timeout)
    {
        tv.tv_sec = 0;
        tv.tv_usec = static_cast<decltype(tv.tv_usec)>(*timeout);
        tvPtr = &tv;
    }

    uint8_t val;

    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(*fd, &fds);

    int res = select(*fd + 1, &fds, nullptr, nullptr, tvPtr);
    if (res == 1 && (FD_ISSET(*fd, &fds) != 0))
    {
        ssize_t result = ::read(*fd, &val, 1);
        if (result != 1)
        { throw std::runtime_error{"Read error."}; }
        return val;
    }

    return {};
}

SerialDevice::FD::FD(int fd) : fd{fd} {}
SerialDevice::FD::~FD() { if (fd >= 0) { close(fd); } }
int SerialDevice::FD::operator*() const { return fd; }

FrameRW::FrameRW(
    std::shared_ptr<CharDevice> dev,
    Checksum checksum) :
    dev{dev},
    checksum{checksum}
{
    if (checksum.generate == nullptr || checksum.verify == nullptr)
    { throw std::runtime_error{"Invalid checksum callbacks."}; }
}

bool FrameRW::valid() const
{ return dev->valid(); }

std::vector<uint8_t> FrameRW::read(TimeoutMicro interByteTimeout)
{
    uint8_t start;
    if (!read({&start, 1}, interByteTimeout))
    { return {}; }
    if (start != frameStart) { return {}; }

    std::array<uint8_t, sizeSize> sizeData;
    if (!read({sizeData}, interByteTimeout))
    { return {}; }

    uint16_t dataSize = bytes::to16(sizeData[0], sizeData[1]);

    std::vector<uint8_t> data(dataSize, 0);
    if (!read({data}, interByteTimeout))
    { return {}; }

    uint8_t checksumData;
    if (!read({&checksumData, 1}, interByteTimeout))
    { return {}; }

    if (!checksum.verify({data}, checksumData))
    { return {}; }

    return data;
}

bool FrameRW::read(Span<uint8_t> buf, TimeoutMicro interByteTimeout)
{
    auto read = dev->read(buf, interByteTimeout);

    // Timed out
    if (buf.size() != read.size())
    { return false; }

    return true;
}

void FrameRW::write(const Span<uint8_t> data)
{
    if (data.size() > std::numeric_limits<uint16_t>::max())
    { throw std::runtime_error{"Data too big."}; }

    std::array<uint8_t, headerSize> header;
    header[0] = frameStart;
    auto sizeBytes = bytes::from16(static_cast<uint16_t>(data.size()));
    header[1] = sizeBytes.first;
    header[2] = sizeBytes.second;

    uint8_t calculatedChecksum = checksum.generate(data);

    dev->write({header});
    dev->write(data);
    dev->write({&calculatedChecksum, 1});
}

}
}
