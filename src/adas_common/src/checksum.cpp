#include "adas_common/checksum.hpp"

#include <numeric>

//bringing span in scope from namespace adas_common
using adas_common::Span;

namespace
{

uint8_t generateXBeeChecksum(const Span<uint8_t> data)
{
    uint16_t sum = std::accumulate(data.begin(), data.end(), 0);

    return 0xff - bytes::lower8(sum);
}

bool verifyXBeeChecksum(const Span<uint8_t> data, const uint8_t givenChecksum)
{
    uint16_t sum = std::accumulate(data.begin(), data.end(), givenChecksum);

    return bytes::lower8(sum) == 0xff;
}

uint8_t generateCrc8Ccitt(const Span<uint8_t> data)
{
    // Taken from avr-libc implementation
    uint8_t crc = 0;

    for (const auto v: data)
    {
        uint8_t update = crc ^ v;

        for (unsigned i = 0; i < 8; i++)
        {
            if ((update & 0x80) != 0)
            {
                update <<= 1;
                update ^= 0x07;
            }
            else
            { update <<= 1; }
        }

        crc = update;
    }

    return crc;
}

bool verifyCrc8Ccitt(const Span<uint8_t> data, const uint8_t givenChecksum)
{
    uint8_t calculatedChecksum = generateCrc8Ccitt(data);
    return givenChecksum == calculatedChecksum;
}

}

namespace adas_common
{

Checksum::Checksum(GenerateFunc generate, VerifyFunc verify) :
    generate{generate},
    verify{verify}
{}

Checksum xBeeChecksum()
{
    return {generateXBeeChecksum, verifyXBeeChecksum};
}

Checksum crc8Ccitt()
{
    return {generateCrc8Ccitt, verifyCrc8Ccitt};
}

}
