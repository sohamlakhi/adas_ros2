#include "adas_common/bytes_helper.hpp"

namespace bytes
{

uint8_t upper8(const uint16_t data)
{ return (data >> 8) & 0xff; }
uint8_t lower8(const uint16_t data)
{ return data & 0xff; }
uint16_t to16(const uint8_t upper, const uint8_t lower)
{ return (upper << 8) | lower; }
std::pair<uint8_t, uint8_t> from16(const uint16_t data)
{ return std::make_pair(upper8(data), lower8(data)); }

}
