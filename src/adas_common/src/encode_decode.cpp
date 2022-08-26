#include "adas_common/encode_decode.hpp"

#include <cstdint>

namespace adas_common
{

Decoder::Decoder(const Span<uint8_t> data) :
    data{data},
    offset{0}
{}

uint16_t Decoder::end() const { return offset == data.size(); }

Encoder::Encoder(size_t size)
{ result.reserve(size); }

std::vector<uint8_t> Encoder::get()
{ return result; }

}
