#ifndef CHECKSUM_HPP_
#define CHECKSUM_HPP_

#include <cstdio>
#include <functional>
#include "adas_common/span.hpp"
#include "adas_common/bytes_helper.hpp"

/**
 * @brief Library to perform checksums for data verification (see: https://en.wikipedia.org/wiki/Checksum)
 * 
 */
namespace adas_common
{

struct Checksum
{
    using GenerateFunc = std::function<uint8_t(const Span<uint8_t>)>;
    using VerifyFunc = std::function<bool(const Span<uint8_t>, const uint8_t)>;

    /**
     * @brief Construct a new Checksum object. Initialise struct members GenerateFunc and VerifyFunc.
     * 
     * @param generate std::function (GenerateFunc = std::function<uint8_t(const Span<uint8_t>)>)
     * @param verify std::function (VerifyFunc = std::function<bool(const Span<uint8_t>, const uint8_t)>)
     */
    Checksum(GenerateFunc generate, VerifyFunc verify);

    GenerateFunc generate;
    VerifyFunc verify;
};

/**
 * @brief instance of checksum to do xbee data verification
 * 
 * @return Checksum 
 */
Checksum xBeeChecksum();
/**
 * @brief instance of checksum for crc verification. See: http://www.sunshine2k.de/articles/coding/crc/understanding_crc.html
 * 
 * @return Checksum 
 */
Checksum crc8Ccitt();

}

#endif
