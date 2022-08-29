#ifndef BYTES_HELPER_HPP_
#define BYTES_HELPER_HPP_

#include <utility>
#include <cstdint>

/**
 * @brief this library is used to manipulate bits 
 * 
 */
namespace bytes
{

/**
 * @brief extracts MS8B
 *
 * 
 * @param data 16 bit input
 * @return uint8_t MS8B
 */
uint8_t upper8(const uint16_t data);
/**
 * @brief extracts LS8B
 * 
 * @param data 16 bit input
 * @return uint8_t LS8B
 */
uint8_t lower8(const uint16_t data);
/**
 * @brief concatenates MS8B and LS8B to create 16bit number
 * 
 * @param upper MS8B
 * @param lower LS8B
 * @return uint16_t 16bit number
 */
uint16_t to16(const uint8_t upper, const uint8_t lower);
/**
 * @brief creates a std::pair by extracting MS8B and LS8B from a 16bit number
 * 
 * @param data 16bit number
 * @return std::pair<uint8_t, uint8_t> pair of MS8B and LS8B
 */
std::pair<uint8_t, uint8_t> from16(const uint16_t data);

}

#endif
