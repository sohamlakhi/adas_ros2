#ifndef ENCODE_DECODE_HPP_
#define ENCODE_DECODE_HPP_

#include <limits>
#include <vector>

#include "adas_common/span.hpp"

/**
 * @brief Encoder and Decoder class. Used to encode and decode data provided in a span (using adas_common::Span)
 * 
 */

namespace adas_common {

class Decoder {
    public:
        /**
         * @brief Construct a new Decoder object
         * 
         * @param data initialise private members data and offset with data to be decoded
         */
        Decoder(const Span<uint8_t> data);

        /**
         * @brief template function to reconstruct data from 8bit chunks. Uses private member data (const Span<uint8_t>)
         * 
         * @tparam T return datatype
         * @return T reconstructed piece of data
         */
        template <typename T> T parseAs() {
            static_assert(std::numeric_limits<T>::is_integer, "Must be an integer.");

            constexpr unsigned dataSize = sizeof(T);
            T val = 0;
            for (unsigned i = 0; i < dataSize; i++) {
                if (offset >= data.size()) { 
                    throw std::runtime_error{"Buffer overrun."}; 
                }
                const uint8_t currData = data[offset++];
                const unsigned shiftIndex = (dataSize - 1) - i;
                val |= (currData << (8 * shiftIndex));
            }

            return val;
        }

        uint16_t end() const;

    private:
        const Span<uint8_t> data;
        uint16_t offset;
};

class Encoder {
    public:
        /**
         * @brief Construct a new Encoder object
         * 
         * @param size type: size_t (represents size of data). Inititalise private members with 
         */
        Encoder(size_t size);

        /**
         * @brief template function to encode data as a stream (stored in std::vector) of 8bit uints. Shifts and extracts 8bits of data
         * 
         * @tparam T input data type
         * @param in input data
         */
        template <typename T> void add(const T in) {
            for (unsigned i = 0; i < sizeof(T); i++){
                const unsigned shift = 8 * ((sizeof(T) - 1) - i);
                const uint8_t chunk = (in >> shift) & 0xff;
                result.push_back(chunk);
            }
        }
        /**
         * @brief getter function to retrieve encoded data 
         * 
         * @return std::vector<uint8_t> private member std::vector<uint8_t> result
         */
        std::vector<uint8_t> get();

    private:
        std::vector<uint8_t> result;
};

}

#endif
