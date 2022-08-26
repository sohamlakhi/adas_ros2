#ifndef SPAN_HPP_
#define SPAN_HPP_

#include <stdexcept>
#include <vector>
#include <array>

/**
 * @brief Header library. Implementation of a span (like std::span) using a pointer. Look at: https://en.cppreference.com/w/cpp/container/span
 * 
 */

namespace adas_common
{

/*
 * A non-owning view into a contiguous memory region, with a strong type.
 * Extremely simplified version of one in GSL (Guideline Support Library),
 * which assumes C++14 support.
 *
 * As it is, spans to a constant pointer cannot be created.
 **/
template <typename T> class Span {
    public:
        using TPtr = T *;
        using iterator = TPtr;
        using const_iterator = const TPtr;

        Span() : storage{nullptr}, N{} {}
        Span(TPtr ptr, const size_t N) : storage{ptr}, N{N} {}
        Span(std::vector<T> &v) : storage{v.data()}, N{v.size()} {}
        template <size_t ArraySize>
        Span(std::array<T, ArraySize> &arr) : storage{arr.data()}, N{ArraySize} {}
        ~Span() = default;

        size_t size() const { return N; }
        const T &operator[](int i) const { return storage[i]; }
        T &operator[](int i) { return storage[i]; }

        iterator begin() { return storage; }
        iterator end() { return storage + N; }
        const_iterator begin() const { return storage; }
        const_iterator end() const { return storage + N; }

        /**
         * @brief make a subspan of a given span
         * 
         * @param offset starting point
         * @param count number of elements
         * @return Span<T> returns sub-span (of type T) of storage span (defined globally in private) 
         */
        Span<T> makeSubspan(const size_t offset, const size_t count)
        {
            if (N != 0)
            {
                if (offset >= N || (offset + count) > N)
                { throw std::runtime_error{"Subspan outside the parent span."}; }
            }
            else if (offset != 0 || count != 0)
            { throw std::runtime_error{"Subspan outside the parent span."}; }

            return {storage + offset, count};
        }

private:
    T *storage;
    size_t N;
};

}

#endif
