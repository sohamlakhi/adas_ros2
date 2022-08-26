#ifndef LERP_HPP_
#define LERP_HPP_

#include <limits>
#include <boost/algorithm/clamp.hpp>

/**
 * @brief Header library. lerp function of template that interpolates a single value between a range. See: https://gamedevbeginner.com/the-right-way-to-lerp-in-unity-with-examples/
 * 
 */
namespace adas_common
{

/**
 * @brief lerp template function. Note: inner values are all static cast to type T
 * 
 * @tparam T function template variable
 * @param a lower limit
 * @param b upper limit
 * @param t interpolation point (like a percentage/ratio)
 * @return T returned point
 */
template <typename T> T lerp(const T a, const T b, const T t) {
    static_assert(std::numeric_limits<T>::is_iec559, "Must be floating point."); //check for floating point

    const T tUse = boost::algorithm::clamp(t, static_cast<T>(0.0), static_cast<T>(1.0));

    return (a * (static_cast<T>(1.0) - tUse)) + (b * tUse);
}

}

#endif
