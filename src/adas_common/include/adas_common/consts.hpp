#ifndef ADAS_COMMON_CONSTS_HPP_
#define ADAS_COMMON_CONSTS_HPP_

#include <cmath>

/**
 * @brief Header library in namespace:adas_common that defines static global constants in struct:Consts
 * 
 */
namespace adas_common
{

struct Consts
{
    static constexpr double steeringCommandMin = -M_PI / 6.0;
    static constexpr double steeringCommandNeutral = 0.0;
    static constexpr double steeringCommandMax = steeringCommandMin;
    static constexpr double throttleCommandMin = 0.0;
    static constexpr double throttleCommandMax = 100.0;
};

}

#endif
