#ifndef DISCRETE_SYSTEM_HELPERS_HPP_
#define DISCRETE_SYSTEM_HELPERS_HPP_

#include "discrete_system.hpp"

namespace adas_filters
{

using FirstOrderSystem = DiscreteSystem<2, 1>;
/*
 * Discretized version of first-order low-pass filter, i.e.
 * G = k / (tau * s + 1).
 */
FirstOrderSystem firstOrderSystem(
    const double k,
    const double tau,
    const double samplingFreqHz,
    const double initialOutput = 0.0);

/*
 * First order filter that has a different tau value depending on if the input is increasing
 * or decreasing.
 */
class AsymmetricFirstOrderSystem
{
public:
    AsymmetricFirstOrderSystem(
        const double k,
        const double increaseTau,
        const double decreaseTau,
        const double samplingFreqHz,
        const double initialOutput = 0);

    double output(const double in);
    void reset();
    void reset(const double newInitialOutput);

private:
    FirstOrderSystem &select(const double in);

    double prev;
    bool beenIncreasing;
    FirstOrderSystem increaseSystem;
    FirstOrderSystem decreaseSystem;
};

}

#endif
