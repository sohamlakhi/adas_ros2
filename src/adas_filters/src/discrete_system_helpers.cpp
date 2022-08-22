#include "adas_filters/discrete_system_helpers.hpp"

namespace adas_filters
{

FirstOrderSystem firstOrderSystem(
    const double k,
    const double tau,
    const double samplingFreqHz,
    const double initialOutput)
{
    const double T = 1.0 / samplingFreqHz;
    const double denom = 2.0 * tau + T;
    const double inCoeff = k * T / denom;
    return {{inCoeff, inCoeff}, {(T - 2.0 * tau) / denom}, initialOutput};
}


AsymmetricFirstOrderSystem::AsymmetricFirstOrderSystem(
    const double k,
    const double increaseTau,
    const double decreaseTau,
    const double samplingFreqHz,
    const double initialOutput) :
    prev(0.0),
    beenIncreasing(true),
    increaseSystem(firstOrderSystem(k, increaseTau, samplingFreqHz, initialOutput)),
    decreaseSystem(firstOrderSystem(k, decreaseTau, samplingFreqHz, initialOutput))
{}

double AsymmetricFirstOrderSystem::output(const double in)
{
    FirstOrderSystem &selected = select(in);
    return selected.output(in);
}

void AsymmetricFirstOrderSystem::reset()
{
    increaseSystem.reset();
    decreaseSystem.reset();
}

void AsymmetricFirstOrderSystem::reset(const double newInitialOutput)
{
    increaseSystem.reset(newInitialOutput);
    decreaseSystem.reset(newInitialOutput);
}

FirstOrderSystem &AsymmetricFirstOrderSystem::select(const double curr)
{
    if (prev < curr && !beenIncreasing)
    {
        // Now increasing. Inherit the state from the decreasing system.
        beenIncreasing = true;
        increaseSystem.assignState(decreaseSystem);
    }
    else if (prev > curr && beenIncreasing)
    {
        // Now decreasing. Inherit the state from the increasing system.
        beenIncreasing = false;
        decreaseSystem.assignState(increaseSystem);
    }

    if (beenIncreasing)
    { return increaseSystem; }
    return decreaseSystem;
}

}
