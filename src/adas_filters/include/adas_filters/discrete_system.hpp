#ifndef DISCRETE_SYSTEM_HPP_
#define DISCRETE_SYSTEM_HPP_

#include "difference_equation.hpp"

namespace adas_filters
{

/*
 * Used for control engineering definition of a system.
 * Input excites the system, initially at rest, producing an output.
 */
template <size_t numInAndPrevs, size_t numOutPrevs>
class DiscreteSystem
{
public:
    using Eqn = DifferenceEquation<numInAndPrevs, numOutPrevs>;

    DiscreteSystem(
        typename Eqn::InCoeffs &&inCoeffs,
        typename Eqn::OutCoeffs &&outCoeffs,
        const double initialOutput = 0) :
        eqn{std::move(inCoeffs), std::move(outCoeffs)},
        initialOutput{initialOutput}
    { reset(); }

    double output(const double input)
    { return eqn.output(input); }

    void reset(const double newInitialOutput)
    {
        initialOutput = newInitialOutput;
        reset();
    }

    void reset()
    {
        typename Eqn::InStates inStates;
        typename Eqn::OutStates outStates;

        // No input is assumed.
        inStates.fill(0.0);
        // System is at rest, at the initial value.
        outStates.fill(initialOutput);

        eqn.assignState(inStates, outStates);
    }

    void assignState(const DiscreteSystem &other)
    {
        eqn.assignState(other.eqn);
    }

private:
    Eqn eqn;
    double initialOutput;
};

}

#endif
