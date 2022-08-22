#ifndef DISCRETE_FILTER_HPP_
#define DISCRETE_FILTER_HPP_

#include "difference_equation.hpp"

#include <algorithm>

#include <boost/optional.hpp>

/*
    TODO: @brief https://en.wikipedia.org/wiki/Digital_filter -> this has been implemented with the help of a difference equation
*/

namespace adas_filters
{

/*
 * Used for signal processing filters. first few samples are unfiltered while the
 * initial values for the filter is being gathered.
 */
template <size_t numInAndPrevs, size_t numOutPrevs>
class DiscreteFilter
{
public:
    using Eqn = DifferenceEquation<numInAndPrevs, numOutPrevs>;

private:
    static constexpr size_t numInitialStates =
        (Eqn::numInStates - 1) > Eqn::numOutStates ? (Eqn::numInStates - 1) : Eqn::numOutStates;
    using InitialValues = std::array<double, numInitialStates>;
    using InitialValuesCollector = std::pair<size_t, InitialValues>;
public:
    DiscreteFilter(
        typename Eqn::InCoeffs &&inCoeffs,
        typename Eqn::OutCoeffs &&outCoeffs) :
        eqn{std::move(inCoeffs), std::move(outCoeffs)},
        // Collector used to gather the initial values for the filter.
        collector{std::make_pair(0, InitialValues{})}
    {}

    double output(const double input)
    {
        // Does the collector exist (i.e. Are we collecting initial values)?
        if (collector)
        {
            InitialValues &initialValues = collector->second;
            size_t &numCollected = collector->first;

            initialValues[numCollected++] = input;

            if (numCollected >= numInitialStates)
            {
                typename Eqn::InStates inInitials{};
                typename Eqn::OutStates outInitials{};

                /*
                 * Configure the filter such that the signal has been passing
                 * through unfiltered till now.
                 * Guaranteed to not go out of bounds.
                 */
                std::copy(
                    initialValues.rbegin(),
                    initialValues.rbegin() + (Eqn::numInStates - 1),
                    inInitials.begin());
                std::copy(
                    initialValues.rbegin(),
                    initialValues.rbegin() + Eqn::numOutStates,
                    outInitials.begin());

                eqn.assignState(inInitials, outInitials);

                // Get rid of the collector (i.e. Done collecting).
                collector = decltype(collector){};
            }

            // Return the unfiltered value for now.
            return input;
        }

        return eqn.output(input);
    }

    void reset()
    {
        // Bring the collector back online.
        collector = decltype(collector){std::make_pair(0, InitialValues{})};
    }

    void assignCoeffs(
        const typename Eqn::InCoeffs &inCoeffs,
        const typename Eqn::OutCoeffs &outCoeffs)
    { eqn.assignCoeffs(inCoeffs, outCoeffs); }

private:

    Eqn eqn;
    //checks to see if the values are being collected (run code only if collector returns true)
    boost::optional<InitialValuesCollector> collector;
};

}

#endif
