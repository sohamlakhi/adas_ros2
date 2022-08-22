#ifndef DISCRETE_FILTER_HELPERS_HPP_
#define DISCRETE_FILTER_HELPERS_HPP_

#include "discrete_filter.hpp"

namespace adas_filters
{

using EWMAFilter = DiscreteFilter<1, 1>;
/*
 * Exponentially weighted moving average.
 * y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
 */
EWMAFilter ewmaFilter(const double alpha);
/*
 * EWMA, constructed with cutoff frequency.
 */
EWMAFilter ewmaFilter(const double cutoffFreqHz, const double samplingFreqHz);

// Modifies the existing EWMA Filter.
void ewmaFilter(EWMAFilter &filt, const double alpha);
void ewmaFilter(EWMAFilter &filt, const double cutoffFreqHz, const double samplingFreqHz);

}

#endif
