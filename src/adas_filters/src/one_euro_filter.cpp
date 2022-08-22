#include "adas_filters/one_euro_filter.hpp"

namespace adas_filters
{

OneEuroFilter::OneEuroFilter(
    const double samplingFreqHz,
    const double minCutoffFreqHz,
    const double cutoffSlope,
    const double dxCutoffFreqHz) :
    samplingFreqHz{samplingFreqHz},
    prevSet{false},
    prev{0.0},
    minCutoffFreqHz{minCutoffFreqHz},
    cutoffSlope{cutoffSlope},
    dxFilter{ewmaFilter(dxCutoffFreqHz, samplingFreqHz)},
    xFilter{ewmaFilter(minCutoffFreqHz, samplingFreqHz)}
{}

double OneEuroFilter::output(const double input)
{
    double dx = 0.0;
    if (prevSet)
    { dx = (input - prev) * samplingFreqHz; }

    // Find out the rate of change of the value
    double dxFiltered = dxFilter.output(dx);

    // Increase the cutoff in proportion to the rate of change.
    double cutoff = minCutoffFreqHz + cutoffSlope * std::fabs(dxFiltered);
    ewmaFilter(xFilter, cutoff, samplingFreqHz);
    double result = xFilter.output(input);

    prev = result;
    prevSet = true;

    return result;
}

void OneEuroFilter::reset()
{
    dxFilter.reset();
    xFilter.reset();
}

}
