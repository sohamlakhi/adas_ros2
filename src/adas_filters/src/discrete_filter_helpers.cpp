#include "adas_filters/discrete_filter_helpers.hpp"

namespace
{

double calculateAlpha(const double cutoffFreqHz, const double samplingFreqHz)
{
    double T = 1.0 / samplingFreqHz;
    double alphaCommon = 2.0 * M_PI * T * cutoffFreqHz;
    return alphaCommon / (alphaCommon + 1);
}

}

namespace adas_filters
{

EWMAFilter ewmaFilter(const double alpha)
{
    return {{alpha}, {-(1.0 - alpha)}};
}

EWMAFilter ewmaFilter(const double cutoffFreqHz, const double samplingFreqHz)
{
    const double alpha = calculateAlpha(cutoffFreqHz, samplingFreqHz);

    return ewmaFilter(alpha);
}

void ewmaFilter(EWMAFilter &filt, const double alpha)
{
    filt.assignCoeffs({alpha}, {-(1.0 - alpha)});
}

void ewmaFilter(EWMAFilter &filt, const double cutoffFreqHz, const double samplingFreqHz)
{
    const double alpha = calculateAlpha(cutoffFreqHz, samplingFreqHz);

    ewmaFilter(filt, alpha);
}

}
