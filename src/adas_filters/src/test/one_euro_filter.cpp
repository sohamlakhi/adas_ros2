#include <gtest/gtest.h>

#include "adas_filters/one_euro_filter.hpp"
#include "adas_filters/discrete_filter_helpers.hpp"

#include "scaffold.hpp"

using namespace adas_filters;

TEST(OneEuroFilter, Basic)
{
    // Zero cutoff slope - Behaves like a normal EWMA
    constexpr double samplingFreqHz = 30.0;
    constexpr double minCutoffFreqHz = 7.5;
    constexpr double dxCutoffFreqHz = 10.0;

    EWMAFilter ewma{ewmaFilter(minCutoffFreqHz, samplingFreqHz)};
    OneEuroFilter filt{samplingFreqHz, minCutoffFreqHz, 0, dxCutoffFreqHz};

    constexpr size_t N = 50;

    std::vector<double> in(N, 0.0);
    std::iota(in.begin(), in.end(), 0.0);

    for (const auto v: in)
    { EXPECT_DOUBLE_EQ(ewma.output(v), filt.output(v)); }
}

TEST(OneEuroFilter, Filtering)
{
    constexpr double samplingFreqHz = 30.0;
    constexpr double noiseStdDev = 1.0 / 3.0; // 99.7% of values fall between [-1, 1]
    constexpr size_t numSamples = std::round(100 * samplingFreqHz);

    constexpr double minCutoffFreqHz = 1.0;
    constexpr double cutoffSlope = 0.01;
    constexpr double dxCutoffFreqHz = 5.0;

    // Degradation between EWMA and One Euro for constant signal must be higher than this
    constexpr double degradationThreshold = 0.85;

    auto noise = gaussianNoise(numSamples, noiseStdDev);

    std::vector<double> step(numSamples, 1.0);
    auto stepNoisy = signalAdd(step, noise);

    EWMAFilter ewma = ewmaFilter(minCutoffFreqHz, samplingFreqHz);
    OneEuroFilter oneEuro{samplingFreqHz, minCutoffFreqHz, cutoffSlope, dxCutoffFreqHz};

    auto ewmaStep = applyFilter(ewma, stepNoisy);
    auto oneEuroStep = applyFilter(oneEuro, stepNoisy);

    const double ewmaStepSNR = rmsPower(step) / rmsPower(residual(step, ewmaStep));
    const double oneEuroStepSNR = rmsPower(step) / rmsPower(residual(step, oneEuroStep));

    const double stepSNRImprove = oneEuroStepSNR / ewmaStepSNR;

    EXPECT_TRUE(stepSNRImprove > degradationThreshold);

    std::vector<double> ramp(numSamples);
    std::iota(ramp.begin(), ramp.end(), 0.0);
    auto rampNoisy = signalAdd(ramp, noise);

    ewma.reset();
    oneEuro.reset();

    auto ewmaRamp = applyFilter(ewma, rampNoisy);
    auto oneEuroRamp = applyFilter(oneEuro, rampNoisy);

    auto ewmaRampShift = calcShift(rampNoisy, ewmaRamp);
    auto oneEuroRampShift = calcShift(rampNoisy, oneEuroRamp);

    // Lag is less for One Euro filter
    EXPECT_TRUE(ewmaRampShift > oneEuroRampShift);
}

int main(int argc, char *argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
