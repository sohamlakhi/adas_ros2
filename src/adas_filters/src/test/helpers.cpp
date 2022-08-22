#include <gtest/gtest.h>

#include <cmath>
#include <vector>
#include <tuple>

#include "scaffold.hpp"

#include "adas_filters/discrete_filter_helpers.hpp"
#include "adas_filters/discrete_system_helpers.hpp"

using namespace adas_filters;

TEST(FirstOrderSystem, Gain)
{
    const double tau = 5.0;
    const double samplingFreqHz = 30.0;
    const double k1 = 3.0;
    const double k2 = 7.5;

    FirstOrderSystem sys1 = firstOrderSystem(k1, tau, samplingFreqHz);
    FirstOrderSystem sys2 = firstOrderSystem(k2, tau, samplingFreqHz);
    // By this time, given a unit step, the value should be within 2% of steady state.
    const unsigned numSettlingTimeSamples = std::round(4 * tau * samplingFreqHz);

    double steadyState1 = 0;
    double steadyState2 = 0;
    for (unsigned i = 0; i < numSettlingTimeSamples; i++)
    {
        steadyState1 = sys1.output(1.0);
        steadyState2 = sys2.output(1.0);
    }

    EXPECT_NEAR(k1, steadyState1, 0.02 * k1);
    EXPECT_NEAR(k2, steadyState2, 0.02 * k2);
}

TEST(FirstOrderSystem, TimeConstant)
{
    // Advancing the output by tau will produce 63% of steady state value.
    const double tau1 = 3.0;
    const double samplingFreqHz = 30.0;
    const double k = 1.0;

    FirstOrderSystem sys1 = firstOrderSystem(k, tau1, samplingFreqHz);
    const unsigned numTau1Samples = std::round(tau1 * samplingFreqHz);

    double out1;
    for (unsigned i = 0; i < numTau1Samples; i++)
    { out1 = sys1.output(1.0); }

    EXPECT_NEAR(0.63 * k, out1, 5e-2);

    const double tau2 = 8.0;
    FirstOrderSystem sys2 = firstOrderSystem(k, tau2, samplingFreqHz);
    const unsigned numTau2Samples = std::round(tau2 * samplingFreqHz);

    double out2;
    for (unsigned i = 0; i < numTau2Samples; i++)
    { out2 = sys2.output(1.0); }
    EXPECT_NEAR(0.63 * k, out2, 5e-2);
}

// TODO AssignState tests
// TODO AsymmetricFirstOrderSystem tests

TEST(EWMAFilter, Basic)
{
    constexpr double alpha = 0.3;
    EWMAFilter filt = ewmaFilter(alpha);

    constexpr size_t N = 50;

    std::vector<double> in(N, 0.0);
    std::iota(in.begin(), in.end(), 0.0);

    std::vector<double> out(in.size(), 0.0);
    out[0] = in[0];

    for (unsigned i = 1; i < N; i++)
    { out[i] = alpha * in[i] + (1.0 - alpha) * out[i - 1]; }

    for (unsigned i = 0; i < N; i++)
    { EXPECT_DOUBLE_EQ(out[i], filt.output(in[i])); }
}

TEST(EWMAFilter, Filtering)
{
    constexpr double samplingFreqHz = 30.0;
    constexpr double cutoffFreqHz = 10.0;
    constexpr double signalAmplitude = 1.0;
    constexpr double signalFreqHz = 0.5;
    constexpr double noiseStdDev = 1.0 / 3.0; // 99.7% of values fall between [-1, 1]
    constexpr size_t numSamples = std::round(100 * samplingFreqHz);
    constexpr double minSNRImprovement = 1.75;

    for (unsigned i = 0; i < 50; i++)
    {
        auto sig = sineSamples(
            numSamples,
            signalAmplitude,
            signalFreqHz,
            samplingFreqHz);
        auto noise = gaussianNoise(numSamples, noiseStdDev);
        auto added = signalAdd(sig, noise);

        EWMAFilter filt = ewmaFilter(cutoffFreqHz, samplingFreqHz);
        auto filtered = applyFilter(filt, added);

        const double unfilteredSNR = rmsPower(sig) / rmsPower(noise);
        const double filteredSNR = rmsPower(sig) / rmsPower(residual(sig, filtered));

        EXPECT_TRUE(minSNRImprovement <= filteredSNR / unfilteredSNR);
    }
}

int main(int argc, char *argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
