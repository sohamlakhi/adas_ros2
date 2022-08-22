#include <gtest/gtest.h>

#include "adas_filters/median_filter.hpp"

#include "scaffold.hpp"

using namespace adas_filters;

TEST(MedianFilter, NoFilter)
{
    constexpr size_t windowSize = 5;
    MedianFilter<> filt{windowSize};

    std::array<double, windowSize> ins{};
    std::iota(ins.begin(), ins.end(), 1.0);

    // First few values pass unfiltered.
    for (unsigned i = 0; i < windowSize - 1; i++)
    { EXPECT_DOUBLE_EQ(ins[i], filt.output(ins[i])); }

    // From this point on, the median value is picked.
    EXPECT_DOUBLE_EQ(ins[windowSize / 2], filt.output(ins[windowSize - 1]));
}

TEST(MedianFilter, LongRun)
{
    constexpr size_t windowSize = 7;
    MedianFilter<> filt{windowSize};

    std::array<double, 10 * windowSize> ins{};
    std::iota(ins.begin(), ins.end(), 1.0);

    // First few values pass unfiltered.
    for (unsigned i = 0; i < windowSize - 1; i++)
    { EXPECT_DOUBLE_EQ(ins[i], filt.output(ins[i])); }

    // From this point on, the median value is picked.
    for (unsigned i = windowSize - 1; i < ins.size(); i++)
    {
        EXPECT_DOUBLE_EQ(ins[i - (windowSize / 2)], filt.output(ins[i]));
    }
}

TEST(MedianChecker, StdDevTrivial)
{
    constexpr size_t windowSize = 5;

    std::vector<double> sig(1000, 0.0);
    auto noise = impulseNoise(sig, windowSize, 0.1, 3.1);
    for (unsigned i = 0; i < windowSize; i++)
    { noise[i] = 0.0; }

    const size_t numImpulses = std::count_if(
        noise.begin(),
        noise.end(),
        [] (const double v) { return v != 0.0; });

    median_filter_checker::StdDevChecker checker{windowSize};

    size_t numDetectedImpulses = 0;
    for (const auto v: noise)
    {
        if (checker(v))
        { numDetectedImpulses++; }
        checker.update(0.0);
    }

    EXPECT_EQ(numImpulses, numDetectedImpulses);
}

TEST(MedianChecker, StdDevSine)
{
    constexpr size_t windowSize = 5;
    constexpr double signalAmplitude = 1.0;
    constexpr double signalFreqHz = 1.0;
    constexpr double samplingFreqHz = 30.0;
    constexpr size_t numSamples = samplingFreqHz * 100;

    auto sig = sineSamples(
        numSamples,
        signalAmplitude,
        signalFreqHz,
        samplingFreqHz);

    auto noise = impulseNoise(sig, windowSize, 0.1, 5.0);
    for (unsigned i = 0; i < windowSize; i++)
    { noise[i] = 0.0; }

    const size_t numImpulses = std::count_if(
        noise.begin(),
        noise.end(),
        [] (const double v) { return v != 0.0; });

    auto added = signalAdd(sig, noise);

    median_filter_checker::StdDevChecker checker{windowSize};

    size_t numDetectedImpulses = 0;
    for (auto its = std::make_pair(sig.begin(), added.begin());
        its.first != sig.end() && its.second != added.end();
        ++its.first, ++its.second)
    {
        const double s = *its.first;
        const double a = *its.second;

        if (checker(a))
        { numDetectedImpulses++; }
        checker.update(s);
    }

    const size_t diff =
        numImpulses > numDetectedImpulses ?
        numImpulses - numDetectedImpulses :
        numDetectedImpulses - numImpulses;
    const double detectionError = static_cast<double>(diff) / numImpulses;
    // Tolerate up to certain percentage median detection error rate
    EXPECT_TRUE(detectionError < 0.20);
}

TEST(MedianFilterChecker, Test)
{
    constexpr size_t windowSize = 5;
    constexpr double signalAmplitude = 1.0;
    constexpr double signalFreqHz = 1.0;
    constexpr double samplingFreqHz = 30.0;
    constexpr size_t numSamples = samplingFreqHz * 100;
    constexpr double minSNRImprovement = 1.75;

    auto sig = sineSamples(
        numSamples,
        signalAmplitude,
        signalFreqHz,
        samplingFreqHz);

    auto noise = impulseNoise(sig, windowSize, 0.1, 5.0);
    for (unsigned i = 0; i < windowSize; i++)
    { noise[i] = 0.0; }

    auto added = signalAdd(sig, noise);

    using Filt = MedianFilter<median_filter_checker::StdDevChecker>;
    Filt filt{windowSize};

    auto filtered = applyFilter(filt, added);

    const double unfilteredSNR = rmsPower(sig) / rmsPower(noise);
    const double filteredSNR = rmsPower(sig) / rmsPower(residual(sig, filtered));

    EXPECT_TRUE(minSNRImprovement <= filteredSNR / unfilteredSNR);
}

int main(int argc, char *argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
