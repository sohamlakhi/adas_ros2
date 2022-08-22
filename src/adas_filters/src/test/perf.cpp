#include <benchmark/benchmark.h>

#include "scaffold.hpp"

#include "adas_filters/difference_equation.hpp"
#include "adas_filters/discrete_filter_helpers.hpp"
#include "adas_filters/median_filter.hpp"
#include "adas_filters/discrete_filter_chain.hpp"
#include "adas_filters/one_euro_filter.hpp"

using namespace adas_filters;

namespace
{

constexpr size_t numSamples = 524288;
constexpr double samplingFreqHz = 30.0;
constexpr double signalFreqHz = 3.0;
constexpr double signalAmplitude = 1.0;

void differenceEquationBench(benchmark::State& st)
{
    const size_t N = static_cast<size_t>(st.range(0));
    auto sig = signalAdd(
            sineSamples(
                N,
                signalAmplitude,
                signalFreqHz,
                samplingFreqHz),
            gaussianNoise(N, 0.1 / 3.0));

    using Eqn = DifferenceEquation<5, 1>;
    Eqn eqn{Eqn::InCoeffs{0.5, 1.5, 0.5, 1.5, 0.5}, Eqn::OutCoeffs{0.1}};

    while (st.KeepRunning())
    {
        double x = 0;
        for (unsigned i = 0; i < N; i++)
        { benchmark::DoNotOptimize(x += eqn.output(sig[i])); }
    }
}
BENCHMARK(differenceEquationBench)
    ->Arg(numSamples)
    ->Unit(benchmark::kMillisecond);

void ewmaFilterBench(benchmark::State &st)
{
    const size_t N = static_cast<size_t>(st.range(0));
    auto sig = signalAdd(
        sineSamples(
            N,
            signalAmplitude,
            signalFreqHz,
            samplingFreqHz),
        gaussianNoise(N, 0.1 / 3.0));

    EWMAFilter filt = ewmaFilter(10.0, samplingFreqHz);

    while (st.KeepRunning())
    {
        double x = 0;
        for (unsigned i = 0; i < N; i++)
        { benchmark::DoNotOptimize(x += filt.output(sig[i])); }

        filt.reset();
    }
}
BENCHMARK(ewmaFilterBench)
    ->Arg(numSamples)
    ->Unit(benchmark::kMillisecond);

void medianFilterBench(benchmark::State &st)
{
    constexpr size_t windowSize = 5;
    const size_t N = static_cast<size_t>(st.range(0));
    auto clean = sineSamples(
        N,
        signalAmplitude,
        signalFreqHz,
        samplingFreqHz);
    auto sig = signalAdd(
        clean,
        impulseNoise(clean, windowSize, 0.3, 3.5));

    MedianFilter<> filt{windowSize};

    while (st.KeepRunning())
    {
        double x = 0;
        for (unsigned i = 0; i < N; i++)
        { benchmark::DoNotOptimize(x += filt.output(sig[i])); }

        filt.reset();
    }
}
BENCHMARK(medianFilterBench)
    ->Arg(numSamples)
    ->Unit(benchmark::kMillisecond);

void checkedMedianFilterBench(benchmark::State &st)
{
    constexpr size_t windowSize = 5;
    const size_t N = static_cast<size_t>(st.range(0));
    auto clean = sineSamples(
        N,
        signalAmplitude,
        signalFreqHz,
        samplingFreqHz);
    auto sig = signalAdd(
        clean,
        impulseNoise(clean, windowSize, 0.3, 3.5));

    using CheckedMedianFilter = MedianFilter<
        median_filter_checker::StdDevChecker>;

    CheckedMedianFilter filt{windowSize};

    while (st.KeepRunning())
    {
        double x = 0;
        for (unsigned i = 0; i < N; i++)
        { benchmark::DoNotOptimize(x += filt.output(sig[i])); }

        filt.reset();
    }
}
BENCHMARK(checkedMedianFilterBench)
    ->Arg(numSamples)
    ->Unit(benchmark::kMillisecond);

void checkedChainedFilterBench(benchmark::State &st)
{
    constexpr size_t windowSize = 5;
    const size_t N = static_cast<size_t>(st.range(0));
    auto clean = sineSamples(
        N,
        signalAmplitude,
        signalFreqHz,
        samplingFreqHz);
    auto sigNoise = signalAdd(clean, gaussianNoise(N, 0.1 / 3.0));
    auto sig = signalAdd(sigNoise, impulseNoise(clean, windowSize, 0.3, 3.5));

    using ChainedFilter = MedianEWMAFilter<
        median_filter_checker::StdDevChecker>;

    ChainedFilter filt{{windowSize}, ewmaFilter(10.0, samplingFreqHz)};

    while (st.KeepRunning())
    {
        double x = 0;
        for (unsigned i = 0; i < N; i++)
        { benchmark::DoNotOptimize(x += filt.output(sig[i])); }

        filt.reset();
    }
}
BENCHMARK(checkedChainedFilterBench)
    ->Arg(numSamples)
    ->Unit(benchmark::kMillisecond);

void oneEuroFilterBench(benchmark::State &st)
{
    const size_t N = static_cast<size_t>(st.range(0));
    auto clean = sineSamples(
        N,
        signalAmplitude,
        signalFreqHz,
        samplingFreqHz);
    auto sigNoise = signalAdd(clean, gaussianNoise(N, 0.1 / 3.0));
    auto sig = signalAdd(sigNoise, gaussianNoise(N, 0.1 / 3.0));

    constexpr double samplingFreqHz = 30.0;
    constexpr double cutoffSlope = 1.0;
    constexpr double minCutoffFreqHz = 10.0;
    constexpr double dxCutoffFreqHz = 10.0;

    OneEuroFilter filt{samplingFreqHz, minCutoffFreqHz, cutoffSlope, dxCutoffFreqHz};

    while (st.KeepRunning())
    {
        double x = 0;
        for (unsigned i = 0; i < N; i++)
        { benchmark::DoNotOptimize(x += filt.output(sig[i])); }

        filt.reset();
    }
}
BENCHMARK(oneEuroFilterBench)
    ->Arg(numSamples)
    ->Unit(benchmark::kMillisecond);

}

BENCHMARK_MAIN();
