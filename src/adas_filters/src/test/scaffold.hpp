#ifndef SCAFFOLD_HPP_
#define SCAFFOLD_HPP_

#include <vector>
#include <tuple>
#include <algorithm>

std::vector<double> sineSamples(
    const size_t numSamples,
    const double amplitude,
    const double signalFreqHz,
    const double samplingFreqHz);

std::vector<double> gaussianNoise(
    const size_t numSamples,
    const double noiseStdDev);

auto impulseNoise(
    const std::vector<double> &src,
    const size_t srcWindowSize,
    const double impulseProbability,
    const double impulseStdDevTimes) -> std::decay<decltype(src)>::type;

auto signalAdd(
    const std::vector<double> &l,
    const std::vector<double> &r) -> std::decay<decltype(l)>::type;

double rmsPower(const std::vector<double> &sig);

std::vector<double> residual(const std::vector<double> &r, const std::vector<double> &y);

template <typename T>
std::vector<double> applyFilter(T &filt, const std::vector<double> &vals)
{
    std::vector<double> res(vals.size(), 0.0);

    std::transform(
        vals.begin(),
        vals.end(),
        res.begin(),
        [&](const double v) { return filt.output(v); });

    return res;
}

size_t calcShift(
    const std::vector<double> &orig,
    const std::vector<double> &lagged);

#endif
