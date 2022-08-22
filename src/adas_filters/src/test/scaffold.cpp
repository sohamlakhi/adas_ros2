#include "scaffold.hpp"

#include <algorithm>
#include <vector>
#include <tuple>
#include <random>
#include <stdexcept>

#include <iostream>

namespace
{
std::random_device rd;
std::mt19937 gen(rd());

template <typename It>
typename std::iterator_traits<It>::value_type mean(It start, It end)
{
    using Ret = typename std::iterator_traits<It>::value_type;
    const size_t N = std::distance(start, end);
    const Ret sum =
        std::accumulate(
            start,
            end,
            0.0,
            std::plus<Ret>{});
    return sum / N;
}

template <typename It>
typename std::iterator_traits<It>::value_type stddev(It start, It end)
{
    using Ret = typename std::iterator_traits<It>::value_type;

    const size_t N = std::distance(start, end);
    const Ret mu = mean(start, end);
    const Ret sumResidualSq = std::accumulate(
        start,
        end,
        0.0,
        [mu] (const Ret acc, const Ret v) { return std::pow(v - mu, 2); });

    return std::sqrt(sumResidualSq / (N - 1));
}

}

std::vector<double> sineSamples(
    const size_t numSamples,
    const double amplitude,
    const double signalFreqHz,
    const double samplingFreqHz)
{
    std::vector<double> res(numSamples);

    std::iota(res.begin(), res.end(), 0.0);
    std::transform(
        res.begin(),
        res.end(),
        res.begin(),
        [=](const double v)
        { return amplitude * std::sin(2.0 * M_PI * signalFreqHz + v / samplingFreqHz); });

    return res;
}

std::vector<double> gaussianNoise(
    const size_t numSamples,
    const double noiseStdDev)
{
    std::vector<double> noise(numSamples);
    std::normal_distribution<double> dist(0.0, noiseStdDev);
    std::generate(noise.begin(), noise.end(), [&] () { return dist(gen); });

    return noise;
}

auto impulseNoise(
    const std::vector<double> &src,
    const size_t srcWindowSize,
    const double impulseProbability,
    const double impulseStdDevTimes) -> std::decay<decltype(src)>::type
{
    if (impulseProbability > 1.0 || impulseProbability < 0)
    { throw std::invalid_argument("Impulse probability must be within range [0, 1]."); }

    if (impulseStdDevTimes < 0)
    { throw std::invalid_argument("Impulse stddev multiplier must be positive."); }

    std::decay<decltype(src)>::type res(src.size());
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    std::uniform_int_distribution<unsigned> sgn(0, 1);

    for (auto it = src.begin(); it != src.end(); ++it)
    {
        if (dist(gen) > impulseProbability) { continue; }
        const size_t i = std::distance(src.cbegin(), it);

        auto window = i < srcWindowSize ?
            std::make_pair(src.cbegin(), it + 1) :
            std::make_pair(it - srcWindowSize, it + 1);

        const double sigma = stddev(window.first, window.second);

        const double deviation = [impulseStdDevTimes, sigma, &sgn]
        {
            double val = impulseStdDevTimes * sigma;
            if (sigma < 1e-5)
            { val = 1.0; }
            if (sgn(gen))
            { val *= -1.0; }

            return val;
        } ();

        res[i] = *it + deviation;
    }

    return res;
}

auto signalAdd(
    const std::vector<double> &l,
    const std::vector<double> &r) -> std::decay<decltype(l)>::type
{
    if (l.size() != r.size())
    { throw std::invalid_argument("Left and right signals to add must match."); }

    std::decay<decltype(l)>::type res(l.size());
    std::transform(l.begin(), l.end(), r.begin(), res.begin(), std::plus<double>{});

    return res;
}

double rmsPower(const std::vector<double> &sig)
{
    return std::accumulate(
            sig.begin(),
            sig.end(),
            0.0,
            [] (const double acc, const double v) { return acc + std::pow(v, 2); }) /
        sig.size();
}

std::vector<double> residual(const std::vector<double> &r, const std::vector<double> &y)
{
    std::vector<double> res(r.size(), 0.0);

    std::transform(
        r.begin(),
        r.end(),
        y.begin(),
        res.begin(),
        [] (const double l, const double r) { return l - r; });

    return res;
}

size_t calcShift(
    const std::vector<double> &orig,
    const std::vector<double> &lagged)
{
    if (orig.size() != lagged.size())
    { throw std::invalid_argument{"Equal size vectors required."}; }

    const size_t vecSize = orig.size();
    // Shift up to only half, to make sure that we do meaningful amount of calculations.
    std::vector<double> distances(vecSize / 2);

    for (size_t i = 0; i < distances.size(); i++)
    {
        size_t samplesTaken = 0;
        for (size_t j = 0; j < vecSize; j++)
        {
            const double origValue = orig[j];
            const size_t leadIdx = j + i;
            if (leadIdx >= lagged.size())
            { break; }
            samplesTaken++;
            const double laggedValue = lagged[leadIdx];

            distances[i] += std::fabs(origValue - laggedValue);
        }
        distances[i] /= samplesTaken;
    }

    auto minItr = std::min_element(distances.begin(), distances.end());
    return std::distance(distances.begin(), minItr);
}
