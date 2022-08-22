#include "adas_filters/median_filter.hpp"

namespace adas_filters
{

namespace median_filter_checker
{


Default::Default(const size_t) {}

bool Default::operator()(const double)
{ return true; }

void Default::update(const double)
{}

void Default::reset()
{}

StdDevChecker::StdDevChecker(const size_t windowSize) : vals{windowSize} {}

bool StdDevChecker::operator()(const double unfiltered)
{
    if (vals.size() != vals.capacity())
    { return false; }

    const double N = static_cast<double>(vals.capacity());

    const double sum = std::accumulate(
        vals.begin(),
        vals.end(),
        0.0,
        std::plus<double>{});
    const double sumSq = std::accumulate(
        vals.begin(),
        vals.end(),
        0.0,
        [] (const double acc, const double v) { return acc + std::pow(v, 2); });

    const double mean = sum / N;
    const double stddev = std::sqrt((sumSq - std::pow(sum, 2) / N) / (N - 1));

    const double dev = std::fabs(unfiltered - mean);

    return dev > 3 * stddev;
}

void StdDevChecker::update(const double filtered)
{ vals.push_front(filtered); }

void StdDevChecker::reset()
{ vals.clear(); }

}

}
