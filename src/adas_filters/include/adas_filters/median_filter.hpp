#ifndef MEDIAN_FILTER_HPP_
#define MEDIAN_FILTER_HPP_

#include "adas_common/scope_guard.hpp"

#include <algorithm>
#include <boost/circular_buffer.hpp>

namespace adas_filters
{

namespace median_filter_checker
{

/*
 * A checker asks: Is the current unfiltered value I am considering an impulse?
 * Two things are provided to the checker:
 * - Current, unfiltered value: operator(), when the decision is made
 * - Result of the current filter iteration: update(), after the filtering
 * As a result, the checker can consider both past filtered and unfiltered values.
 * NOTE: It is up to the caller to guarantee that the corresponding functions are
 * appropriately called.
 */

struct Default
{
    Default(const size_t windowSize);
    bool operator()(const double unfiltered);
    void update(const double filtered);
    void reset();
};

/*
 * Check the current value's deviation against the previous filtered values'
 * 3 * stddev range. If the value is outside this range, it is considered
 * an impulse.
 */
struct StdDevChecker
{
    StdDevChecker(const size_t windowSize);

    bool operator()(const double unfiltered);
    void update(const double filtered);
    void reset();

    boost::circular_buffer<double> vals;
};

}

template
<typename Checker = median_filter_checker::Default>
class MedianFilter
{
public:
    MedianFilter(const size_t windowSize) :
        windowSize{windowSize},
        scratch(windowSize),
        vals{windowSize},
        checker{windowSize}
    {}

    double output(const double in)
    {
        vals.push_front(in);

        // Is the median window filled?
        const bool medianWindowFull = vals.size() == vals.capacity();
        // Is there an impulse being detected?
        const bool impulseDetected = checker(in);

        // Guarantee that the return value is used to update the checker.
        double res = in;
        auto guard = makeGuard([this, &res] () { checker.update(res); });

        if (!medianWindowFull || !impulseDetected)
        { return res; }

        std::copy(vals.begin(), vals.end(), scratch.begin());

        const size_t midIndex = windowSize / 2;
        std::nth_element(scratch.begin(), scratch.begin() + midIndex, scratch.end());

        res = scratch[midIndex];
        return res;
    }

    void reset()
    {
        checker.reset();
        vals.clear();
    }
private:
    size_t windowSize;
    std::vector<double> scratch;
    boost::circular_buffer<double> vals;
    Checker checker;
};

}

#endif
