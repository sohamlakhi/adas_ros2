#ifndef DISCRETE_FILTER_CHAIN_HPP_
#define DISCRETE_FILTER_CHAIN_HPP_

#include "discrete_filter_helpers.hpp"
#include "median_filter.hpp"

namespace adas_filters
{

template <typename Filter1, typename Filter2>
class FilterChain
{
public:
    FilterChain(Filter1 &&filt1, Filter2 &&filt2) : filt1(filt1), filt2(filt2) {}

    double output(const double input)
    {
        return filt2.output(filt1.output(input));
    }

    void reset()
    {
        filt1.reset();
        filt2.reset();
    }
private:
    Filter1 filt1;
    Filter2 filt2;
};

template <typename Checker>
using MedianEWMAFilter = FilterChain<MedianFilter<Checker>, EWMAFilter>;

}

#endif
