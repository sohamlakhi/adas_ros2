#ifndef ONE_EURO_FILTER_HPP_
#define ONE_EURO_FILTER_HPP_

#include "adas_filters/discrete_filter_helpers.hpp"

/* Very simple adaptive filter as described in
 * http://cristal.univ-lille.fr/~casiez/1euro/
 * RC cars used in this project currently are off-the-shelf
 * models and thus built-to-cost. This makes an accurate enough
 * dynamic modeling of the system required for a good Kalman
 * filter performance hard, hence the use of this filter.
 */

namespace adas_filters
{

class OneEuroFilter
{
public:
    OneEuroFilter(
        const double samplingFreqHz,
        const double minCutoffFreqHz,
        const double cutoffSlope,
        const double dxCutoffFreqHz);

    double output(const double input);

    void reset();
private:
    double samplingFreqHz;

    // Used for calculating the derivative of the value.
    bool prevSet;
    double prev;

    // Lowest cutoff frequency given no motion.
    double minCutoffFreqHz;
    /* Rate of increase of cutoff frequency in relation to derivative.
     * Referred to as "beta."
     */
    double cutoffSlope;

    EWMAFilter dxFilter;
    EWMAFilter xFilter;
};

}

#endif
