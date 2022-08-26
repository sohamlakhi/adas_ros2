#ifndef TIME_DIFF_HPP
#define TIME_DIFF_HPP

#include <type_traits>
#include <limits>

namespace adas_common
{

template <typename T>
class TimeDiff
{
public:
    static_assert(std::is_integral<T>::value, "Time difference operates on integral values.");

    TimeDiff() : prevSet(false) {}

    bool update(T curr, T &diff)
    {
        if (!prevSet)
        {
            prevSet = true;
            prev = curr;
            return false;
        }

        diff = curr - prev;
        if (prev > curr)
        {
            // Rollover scenario.
            diff = (std::numeric_limits<T>::max() - prev) + curr + 1;
        }

        prev = curr;

        return true;
    }

    void reset()
    {
        prevSet = false;
    }

private:
    bool prevSet;
    T prev;
};

}

#endif
