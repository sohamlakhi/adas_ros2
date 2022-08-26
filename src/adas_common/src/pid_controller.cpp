#include "adas_common/pid_controller.hpp"

#include <limits>
#include <boost/algorithm/clamp.hpp>
#include <stdexcept>

namespace adas_common
{

PIDController::PIDController(
    double p,
    double i,
    double d,
    double saturationMin,
    double saturationMax)
    : p(p),
      i(i),
      d(d),
      saturationMin(saturationMin),
      saturationMax(saturationMax),
      prevValid(false),
      goalValid(false),
      accumulate(true),
      accum(0)
{
    checkSaturation();
}

void PIDController::setGoal(const double r)
{
    goalValid = true;
    goal = r;
}

void PIDController::setSaturation(const double min, const double max)
{
    saturationMin = min;
    saturationMax = max;

    // Inherit the accumulation status, but trim the accum value
    accum = boost::algorithm::clamp(accum, saturationMin, saturationMax);

    checkSaturation();
}

void PIDController::checkSaturation()
{
    if (saturationMin > saturationMax)
    { throw std::invalid_argument{"Saturation min value must be smaller than the max value."}; }
}

double PIDController::commandStep(
    const double feedforward,
    const double y,
    const double dt,
    const bool doAccumulate)
{
    if (!goalValid) { return 0.0; }

    double e = goal - y;

    double pTerm = p * e;
    double totalTerm = pTerm + feedforward;

    if (!prevValid)
    {
        prevError = e;
        prevValid = true;
        return boost::algorithm::clamp(totalTerm, saturationMin, saturationMax);
    }

    // Accumulate if told by user AND windup prevention not in place
    if (doAccumulate && accumulate)
    {
        // Trapezoidal integration
        accum += (prevError + e) / 2 * dt;
    }

    double iTerm = i * accum;
    double dTerm = d * (e - prevError) / dt;

    totalTerm += iTerm + dTerm;

    // Update prev
    prevError = e;

    // Update windup prevention
    bool saturating = totalTerm < saturationMin || totalTerm > saturationMax;
    bool errorAccumSignMatch = accum * prevError > 0;
    accumulate = !(saturating && errorAccumSignMatch);

    return boost::algorithm::clamp(totalTerm, saturationMin, saturationMax);
}

void PIDController::reset()
{
    accum = 0.0;
    prevValid = false;
    accumulate = true;
}

}
