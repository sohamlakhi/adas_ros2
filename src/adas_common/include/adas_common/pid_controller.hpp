#ifndef ADAS_PID_CONTROLLER_HPP_
#define ADAS_PID_CONTROLLER_HPP_

#include <stdint.h>

namespace adas_common
{
/**
 * @brief class to return PID output packet (discrete signal) given input packet (discrete signal)
 *  see: https://www.csimn.com/CSI_pages/PIDforDummies.html
 */

class PIDController
{
public:
    /**
     * @brief Construct a new PIDController object
     *  default constructor with everything initialised to base values
     * 
     */
    PIDController() :
        p(0),
        i(0),
        d(0),
        saturationMin(0),
        saturationMax(0),
        prevValid(false),
        goalValid(false),
        accumulate(true),
        accum(0)
    {}

    PIDController(
        const double p,
        const double i,
        const double d,
        const double saturationMin,
        const double saturationMax);
    /**
     * @brief Set the private member goal
     * 
     * @param r goal 
     */
    void setGoal(const double r);
    /**
     * @brief Get the private member goal
     * 
     * @return double goal
     */
    double getGoal() const { return goal; }

    /**
     * @brief PID control step 
     * 
     * @param feedforward feedforward signal (see: https://blog.incatools.com/benefits-feedforward-pid-controllers )
     * @param y previous output
     * @param dt time step
     * @param doAccumulate boolean to turn on accumlation
     * @return double 
     */
    double commandStep(
        const double feedforward,
        const double y,
        const double dt,
        const bool doAccumulate = true);
    void reset();

    double p;
    double i;
    double d;

    void setSaturation(const double min, const double max);
private:
    void checkSaturation();

    double saturationMin;
    double saturationMax;

    bool prevValid;
    double prevError;
    bool goalValid;
    double goal;
    bool accumulate;
    double accum;
};

}

#endif
