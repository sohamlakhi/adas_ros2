#include "adas_common/ros_time_diff.hpp"

namespace adas_common
{

RosTimeDiff::RosTimeDiff()
    : prevTime(0)
{}

bool RosTimeDiff::update(ros::Duration &diff)
{
    ros::Time currTime = ros::Time::now();

    if (prevTime == ros::Time(0))
    {
        prevTime = currTime;
        return false;
    }

    /*
     * Under an extremely heavy load, or in simulation (Where an external program publishes
     * to /clock), it is very possible that the time hasn't progressed. This _should_ be an
     * extremely transient condition, so let go of the CPU until it is resolved.
     */
    while (currTime == prevTime && ros::ok())
    {
        currTime = ros::Time::now();
        sched_yield();
    }

    if (!ros::ok())
    { return false; }

    diff = currTime - prevTime;
    prevTime = currTime;

    return true;
}

void RosTimeDiff::reset()
{
    prevTime = ros::Time(0);
}

}
