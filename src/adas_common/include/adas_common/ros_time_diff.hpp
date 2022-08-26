#ifndef ADAS_ROS_TIME_DIFF_HPP_
#define ADAS_ROS_TIME_DIFF_HPP_

#include <ros/ros.h>

namespace adas_common
{

class RosTimeDiff
{
public:
    RosTimeDiff();
    bool update(ros::Duration &diff);
    void reset();
private:
    ros::Time prevTime;
};

}

#endif
