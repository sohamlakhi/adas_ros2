#ifndef ADAS_ROS_TIME_DIFF_HPP_
#define ADAS_ROS_TIME_DIFF_HPP_

#include "rclcpp/rclcpp.hpp"

namespace adas_common
{

class RosTimeDiff
{
public:
    RosTimeDiff();
    bool update(rclcpp::Duration &diff, rclcpp::Node *node);
    void reset();
private:
    rclcpp::Time prevTime;
};

}

#endif
