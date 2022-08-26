#ifndef PROFILER_HPP_
#define PROFILER_HPP_

#include <ros/ros.h>

#include "adas_common/ProfileTime.h"

namespace adas_common
{

class Profiler
{
public:
    Profiler(ros::NodeHandle &node, const std::string &name);

    void tick();
    void tock(const uint8_t id);
private:
    ros::Time prev;
    bool measuring;
    bool enabled;
    ros::Publisher pub;
};

}

#endif
