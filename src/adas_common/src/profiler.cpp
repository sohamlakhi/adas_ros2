#include "adas_common/profiler.hpp"
#include "adas_common/params.hpp"

namespace adas_common
{
Profiler::Profiler(
    ros::NodeHandle &node,
    const std::string &name) :
    measuring{false},
    enabled{adas_common::boolParamWithDefault(node, "/profiler/" + name + "/enabled", false)}
{
    if (enabled)
    { pub = node.advertise<adas_common::ProfileTime>("/profiler/" + name, 1); }
}

void Profiler::tick()
{
    if (!enabled) { return; }
    prev = ros::Time::now();
    measuring = true;
}

void Profiler::tock(const uint8_t id)
{
    if (!enabled) { return; }

    ros::Time curr = ros::Time::now();
    adas_common::ProfileTime msg;

    msg.stamp = curr;
    msg.id = id;
    msg.duration = (curr - prev).toSec();

    measuring = false;

    pub.publish(msg);
}

}
