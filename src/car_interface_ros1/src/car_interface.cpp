#include <ros/ros.h>
#include <boost/algorithm/clamp.hpp>
#include "car_interface/car_interface.hpp"

//C++ headers
#include <algorithm>
#include <cmath>

#include <adas_common/params.hpp>

static const double maxAngle = M_PI / 6;

namespace ros_car_interface
{

CarInterface::CarInterface(ros::NodeHandle &node, ros::NodeHandle &nodePriv)
    : node(node),
      nodePriv(nodePriv),
      commandSub(node.subscribe("command", 1, &CarInterface::commandCallback, this)),
      resetSub(node.subscribe(
          "reset",
          1,
          &CarInterface::resetCallback,
          this)),
      safetySub{node.subscribe("/is_safe", 1, &CarInterface::safetyCallback, this)},
      commandValuePub(node.advertise<car_interface::CarCommandValue>("command_value", 1)),
      safe{false},
      steerNeutral(adas_common::integerParam<uint16_t>(node, "interface/steer_neutral", 0)),
      // Maximum left/right value offset from the neutral value
      steerMaxOffset(adas_common::integerParam<uint16_t>(node, "interface/steer_offset", 0)),
      // Flip from positive -> above neutral to positive -> below neutral
      steerReverse(adas_common::boolParam(node, "interface/steer_reverse")),
      throttleNeutral(adas_common::integerParam<uint16_t>(node, "interface/throttle_neutral", 0)),
      throttleMax(adas_common::integerParam<uint16_t>(node, "interface/throttle_max", 0)),
      steerTrim{adas_common::integerParamWithDefault<int16_t>(
          node, "interface/steer_trim", 0, -1000, 1000)},
      throttleTrim{adas_common::integerParamWithDefault<int16_t>(
          node, "interface/throttle_trim", 0, -1000, 1000)},
      configMutex{},
      server{configMutex},
      f{boost::bind(&CarInterface::trimCallback, this, _1, _2)},
      firstConfig{true}
{
    server.setCallback(f);

    car_interface::TrimConfig cfg;
    cfg.steer_trim = steerTrim;
    cfg.throttle_trim = throttleTrim;
    server.updateConfig(cfg);

    ROS_ASSERT(steerMaxOffset <= steerNeutral);

    publishStop();
}

void CarInterface::commandCallback(const car_interface::CarCommandConstPtr &msg)
{
    if (!safe)
    {
        publishStop();
        return;
    }

    car_interface::CarCommandValue commandValue;

    // msg->steer is in radians,msg->throttle is in percent
    double steerOffset = std::max(std::min(msg->steer, maxAngle), -maxAngle) / maxAngle * steerMaxOffset;

    const int16_t steerMultiplier = [this] () {
        if (steerReverse) { return -1; }
        return 1; } ();

    // Ensure within range with trim
    commandValue.steer = static_cast<uint16_t>(boost::algorithm::clamp(
        steerNeutral + steerMultiplier * (boost::math::iround(steerOffset) + steerTrim),
        steerNeutral - steerMaxOffset,
        steerNeutral + steerMaxOffset));

    double throttleOffset = std::max(std::min(msg->throttle, 100.0), 0.0) / 100.0 *
                            (throttleMax - throttleNeutral);

    // Ensure within range with trim
    commandValue.throttle = static_cast<uint16_t>(boost::algorithm::clamp(
        throttleNeutral + throttleTrim + throttleOffset,
        throttleNeutral,
        throttleMax));

    commandValuePub.publish(commandValue);
}

void CarInterface::resetCallback(const std_msgs::TimeConstPtr &msg)
{
    ROS_INFO(
        "Stopping car %u...",
        adas_common::integerParam<unsigned int>(node, "car_id"));
    car_interface::CarCommandValue commandValue;

    publishStop();
}

void CarInterface::trimCallback(car_interface::TrimConfig &cfg, uint32_t)
{
    // First update is meaningless, as it is the defaults from dynamic_reconfigure.
    if (firstConfig)
    {
        firstConfig = false;
        return;
    }
    steerTrim = static_cast<int16_t>(cfg.steer_trim);
    throttleTrim = static_cast<int16_t>(cfg.throttle_trim);
}

void CarInterface::safetyCallback(const std_msgs::BoolConstPtr &msg)
{
    safe = msg->data;
    if (!safe)
    { publishStop(); }
}

void CarInterface::publishStop()
{
    car_interface::CarCommandValue commandValue;
    commandValue.steer = steerNeutral;
    commandValue.throttle = throttleNeutral;
    commandValuePub.publish(commandValue);
}

}
