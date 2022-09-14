#ifndef PROFILER_HPP_
#define PROFILER_HPP_

//RCL header
#include "rclcpp/rclcpp.hpp"

//msg headers
#include "adas_interfaces/msg/profile_time.hpp"

namespace adas_common
{

class Profiler
{
public:
    Profiler(rclcpp::Node *node, const std::string &name);

    void tick(rclcpp::Node *node);
    void tock(rclcpp::Node *node, const uint8_t id);
private:
    rclcpp::Time prev;
    bool measuring;
    bool enabled;
    rclcpp::Publisher<adas_interfaces::msg::ProfileTime>::SharedPtr pub;  

};

}

#endif
