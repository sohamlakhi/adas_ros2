#include "adas_common/profiler.hpp"
#include "adas_common/params.hpp"

//this might create issues
//using namespace adas_common 
namespace adas_common
{
Profiler::Profiler(rclcpp::Node *node, const std::string &name) : measuring{false}, enabled{adas_common::boolParamWithDefault(node, "/profiler/" + name + "/enabled", false)} {
    
    if (enabled) { 
        pub = node->create_publisher<adas_interfaces::msg::ProfileTime>("/profiler/" + name, 1);
    }
}

void Profiler::tick(rclcpp::Node *node)
{
    if (!enabled) { return; }
    prev = node->get_clock()->now();
    measuring = true;
}

void Profiler::tock(rclcpp::Node *node, const uint8_t id)
{
    if (!enabled) { return; }

    rclcpp::Time curr = node->get_clock()->now();
    auto msg = adas_interfaces::msg::ProfileTime();

    msg.stamp = curr;
    msg.id = id;
    /**
     * NOTE: might have to use .seconds()
     * 
     */
    msg.duration = (curr - prev).nanoseconds(); 

    measuring = false;

    pub->publish(msg);
}

}
/**
 * need to add dependencies in cmakelists to adas_interfaces
 * 
 */
