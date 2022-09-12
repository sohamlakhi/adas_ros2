#ifndef CAR_INTERFACE_HPP
#define CAR_INTERFACE_HPP

#include <ros/ros.h>

#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Time.h>
#include <std_msgs/Bool.h>

//custom msg headers. Replace with ackermann drive stamped.
#include "car_interface/CarCommandValue.h"
#include "car_interface/CarCommand.h"

//dynamic_reconfigure headers
#include <dynamic_reconfigure/server.h>
#include <car_interface/TrimConfig.h>

namespace ros_car_interface
{

class CarInterface
{
public:
    CarInterface(ros::NodeHandle &node, ros::NodeHandle &nodePriv);

private:
    void commandCallback(const car_interface::CarCommandConstPtr &msg);
    void resetCallback(const std_msgs::TimeConstPtr &msg);
    void trimCallback(car_interface::TrimConfig &cfg, uint32_t);
    void safetyCallback(const std_msgs::BoolConstPtr &msg);

    void publishStop();

    ros::NodeHandle node;
    ros::NodeHandle nodePriv;

    ros::Subscriber commandSub;
    ros::Subscriber resetSub;
    ros::Subscriber safetySub;
    ros::Publisher commandValuePub;

    bool safe;

    uint16_t steerNeutral;
    uint16_t steerMaxOffset;
    bool steerReverse;
    uint16_t throttleNeutral;
    uint16_t throttleMax;
    int16_t steerTrim;
    int16_t throttleTrim;

    // We're giving manual updates to the dynamic parameter server - Need to provide separate mutex
    boost::recursive_mutex configMutex;
    dynamic_reconfigure::Server<car_interface::TrimConfig> server;
    decltype(server)::CallbackType f;
    bool firstConfig;
};

}

#endif
