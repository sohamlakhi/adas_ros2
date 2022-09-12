#include <ros/ros.h>

#include "car_interface/car_interface.hpp"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "car_interface");
    ros::NodeHandle node;
    ros::NodeHandle nodePriv("~");

    ros_car_interface::CarInterface interface(node, nodePriv);

    ros::spin();

    return 0;
}
