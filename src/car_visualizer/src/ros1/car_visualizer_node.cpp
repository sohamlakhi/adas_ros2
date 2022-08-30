#include <ros/ros.h>

#include "car_visualizer/car_visualizer.hpp"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "car_visualizer");
    ros::NodeHandle node;
    ros::NodeHandle nodePriv("~");

    ros_car_visualizer::CarVisualizer vis(node, nodePriv);

    ros::spin();

    return 0;
}
