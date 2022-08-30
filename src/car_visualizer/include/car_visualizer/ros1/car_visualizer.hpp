#ifndef CAR_VISUALIZER_HPP
#define CAR_VISUALIZER_HPP

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Time.h>
#include <visualization_msgs/MarkerArray.h>

#include "car_visualizer/car_bounding_box.hpp"

namespace ros_car_visualizer
{

class CarVisualizer
{
public:
    CarVisualizer(ros::NodeHandle &node, ros::NodeHandle &nodePriv);

private:
    void poseCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    void resetCallback(const std_msgs::TimeConstPtr &msg);

    visualization_msgs::MarkerArray populateMsg();

    ros::NodeHandle node;
    ros::NodeHandle nodePriv;

    unsigned int carId;
    car_bounding_box::BoundingBox box;
    double textHeight;

    ros::Subscriber poseSub;
    ros::Subscriber resetSub;
    ros::Publisher visualizePub;
};

}

#endif
