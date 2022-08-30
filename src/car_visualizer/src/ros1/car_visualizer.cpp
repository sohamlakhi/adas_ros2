#include <ros/ros.h>
#include <adas_common/params.hpp>

#include <algorithm>
#include <stdexcept>

#include "car_visualizer/car_visualizer.hpp"

namespace ros_car_visualizer
{

namespace
{
std::vector<double> getBoundingBox(ros::NodeHandle &node)
{
    return adas_common::vectorParam<double>(
        node,
        "bounding_box",
        4);
}

std::pair<double, double> getBoundingBoxOffset(ros::NodeHandle &node)
{
    std::vector<double> boundingBox = getBoundingBox(node);

    return std::make_pair(boundingBox[0], boundingBox[1]);
}

std::pair<double, double> getBoundingBoxSize(ros::NodeHandle &node)
{
    std::vector<double> boundingBox = getBoundingBox(node);

    if (boundingBox[2] < 0 || boundingBox[3] < 0)
    { throw std::runtime_error{"Bounding box size cannot be negative."}; }

    return std::make_pair(boundingBox[2], boundingBox[3]);
}
}

CarVisualizer::CarVisualizer(ros::NodeHandle &node, ros::NodeHandle &nodePriv)
    : node(node),
      nodePriv(nodePriv),
      carId(adas_common::integerParam<unsigned int>(node, "car_id")),
      box(getBoundingBoxOffset(node), getBoundingBoxSize(node)),
      textHeight(getBoundingBoxSize(node).second * 0.75),
      poseSub(node.subscribe("tag_pose", 1, &CarVisualizer::poseCallback, this)),
      resetSub(node.subscribe(
          "reset",
          1,
          &CarVisualizer::resetCallback,
          this)),
      visualizePub(node.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 1))
{
}

void CarVisualizer::poseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    visualization_msgs::MarkerArray carPoints = populateMsg();

    carPoints.markers[0].pose = msg->pose;
    carPoints.markers[0].scale.x = 0.01;
    carPoints.markers[0].color.a = 1.0;
    carPoints.markers[0].color.r = 0.0;
    carPoints.markers[0].color.g = 0.0;
    carPoints.markers[0].color.b = 1.0;

    carPoints.markers[1].pose = msg->pose;
    carPoints.markers[1].scale.x = 0.0;
    carPoints.markers[1].scale.z = textHeight;
    carPoints.markers[1].text = std::to_string(carId);
    carPoints.markers[1].color.a = 1.0;
    carPoints.markers[1].color.r = 1.0;
    carPoints.markers[1].color.g = 1.0;
    carPoints.markers[1].color.b = 1.0;

    const std::vector<geometry_msgs::Point> &bounds = box.get();

    carPoints.markers[0].points = bounds;

    visualizePub.publish(carPoints);
}

void CarVisualizer::resetCallback(const std_msgs::TimeConstPtr &msg)
{
    visualization_msgs::MarkerArray carPoints = populateMsg();

    carPoints.markers[0].action = visualization_msgs::Marker::DELETE;
    carPoints.markers[1].action = visualization_msgs::Marker::DELETE;

    visualizePub.publish(carPoints);
}

visualization_msgs::MarkerArray CarVisualizer::populateMsg()
{
    visualization_msgs::MarkerArray carPoints;
    carPoints.markers = std::vector<visualization_msgs::Marker>(2);

    carPoints.markers[0].header.frame_id = "map";
    carPoints.markers[0].header.stamp = ros::Time();
    carPoints.markers[0].ns = "car";
    carPoints.markers[0].id = 2 * carId;
    carPoints.markers[0].type = visualization_msgs::Marker::LINE_STRIP;
    carPoints.markers[0].action = visualization_msgs::Marker::ADD;

    carPoints.markers[1] = carPoints.markers[0];
    carPoints.markers[1].id = 2 * carId + 1;
    carPoints.markers[1].type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    return carPoints;
}

}
