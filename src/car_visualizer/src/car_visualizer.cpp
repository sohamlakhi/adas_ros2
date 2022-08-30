//C++ STL headers
#include <algorithm>
#include <stdexcept>

//ROS2 headers
#include "rclcpp/rclcpp.hpp"

//message headers
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
//TODO: missing std_msgs time header

//your headers
#include "car_visualizer/car_bounding_box.hpp"

//macros
#define NODE_NAME "car_visualiser"

using namespace std;
using std::placeholders::_1;

namespace ros_car_visualizer {

    // namespace {
    //     std::vector<double> getBoundingBox(ros::NodeHandle &node)
    //     {
    //         return adas_common::vectorParam<double>(
    //             node,
    //             "bounding_box",
    //             4);
    //     }

    //     std::pair<double, double> getBoundingBoxOffset(ros::NodeHandle &node)
    //     {
    //         std::vector<double> boundingBox = getBoundingBox(node);

    //         return std::make_pair(boundingBox[0], boundingBox[1]);
    //     }

    //     std::pair<double, double> getBoundingBoxSize(ros::NodeHandle &node)
    //     {
    //         std::vector<double> boundingBox = getBoundingBox(node);

    //         if (boundingBox[2] < 0 || boundingBox[3] < 0)
    //         { throw std::runtime_error{"Bounding box size cannot be negative."}; }

    //         return std::make_pair(boundingBox[2], boundingBox[3]);
    //     }
    // }
    
    class CarVisualizer : public rclcpp::Node {

        public:
            CarVisualizer() : Node(NODE_NAME) {
                //initialise pub and subs here
                poseSub = this->create_subscription<geometry_msgs::msg::PoseStamped>("tag_pose", 1, bind(&ros_car_visualizer::CarVisualizer::poseCallback, this, _1));
                //resetSub = this->create_subscription<?>("reset", 1, bind(&ros_car_visualizer::CarVisualizer::resetCallback, this, _1));

                visualizePub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization_marker_array", 1);

                //TODO: need to initialise private members through parameter management. Initialised at 0 for now
                carId = 0;

                textHeight = 0.0;
            }

        private:

            //private objects and variables
            unsigned int carId;
            //TODO: add this 
            //car_bounding_box::BoundingBox box;
            double textHeight;

            //subscribers
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr poseSub;
            //rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr resetSub; -> TODO: need to find time msg

            //publishers
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualizePub; 


            visualization_msgs::msg::MarkerArray populateMsg() {
                auto carPoints = visualization_msgs::msg::MarkerArray();
                
                carPoints.markers = std::vector<visualization_msgs::msg::Marker>(2);

                carPoints.markers[0].header.frame_id = "map";
                carPoints.markers[0].header.stamp = this->get_clock()->now();
                carPoints.markers[0].ns = "car";
                carPoints.markers[0].id = 2 * carId;
                carPoints.markers[0].type = visualization_msgs::msg::Marker::LINE_STRIP;
                carPoints.markers[0].action = visualization_msgs::msg::Marker::ADD;

                carPoints.markers[1] = carPoints.markers[0];
                carPoints.markers[1].id = 2 * carId + 1;
                carPoints.markers[1].type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;

                return carPoints;

            }

            void poseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
                auto carPoints = visualization_msgs::msg::MarkerArray();
                carPoints = populateMsg();

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

                // const std::vector<geometry_msgs::Point> &bounds = box.get();

                // carPoints.markers[0].points = bounds;

                visualizePub->publish(carPoints);
            }

            // void resetCallback() {

            //     auto carPoints = visualisation_msgs::msg::MarkerArray();
            //     carPoints = populateMsg();

            //     carPoints.markers[0].action = visualization_msgs::Marker::DELETE;
            //     carPoints.markers[1].action = visualization_msgs::Marker::DELETE;

            //     visualizePub->publish(carPoints);

            // }

    }; 
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node_ptr = make_shared<ros_car_visualizer::CarVisualizer>(); // initialise node pointer
    rclcpp::spin(node_ptr);
    rclcpp::shutdown();
    return 0;
}

/**
 * TODO: tomorrow: figure out time message. Port params code (figure out what is happening)
 *       look at how to retrieve parameters,etc. (having multiple agents and multiple instances of the node)
 *       finish this package accordingly
 *      
 *       need to do bounding box class and this node (do cmakelists and package.xml)
 * 
 * come back and evaluate the the use of parameters. see if you can skip it for now
 * 
 * 
 * TODO: need to figure out parameters and stuff
 *       need to figure out reset callback type and initialisation
 */