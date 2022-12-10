#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <string>
#include <functional>

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/msg/pose.hpp"

#define NODE_NAME "tf2test" 

class bigtest : public rclcpp::Node {

    public:
        bigtest() : Node(NODE_NAME) {
            
            RCLCPP_INFO (this->get_logger(), "%s node has been launched", NODE_NAME);
            
            pose_geom.position.x = 2423.4;
            pose_geom.orientation.y = 5.0;
            tf2::convert (pose_geom, pose_tf2);

            tf2::Vector3 tong = pose_tf2.getOrigin();
            RCLCPP_INFO (this->get_logger(), "geom: <%f, %f>", pose_geom.position.x, pose_geom.orientation.y);
            RCLCPP_INFO (this->get_logger(), "tf2: <%f>", tong.getX());
           
        }

    private:

        geometry_msgs::msg::Pose pose_geom;
        tf2::Transform pose_tf2;
};

//TODO: multithread and spin here
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node_ptr = std::make_shared<bigtest>(); // initialise node pointer
    rclcpp::spin(node_ptr);
    rclcpp::shutdown();
    return 0;
    
}