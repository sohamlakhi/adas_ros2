//C++ library includes
#include <memory> //for smart pointers
#include <chrono> //for time 

//ROS related headers
#include "rclcpp/rclcpp.hpp"
//message header
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

#include <std_msgs/msg/UInt16.h>
#include <std_msgs/msg/float64.h>
//#include <std_msgs/Time.h>
#include <std_msgs/msg/bool.h>

//headers to code written by you
#include "adas_common/bytes_helper.hpp"
#include "adas_common/checksum.hpp"

//other macros
#define NODE_NAME "car_interface" 
#define _USE_MATH_DEFINES

//using namespaces 
//used for bind (uncomment below)
using std::placeholders::_1;
using namespace std;
using namespace std::chrono_literals;


class CarInterface : public rclcpp::Node {

public:
    CarInterface() : Node(NODE_NAME) {
        
        RCLCPP_INFO (this->get_logger(), "%s node has been launched", NODE_NAME);

        
    }

private:

    uint16_t data;
    adas_common::Span<uint8_t> myname;



};


int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node_ptr = make_shared<CarInterface>(); // initialise node pointer
    rclcpp::spin(node_ptr);
    rclcpp::shutdown();
    return 0;
    
}
