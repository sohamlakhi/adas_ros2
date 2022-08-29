//C++ library includes
#include <memory> //for smart pointers
#include <chrono> //for time 

//ROS related headers
#include "rclcpp/rclcpp.hpp"
//message header

//headers to code written by you
#include "adas_common/bytes_helper.hpp"
#include "adas_common/checksum.hpp"

//other macros
#define NODE_NAME "bytes_tester" 
#define _USE_MATH_DEFINES

//using namespaces 
//used for bind (uncomment below)
using std::placeholders::_1;
using namespace std;
using namespace std::chrono_literals;






class bytestest : public rclcpp::Node {


public:
    bytestest() : Node(NODE_NAME) {
        
        RCLCPP_INFO (this->get_logger(), "%s node has been launched", NODE_NAME);

        data = 255;
        
        std::vector<uint8_t> v = {0,0,0,0};
        myname = adas_common::Span(v);

        uint8_t x = bytes::upper8(data);
        RCLCPP_INFO (this->get_logger(), "%f", x);
    }

private:

    uint16_t data;
    adas_common::Span<uint8_t> myname;



};

//TODO: multithread and spin here
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node_ptr = make_shared<bytestest>(); // initialise node pointer
    rclcpp::spin(node_ptr);
    rclcpp::shutdown();
    return 0;
    
}
