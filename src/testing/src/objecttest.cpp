#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <string>
#include <functional>

#define NODE_NAME "objecttest" 

class paramTest {
    public:
        static void get_parameter_from_class (rclcpp::Node *node) {
            std::string parameter_string_;

            node->get_parameter("my_parameter", parameter_string_);
            RCLCPP_INFO(node->get_logger(), "from class: %s", parameter_string_.c_str());
        }
};

namespace {
    // void get_param_from_local_ns (rclcpp::Node *node) {
    //     std::string parameter_string_;

    //     node->get_parameter("my_parameter", parameter_string_);
    //     RCLCPP_INFO(node->get_logger(), "from ns (pointer): %s", parameter_string_.c_str());
    // }

    void get_param_from_local_ns (rclcpp::Node *node) {
        int parameter_string_;

        node->get_parameter("my_parameter2", parameter_string_);
        RCLCPP_INFO(node->get_logger(), "from ns (by referebce): %d", parameter_string_);
    }
}

class bigtest : public rclcpp::Node {

    public:
        bigtest() : Node(NODE_NAME) {
            
            RCLCPP_INFO (this->get_logger(), "%s node has been launched", NODE_NAME);
            this->declare_parameter<std::string>("my_parameter", "world");
            this->declare_parameter<int>("my_parameter2", 29);

            get_param_from_local_ns (this);
            //get_param_from_local_ns (this);
            paramTest::get_parameter_from_class (this);
        }

};

//TODO: multithread and spin here
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node_ptr = std::make_shared<bigtest>(); // initialise node pointer
    rclcpp::spin(node_ptr);
    rclcpp::shutdown();
    return 0;
    
}

/**
 * NOTE: this is a POINTER to the current object. Hence, you need to pass it as a pointer 
 * you can also explicitly cast -> static_cast, dynamic_cast
 * https://www.geeksforgeeks.org/passing-reference-to-a-pointer-in-c/
 * https://www.geeksforgeeks.org/this-pointer-in-c/
 * https://www.internalpointers.com/post/move-smart-pointers-and-out-functions-modern-c
 * 
 * add this to the params notes as well.
 */