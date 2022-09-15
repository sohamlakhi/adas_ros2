//C++ library includes
#include <memory> //for smart pointers
#include <chrono> //for time 
#include <utility>
#include <cmath>

//ROS related headers
#include "rclcpp/rclcpp.hpp"

//message header
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/uint8.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

//message filters
/**
 * See: https://index.ros.org/p/message_filters/#foxy-overview https://github.com/intel/ros2_message_filters https://github.com/ros2/message_filters/tree/foxy/
http://docs.ros.org/en/humble/p/message_filters/ http://docs.ros.org/en/humble/p/message_filters/
 * 
 */
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

//tf2 headers
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

//headers to code written by you
#include <adas_common/pid_controller.hpp>
#include <adas_common/lateral_controllers.hpp>
#include <adas_common/params.hpp>
#include <adas_common/consts.hpp>
#include <adas_common/lerp.hpp>
#include <adas_common/ros_time_diff.hpp>
#include <adas_common/profiler.hpp>

//other macros
#define NODE_NAME "car_controller" 
#define _USE_MATH_DEFINES

//using namespaces 
//used for bind (uncomment below)
using std::placeholders::_1;
using namespace std;
using namespace std::chrono_literals;
using namespace adas_common;


namespace ros_car_controller {

    class CarController : public rclcpp::Node {
        public:
            struct PIDParams {
                    double p;
                    double i;
                    double d;
                    PIDParams(const double p, const double i, const double d) : p{p}, i{i}, d{d} {

                    }
                    
            };

            struct SaturationParams {
                    double min;
                    double max;
                    SaturationParams(const double min, const double max) : min{min}, max{max} {

                    }
                   
            };

            CarController() : Node(NODE_NAME) {
                //initialize stuff here

                //declare subscribers
                goalSub = this->create_subscription<geometry_msgs::msg::PoseStamped>("goal", 10, bind(&CarController::goalPoseCallback, this, _1));
                treadmillVelSub = this->create_subscription<geometry_msgs::msg::TwistStamped>("/treadmill/velocity", 10, bind(&CarController::treadmillVelCallback, this, _1));

                //initialize publishers
                commandPub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("command", 10);

                //initialize message_filter object
                poseSub = (this, "pose", 1);
                twistSub = (this, "twist", 1);
                sync = (poseSub, twistSub, 2);
                sync.registerCallback(std::bind(&CarController::syncCallback, this, _1, _2));

                //PIDcontroller objects
                // velController.p = doubleParam(this, "controller/vel/p");
                // velController.i = doubleParam(this, "controller/vel/i");
                // velController.d = doubleParam(this, "controller/vel/d");
                // velController.saturationMin = Consts::throttleCommandMin;
                // velController.saturationMax = Consts::throttleCommandMax;

                //constructing object with constructor (refer PIDController.hpp)
                velController = PIDController(doubleParam(this, "controller/vel/p"), doubleParam(this, "controller/vel/i"), doubleParam(this, "controller/vel/d"), Consts::throttleCommandMin, Consts::throttleCommandMax);

                posController = PIDController(doubleParam(this, "controller/pos/p"), doubleParam(this, "controller/pos/i"), doubleParam(this, "controller/pos/d"), doubleParam(this, "controller/vel/min"), doubleParam(this, "controller/vel/max"));

                headController = StanleyController(doubleParam(this, "controller/heading/k_p_heading"), doubleParam(this, "controller/heading/k_d_heading"), doubleParam(this, "controller/heading/k_crosstrack"), doubleParam(this, "controller/heading/vel_damping"), doubleParam(this, "controller/heading/axle_distance", 0.0));

                velKFeedforward = adas_common::doubleParam(node, "controller/vel/k_feedforward", 0.0);
                posIThreshold = adas_common::doubleParam(node, "controller/pos/i_threshold", 0.0);
                discontinuityThreshold = adas_common::doubleParam(node, "controller/pos/discontinuity_threshold", 0.0);
                goalReceived = false;

                profiler = Profiler(this, "car_controller_" + std::to_string(integerParam<unsigned int>(this, "car_id")));
            }

        protected:
            void setPosPIDParams(const PIDParams &posParams) {
                posController.p = posParams.p;
                posController.i = posParams.i;
                posController.d = posParams.d;
            }

            void setVelSatParams(const SaturationParams &velSatParams) {
                // On position control as it defines the max velocity command
                posController.setSaturation(velSatParams.min, velSatParams.max);
            }

            void setVelPIDParams(const PIDParams &velParams) {
                velController.p = velParams.p;
                velController.i = velParams.i;
                velController.d = velParams.d;
            }

            void setVelKFeedforward(const double kFeedforward) {
                velKFeedforward = kFeedforward;
            }

            void setKPHeading(const double kPHeading) {
                headController.setKPHeading(kPHeading);
            }

            void setKDHeading(const double kDHeading) {
                headController.setKDHeading(kDHeading);
            }

            void setKCrosstrack(const double kCrosstrack) {
                headController.setKCrosstrack(kCrosstrack);
            }

            void setVelDamping(const double velDamping) {
                headController.setVelDamping(velDamping);
            }

        private:

            void syncCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose, const geometry_msgs::msg::TwistStamped::ConstSharedPtr twist) {
                if (!goalReceived) {
                    return;
                }

                rclcpp::Duration diff;

                /**
                 * @brief if time has not progressed, exit synccallback
                 * 
                 */
                if (!timediff.update(diff, this)) {
                    return;
                }
                
                //tick time forward if profiler has been created (which it has below)
                profiler.tick();

                tf2::Transform observedPose;
                tf2::convert (pose, observedPose); //(data_in, data_out)

                const double velCommand = posController.commandStep(0.0, pose->pose.position.x, diff.seconds());

                
                
            }

            void goalPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);

            void treadmillVelCallback(const geometry_msgs::TwistStampedConstPtr &msg);

            //void resetCallback(const std_msgs::TimeConstPtr &msg);

            //subsribers
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goalSub;
            rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr treadmillVelSub;
            //need to add reset callback

            //publishers 
            /**
             * NOTE: replaced car_interface::CarCommand custom message with ackermann drive stamped
             * 
             */
            rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr commandPub;  

            //message filter objects
            message_filters::Subscriber<geometry_msgs::msg::PoseStamped> poseSub;
            message_filters::Subscriber<geometry_msgs::msg::TwistStamped> twistSub;
            message_filters::TimeSynchronizer<geometry_msgs::msg::PoseStamped, geometry_msgs::msg::TwistStamped> sync;

            adas_common::PIDController velController;
            adas_common::PIDController posController;
            
            adas_common::StanleyController headController;

            double velKFeedforward;
            double treadmillVel;
            double posIThreshold;
            double discontinuityThreshold;

            bool goalReceived;

            adas_common::Profiler profiler;
            adas_common::RosTimeDiff timeDiff;

        };

}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node_ptr = make_shared<bytestest>(); // initialise node pointer
    rclcpp::spin(node_ptr);
    rclcpp::shutdown();
    return 0;
    
}