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
/**
 * std_msgs::msg::uint8 has been deprecated as of foxy. 
 * Hence, built a custom version in adas_interfaces
 */
//#include "std_msgs/msg/uint8.hpp"
#include "adas_interfaces/msg/u_int8.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "builtin_interfaces/msg/time.hpp"

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
using std::placeholders::_2;
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
                resetSub = this->create_subscription<builtin_interfaces::msg::Time>("reset", 1, bind(&CarController::resetCallback, this, _1));

                //initialize publishers
                commandPub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("command", 10);

                //initialize message_filter object
                poseSub = message_filters::Subscriber(this, "pose", 1);
                twistSub = message_filters::Subscriber(this, "twist", 1);
                sync = message_filters::TimeSynchronizer(poseSub, twistSub, 2);
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

                headController = StanleyController(doubleParam(this, "controller/heading/k_p_heading"), doubleParam(this, "controller/heading/k_d_heading"), doubleParam(this, "controller/heading/k_crosstrack"), doubleParam(this, "controller/heading/vel_damping"), doubleParam(this, "controller/heading/axle_distance"), 0.0);

                velKFeedforward = adas_common::doubleParam(this, "controller/vel/k_feedforward", 0.0);
                posIThreshold = adas_common::doubleParam(this, "controller/pos/i_threshold", 0.0);
                discontinuityThreshold = adas_common::doubleParam(this, "controller/pos/discontinuity_threshold", 0.0);
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

                rclcpp::Duration diff(0,0); //TODO: might have to build with std::chrono::nanoseconds constructor (look at api reference)

                /**
                 * @brief if time has not progressed, exit synccallback
                 * 
                 */
                if (!timeDiff.update(diff, this)) {
                    return;
                }
                
                //tick time forward if profiler has been created (which it has below)
                profiler.tick(this);

                tf2::Transform observedPose;
                /**
                 * @brief the tf2_geometry_msgs package overloads the doTransform templated function and provides toMsg and fromMsg functions
                 * that is suficient to call the tf2::convert method (all objects passed by reference)
                 * See:
                 * http://wiki.ros.org/tf2/Tutorials/Migration/DataConversions
                 * https://wiki.ros.org/tf2_ros
                 * http://wiki.ros.org/tf2_geometry_msgs
                 * http://docs.ros.org/en/noetic/api/tf2_geometry_msgs/html/c++/
                 * http://docs.ros.org/en/noetic/api/tf2/html/namespacetf2.html#af1255c37833c838543b512febce5e9b1
                 * http://wiki.ros.org/tf2/Tutorials/Create%20Data%20Conversion%20Package%20%28C%2B%2B%29
                 * 
                 */
                tf2::convert (pose->pose, observedPose); //(data_in, data_out)

                /**
                 * @brief velocity at which goal should be approached
                 * 
                 */
                const double velCommand = posController.commandStep(0.0, pose->pose.position.x, diff.seconds());

                const double desiredVel = velCommand + treadmillVel;

                velController.setGoal(desiredVel);

                 // Explicit decision: No reset of D term calculation. It's not user-controlled and is assumed to be "reasonably continuous."
                /*
                * Now calculate the throttle required for the specified velocity.
                * NOTE: Accumulate I term iff we're close to the goal.
                */

                const double throtCommand = velController.commandStep(treadmillVel*velKFeedforward, twist->twist.linear.x+treadmillVel, diff.seconds(), std::fabs(pose->pose.position.x - posController.getGoal())<posIThreshold);
                
                tf2::Vector3 linearVel(treadmillVel+twist->twist.linear.x, twist->twist.linear.y, 0.0);

                double headCommand = headController.commandStep(observedPose, linearVel, twist->twist.angular.z, diff.seconds());

                auto command = ackermann_msgs::msg::AckermannDriveStamped();
                command.drive.steering_angle = headCommand;
                command.drive.speed = throtCommand;

                commandPub -> publish(command);
                profiler.tock(this, 0);
            }

            void goalPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
                tf2::Transform goal;
                tf2::convert (msg->pose, goal);
                headController.setGoal(goal);

                double prevGoalX = posController.getGoal();
                double newGoalX = goal.getOrigin().getX(); //chaining function calls. first function returns tf2::Vector

                if (goalReceived && std::fabs(prevGoalX - newGoalX) > discontinuityThreshold){
                    RCLCPP_WARN(this->get_logger(), "Large path discontinuity detected. Resetting Controller.");
                    posController.reset();
                    velController.reset();
                }

                posController.setGoal(newGoalX);

                goalReceived = true;
            }

            void treadmillVelCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
                treadmillVel = msg->twist.linear.x;
            }

            void resetCallback(const builtin_interfaces::msg::Time::ConstSharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "Resetting controller for car %u", integerParam<unsigned int>(this, "car_id"));
                timeDiff.reset();
                posController.reset();
                velController.reset();
                headController.reset();
            }

            //subsribers
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goalSub;
            rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr treadmillVelSub;
            rclcpp::Subscription<builtin_interfaces::msg::Time>::SharedPtr resetSub;

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

        class CarControllerWithUrgency : public CarController {
            public:
                //call default base class constructor. Hence, node constructed as well
                CarControllerWithUrgency() {
                    urgencySub = this->create_subscription<adas_interfaces::msg::UInt8>("urgency", 1, std::bind(&CarControllerWithUrgency::urgencyCallback, this, _1));
                    /**
                     * TODO: add all private member initialisations
                     * 
                     */

                    velParams = make_pair(PIDParams{doubleParam(this, "controller/vel/p"), doubleParam(this, "controller/vel/i"), doubleParam(this, "controller/vel/d")},PIDParams{doubleParam(this, "controller/vel/p_urgent"), doubleParam(this, "controller/vel/i_urgent"), doubleParam(this, "controller/vel/d_urgent")};
                    velSatParams = make_pair(SaturationParams{doubleParam(this, "controller/vel/min"), doubleParam(this, "controller/vel/max")}, SaturationParams{doubleParam(this, "controller/vel/min_urgent"), doubleParam(this, "controller/vel/max_urgent")});
                    posParams = make_pair(PIDParams{doubleParam(this, "controller/pos/p"), doubleParam(this, "controller/pos/i"), doubleParam(this, "controller/pos/d")}, PIDParams{doubleParam(this, "controller/pos/p_urgent"), doubleParam(this, "controller/pos/i_urgent"), doubleParam(this, "controller/pos/d_urgent")});
                    kCrosstrack = make_pair(doubleParam(this, "controller/heading/k_crosstrack"), doubleParam(this, "controller/heading/k_crosstrack_urgent"));

                }

            private:
                rclcpp::Subscription<adas_interfaces::msg::UInt8>::SharedPtr urgencySub;

                void urgencyCallback(adas_interfaces::msg::UInt8::ConstPtr msg) {
                    const double urgencyT = static_cast<double>(msg->data)/numeric_limits<uint8_t>::max();

                    setVelPIDParams({lerp(velParams.first.p, velParams.second.p, urgencyT), lerp(velParams.first.i, velParams.second.i, urgencyT), lerp(velParams.first.d, velParams.second.d, urgencyT)});
                    setVelSatParams({lerp(velSatParams.first.min, velSatParams.second.min, urgencyT), lerp(velSatParams.first.max, velSatParams.second.max, urgencyT)});
                    setPosPIDParams({lerp(posParams.first.p, posParams.second.p, urgencyT), lerp(posParams.first.i, posParams.second.i, urgencyT), lerp(posParams.first.d, posParams.second.d, urgencyT)});
                    setKCrosstrack(lerp(kCrosstrack.first, kCrosstrack.second, urgencyT));
    
                }

                // Nominal-Urgent param pairs
                std::pair<PIDParams, PIDParams> velParams;
                std::pair<SaturationParams, SaturationParams> velSatParams;
                std::pair<PIDParams, PIDParams> posParams;
                std::pair<double, double> kCrosstrack;
        };

}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    /**
     * @brief if you are using tunable, then can do the following:
     * 
     
    auto carController = make_shared<ros_car_controller::CarController>;

    using CarControllerPtr = shared_ptr<ros_car_controller::CarController>;

    CarControllerPtr carController = [=] () {
        if (boolParamWithDefault(carController.get(), "/tune", false)) {
            RCLCPP_WARN(carController.get()->get_logger(), "Launching with dynamic parameters for tuning.\n"
                "Parameters will take default, zero values.\n"
                "Urgency is not supported.\n");

            return CarControllerPtr{new ros_car_controller::CarControllerTunable()};
        }

        return {new ros_car_controller::CarControllerWithUrgency()};

    }
    
    */

    auto node_ptr = make_shared<ros_car_controller::CarControllerWithUrgency>();
    //auto node_ptr = make_shared<bytestest>(); // initialise node pointer
    rclcpp::spin(node_ptr);
    rclcpp::shutdown();
    return 0;
    
}

/**
 * @brief finish porting functions and urgency class -> DONE
 * 
 * write main function call stuff
 * 
 * compile it
 * 
 * commit and save. Figure out car controller and power (look at appropriate courses for that)
 * 
 */