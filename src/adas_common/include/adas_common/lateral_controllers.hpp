#ifndef ADAS_PURE_PURSUIT_CONTROLLER_HPP_
#define ADAS_PURE_PURSUIT_CONTROLLER_HPP_

//#include <tf/transform_datatypes.h>
#include "tf2/utils.h"

/**
 * @brief Library to implement a stanley and pure pursuit controller. See: https://dingyan89.medium.com/three-methods-of-vehicle-lateral-control-pure-pursuit-stanley-and-mpc-db8cc1d32081
 * TODO: rewrite using tf2 datatypes
 * NOTE: could use Eigen data structures as well (Refer to f1tenth pure pursuit cmakelists.txt)
 * See: http://wiki.ros.org/tf/data%20types http://wiki.ros.org/tf2/Tutorials/Migration/DataConversions
 * look at bullet type transformations
 * 
 * TODO: convert all tf to tf2. Look for tf::Pose and see how you can change that
 */

namespace adas_common {
    /**
     * @brief overloaded function to find perpendicular intersection between point and line
     * 
     * @param point point of interest
     * @param linePoint point defining line
     * @param lineAngle angle defining line (relative to x-axis in right hand co-ordinate system)
     * @return tf::Vector3 intersection point
     */
    tf2::Vector3 findIntersection(const tf2::Vector3 &point, const tf2::Vector3 &linePoint, const double lineAngle);

    /**
     * @brief overloaded function to find perpendicular intersection between point and line
     * 
     * @param point point of interest
     * @param linePoint point defining line
     * @param lineSlope direction vector defining line 
     * @return tf::Vector3 intersection point
     */
    tf2::Vector3 findIntersection(const tf2::Vector3 &point, const tf2::Vector3 &linePoint, const tf2::Vector3 &lineSlope);

/**
 * @brief parent class defining lateral controllers
 * 
 */
class LateralController {
    public:
        LateralController() : goalReceived{false} {}

        void setGoal(const tf2::Transform &g) {
            goalReceived = true;
            goal = g;
        }
        const tf2::Transform &getGoal() const { 
            return goal; 
        }

    protected:
        bool goalReceived;
        tf2::Transform goal;
};

class PurePursuitController : public LateralController {
    public:
        PurePursuitController(const double lookahead, const double axleDistance);

        void setLookahead(const double l);

        double commandStep(const tf2::Transform &observedPose);

    private:
        void verifyLookahead() const;

        double lookahead;
        double axleDistance;
};

class StanleyController : public LateralController {
    public:
        StanleyController(const double kPHeading, const double kDHeading, const double kCrosstrack, const double velDamping, const double wheelbase);

        double commandStep(const tf2::Transform &observedPose, const tf2::Vector3 &linearVel, const double angularVel, const double dt);
        void reset();

        void setKPHeading(const double kP);
        void setKDHeading(const double kD);
        void setKCrosstrack(const double kCross);
        void setVelDamping(const double vDamping);
    private:

        void verifyControlParams() const;

        double kPHeading;
        double kDHeading;
        double kCrosstrack;
        double velDamping;
        double wheelbase;

        double prevHeadingError;
};

}

#endif

/**
 * NOTES: tf::Pose is a typedef of tf::Transform. Can define typedef OR change it everywhere and make sure the corresponding functions exist
 *        getYaw() is a namespace function in tf2 and tf1
 * 
 * CHANGES: changed tf to tf2 for Vector3
 *          changed tf::Pose to tf2::Transform (will have to do that for the other codes that our calling these functions)
 * 
 */