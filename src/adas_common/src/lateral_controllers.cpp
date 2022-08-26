#include "adas_common/lateral_controllers.hpp"

#include <boost/algorithm/clamp.hpp>

#include <iostream>
#include <stdexcept>

namespace adas_common
{

tf2::Vector3 findIntersection(const tf2::Vector3 &point,const tf2::Vector3 &linePoint,const double lineAngle) {
    return findIntersection(point, linePoint, {1, std::tan(lineAngle), 0});
}

tf2::Vector3 findIntersection(const tf2::Vector3 &point, const tf2::Vector3 &linePoint, const tf2::Vector3 &lineSlope)
{
    tf2::Vector3 pointSlope{lineSlope.getY(), -lineSlope.getX(), 0};

    /*
     * Two parameteric equations:
     * L0 = lineSlope * t + linePoint
     * L1 = pointSlope * u + point
     * Solve for t.
     */
    double intersectT =
        (pointSlope.getY() * (point.getX() - linePoint.getX()) -
        pointSlope.getX() * (point.getY() - linePoint.getY())) /
        (lineSlope.getX() * pointSlope.getY() - pointSlope.getX() * lineSlope.getY());

    return lineSlope * intersectT + linePoint;
}

PurePursuitController::PurePursuitController(
    const double lookahead,
    const double axleDistance)
    : lookahead{lookahead},
      axleDistance{axleDistance}
{
    verifyLookahead();

    if (axleDistance <= 0)
    { throw std::invalid_argument{"Axle distance must be positive."}; }
}

void PurePursuitController::verifyLookahead() const
{
    if (lookahead <= 0)
    { throw std::invalid_argument{"Lookahead must be positive."}; }
}

void PurePursuitController::setLookahead(const double l)
{
    lookahead = l;
    verifyLookahead();
}

// Given a straight trajectory, and the current pose, calculate the steering value
double PurePursuitController::commandStep(
    const tf2::Transform &observedPose)
{
    if (!goalReceived) { return 0.0; }

    tf2::Vector3 goalSlope(1, std::tan(tf2::getYaw(goal.getRotation())), 0);
    tf2::Vector3 intersection = findIntersection(
        observedPose.getOrigin(),
        goal.getOrigin(),
        goalSlope);


    double intersectionDistance2 = intersection.distance2(observedPose.getOrigin());

    double lookahead2 = std::pow(lookahead, 2);

    tf2::Vector3 lookaheadPoint;

    // Direction unit vector of the goal trajectory.
    tf2::Vector3 trajectoryDir = goalSlope.normalized();
    if (intersectionDistance2 >= lookahead2)
    {
        /*
         * Distance to the intersection is larger than the lookahead distance.
         * Simply look ahead a fixed amount.
         */
        lookaheadPoint = intersection + lookahead* trajectoryDir;
    }
    else
    {
        /*
         * Lookahead distance (Forward from the intersection) decreases the further
         * you are away from the trajectory.
         */
        lookaheadPoint = intersection +
                         std::sqrt(lookahead2 - intersectionDistance2) *
                         trajectoryDir;
    }

    tf2::Vector3 poseToTarget = lookaheadPoint - observedPose.getOrigin();
    double goalYaw = tf2::Vector3(1, 0, 0).angle(poseToTarget);

    // tf::Vector3::angle() always returns positive - give the directional context.
    if (poseToTarget.getY() < 0)
    {
        goalYaw *= -1;
    }

    double poseYaw = tf2::getYaw(observedPose.getRotation());

    // Now use the pure pursuit equation to calculate the desired steering angle.
    double goalCurvature = 2.0 * std::sin(goalYaw - poseYaw) / lookahead;
    return std::atan(goalCurvature * axleDistance);
}

StanleyController::StanleyController(
        const double kPHeading,
        const double kDHeading,
        const double kCrosstrack,
        const double velDamping,
        const double wheelbase) :
    kPHeading{kPHeading},
    kDHeading{kDHeading},
    kCrosstrack{kCrosstrack},
    velDamping{velDamping},
    wheelbase{wheelbase},
    prevHeadingError{0.0}
{
    if (wheelbase <= 0)
    { throw std::invalid_argument{"Wheelbase must be larger than zero."}; }

    verifyControlParams();
}

void StanleyController::verifyControlParams() const
{
    if (kPHeading < 0)
    { throw std::invalid_argument{"P gain for heading error cannot be negative."}; }
    if (kDHeading < 0)
    { throw std::invalid_argument{"D gain for heading error cannot be negative."}; }
    if (kCrosstrack < 0)
    { throw std::invalid_argument{"Crosstrack gain cannot be negative."}; }
    if (velDamping < 0)
    { throw std::invalid_argument{"Velocity damping constant cannot be negative."}; }
}

void StanleyController::setKPHeading(const double kP)
{
    kPHeading = kP;
    verifyControlParams();
}

void StanleyController::setKDHeading(const double kD)
{
    kDHeading = kD;
    verifyControlParams();
}

void StanleyController::setKCrosstrack(const double kCross)
{
    kCrosstrack = kCross;
    verifyControlParams();
}

void StanleyController::setVelDamping(const double vDamping)
{
    velDamping = vDamping;
    verifyControlParams();
}

double StanleyController::commandStep(
        const tf2::Transform &observedPose,
        const tf2::Vector3 &linearVel,
        const double angularVel,
        const double dt)
{
    if (!goalReceived) { return 0.0; }

    double yaw = tf2::getYaw(observedPose.getRotation());

    const double headingError = tf2::getYaw(goal.getRotation()) - yaw;
    double headingCorrection =
        kPHeading * headingError +
        kDHeading * ((headingError - prevHeadingError) / dt);

    // Incoming pose and velocity are for the rear axles - convert to front.
    tf2::Transform frontPose{
        observedPose.getRotation(),
        {
            observedPose.getOrigin().getX() + wheelbase * std::cos(yaw),
            observedPose.getOrigin().getY() + wheelbase * std::sin(yaw),
            0}};
    tf2::Vector3 frontLinearVel{
        linearVel.getX() + wheelbase * angularVel * std::sin(yaw),
        linearVel.getY() + wheelbase * angularVel * std::cos(yaw),
        0};

    tf2::Vector3 intersection = findIntersection(
        frontPose.getOrigin(),
        goal.getOrigin(),
        tf2::getYaw(goal.getRotation()));

    double crosstrackError = intersection.distance(frontPose.getOrigin());

    // Goal line always points forward - so, if pose.y is bigger, pose is to the left.
    if (goal.getOrigin().getY() < frontPose.getOrigin().getY())
    { crosstrackError *= -1.0; }

    double crosstrackCorrection = std::atan2(
        kCrosstrack * crosstrackError,
        velDamping + frontLinearVel.length());
    const double totalCommand = headingCorrection + crosstrackCorrection;

    prevHeadingError = headingError;

    return totalCommand;
}

void StanleyController::reset()
{ prevHeadingError = 0.0; }

}
