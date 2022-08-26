#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
import numpy as np

def x_y(msg):
    return msg.pose.position.x, msg.pose.position.y

class DistToGoal:
    def __init__(self):
        rospy.init_node('dist_to_goal')
        self.goal = None
        rospy.Subscriber('goal', PoseStamped, self.goal_callback)
        rospy.Subscriber('pose', PoseStamped, self.pose_callback)
        self.pub = rospy.Publisher('dist_to_goal', Float64, queue_size=1)

    def goal_callback(self, msg):
        self.goal = x_y(msg)

    def pose_callback(self, msg):
        pose = x_y(msg)
        if not self.goal:
            return
        dist = Float64(np.sqrt(np.power(self.goal[0] - pose[0], 2) + np.power(self.goal[1] - pose[1], 2)))
        self.pub.publish(dist)

def main():
    node = DistToGoal()

    rospy.spin()

if __name__ == '__main__':
    main()
