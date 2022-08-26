import rospy
import numpy as np

class TreadmillInfo:
    def __init__(self):
        self.x_max = rospy.get_param('/treadmill/length')
        self.lanes = rospy.get_param('/treadmill/lanes')
        self.y_max = self.lanes[-1]

def x_y_min_max(treadmill, car_geometry, padding):
    x_pad, y_pad = padding

    x_min = car_geometry.back_length + x_pad
    x_max = treadmill.x_max - car_geometry.front_length - x_pad

    y_min = car_geometry.width / 2 + y_pad
    y_max = treadmill.y_max - car_geometry.width / 2 - y_pad

    return (x_min, x_max, y_min, y_max)

def position_in_boundary(pos, treadmill, car_geometry, padding):
    x, y = pos
    x_min, x_max, y_min, y_max = x_y_min_max(treadmill, car_geometry, padding)

    if x < x_min or x > x_max or y < y_min or y > y_max:
        return False
    return True

def clip(pos, treadmill, car_geometry, padding):
    x, y = pos
    x_min, x_max, y_min, y_max = x_y_min_max(treadmill, car_geometry, padding)

    return (min(x_max, max(x_min, x)), min(y_max, max(y_min, y)))

def _get_other(this, that):
    this, this_d, this_init = this
    that_d, that_init = that
    t = 0.0
    if not np.isclose(this_d, 0.0):
        t = (this - this_init) / this_d
    that = that_d * t + that_init
    return that

def clip_trajectory(goal, pos, treadmill, car_geometry, padding):
    if position_in_boundary(goal, treadmill, car_geometry, padding):
        return goal

    goal_x, goal_y = goal
    pos_x, pos_y = pos
    dx = goal_x - pos_x
    dy = goal_y - pos_y
    clip_x, clip_y = clip(goal, treadmill, car_geometry, padding)

    new_goal = (clip_x, _get_other((clip_x, dx, pos_x), (dy, pos_y)))
    if position_in_boundary(new_goal, treadmill, car_geometry, padding):
        return new_goal

    return (_get_other((clip_y, dy, pos_y), (dx, pos_x)), clip_y)
