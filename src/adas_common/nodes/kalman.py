#!/usr/bin/env python

import rospy
import numpy as np
import argparse
import yaml
from importlib import import_module
from filterpy.kalman import KalmanFilter
from adas_common.msg import StateWithCovariance
from std_msgs.msg import Bool

def msg():
    return '''%(prog)s [options]

Linear Kalman filter node. Assumes <predict hz> == <observation hz>

Config format:

hz: <Kalman update frequency>
inputs:
    - <topic>: [<type>, <subfield>]
    - ...
observations:
    - <topic>: [<type>, <subfield>]
    - ...
initial_state: <Initial state vector>
initial_covariance: <Initial covariance matrix, row-wise>
A: <Model update matrix>
B: <Model input matrix>
C: <Observation matrix>
Q: <Process noise matrix>
R: <Observation noise matrix>

- Vectors are specified as lists.
- All matrices are specified as a 1D list, row-wise.
'''

def yaml_from_file(path):
    with open(path, 'r') as f:
        return yaml.load(f.read())

def rgetattr(obj, attrs):
    if len(attrs) == 0:
        raise RuntimeError('Empty attributes array passed.')
    if len(attrs) == 1:
        return getattr(obj, attrs[0])
    return rgetattr(getattr(obj, attrs[0]), attrs[1:])

class TopicValueConfig:
    def __init__(self, topic, ros_type_name, subfield):
        self.topic = topic
        type_pkg, type_name = ros_type_name.split('/')
        self.type = getattr(import_module('{}.msg'.format(type_pkg)), type_name)
        self.subfield = subfield.split('.')

def get_topic_value_configs(l):
    result = []
    for d in l:
        if len(d) != 1:
            raise RuntimeError('Malformed Config.')
        for topic, (ros_type_name, subfield) in d.iteritems():
            result.append(TopicValueConfig(topic, ros_type_name, subfield))
    return result

def numpy_2d_from_list(l, rows, cols):
    if len(l) != rows * cols:
        raise RuntimeError('Malformed array.')
    result = np.zeros((rows, cols))
    for i in range(0, rows):
        for j in range(0, cols):
            result[i, j] = l[i * cols + j]
    return result

def get_sub_data(topic_value_configs, topic_type):
    sub_data = {}
    for i, topic_value in enumerate(topic_value_configs):
        if not topic_value.topic in topic_type:
            topic_type[topic_value.topic] = topic_value.type
        if not topic_value.topic in sub_data:
            sub_data[topic_value.topic] = []
        sub_data[topic_value.topic].append((i, topic_value.subfield))

    return topic_type, sub_data

class Kalman:
    def __init__(self, config, hz):
        rospy.init_node('kalman')

        self.enabled = False
        rospy.Subscriber('~enabled', Bool, self.enable_callback)

        num_states = len(config['initial_state'])
        num_inputs = len(config['inputs'])
        num_observations = len(config['observations'])

        topic_type = {}
        topic_type, input_sub_data = get_sub_data(
            get_topic_value_configs(config['inputs']),
            topic_type)
        topic_type, obsv_sub_data = get_sub_data(
            get_topic_value_configs(config['observations']),
            topic_type)

        for topic, idx_subfield in input_sub_data.iteritems():
            rospy.Subscriber(topic, topic_type[topic], self.input_callback, idx_subfield)

        for topic, idx_subfield in obsv_sub_data.iteritems():
            rospy.Subscriber(topic, topic_type[topic], self.observation_callback, idx_subfield)

        A = numpy_2d_from_list(config['A'], num_states, num_states)
        B = numpy_2d_from_list(config['B'], num_states, num_inputs)
        C = numpy_2d_from_list(config['C'], num_observations, num_states)
        Q = numpy_2d_from_list(config['Q'], num_states, num_states)
        R = numpy_2d_from_list(config['R'], num_observations, num_observations)

        self.initial_state = np.array(config['initial_state'])
        self.initial_covariance = numpy_2d_from_list(config['initial_covariance'], num_states, num_states)

        self.u = np.zeros(num_inputs)
        self.y = np.zeros(num_observations)

        self.kf = KalmanFilter(dim_x=num_states, dim_z=num_observations, dim_u=num_inputs)
        self.kf.F = A
        self.kf.B = B
        self.kf.H = C
        self.kf.Q = Q
        self.kf.R = R

        rospy.Timer(rospy.Duration(1.0 / hz), self.timer_callback)

        self.pub = rospy.Publisher('~state', StateWithCovariance, queue_size=1)

    def input_callback(self, msg, idx_subfield):
        for idx, subfield in idx_subfield:
            self.u[idx] = rgetattr(msg, subfield)

    def observation_callback(self, msg, idx_subfield):
        for idx, subfield in idx_subfield:
            self.y[idx] = rgetattr(msg, subfield)

    def enable_callback(self, msg):
        prev_enabled = self.enabled
        self.enabled = msg.data
        if not prev_enabled and self.enabled:
            self.kf.x = self.initial_state.copy()
            self.kf.P = self.initial_covariance.copy()

    def timer_callback(self, evt):
        if not self.enabled:
            return

        self.kf.predict(self.u)
        self.kf.update(self.y)

        msg = StateWithCovariance()
        msg.stamp = rospy.Time.now()
        msg.num_states = self.kf.x.shape[0]
        msg.num_observations = self.kf.y.shape[0]
        msg.state = self.kf.x.tolist()
        msg.covariance = [v for row in self.kf.P.tolist() for v in row]
        msg.residual = self.kf.y.tolist()
        self.pub.publish(msg)

def main():
    parser = argparse.ArgumentParser(usage=msg())
    parser.add_argument(
        'config',
        type=yaml_from_file,
        help='Config YAML.')

    args = parser.parse_args(rospy.myargv()[1:])
    kalman = Kalman(args.config, rospy.get_param('/target_hz'))

    rospy.spin()

if __name__ == '__main__':
    main()
