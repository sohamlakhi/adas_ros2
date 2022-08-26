#!/usr/bin/env python

import rospy
import rostopic
import argparse
import math

def rgetattr(obj, attrs):
    if len(attrs) == 0:
        raise RuntimeError('Empty attributes array passed.')
    if len(attrs) == 1:
        return getattr(obj, attrs[0])
    return rgetattr(getattr(obj, attrs[0]), attrs[1:])

def rsetattr(obj, attrs, val):
    if len(attrs) == 0:
        raise RuntimeError('Empty attributes array passed.')
    if len(attrs) == 1:
        return setattr(obj, attrs[0], val)
    return rsetattr(getattr(obj, attrs[0]), attrs[1:], val)

def floor_to(v, bin_size):
    return math.floor(float(v) / bin_size) * bin_size

class BinTopic:
    def __init__(self):
        sub_topic, postfix, self.field_bins = self._parse_args()

        sub_type, _, _ = rostopic.get_topic_class(sub_topic, blocking=True)

        rospy.init_node('bin_topic', anonymous=True)
        rospy.Subscriber(sub_topic, sub_type, self.callback)
        self.pub = rospy.Publisher(sub_topic + postfix, sub_type, queue_size=1)

    def callback(self, msg):
        for field, bin_size in self.field_bins.iteritems():
            fields = field.split('.')
            val = rgetattr(msg, fields)
            rsetattr(msg, fields, floor_to(val, bin_size))
        self.pub.publish(msg)

    def _parse_args(self):
        parser = argparse.ArgumentParser(description="Floor a topic's field value(s) according to specified bin size.")

        parser.add_argument(
            '--topic',
            required=True,
            help='Topic to subscribe to.')

        parser.add_argument(
            '--postfix',
            default='_binned',
            help='postfix to add for the publisher.')

        parser.add_argument(
            '--field_bins',
            nargs='+',
            required=True,
            help='Pairs of (field name) (bin size)')

        args = parser.parse_args()

        if (len(args.field_bins)) % 2 != 0:
            raise RuntimeError('Odd number of pair elements provided.')
        field_bins = {}
        for i in range(0, len(args.field_bins), 2):
            field = args.field_bins[i]
            bin_size = float(args.field_bins[i + 1])
            if bin_size <= 0:
                raise RuntimeError('Invalid bin size.')
            field_bins[field] = bin_size

        return args.topic, args.postfix, field_bins

def main():
    node = BinTopic()

    rospy.spin()

if __name__ == '__main__':
    main()
