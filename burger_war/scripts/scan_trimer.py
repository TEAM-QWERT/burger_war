#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import rospy
import math
import roslib.packages
from sensor_msgs.msg import LaserScan

class ScanTrimer:
    def __init__(self):
        self.sub_scan        = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.pub_scan_trimed = rospy.Publisher('scan_trimed', LaserScan, queue_size=10)

    def scan_callback(self, msg):

        triming_point_num = 30
        center_pt = len(msg.ranges) / 2
        start_pt  = center_pt - (triming_point_num / 2)
        end_pt    = center_pt + (triming_point_num / 2)

        ranges_list = list(msg.ranges)
        for i in range(start_pt, end_pt):
            ranges_list[i] = 0
        msg.ranges = tuple(ranges_list)

        self.pub_scan_trimed.publish(msg)

if __name__ == '__main__':

    rospy.init_node('scan_trimer_node')
    st = ScanTrimer()
    rospy.loginfo("Scan Trimer Node Start.")
    rospy.spin()
