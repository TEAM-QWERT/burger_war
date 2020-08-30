#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
from geometry_msgs.msg    import Twist

pub_twist   = rospy.Publisher("cmd_vel", Twist, queue_size=10)

def publish_rotate_twist(radian):

    send_data = Twist()
    send_data.angular.z = radian

    if math.degrees(send_data.angular.z) > 100:
        send_data.angular.z = math.radians(100)
    elif math.degrees(send_data.angular.z) < -100:
        send_data.angular.z = math.radians(-100)

    pub_twist.publish(send_data)
    rospy.sleep(0.3)
    pub_twist.publish(Twist())


def publish_back_twist():

    send_data = Twist()
    send_data.linear.x = -0.15

    pub_twist.publish(send_data)
    rospy.sleep(0.5)
    pub_twist.publish(Twist())


def publish_forward_twist():

    send_data = Twist()
    send_data.linear.x = 0.15

    pub_twist.publish(send_data)
    rospy.sleep(0.5)
    pub_twist.publish(Twist())
