#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf
import math

tf_listener = tf.TransformListener()

def get_the_length_to_enemy():
    try:
        trans,rot = tf_listener.lookupTransform("/base_footprint", "/enemy_closest", rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr("TF lookup error [base_footprint -> enemy_closest]")
        return 1.0 #敵が検出されない場合
        
    return math.sqrt(pow(trans[0],2)+pow(trans[1],2))


def get_the_radian_to_enemy():
    try:
        trans,rot = tf_listener.lookupTransform("/base_footprint", "/enemy_closest", rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr("TF lookup error [base_footprint -> enemy_closest]")
        return 0.0 #敵が検出されない場合

    return math.atan2(trans[1],trans[0])


def get_current_enemy_zone(current_area):
    try:
        trans,rot = tf_listener.lookupTransform("/map", "/enemy_closest", rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return current_area
    
    x,y  = trans[0],trans[1]
    zone = ""
    if (y-x) > 0.0:
        if (y+x) > 0.0:
            zone = "west"
        else:
            zone = "south"
    else:
        if (y+x) > 0.0:
            zone = "north"
        else:
            zone = "east"
    return zone


def get_current_my_zone():
    try:
        trans,rot = tf_listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr("TF lookup error [base_footprint -> base_footprint]")
        return "south"#slamが動いてる限りエラーにはならないはず、、とりあえずsouthで返す
    
    x,y  = trans[0],trans[1]
    zone = ""
    if (y-x) > 0.0:
        if (y+x) > 0.0:
            zone = "west"
        else:
            zone = "south"
    else:
        if (y+x) > 0.0:
            zone = "north"
        else:
            zone = "east"
    return zone