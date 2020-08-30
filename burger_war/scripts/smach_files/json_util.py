#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import json
import roslib
from move_base_msgs.msg import MoveBaseGoal


#location_listの読み込み
file_path = roslib.packages.get_pkg_dir('burger_war') + "/location_list/location_list.json"
file = open(file_path, 'r')
location_list_dict = json.load(file)


def generate_movebasegoal_from_locationname(location_name):
    try:
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp    = rospy.Time.now()
        goal.target_pose.header.frame_id = rospy.get_param('~robot_name', '') + "/map"

        goal.target_pose.pose.position.x    = location_list_dict[location_name]["translation"]["x"]
        goal.target_pose.pose.position.y    = location_list_dict[location_name]["translation"]["y"]
        goal.target_pose.pose.position.z    = location_list_dict[location_name]["translation"]["z"]
        goal.target_pose.pose.orientation.x = location_list_dict[location_name]["rotation"]["x"]
        goal.target_pose.pose.orientation.y = location_list_dict[location_name]["rotation"]["y"]
        goal.target_pose.pose.orientation.z = location_list_dict[location_name]["rotation"]["z"]
        goal.target_pose.pose.orientation.w = location_list_dict[location_name]["rotation"]["w"]

        return goal
    except:
        return False
