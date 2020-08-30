# coding: UTF-8
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import sendIdToJudge
from tf.transformations import euler_from_quaternion
import subprocess
import sendIdToJudge
import tf
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs
import json
import math
import roslib

#location_listの読み込み
file_path = roslib.packages.get_pkg_dir('burger_war') + "/location_list/location_list.json"
file = open(file_path, 'r')
location_list_dict = json.load(file)
check_points     = ["south_right", "south_center", "south_left", "west_right", "west_center", "west_left", "north_right", "north_center", "north_left", "east_right", "east_center", "east_left"]
        #q=tf.transformations.quaternion_from_euler(0,0,yaw)

for i in check_points:
    q=tf.transformations.euler_from_quaternion([location_list_dict[i]["rotation"]["x"],
    location_list_dict[i]["rotation"]["y"],
    location_list_dict[i]["rotation"]["z"],
    location_list_dict[i]["rotation"]["w"]])
    with open("test.log", "w") as f:
        """
        f.writelines([i,str(location_list_dict[i]["translation"]["x"]),
        str(location_list_dict[i]["translation"]["y"]),str(q[2]*360/3.14),"\n"])
        print(q)
        """
        print(i,"{:.3f} {:.3f} {:.3f}".format(location_list_dict[i]["translation"]["x"],
        location_list_dict[i]["translation"]["y"],(q[2]*180/3.14),file=f))
print("end")
