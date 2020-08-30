#!/usr/bin/env python
# -*- coding: utf-8 -*-


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
import numpy as np
import requests
from std_msgs.msg import Float32

class TekiMitsukeru():

    def __init__(self):
        self.camera_sub = rospy.Subscriber('image_raw', Image, self.imageCallback)
        self.teki_pub = rospy.Publisher("en_distance_fromImage", Float32, queue_size=1)
        self.bridge = CvBridge()

    # camera image call back sample
    # comvert image topic to opencv object and show
    def imageCallback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        image, enemy_dis = self.calcEnemyDistance(self.img)
        cv2.imshow("Image window2", image)
        cv2.waitKey(1)

    def calcEnemyDistance(self,img):
        distance = -1
        hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

        hsvLower = np.array([0, 128, 0])
        hsvUpper = np.array([30, 255, 255])
        mask1 = cv2.inRange(hsv, hsvLower, hsvUpper)

        hsvLower = np.array([150, 128, 0])
        hsvUpper = np.array([179, 255, 255])
        mask2 = cv2.inRange(hsv, hsvLower, hsvUpper)
        
        mask = mask1 + mask2

        masked_hsv = cv2.bitwise_and(img, img, mask=mask)
        gray = cv2.cvtColor(masked_hsv,cv2.COLOR_BGR2GRAY)

        ret,thresh = cv2.threshold(gray,0,255,cv2.THRESH_BINARY)
        image, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        try:
            (x,y), radius = cv2.minEnclosingCircle(contours[0])
            center = (int(x), int(y))
            radius = int(radius)
            image = cv2.circle(img,center,radius,(0,255,0),-1)
            
            height, width, channels = img.shape[:3]
            hani = 30
            if center[0] > width/2 - hani and center[0] < width/2 + hani:
                distance = float(20.0/radius)
                self.teki_pub.publish(distance)
                cv2.putText(image,"Find enemy "+'R='+str(radius)+";D="+str(distance)+";x,y"+str(int(x))+","+str(int(y)),(50,50), cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,0),2,cv2.LINE_AA)

        except:
            image = img
            cv2.putText(image,"Lost enemy",(50,50), cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,0),2,cv2.LINE_AA)

        #return cv2.contourArea(contours[0])
        """
        protected int getDistance(double x, double y, double x2, double y2){
         double distance = Math.sqrt((x2 - x) * (x2 - x) + (y2 - y) * (y2 - y));

         return (int) distance;   
        }
        """
        return image,distance


if __name__ == '__main__':
    rospy.init_node('teki_mitsukeru')
    teki = TekiMitsukeru()
    rospy.spin()

