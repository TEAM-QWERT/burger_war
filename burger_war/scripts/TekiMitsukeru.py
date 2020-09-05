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
from std_msgs.msg import Float32, Int16

class TekiMitsukeru():

    def __init__(self):
        self.camera_sub = rospy.Subscriber('image_raw', Image, self.imageCallback)
        self.teki_pub = rospy.Publisher("en_distance_fromImage", Float32, queue_size=1)
        self.teki_find_pub = rospy.Publisher("en_find_fromImage", Int16, queue_size=1)
        self.img_pub = rospy.Publisher("en_image", Image, queue_size=1)
        self.bridge = CvBridge()

    # camera image call back sample
    # comvert image topic to opencv object and show
    def imageCallback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        image, enemy_dis = self.calcEnemyDistance(self.img)
        self.img_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
        #cv2.imshow("Image window2", image)
        #cv2.waitKey(1)

    def calcEnemyDistance(self,img):
        distance = -1
        hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

        hsvLower = np.array([170, 100, 70])
        hsvUpper = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv, hsvLower, hsvUpper)

        hsvLower = np.array([0, 100, 70])
        hsvUpper = np.array([10, 255, 255])
        mask2 = cv2.inRange(hsv, hsvLower, hsvUpper)
        """
        #シミュレーション用
        hsvLower = np.array([0, 128, 0])
        hsvUpper = np.array([30, 255, 255])
        mask1 = cv2.inRange(hsv, hsvLower, hsvUpper)

        hsvLower = np.array([150, 128, 0])
        hsvUpper = np.array([179, 255, 255])
        mask2 = cv2.inRange(hsv, hsvLower, hsvUpper)
        """
        mask_R = mask1 + mask2

        hsvLower = np.array([40, 100, 70])
        hsvUpper = np.array([60, 255, 255])
        """
        #シミュレーション用
        hsvLower = np.array([60, 128, 0])
        hsvUpper = np.array([90, 255, 255])
        """
        mask_G = cv2.inRange(hsv, hsvLower, hsvUpper)

        masked_hsv_R = cv2.bitwise_and(img, img, mask=mask_R)
        gray_R = cv2.cvtColor(masked_hsv_R,cv2.COLOR_BGR2GRAY)

        masked_hsv_G = cv2.bitwise_and(img, img, mask=mask_G)
        gray_G = cv2.cvtColor(masked_hsv_G,cv2.COLOR_BGR2GRAY)

        ret,thresh_R = cv2.threshold(gray_R,0,255,cv2.THRESH_BINARY)
        image, contours_R, hierarchy_R = cv2.findContours(thresh_R,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        ret,thresh_G = cv2.threshold(gray_G,0,255,cv2.THRESH_BINARY)
        image, contours_G, hierarchy_G = cv2.findContours(thresh_G,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        hoge_img = img
        try:
            (x,y), radius = cv2.minEnclosingCircle(contours_G[0])
            center = (int(x), int(y))
            radius = int(radius)
            image = cv2.circle(hoge_img,center,radius,(255,0,0),-1)
            hoge_img = image
            height, width, channels = hoge_img.shape[:3]
            hani = 60
            if center[0] > width/2 - hani and center[0] < width/2 + hani:
                self.teki_find_pub.publish(1)
        except:
            image = img
        try:
            (x,y), radius = cv2.minEnclosingCircle(contours_R[0])
            center = (int(x), int(y))
            radius = int(radius)
            image = cv2.circle(hoge_img,center,radius,(0,255,0),-1)
            
            height, width, channels = img.shape[:3]
            hani = 30
            if center[0] > width/2 - hani and center[0] < width/2 + hani:
                distance = float(20.0/radius)

                self.teki_pub.publish(distance)
                cv2.putText(image,"Find enemy "+'R='+str(radius)+";D="+str(distance)+";x,y"+str(int(x))+","+str(int(y)),(50,50), cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,0),2,cv2.LINE_AA)

        except:
            image = hoge_img
            cv2.putText(image,"Lost enemy",(50,50), cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,0),2,cv2.LINE_AA)

        return image, distance



if __name__ == '__main__':
    rospy.init_node('teki_mitsukeru')
    teki = TekiMitsukeru()
    rospy.spin()

