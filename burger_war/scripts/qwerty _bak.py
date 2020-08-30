#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
This is rumdom run node.
subscribe No topcs.
Publish 'cmd_vel' topic. 
mainly use for simple sample program

by Takuya Yamaguhi.
'''
import rospy
import cv2
import numpy as np
import sendIdToJudge
import subprocess
import sendIdToJudge
import tf
import actionlib
import actionlib_msgs
import json
import math
import roslib
import numpy as np
import requests
import smach
import smach_ros
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

#location_listの読み込み
file_path = roslib.packages.get_pkg_dir('burger_war') + "/location_list/location_list.json"
file = open(file_path, 'r')
location_list_dict = json.load(file)

class Qwerty():
    def __init__(self, bot_name="NoName",
                 use_lidar=False, use_camera=False, use_imu=False,
                 use_odom=False, use_joint_states=False, use_war_state=False):
        self.judge_url = rospy.get_param('~judge_url', 'http://127.0.0.1:5000')
        self.name = bot_name
        self.my_field_points = np.zeros(12)
        self.my_score = 0
        self.en_field_points = np.zeros(12)
        self.en_score = 0
        self.my_col = "n"
        self.en_col = "n"
        self.need2get_fields = []
        #self.direction = [[],[],[],[]]#[S,W,N,E]
        self.direction = {"S":[],"W":[],"N":[],"E":[]}#[S,W,N,E]
        self.check_points     = ["S_right", "S_center", "S_left", 
                                "W_right", "W_center", "W_left", 
                                "N_right", "N_center", "N_left", 
                                "E_right", "E_center", "E_left"]
        self.direction_name = []

        """
        周る順番
        "OctopusWiener_S",
        "FriedShrimp_S",
        "Pudding_S",
        "Pudding_N",
        "FriedShrimp_W",
        "Tomato_S",
        "FriedShrimp_N",
        "Tomato_N",
        "Omelette_N",
        "Omelette_S",
        "FriedShrimp_E",
        "OctopusWiener_N"
        """
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

        # war state subscriber
        if use_war_state:
            self.war_state_sub = rospy.Subscriber('war_state', String, self.callback_war_state)

        # lidar scan subscriber
        if use_lidar:
            self.scan = LaserScan()
            self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.lidarCallback)

        # camera subscribver
        # please uncoment out if you use camera
        if use_camera:
            # for convert image topic to opencv obj
            self.img = None
            self.bridge = CvBridge()
            self.image_sub = rospy.Subscriber('image_raw', Image, self.imageCallback)
            self
        # imu subscriber
        if use_imu:
            self.imu_sub = rospy.Subscriber('imu', Imu, self.imuCallback)

        # odom subscriber
        if use_odom:
            self.odom_sub = rospy.Subscriber('odom', Odometry, self.odomCallback)

        # joint_states subscriber
        if use_joint_states:
            self.odom_sub = rospy.Subscriber('joint_states', JointState, self.jointstateCallback)

    # lidar scan topic call back sample
    # update lidar scan state
    def lidarCallback(self, data):
        self.scan = data
        rospy.loginfo(self.scan)

    # camera image call back sample
    # comvert image topic to opencv object and show
    def imageCallback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        '''
        self.S = self.area(self.img)
        if self.S != None:
            print(self.S)
        '''
        cv2.imshow("Image window", self.img)
        #cv2.imshow("Image window2", self.area(self.img))
        cv2.waitKey(1)

    # imu call back sample
    # update imu state
    def imuCallback(self, data):
        self.imu = data
        rospy.loginfo(self.imu)

    # odom call back sample
    # update odometry state
    def odomCallback(self, data):
        self.pose_x = data.pose.pose.position.x
        self.pose_y = data.pose.pose.position.y
        rospy.loginfo("odom pose_x: {}".format(self.pose_x))
        rospy.loginfo("odom pose_y: {}".format(self.pose_y))

    # jointstate call back sample
    # update joint state
    def jointstateCallback(self, data):
        self.wheel_rot_r = data.position[0]
        self.wheel_rot_l = data.position[1]
        rospy.loginfo("joint_state R: {}".format(self.wheel_rot_r))
        rospy.loginfo("joint_state L: {}".format(self.wheel_rot_l))

    def area(self,img):
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

            Distance = float(20.0/radius)
            cv2.putText(image,'R='+str(radius)+";D="+str(Distance)+";x,y"+str(int(x))+","+str(int(y)),(50,50), cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,0),2,cv2.LINE_AA)
            cv2.putText(image,"width: " + str(width)+";height: " + str(height),(50,100), cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,0),2,cv2.LINE_AA)

        except:
            image = img
            cv2.putText(image,"OUTCH",(50,50), cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,0),2,cv2.LINE_AA)
        #return cv2.contourArea(contours[0])
        """
        protected int getDistance(double x, double y, double x2, double y2){
         double distance = Math.sqrt((x2 - x) * (x2 - x) + (y2 - y) * (y2 - y));

         return (int) distance;   
        }
        """
        return image

    def set_players(self,data):
        if data["players"]["r"] == "you":
            my_col = "r"
            en_col = "b"
        else:
            my_col = "b"
            en_col = "r"
        return my_col, en_col

    def calc_war_state(self, data, my_col, en_col):
        my_field_points = np.zeros(12) #自分の取得フィールド点
        en_field_points = np.zeros(12) #相手の取得フィールド点
        my_score = data["scores"][my_col] #自分の点数
        en_score = data["scores"][en_col] #相手の点数
        #direction = [[],[],[],[]]#[S,W,N,E]
        direction = {"S":[],"W":[],"N":[],"E":[]}
        CHANGE = {"Tomato_N": "N_center", "Tomato_S": "W_left", "Omelette_N":	"N_left", "Omelette_S": "E_right",
                "Pudding_N": "W_right", "Pudding_S": "S_left", "OctopusWiener_N": "E_left", "OctopusWiener_S": "S_right",
                "FriedShrimp_N": "N_right", "FriedShrimp_E": "E_center", "FriedShrimp_W": "W_center", "FriedShrimp_S": "S_center"}

        #print(data["targets"])

        #フィールド点の計算
        for i,j in zip(range(6,18),range(0,12)):
            if data["targets"][i]["player"] == self.my_col:
                my_field_points[j] = 1
                en_field_points[j] = 0
                temp = CHANGE[data["targets"][i]["name"]]
                if "S" in temp:
                    #direction[0].append(temp)
                    direction["S"].append(temp)
                elif "W" in temp:
                    #direction[1].append(temp)
                    direction["W"].append(temp)
                elif "N" in temp:
                    #direction[2].append(temp)
                    direction["N"].append(temp)
                elif "E" in temp:
                    #direction[3].append(temp)
                    direction["E"].append(temp)

            elif data["targets"][i]["player"] == self.en_col:
                my_field_points[j] = 0
                en_field_points[j] = 1
        #rospy.loginfo(temp)
        #rospy.loginfo(my_field_points)
        return my_score, en_score, my_field_points, en_field_points, direction
    
    def scan_field(self, field_points):
        result = []
        """
        WAR_STATE = ["BL_B","BL_L","BL_R","RE_B","RE_L","RE_R","Tomato_N", "Tomato_S","Omelette_N","Omelette_S",
                    "Pudding_N","Pudding_S","OctopusWiener_N","OctopusWiener_S",
                    "FriedShrimp_N","FriedShrimp_E","FriedShrimp_W","FriedShrimp_S"]
        """
        WAR_STATE = ["Tomato_N", "Tomato_S","Omelette_N","Omelette_S",
                    "Pudding_N","Pudding_S","OctopusWiener_N","OctopusWiener_S",
                    "FriedShrimp_N","FriedShrimp_E","FriedShrimp_W","FriedShrimp_S"]
        CHANGE = {"Tomato_N": "N_center", "Tomato_S": "W_left", "Omelette_N":	"N_left", "Omelette_S": "E_right",
                "Pudding_N": "W_right", "Pudding_S": "S_left", "OctopusWiener_N": "E_left", "OctopusWiener_S": "S_right",
                "FriedShrimp_N": "N_right", "FriedShrimp_E": "E_center", "FriedShrimp_W": "W_center", "FriedShrimp_S": "S_center"}

        for i,j in zip(range(12), field_points):
            if j == 0:
                result.append(CHANGE[WAR_STATE[i]])
        return result

    def callback_war_state(self, data):
        resp = requests.get(self.judge_url + "/warState")
        war_state_dic = resp.json()
        if self.my_col == "n":
            self.my_col,self.en_col = self.set_players(war_state_dic)
            #rospy.loginfo(self.my_col)
        else:
            self.my_score, self.en_score, self.my_field_points, self.en_field_points, self.direction = self.calc_war_state(war_state_dic,my_col=self.my_col, en_col=self.en_col)
            self.need2get_fields = self.scan_field(self.my_field_points)
            self.en_fields = self.scan_field(self.en_field_points)
            #rospy.loginfo(self.my_field_points)
            #rospy.loginfo(self.need2get_fields)
            rospy.loginfo(self.direction)
            #rospy.loginfo(self.my_field_points)
            """
            with open("/home/majima/catkin_ws/src/burger_war/burger_war/scripts/fieldpoint.log", mode="a") as f:
                f.writelines([str(self.need2get_fields),"/n"])
            """

    def GetFieldPoint(self, my_xy, en_xy):
        HOUI = ["S", "W", "N", "E"]
        iranaikedo = {}
        local_need2getfelds = self.need2get_fields
        kyori = 0.0
        en_field = {}
        my_field = {}

        for i in iruten:
            kyori = math.sqrt(math.pow(my_xy.x - location_list_dict[i]["translation"]["x"],2) + math.pow(my_xy.y - location_list_dict[i]["translation"]["y"],2))
            iranaikedo["i"] = kyori
        for i in HOUI:
            kyori = math.sqrt(math.pow(en_xy.x - location_list_dict[i+"_center"]["translation"]["x"],2) + math.pow(en_xy.y - location_list_dict[i+"_center"]["translation"]["y"],2))
            kyori_my = math.sqrt(math.pow(my_xy.x - location_list_dict[i+"_center"]["translation"]["x"],2) + math.pow(my_xy.y - location_list_dict[i+"_center"]["translation"]["y"],2))
            en_field[i] = kyori
            my_field[i] = kyori_my
        ittehaikenai = sorted(en_field.items(), key=lambda x:x[1])[0][0]
        my_basyo = sorted(my_field.items(), key=lambda x:x[1])[0][0]
        saitan = sorted(iranaikedo.items(), key=lambda x:x[1])
        saitan_houi = saitan[0][0].split("_")#NWSEを取得
        if saitan_houi == HOUI[HOUI.index(houi) - 2]:
            #最短が自分の領域と逆側
            if ittehaikenai[0][0] == HOUI[HOUI.index(houi) - 1]:
                return HOUI[HOUI.index(houi) - 3] + "_center"#SWNEの中心
            elif ittehaikenai[0][0] == HOUI[HOUI.index(houi) - 3]:
                return  HOUI[HOUI.index(houi) - 1] + "_center"#SWNEの中心
        return saitan[0][0]
    
    def calcTwist(self):
        x = 0
        th = 0
        twist = Twist()
        twist.linear.x = x; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
        return twist


    def setGoal(self,location_name):
        self.client.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "/map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = location_list_dict[location_name]["translation"]["x"]
        goal.target_pose.pose.position.y = location_list_dict[location_name]["translation"]["y"]

        # Euler to Quartanion
        q=tf.transformations.quaternion_from_euler(0,0,location_list_dict[location_name]["rotation"]["yaw"] * math.pi / 180.0)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result() 
            
    def strategy(self):
        r = rospy.Rate(5) # change speed 1fps

        while(1):
            self.check_points.append(self.check_points[0])
            self.setGoal(self.check_points[0])
            self.check_points.pop(0)
        """
        while not rospy.is_shutdown():
            twist = self.calcTwist()
            print(twist)
            self.vel_pub.publish(twist)

            r.sleep()
        """
class Commander(smach.State):
    def __init__(self):
            smach.State.__init__(self, outcomes=["get_point", "fight_enemy", "search_enemy", "commander", "game_finish"])
            #敵と接敵したと判定する距離[m]
            self.close_enemy_th = 0.9
            #敵を検知できなくなった時に、見失うまでの時間[s]
            self.lost_enemy_time = 3
            self.last_notice_time = rospy.Time.now()
            self.close_enemy  = False
            self.find_enemy = False
            self.enemy_pos = 
            GetFieldPoint
            self.sub_enemy_fromLRF   = rospy.Subscriber('robot2enemy', Float32, self.enemy_callback_fromLRF)
            self.sub_enemy_fromImage = rospy.Subscriber('en_distance_fromImage', Float32, self.enemy_callback_fromImage)
            self.enemy_frame_name = rospy.get_param('~robot_name',"") + '/enemy_closest'
            self.my_frame_name = rospy.get_param('~robot_name',"") + "/base_footprint"
            self.map_frame_name = "/map"
            #M
            self.judge_url = rospy.get_param('~judge_url', 'http://127.0.0.1:5000')
            
        def execute(self, userdata):
            #敵検知から指定時間立った場合は、敵情報をリセット
            if (rospy.Time.now() - self.last_notice_time) > self.lost_enemy_time:
                self.find_enemy = False
                self.enemy_close = False
            
            #各状況に合わせて状態遷移
            #敵が近くにいる場合、fightEnemy状態に遷移
            if self.find_enemy == True and self.is_enemy_close == True:
                return "fight_enemy"
            #敵を発見できているが、近くにはいない場合、getPoint状態に遷移
            else if self.find_enemy == True and self.is_enemy_close == False:
                return "get_point"
            #発見できている敵がいない場合、searchEnemy状態に遷移
            else if self.find_enemy == False:
                return "search_enemy"
            else:
                return "commander"

        #LRFから敵を検知し、指定距離に入った場合フラグを立てる
        def enemy_callback_fromLRF(self, msg):
            self.last_notice_time = rospy.Time.now()
            self.find_enemy = True
            if msg.data <= close_enemy_th and self.is_enemy_close == False:
                self.is_enemy_close   = True
            elif msg.data > close_enemy_th and self.is_enemy_close == True:
                self.is_enemy_close == False

        #カメラ画像から敵を検知し、指定距離に入った場合フラグを立てる
        def enemy_callback_fromImage(self, msg):
            self.last_notice_time = rospy.Time.now()
            self.find_enemy = True
            if msg.data <= close_enemy_th and self.is_enemy_close == False:
                self.is_enemy_close   = True
            elif msg.data > close_enemy_th and self.is_enemy_close == True:
                self.is_enemy_close == False  

class GetPoint(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["finish"])

    def execute(self, userdata):


class FightEnemy(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["finish"])

    def execute(self, userdata):

class searchEnemy(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["finish"])

    def execute(self, userdata):   

if __name__ == '__main__':
    rospy.init_node('all_sensor_sample')
    bot = Qwerty(bot_name='qwerty', use_lidar=False, use_camera=True,
                 use_imu=False, use_odom=False, use_joint_states=False,use_war_state=True)
    bot.strategy()

