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
import sys
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
from std_msgs.msg import Float32

import pathlib
current_dir = pathlib.Path(__file__).resolve().parent
sys.path.append(str(current_dir))
from smach_files import *
 
g_enemy_xy = [0,0]

class Commander(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["get_point", "fight_enemy", "search_enemy", "commander", "game_finish"])
        #敵と接敵したと判定する距離[m]
        self.close_enemy_th = 0.6
        #敵を検知できなくなった時に、見失うまでの時間[s]
        self.lost_enemy_time = rospy.Duration(secs=100)
        self.last_notice_time = rospy.Time.now()
        self.close_enemy  = False
        self.find_enemy = False
        self.tf_listener = tf.TransformListener()
        self.sub_enemy_fromLRF   = rospy.Subscriber('robot2enemy', Float32, self.enemy_callback_fromLRF)
        self.sub_enemy_fromImage = rospy.Subscriber('en_distance_fromImage', Float32, self.enemy_callback_fromImage)
        self.enemy_frame_name = rospy.get_param('~robot_name',"") + '/enemy_closest'
        self.my_frame_name = rospy.get_param('~robot_name',"") + "/base_footprint"
        self.map_frame_name = "/map"
        self.pub_twist = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        
    def execute(self, userdata):
        #敵検知から指定時間立った場合は、敵情報をリセット
        if ((rospy.Time.now() - self.last_notice_time) > self.lost_enemy_time):
            self.enemy_close = False
        
        #各状況に合わせて状態遷移
        #敵が近くにいる場合、fightEnemy状態に遷移

        if self.close_enemy == True:
            return "fight_enemy"
        #敵を発見できているが、近くにはいない場合、getPoint状態に遷移
        elif self.close_enemy == False:
            return "get_point"
        else:
            return "commander"

    #LRFから敵を検知し、指定距離に入った場合フラグを立てる
    def enemy_callback_fromLRF(self, msg):
        global g_enemy_xy
        enemy_frame_name = rospy.get_param('~robot_name',"") + '/enemy_closest'
        my_frame_name = rospy.get_param('~robot_name',"") + "/base_footprint"
        map_frame_name = "/map"
        enemy_pos = self.tf_listener.lookupTransform(map_frame_name, enemy_frame_name, rospy.Time(0))
        g_enemy_xy = [enemy_pos[0][0], enemy_pos[0][1]]
        rospy.logerr(["LRF",g_enemy_xy])
        self.last_notice_time = rospy.Time.now()
        self.find_enemy = True
        if msg.data <= self.close_enemy_th:
            self.close_enemy   = True
        elif msg.data > self.close_enemy_th:
            self.close_enemy == False


    #カメラ画像から敵を検知し、指定距離に入った場合フラグを立てる
    def enemy_callback_fromImage(self, msg):
        max_enemy_distance = 3.5
        global g_enemy_xy
        if msg.data > max_enemy_distance:
            enemy_distance = max_enemy_distance
        else:
            enemy_distance = msg.data
        enemy_frame_name = rospy.get_param('~robot_name',"") + '/enemy_closest'
        my_frame_name = rospy.get_param('~robot_name',"") + "/base_footprint"
        map_frame_name = "/map"
        my_pos =  self.tf_listener.lookupTransform(map_frame_name, my_frame_name, rospy.Time(0))
        #rospy.logerr(["my_pos:",my_pos])
        #my_posに座標変換した座標を入れて、グローバル変数g_enemy_posに角度と距離から敵の位置を推定したものを入れる
        #global g_enemy_pos =  [[my_pos[0][0]+msg.data*math.cos(my_pos[1]),my_pos[0][1],my_pos[0][2]],[my_pos[1][0],my_pos[1][1],my_pos[1][2],my_pos[1][3]]
        self.last_notice_time = rospy.Time.now()
        shita = tf.transformations.euler_from_quaternion((my_pos[1][0], my_pos[1][1], my_pos[1][2], my_pos[1][3]))
        angle_z = float(shita[2]) + math.pi / 2.0 
        #rospy.logerr(["angle_z",angle_z])
        #rospy.logerr(["diatance",msg.data])
        add_xy = [enemy_distance * math.sin(angle_z),enemy_distance * math.cos(angle_z)]
        #global g_enemy_pos = [[my_pos[0][0]+msg.data*math.cos(shita), my_pos[0][1]+msg.data*math.sin(shita), 0], [[0]*4]]
        g_enemy_xy = [my_pos[0][0] + add_xy[0], my_pos[0][1] + add_xy[1]]
        rospy.logerr(["Image",g_enemy_xy])
        #rospy.logerr(["g_enemy_pos",g_enemy_pos])
        self.find_enemy = True
        if msg.data <= self.close_enemy_th:
            self.close_enemy   = True
        elif msg.data > self.close_enemy_th:
            self.close_enemy == False  


class GetPoint(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["finish"])
        #location_listの読み込み
        file_path = roslib.packages.get_pkg_dir('burger_war') + "/location_list/location_list.json"
        self.judge_url = rospy.get_param('~judge_url', 'http://127.0.0.1:5000')
        file = open(file_path, 'r')
        self.location_list_dict = json.load(file)
        self.tf_listener = tf.TransformListener()
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.my_field_points = np.zeros(12)
        self.en_field_points = np.zeros(12)
        self.my_score = 0
        self.en_score = 0
        self.need2get_fields = []
        self.enemy_frame_name = rospy.get_param('~robot_name',"") + '/enemy_closest'
        self.my_frame_name = rospy.get_param('~robot_name',"") + "/base_footprint"
        self.map_frame_name = "/map"
        self.my_xy = [-1.5, 0]
        self.direction = {"S":[],"W":[],"N":[],"E":[]}#[S,W,N,E]
        self.fieldPointName_onMap = {"Tomato_N": "N_right", "Tomato_S": "W_left", "Omelette_N": "N_left", "Omelette_S": "E_right",
                "Pudding_N": "W_right", "Pudding_S": "S_left", "OctopusWiener_N": "E_left", "OctopusWiener_S": "S_right",
                "FriedShrimp_N": "N_center", "FriedShrimp_E": "E_center", "FriedShrimp_W": "W_center", "FriedShrimp_S": "S_center"}
        self.resp = requests.get(self.judge_url + "/warState")
        self.war_state_dic = self.resp.json()
        if self.war_state_dic["players"]["r"] == "you":
            self.my_col = "r"
            self.en_col = "b"
        else:
            self.my_col = "b"
            self.en_col = "r"
        self.war_state_sub = rospy.Subscriber('war_state', String, self.callback_war_state)
        self.pub_twist = rospy.Publisher('cmd_vel', Twist,queue_size=1)

        self.check_points     = ["S_center", "E_left", "S_right", "S_left", 
                                "W_right", "W_center", "W_left", 
                                "N_right", "N_center", "N_left", 
                                "E_right", "E_center", "E_left"]

    def calc_war_state(self, data, my_col, en_col):
        my_field_points = np.zeros(12) #自分の取得フィールド点
        en_field_points = np.zeros(12) #相手の取得フィールド点
        my_score = data["scores"][my_col] #自分の点数
        en_score = data["scores"][en_col] #相手の点数
        direction = {"S":[],"W":[],"N":[],"E":[]}

        #フィールド点の計算
        for i,j in zip(range(6,18),range(0,12)):
            if data["targets"][i]["player"] == self.my_col:
                my_field_points[j] = 1
                en_field_points[j] = 0
                temp = self.fieldPointName_onMap[data["targets"][i]["name"]]
                if "S" in temp:
                    direction["S"].append(temp)
                elif "W" in temp:
                    direction["W"].append(temp)
                elif "N" in temp:
                    direction["N"].append(temp)
                elif "E" in temp:
                    direction["E"].append(temp)

            elif data["targets"][i]["player"] == self.en_col:
                my_field_points[j] = 0
                en_field_points[j] = 1

        return my_score, en_score, my_field_points, en_field_points, direction
    
    def scan_field(self, field_points):
        result = []
        fieldPointName_onJudge = ["Tomato_N", "Tomato_S","Omelette_N","Omelette_S",
                                "Pudding_N","Pudding_S","OctopusWiener_N","OctopusWiener_S",
                                "FriedShrimp_N","FriedShrimp_E","FriedShrimp_W","FriedShrimp_S"]
        CHANGE = {"Tomato_N": "N_right", "Tomato_S": "W_left", "Omelette_N": "N_left", "Omelette_S": "E_right",
                "Pudding_N": "W_right", "Pudding_S": "S_left", "OctopusWiener_N": "E_left", "OctopusWiener_S": "S_right",
                "FriedShrimp_N": "N_center", "FriedShrimp_E": "E_center", "FriedShrimp_W": "W_center", "FriedShrimp_S": "S_center"}
        for i,j in zip(range(12), field_points):
            if j == 0:
                result.append(CHANGE[fieldPointName_onJudge[i]])
        return result

    def callback_war_state(self, data):
        resp = requests.get(self.judge_url + "/warState")
        war_state_dic = resp.json()
        if self.my_col == "n":
            self.my_col,self.en_col = self.set_players(war_state_dic)
        else:
            self.my_score, self.en_score, self.my_field_points, self.en_field_points, self.direction = self.calc_war_state(war_state_dic,my_col=self.my_col, en_col=self.en_col)
            self.need2get_fields = self.scan_field(self.my_field_points)
            self.en_fields = self.scan_field(self.en_field_points)

    def GetFieldPoint(self, my_xy, en_xy):
        HOUI = ["S", "W", "N", "E"]
        kyori_with_name = {}
        local_need2getfelds = self.need2get_fields
        kyori = 0.0
        en_field = {}
        my_field = {}
        
        if local_need2getfelds == []:
            return "S_center"
        for i in local_need2getfelds:
            kyori = math.sqrt(math.pow(my_xy[0] - self.location_list_dict[i]["translation"]["x"],2) + math.pow(my_xy[1] - self.location_list_dict[i]["translation"]["y"],2))
            kyori_with_name[i] = kyori
        for i in HOUI:
            kyori = math.sqrt(math.pow(en_xy[0] - self.location_list_dict[i+"_center"]["translation"]["x"],2) + math.pow(en_xy[1] - self.location_list_dict[i+"_center"]["translation"]["y"],2))
            kyori_my = math.sqrt(math.pow(my_xy[0] - self.location_list_dict[i+"_center"]["translation"]["x"],2) + math.pow(my_xy[1] - self.location_list_dict[i+"_center"]["translation"]["y"],2))
            en_field[i] = kyori
            my_field[i] = kyori_my
        ikuna = sorted(en_field.items(), key=lambda x:x[1])[0][0]
        ikuna_houi = ikuna[0][0].split("_")[0]#NWSEを取得
        
        iru = sorted(my_field.items(), key=lambda x:x[1])[0][0]
        iru_houi = iru[0][0].split("_")[0]#NWSEを取得
        MUKI = ["_center","_left","_right"]
        for i in MUKI:
            kyori_with_name.pop(ikuna_houi+i,None)
        saitan = sorted(kyori_with_name.items(), key=lambda x:x[1])
        
        #rospy.logerr(["ikuna_houi+muki:",ikuna_houi+MUKI[0]])
        try:
            saitan_houi = saitan[0][0].split("_")[0]#NWSEを取得
        except:
            pass
        #rospy.logerr(saitan)
        #rospy.logerr(sorted(en_field.items(), key=lambda x:x[1]))
        #rospy.logerr(["TEKI_houi:",ikuna_houi])
        #rospy.logerr(["SAITAN_houi:",saitan_houi])
        #rospy.logerr(["HOUI:",HOUI[HOUI.index(saitan_houi)]])
        if len(kyori_with_name) == 0:
            return iru_houi+"_st"

        if saitan_houi == HOUI[HOUI.index(iru) - 2]:
            #最短が自分の領域と逆側
            if ikuna_houi == HOUI[HOUI.index(saitan_houi) - 1]:
                return HOUI[HOUI.index(saitan_houi) - 3] + "_center"#SWNEの中心
            elif ikuna_houi == HOUI[HOUI.index(saitan_houi) - 3]:
                return  HOUI[HOUI.index(saitan_houi) - 1] + "_center"#SWNEの中心
        #rospy.logerr(saitan[0][0])
        #return "S_right"

        return saitan[0][0]
    
    def calcTwist(self):
        x = 0
        th = 0
        twist = Twist()
        twist.linear.x = x; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
        return twist


    def setGoal(self,location_name):
        #self.client.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "/map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.location_list_dict[location_name]["translation"]["x"]
        goal.target_pose.pose.position.y = self.location_list_dict[location_name]["translation"]["y"]

        # Euler to Quartanion
        q=tf.transformations.quaternion_from_euler(0,0,self.location_list_dict[location_name]["rotation"]["yaw"] * math.pi / 180.0)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        #self.client.send_goal(goal)
        move_base.send_goal_and_wait_result(goal)
        #wait = self.client.wait_for_result()
        #hoge = location_name.split("_")[0]
        #if location_name in self.direction[hoge]:
        #    move_base.cancel_goal()
        #if not wait:
        #    rospy.logerr("Action server not available!")
        #    rospy.signal_shutdown("Action server not available!")
        #else:
        #    return self.client.get_result() 
    def execute(self, userdata):
        global g_enemy_xy
        overlaytext.publish('STATE: Get filed point')
        enemy_frame_name = rospy.get_param('~robot_name',"") + '/enemy_closest'
        my_frame_name = rospy.get_param('~robot_name',"") + "/base_footprint"
        map_frame_name = "/map"
        self.pub_twist.publish(Twist())
        #move_base.cancel_goal()

        #敵情報取得
        try:
            my_pos = self.tf_listener.lookupTransform(map_frame_name, my_frame_name, rospy.Time(0))
            self.my_xy= [my_pos[0][0], my_pos[0][1]]
        except:
            pass
        #自分の情報取得

        #移動開始
        #self.check_points.append(self.check_points[0])
        #result = move_base.send_goal_and_wait_result(self.setGoal(self.check_points[0]))
        #result = move_base.send_goal_and_wait_result(self.setGoal("E_left"))
        #self.check_points.pop(0)

        #set_field = self.GetFieldPoint([-0.63,0],[0.0,0.53])
        set_field = self.GetFieldPoint(self.my_xy,g_enemy_xy)
        #result = move_base.send_goal_and_wait_result(self.setGoal(set_field))
        #result = move_base.send_goal(self.setGoal(set_field))
        self.setGoal(set_field)
        #result = move_base.send_goal_and_wait_result(self.setGoal("N_left"))
        return "finish"
        

class FightEnemy(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["finish"])
        # velocity publisher
        self.pub_twist = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        #self.enemy_frame_name = rospy.get_param('~robot_name',"") + '/enemy_closest'
        #self.my_frame_name = rospy.get_param('~robot_name',"") + "/base_footprint"
        #self.map_frame_name = "/map"
        self.tf_listener = tf.TransformListener()

    def execute(self, userdata):
        overlaytext.publish('STATE: Fight enemy')

        move_base.cancel_goal()
        enemy_frame_name = rospy.get_param('~robot_name',"") + '/enemy_closest'
        my_frame_name = rospy.get_param('~robot_name',"") + "/base_footprint"
        map_frame_name = "/map"
        # Twistのpublish
        send_data = Twist()
        try:
            trans,rot = self.tf_listener.lookupTransform(my_frame_name, enemy_frame_name, rospy.Time(0))
            send_data.angular.z =  math.atan2(trans[1],trans[0]) * 1

            if math.degrees(send_data.angular.z) > 100:
                send_data.angular.z = math.radians(100)
            elif math.degrees(send_data.angular.z) < -100:
                send_data.angular.z = math.radians(-100)

            self.pub_twist.publish(send_data)
            rospy.sleep(0.3)
            send_data.angular.z =  0
            self.pub_twist.publish(send_data)
        except:
            pass
        #rospy.sleep(1)
        return "finish"

class SearchEnemy(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["finish"])
        # velocity publisher
        self.pub_twist = rospy.Publisher('cmd_vel', Twist,queue_size=1)
    def execute(self, userdata):   
        overlaytext.publish('STATE: Search enemy')

        move_base.cancel_goal()
        # Twistのpublish
        send_data = Twist()
        send_data.angular.z = 0#math.radians(100)
        self.pub_twist.publish(send_data)
        rospy.sleep(0.1)
        self.pub_twist.publish(Twist())
        #rospy.sleep(1)
        return "finish"

if __name__ == '__main__':
    rospy.init_node('all_sensor_sample')
    sm = smach.StateMachine(outcomes=["Game_finish"])
    with sm:
        smach.StateMachine.add("Commander", Commander(), transitions={"get_point": "GetPoint", "fight_enemy": "FightEnemy", "search_enemy": "SearchEnemy", "commander": "Commander", "game_finish": "Game_finish"})
        smach.StateMachine.add("GetPoint",  GetPoint(),  transitions={"finish": "Commander"})
        smach.StateMachine.add("FightEnemy",  FightEnemy(),  transitions={"finish": "Commander"})
        smach.StateMachine.add("SearchEnemy", SearchEnemy(), transitions={"finish": "Commander"})

    #sis = smach_ros.IntrospectionServer("server", sm, "/BURGER_WAR_TASK")
    #sis.start()
    sm.execute()
    #sis.stop()