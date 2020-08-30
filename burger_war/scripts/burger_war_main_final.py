#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random
import tf
import json
import math
import random
import actionlib
import actionlib_msgs
import smach
import smach_ros
import roslib.packages
from subprocess        import Popen
from geometry_msgs.msg import Twist
from std_msgs.msg      import Float32

rospy.init_node("burger_war_main_node")#smath_filesでtfを使用するため,init_nodeする前にtf_listerner()があるとエラー
from smach_files          import *

# Global変数
target_location_global = ""


class Setup(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=["finish"])

    def execute(self, userdata):

        overlaytext.publish("Setup now ...")
        rospy.sleep(1)
        my_side = rospy.get_param("/my_side")

        if my_side == "r":
            move_base.pub_initialpose_for_red_side()
        else:
            move_base.pub_initialpose_for_blue_side()

        rospy.sleep(1)
        return "finish"


class Commander(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=["move", "fight", "commander", "game_finish"])

        self.is_enemy_close = False
        self.notice_length  = 0.8

    def execute(self, userdata):
        global target_location_global

        self.is_enemy_close = True if tf_util.get_the_length_to_enemy() < self.notice_length else False

        #各状況に合わせて状態遷移
        if self.is_enemy_close == True:
            return "fight"
        else:
            target_location_global = maker.get_next_location_name()
            if target_location_global == "":
                rospy.sleep(1)
                return "commander"
            else:
                return "move"

class Move(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=["finish"])
        self.notice_length = 0.8

    def execute(self, userdata):
        global target_location_global

        goal = json_util.generate_movebasegoal_from_locationname(target_location_global)
        overlaytext.publish("Move to " + target_location_global)

        #移動開始
        move_base.send_goal(goal)        
        start_moving_time = rospy.Time.now()

        while True:
            rospy.sleep(0.5)

            if (tf_util.get_the_length_to_enemy() < self.notice_length) or (maker.get_next_location_name() != target_location_global):#敵が近づいてきた場合or目標地点が変化した場合
                move_base.cancel_goal()
                break
            elif maker.is_maker_mine(target_location_global) == True:
                move_base.cancel_goal()
                overlaytext.publish("Get the maker[" + target_location_global + "].")
                rospy.sleep(1.0)
                break
            elif move_base.get_current_status() == "SUCCEEDED":#到着したがマーカーを取れていない
                twist.publish_back_twist()
                break
            elif move_base.get_current_status() != "SUCCEEDED" and move_base.get_current_status() != "ACTIVE":#slam失敗した場合
                move_base.cancel_goal()
                if random.random() > 0.5:#slamに失敗した時だけガチャを引く
                    twist.publish_back_twist()
                else:
                    twist.publish_forward_twist()
                break
            elif (start_moving_time + rospy.Duration(13)) < rospy.Time.now():
                print "Move_Base Timeout"
                move_base.cancel_goal()
                if random.random() > 0.5:#slamに失敗した時だけガチャを引く
                    twist.publish_back_twist()
                else:
                    twist.publish_forward_twist()
                break

        return "finish"


class Fight(smach.State):#敵が付近に存在する場合は、敵のマーカーをトラッキング

    def __init__(self):
        smach.State.__init__(self, outcomes=["finish"])
        self.angular_weight = 1.50
        self.notice_length  = 0.80

    def execute(self, userdata):

        start_fight_time = rospy.Time.now()
        cycle_start_time = rospy.Time.now()
        cycle_count      = 0
        while True:
            move_base.cancel_goal()
            overlaytext.publish("STATE: Fight\nlength = " + str(tf_util.get_the_length_to_enemy())[:6])
            if tf_util.get_the_length_to_enemy() > self.notice_length:
                break
            elif start_fight_time + rospy.Duration(20) < rospy.Time.now() and maker.get_current_get_marker_num() >= 3 and maker.am_i_win() == False:#現状負けてて、自分が動いてマーカすべて取られてもKOにならない場合、敵を無視する
                Popen(["rosnode", "kill", "/enemy_detector_node"])
                print "\n\n無双モード\n\n"
                break
            elif cycle_start_time + rospy.Duration(3) < rospy.Time.now() and cycle_count < 5 and maker.am_i_win() == False:#3sの周期で的に対し少し距離を置く
                twist.publish_back_twist()
                cycle_start_time = rospy.Time.now()
                cycle_count += 1
                #break

            # Twistのpublish
            target_angular = tf_util.get_the_radian_to_enemy() * self.angular_weight
            twist.publish_rotate_twist(target_angular)

        return "finish"


if __name__ == "__main__":

    rospy.init_node("burger_war_main_node")
    rospy.loginfo("Start burger war main program.")

    sm = smach.StateMachine(outcomes=["Game_finish"])
    with sm:
        smach.StateMachine.add("Setup",     Setup(),     transitions={"finish": "Commander"})
        smach.StateMachine.add("Commander", Commander(), transitions={"move": "Move", "fight": "Fight", "commander": "Commander", "game_finish": "Game_finish"})
        smach.StateMachine.add("Move",      Move(),      transitions={"finish": "Commander"})
        smach.StateMachine.add("Fight",     Fight(),     transitions={"finish": "Commander"})

    sis = smach_ros.IntrospectionServer("server", sm, "/BURGER_WAR_TASK")
    sis.start()
    sm.execute()
    sis.stop()
