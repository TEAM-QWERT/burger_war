#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import rospy
import math
import tf
import pathlib
import roslib.packages
current_dir = pathlib.Path(__file__).resolve().parent
sys.path.append(str(current_dir)+'/../../')
from obstacle_detector.msg import Obstacles
from std_msgs.msg          import Float32, Int16
ignore_pos = []

class EnemyDetector:
    def __init__(self):
        #self.map_data#このクラスが持つ「num」変数に引数を格納
        self.tf_broadcaster  = tf.TransformBroadcaster()
        self.tf_listener     = tf.TransformListener()
        self.sub_obstacles   = rospy.Subscriber('obstacles', Obstacles, self.obstacles_callback)
        self.pub_robot2enemy = rospy.Publisher('robot2enemy', Float32, queue_size=10)
        self.robot_name      = rospy.get_param('~robot_name', '')
        self.sub_balus_com   = rospy.Subscriber('balus',Int16, self.balus_callback) 
        self.enemy_frame_name= rospy.get_param('~robot_name',"") + '/enemy_closest'
        self.map_frame_name  = "/map"
        #self.ignore_pos      = []
        #無視するエリアの拡張幅[m]
        self.ignore_enh      = 0.1 

    def obstacles_callback(self, msg):
        global ignore_pos
        closest_enemy_len = sys.float_info.max
        closest_enemy_x   = 0
        closest_enemy_y   = 0
        #rospy.logerr(["balus list;",ignore_pos])
        for num in range(len(msg.circles)):

            temp_x = msg.circles[num].center.x
            temp_y = msg.circles[num].center.y

            #フィールド内のオブジェクトであればパス
            if self.is_point_emnemy(temp_x, temp_y) == False:
                continue

            #敵の座標をTFでbroadcast
            enemy_frame_name = self.robot_name + '/enemy_' + str(num)
            map_frame_name   = self.robot_name + "/map"
            self.tf_broadcaster.sendTransform((temp_x,temp_y,0), (0,0,0,1), rospy.Time.now(), enemy_frame_name, map_frame_name)

            #ロボットから敵までの距離を計算
            try:
                target_frame_name = self.robot_name + '/enemy_' + str(num)
                source_frame_name = self.robot_name + "/base_footprint"
                (trans,rot) = self.tf_listener.lookupTransform(source_frame_name, target_frame_name, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            len_robot2enemy = math.sqrt(pow(trans[0],2) + pow(trans[1],2))

            if closest_enemy_len > len_robot2enemy:
                closest_enemy_len = len_robot2enemy
                closest_enemy_x   = temp_x
                closest_enemy_y   = temp_y

        #敵を検出している場合、その座標と距離を出力
        if closest_enemy_len < sys.float_info.max:

            map_frame_name   = self.robot_name + "/map"
            enemy_frame_name = self.robot_name + "/enemy_closest"
            self.tf_broadcaster.sendTransform((closest_enemy_x,closest_enemy_y,0), (0,0,0,1), rospy.Time.now(), enemy_frame_name, map_frame_name)

            #ロボットから敵までの距離をpublish
            self.pub_robot2enemy.publish(closest_enemy_len)

    def balus_callback(self, msg):
        global ignore_pos
        try:
            enemy_pos = self.tf_listener.lookupTransform(self.map_frame_name, self.enemy_frame_name, rospy.Time(0))
            enemy_xy = [enemy_pos[0][0], enemy_pos[0][1]]
            for ignore_pos_xy in ignore_pos:
                if (math.sqrt(pow(enemy_xy[0]-ignore_pos_xy[0],2))+math.sqrt(pow(enemy_xy[1]-ignore_pos_xy[1],2))) < 0.1:
                    return 0
            ignore_pos.append(enemy_xy)
        except:
            pass

    def is_point_emnemy(self, point_x, point_y):
        global ignore_pos
        #フィールド内の物体でない、敵と判定する閾値（半径）
        thresh_corner = 0.130
        thresh_center = 0.250

        #フィールド内かチェック
        if   point_y > (-point_x + 1.6):
            return False
        elif point_y < (-point_x - 1.6):
            return False
        elif point_y > ( point_x + 1.6):
            return False
        elif point_y < ( point_x - 1.6):
            return False

        #フィールド内の物体でないかチェック
        len_p1 = math.sqrt(pow((point_x - 0.53), 2) + pow((point_y - 0.53), 2))
        len_p2 = math.sqrt(pow((point_x - 0.53), 2) + pow((point_y + 0.53), 2))
        len_p3 = math.sqrt(pow((point_x + 0.53), 2) + pow((point_y - 0.53), 2))
        len_p4 = math.sqrt(pow((point_x + 0.53), 2) + pow((point_y + 0.53), 2))
        len_p5 = math.sqrt(pow(point_x         , 2) + pow(point_y         , 2))

        try:
            for ignore_pos_xy in ignore_pos:
                dis = math.sqrt(pow((point_x - ignore_pos_xy[0]), 2) + pow((point_y - ignore_pos_xy[1]), 2))
                if dis < self.ignore_enh:
                    return False
        except:
            pass

        if len_p1 < thresh_corner or len_p2 < thresh_corner or len_p3 < thresh_corner or len_p4 < thresh_corner or len_p5 < thresh_center:
            return False
        else:
            return True


if __name__ == '__main__':

    rospy.init_node('enemy_detector')
    ed = EnemyDetector()
    rospy.loginfo("Enemy Detector Start.")
    rospy.spin()
