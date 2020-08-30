#!/usr/bin/env python
# coding: utf-8
import sys
import rospy
import roslib
import json
import copy
import math
from burger_war.msg     import MarkerList
from smach_files          import overlaytext, tf_util

#陣地の情報を取得
my_side    = rospy.get_param("/my_side")
enemy_side = rospy.get_param("/enemy_side")
print "my_side = {}".format(my_side)
print "enemy_side = {}".format(enemy_side)

#location_listの読み込み
file_path = roslib.packages.get_pkg_dir('burger_war') + "/location_list/target_list.json"
file = open(file_path, 'r')
location_list_dict = json.load(file)

#global変数
next_target_location             = ""
enemy_last_get_maker_name        = ""
previous_my_markers_name_list    = []
previous_enemy_markers_name_list = []


def get_current_get_marker_num():
    global previous_my_markers_name_list
    print "get_current_get_marker_num = {}".format(len(previous_my_markers_name_list))
    return len(previous_my_markers_name_list)


def am_i_win():
    global previous_my_markers_name_list, previous_enemy_markers_name_list
    if len(previous_my_markers_name_list) > len(previous_enemy_markers_name_list):
        return True
    else:
        return False


def get_next_location_name():
    return next_target_location


def is_maker_mine(location_name):
    global previous_my_markers_name_list
    if location_name in previous_my_markers_name_list:
        return True
    else:
        return False


def get_erea_name(location_name):
    try:
        return location_list_dict[location_name]["option"]
    except:
        return ""

def get_location_point(location_name):
    x = location_list_dict[location_name]["translation"]["x"]
    y = location_list_dict[location_name]["translation"]["y"]
    return (x, y)

def get_location_weight(target_erea, source_erea):

    if source_erea == "north":
        if target_erea   == "north":
            return 1
        elif target_erea == "west" or target_erea == "east":
            return 0.5
        else:#south
            return 0
    elif source_erea == "west":
        if target_erea   == "west":
            return 1
        elif target_erea == "north" or target_erea == "south":
            return 0.5
        else:#east
            return 0
    elif source_erea == "east":
        if target_erea   == "east":
            return 1
        elif target_erea == "north" or target_erea == "south":
            return 0.5
        else:#west
            return 0
    else:#south
        if target_erea   == "south":
            return 1
        elif target_erea == "west" or target_erea == "east":
            return 0.5
        else:#north
            return 0


def is_location_denger(target_erea, enemy_erea):
    if target_erea == enemy_erea:
        return True
    else:
        return False


def switch_location(next_target_location, my_zone, enemy_zone):
    target_erea = get_erea_name(next_target_location)
    if target_erea == "west" and my_zone == "east":
        if enemy_zone == "north":
            return "CheckPoint_S"
        elif enemy_zone == "south":
            return "CheckPoint_N"
        else:
            return next_target_location
    elif target_erea == "north" and my_zone == "south":
        if enemy_zone == "west":
            return "CheckPoint_E"
        elif enemy_zone == "east":
            return "CheckPoint_W"
        else:
            return next_target_location
    elif target_erea == "east" and my_zone == "west":
        if enemy_zone == "north":
            return "CheckPoint_S"
        elif enemy_zone == "south":
            return "CheckPoint_N"
        else:
            return next_target_location
    elif target_erea == "south" and my_zone == "north":
        if enemy_zone == "west":
            return "CheckPoint_E"
        elif enemy_zone == "east":
            return "CheckPoint_W"
        else:
            return next_target_location
    else:
        return next_target_location



def cb_marker_status(msg):
    global previous_enemy_markers_name_list, previous_my_markers_name_list
    global enemy_last_get_maker_name, my_last_get_maker_name, next_target_location

    enemy_markers_name_list = []
    none_markers_name_list  = []
    my_markers_name_list    = []

    #誰がどのマーカーを確保しているか情報整理
    for marker in msg.markers:
        if marker.owner == enemy_side:
            enemy_markers_name_list.append(marker.marker_name)
            #敵が新しく取得したマーカーかどうか確認
            if marker.marker_name not in previous_enemy_markers_name_list:
                enemy_last_get_maker_name = marker.marker_name
        elif marker.owner == "n":
            none_markers_name_list.append(marker.marker_name)
        elif marker.owner == my_side:
            my_markers_name_list.append(marker.marker_name)
    #保存
    previous_my_markers_name_list    = copy.copy(my_markers_name_list)
    previous_enemy_markers_name_list = copy.copy(enemy_markers_name_list)

    #敵のいる領域を推定
    enemy_zone = tf_util.get_current_enemy_zone(get_erea_name(enemy_last_get_maker_name))
    my_zone    = tf_util.get_current_my_zone()

    #各マーカーの優先度を評価
    evaluate_location_name = enemy_markers_name_list
    evaluate_location_name.extend(none_markers_name_list)
    location_value_dict = {}
    for name in evaluate_location_name:
        evaluation_value = 1.0 if name in none_markers_name_list else 2.0 #noneのマーカは1点、敵のマーカは2点の評価
        evaluation_value += get_location_weight(get_erea_name(name), my_zone)#自分に近いマーカーは評価を上げる
        if is_location_denger(get_erea_name(name), enemy_zone) == True:#敵が存在する場所には行かない
            evaluation_value = 0.0
        location_value_dict[name] = evaluation_value

    if len(location_value_dict.keys()) == 0:
        next_target_location = ""
        return
    
    sorted_location_value_dict = sorted(location_value_dict.items(), key=lambda x:x[1], reverse=True)
    next_target_location = sorted_location_value_dict[0][0]#最も評価が高い場所に移動する
    next_target_location = switch_location(next_target_location, my_zone, enemy_zone)#中継地点が必要な場合は目標値を入れ替える

    pub_text = "[Evaluation Value]\n"
    for key, value in sorted_location_value_dict:
        pub_text += (key + "\t\t: " + str(value) + "\n")
    overlaytext.pub_maker_score(pub_text)


sub_makers_states = rospy.Subscriber('/marker_status', MarkerList, cb_marker_status)
