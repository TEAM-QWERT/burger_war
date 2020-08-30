#!/usr/bin/env python
# coding: utf-8
import sys
import rospy
import actionlib
import copy
import math
import tf
from move_base_msgs.msg   import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg   import GoalID, GoalStatusArray
from geometry_msgs.msg    import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg         import Path
from std_msgs.msg         import Time

action_goal_status = ["PENDING", "ACTIVE", "PREEMPTED", "SUCCEEDED", "ABORTED", "REJECTED", "PREEMPTING", "RECALLING", "RECALLED", "LOST"]
tf_broadcaster  = tf.TransformBroadcaster()
tf_listener     = tf.TransformListener()
obtained_path = Path()


def get_current_status():
    data = ac_move_base_client.get_state()
    current_status = action_goal_status[data]
    return current_status


def send_goal_and_wait_result(goal):
    if type(goal) != type(MoveBaseGoal()):
        return False
    else:
        ac_move_base_client.wait_for_server()
        ac_move_base_client.send_goal(goal)
        result = ac_move_base_client.wait_for_result(rospy.Duration(25))
        return result


def send_goal(goal):
    if type(goal) != type(MoveBaseGoal()):
        return False
    else:
        ac_move_base_client.wait_for_server()
        ac_move_base_client.send_goal(goal)
        return True


def cancel_goal():
    ac_move_base_client.cancel_all_goals()


def pub_initialpose_for_burger_war():

    send_data = PoseWithCovarianceStamped()
    send_data.header.frame_id = "/map"
    send_data.pose.pose.position.x = -1.3
    send_data.pose.pose.position.y = 0
    send_data.pose.pose.position.z = 0
    send_data.pose.pose.orientation.x = 0
    send_data.pose.pose.orientation.y = 0
    send_data.pose.pose.orientation.z = 0
    send_data.pose.pose.orientation.w = 1
    pub_initial_pose.publish(send_data)


def pub_initialpose_for_red_side():

    send_data = PoseWithCovarianceStamped()
    send_data.header.frame_id = "/map"
    send_data.pose.pose.position.x = -1.3
    send_data.pose.pose.position.y = 0
    send_data.pose.pose.position.z = 0
    send_data.pose.pose.orientation.x = 0
    send_data.pose.pose.orientation.y = 0
    send_data.pose.pose.orientation.z = 0
    send_data.pose.pose.orientation.w = 1
    pub_initial_pose.publish(send_data)


def pub_initialpose_for_blue_side():

    send_data = PoseWithCovarianceStamped()
    send_data.header.frame_id = "/map"
    send_data.pose.pose.position.x = 1.3
    send_data.pose.pose.position.y = 0
    send_data.pose.pose.position.z = 0
    send_data.pose.pose.orientation.x = 0
    send_data.pose.pose.orientation.y = 0
    send_data.pose.pose.orientation.z = 1
    send_data.pose.pose.orientation.w = 0
    pub_initial_pose.publish(send_data)


def cb_global_path(msg):
    global obtained_path
    obtained_path = copy.copy(msg)
    #print obtained_path


def calculate_rotate_goal_from_global_path(time_out_sec, target_length):
    global obtained_path
    start_time     = rospy.Time.now()
    threshold_time = 1

    while start_time + rospy.Duration(time_out_sec) > rospy.Time.now():
        if obtained_path.header.stamp + rospy.Duration(threshold_time) > rospy.Time.now():
            break
        else:
            rospy.sleep(0.1)

    path_poses_len = len(obtained_path.poses)
    if path_poses_len == 0:
        return False

    source_pt = obtained_path.poses[0].pose.position
    target_pt = None
    for i in range(path_poses_len):
        eval_pt     = obtained_path.poses[i].pose.position
        eval_length = math.sqrt((eval_pt.x - source_pt.x)**2 + ((eval_pt.y - source_pt.y)**2))
        if eval_length > target_length:
            target_pt = eval_pt
            break

    if target_pt == None:
        target_pt = obtained_path.poses[-1].pose.position#ゴールが近すぎた場合は最後のpositionを利用する.

    #目標の座標をTFでbroadcast
    target_frame_name = "/rotate_target"
    robot_frame_name  = "/base_footprint"
    source_frame_name = obtained_path.header.frame_id
    tf_broadcaster.sendTransform((target_pt.x, target_pt.y, 0), (0,0,0,1), rospy.Time.now(), target_frame_name, source_frame_name)
    rospy.sleep(0.1)

    #ロボットから敵までの距離を計算
    try:
        (trans,rot) = tf_listener.lookupTransform(robot_frame_name, target_frame_name, rospy.Time(0))
        rorate_quartanion = tf.transformations.quaternion_from_euler(0,0,math.atan2(trans[1], trans[0]))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr("tf.lookupTransform error. [base_footprint -> rotate_target]")
        return False

    goal = MoveBaseGoal()
    goal.target_pose.header.stamp    = rospy.Time.now()
    goal.target_pose.header.frame_id = robot_frame_name

    goal.target_pose.pose.position.x    = 0
    goal.target_pose.pose.position.y    = 0
    goal.target_pose.pose.position.z    = 0
    goal.target_pose.pose.orientation.x = rorate_quartanion[0]
    goal.target_pose.pose.orientation.y = rorate_quartanion[1]
    goal.target_pose.pose.orientation.z = rorate_quartanion[2]
    goal.target_pose.pose.orientation.w = rorate_quartanion[3]
    return goal


ac_move_base_client        = actionlib.SimpleActionClient('move_base',       MoveBaseAction)
pub_initial_pose           = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
sub_move_base_dummy_path   = rospy.Subscriber('move_base/DWAPlannerROS/global_plan', Path, cb_global_path)
#ac_move_base_dummy_client  = actionlib.SimpleActionClient('move_base_dummy', MoveBaseAction)
#pub_dymmy_goal             = rospy.Publisher('momove_base_simple/goal', PoseStamped, queue_size=10)
#sub_move_base_status = rospy.Subscriber('move_base/status', GoalStatusArray, cb_goalstatus)
