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
obtained_path = Path()
ac_move_base_client        = actionlib.SimpleActionClient('move_base',       MoveBaseAction)

def get_current_status():
    data = ac_move_base_client.get_state()
    if len(data.status_list) > 0:
        current_status = action_goal_status[data.status_list[0].status]
        return current_status

def send_goal_and_wait_result(goal):
    if type(goal) != type(MoveBaseGoal()):
        return False
    else:
        ac_move_base_client.wait_for_server()
        ac_move_base_client.send_goal(goal)
        result = ac_move_base_client.wait_for_result(rospy.Duration(1))
        #result = ac_move_base_client.wait_for_result(rospy.Duration(1,500000000))
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


