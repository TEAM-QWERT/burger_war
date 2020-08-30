#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import requests
import json
from time import sleep
from burger_war.msg import MarkerStatus, MarkerList

def urlreq():
    resp = requests.get("http://localhost:5000/warState")
    data = resp.json()
    return data

def publisher():
    pub = rospy.Publisher('marker_status', MarkerList, queue_size=10)
    rospy.init_node('marker_status_publisher_node', anonymous=True)
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        state_json = urlreq()

        known_list = ['BL_B', 'BL_L', 'BL_R', 'RE_B', 'RE_L', 'RE_R']
        marker_list = MarkerList()
        for number in range(len(state_json["targets"])):
            marker_msg = MarkerStatus()
            if state_json["targets"][number]["name"] not in known_list:
                marker_msg.marker_name = state_json["targets"][number]["name"].encode()
                marker_msg.owner = state_json["targets"][number]["player"].encode()
                marker_list.markers.append(marker_msg)

        pub.publish(marker_list)
        r.sleep()

    # print json.dumps(state_json["targets"], indent=4)

if __name__ == "__main__":
     try:
        publisher()
     except rospy.ROSInterruptException: pass