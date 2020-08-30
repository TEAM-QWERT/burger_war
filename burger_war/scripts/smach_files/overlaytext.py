#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from jsk_rviz_plugins.msg import OverlayText

pub_overlay_text     = rospy.Publisher("overlaytext", OverlayText, queue_size=1, latch=True)
pub_maker_score_text = rospy.Publisher("maker_score", OverlayText, queue_size=1, latch=True)

def publish(text):

    rospy.loginfo(text)
    send_data = OverlayText()
    send_data.text = text
    pub_overlay_text.publish(send_data)


def pub_maker_score(text):

    send_data = OverlayText()
    send_data.text = text
    pub_maker_score_text.publish(send_data)

