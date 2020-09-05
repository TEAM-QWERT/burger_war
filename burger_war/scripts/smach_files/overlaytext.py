#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from jsk_rviz_plugins.msg import OverlayText

pub_overlay_text = rospy.Publisher("overlaytext", OverlayText, queue_size=1, latch=True)
pub_overlay_text2 = rospy.Publisher("overlaytext2", OverlayText, queue_size=1, latch=True)
pub_overlay_text3 = rospy.Publisher("overlaytext3", OverlayText, queue_size=1, latch=True)
pub_overlay_text4 = rospy.Publisher("overlaytext4", OverlayText, queue_size=1, latch=True)
pub_overlay_text5 = rospy.Publisher("overlaytext5", OverlayText, queue_size=1, latch=True)
pub_overlay_text6 = rospy.Publisher("overlaytext6", OverlayText, queue_size=1, latch=True)
pub_overlay_text7 = rospy.Publisher("overlaytext7", OverlayText, queue_size=1, latch=True)
pub_overlay_text8 = rospy.Publisher("overlaytext8", OverlayText, queue_size=1, latch=True)

def publish(text):
    send_data = OverlayText()
    send_data.text = text
    pub_overlay_text.publish(send_data)

def publish2(text):
    send_data = OverlayText()
    send_data.text = text
    pub_overlay_text2.publish(send_data)

def publish3(text):
    send_data = OverlayText()
    send_data.text = text
    pub_overlay_text3.publish(send_data)

def publish4(text):
    send_data = OverlayText()
    send_data.text = text
    pub_overlay_text4.publish(send_data)

def publish5(text):
    send_data = OverlayText()
    send_data.text = text
    pub_overlay_text5.publish(send_data)

def publish6(text):
    send_data = OverlayText()
    send_data.text = text
    pub_overlay_text6.publish(send_data)

def publish7(text):
    send_data = OverlayText()
    send_data.text = text
    pub_overlay_text7.publish(send_data)

def publish8(text):
    send_data = OverlayText()
    send_data.text = text
    pub_overlay_text8.publish(send_data)