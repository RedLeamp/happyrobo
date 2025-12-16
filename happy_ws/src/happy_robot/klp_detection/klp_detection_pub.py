#! /usr/bin/env python3
import requests
import json
from glob import glob
import torch
import easyocr
import numpy as np
import cv2
from PIL import ImageFont, ImageDraw, Image
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
PATH = os.getcwd() + '/src/happy_robot/klp_detection'
os.chdir(PATH)
        
if __name__ == '__main__':

    rospy.init_node("init_klp_detection_pub")
    rospy.loginfo("init_klp_detection_pub")
    image = cv2.imread("klp_01.jpeg")
    bridge = CvBridge()
    image_message = bridge.cv2_to_imgmsg(image, encoding="passthrough")
    pub = rospy.Publisher('/rgb/image_raw', Image, queue_size=1)
    loop_rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub.publish(image_message)
        rospy.loginfo("pub")
        loop_rate.sleep
