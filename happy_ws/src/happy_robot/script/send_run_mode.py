#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
import os

def set_mode(mode):
    rospy.init_node('set_mode_publisher', anonymous=True)
    pub = rospy.Publisher('/run_mode', Int32, queue_size=10)
    rate = rospy.Rate(1)  # 1Hz
    msg = Int32()
    msg.data = mode
    rospy.sleep(1)  # 퍼블리셔 초기화 대기
    pub.publish(msg)
    rospy.loginfo("Published run_mode: %d", mode)
    rate.sleep()

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            mode = int(input("Enter run_mode (0: 2D, 1: 3D, 2: Dual): "))
            if (mode == 'q') : os._exit(0)
            else : set_mode(mode)
    except rospy.ROSInterruptException:
        pass