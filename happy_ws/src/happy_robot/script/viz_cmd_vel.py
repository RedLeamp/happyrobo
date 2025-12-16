#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion
import tf.transformations as tf

class CmdVelPoseStampedPublisher:
    def __init__(self):
        rospy.init_node("cmd_vel_pose_stamped_publisher", anonymous=True)

        # Publisher
        self.pose_pub = rospy.Publisher("/cmd_vel_pose", PoseStamped, queue_size=1)

        # Subscriber
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)

        # 내부 상태
        self.last_yaw = 0.0
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_time = rospy.Time.now()

        rospy.loginfo("✅ CmdVelPoseStampedPublisher node started (geometry_msgs/PoseStamped).")

        rospy.spin()

    def cmd_vel_callback(self, msg: Twist):
        # 현재 시간 및 dt 계산
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time
        if dt <= 0.0:
            return

        # 선속도 및 각속도 추출
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        self.last_x += vx * dt
        self.last_y += vy * dt

        # yaw 적분 (angular.z 반영)
        self.last_yaw += wz * dt
        self.last_yaw = math.atan2(math.sin(self.last_yaw), math.cos(self.last_yaw))  # -pi~pi 정규화

        # PoseStamped 메시지 생성
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "map"

        # position (항상 원점, z는 속도 크기 시각화용)
        pose_stamped.pose.position.x = self.last_x
        pose_stamped.pose.position.y = self.last_y
        pose_stamped.pose.position.z = 0

        # orientation (yaw를 쿼터니언으로 변환)
        q = tf.quaternion_from_euler(0, 0, self.last_yaw)
        pose_stamped.pose.orientation = Quaternion(*q)

        # 퍼블리시
        self.pose_pub.publish(pose_stamped)

        rospy.loginfo_throttle(1.0,
            f"PoseStamped -> vx={vx:.2f}, vy={vy:.2f}, wz={wz:.2f}, yaw={math.degrees(self.last_yaw):.1f}°"
        )


if __name__ == "__main__":
    try:
        CmdVelPoseStampedPublisher()
    except rospy.ROSInterruptException:
        pass
