#!/usr/bin/env python
import rospy
import tf
import math
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatus

class CustomPathPlanner:
    def __init__(self):
        rospy.init_node('custom_path_planner', anonymous=True)
        
        # 파라미터 설정
        self.linear_speed = rospy.get_param('~linear_speed', 0.3)  # m/s
        self.angular_speed = rospy.get_param('~angular_speed', 0.5)  # rad/s
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.2)  # m
        self.angular_tolerance = rospy.get_param('~angular_tolerance', 0.1)  # rad
        
        # 현재 위치와 목표
        self.current_pose = None
        self.goal_pose = None
        self.goal_reached = False
        
        # 퍼블리셔와 서브스크라이버
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.goal_status_pub = rospy.Publisher('/move_base/status', GoalStatus, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        
        # TF 리스너
        self.tf_listener = tf.TransformListener()
        
        # PID 제어 파라미터
        self.kp_linear = 0.5
        self.kp_angular = 1.0

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def goal_callback(self, msg):
        if self.current_pose is None:
            rospy.logwarn("Current pose not available yet")
            return
        
        # 목표 프레임이 map인지 확인
        if msg.header.frame_id != "map":
            rospy.logwarn("Goal must be in 'map' frame")
            return
        
        self.goal_pose = msg.pose
        self.goal_reached = False
        rospy.loginfo("Received new goal: x=%.2f, y=%.2f", 
                      self.goal_pose.position.x, self.goal_pose.position.y)

    def compute_distance(self):
        if self.current_pose is None or self.goal_pose is None:
            return float('inf')
        
        dx = self.goal_pose.position.x - self.current_pose.position.x
        dy = self.goal_pose.position.y - self.current_pose.position.y
        return math.sqrt(dx**2 + dy**2)

    def compute_angle(self):
        if self.current_pose is None or self.goal_pose is None:
            return 0.0
        
        dx = self.goal_pose.position.x - self.current_pose.position.x
        dy = self.goal_pose.position.y - self.current_pose.position.y
        target_yaw = math.atan2(dy, dx)
        
        # 현재 yaw 계산
        quaternion = (
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w
        )
        _, _, current_yaw = tf.transformations.euler_from_quaternion(quaternion)
        
        # 각도 차이
        angle_diff = target_yaw - current_yaw
        # -pi ~ pi 범위로 정규화
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
            
        return angle_diff

    def run(self):
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            if self.goal_pose is None or self.current_pose is None:
                rate.sleep()
                continue
                
            # 목표까지 거리 계산
            distance = self.compute_distance()
            if distance < self.goal_tolerance:
                self.goal_reached = True
                cmd_vel = Twist()
                self.cmd_vel_pub.publish(cmd_vel)
                status = GoalStatus()
                status.status = GoalStatus.SUCCEEDED
                status.text = "Goal reached"
                self.goal_status_pub.publish(status)
                rospy.loginfo("Goal reached!")
                self.goal_pose = None
                rate.sleep()
                continue
                
            # 속도 명령 생성
            cmd_vel = Twist()
            
            # 선형 속도
            cmd_vel.linear.x = min(self.kp_linear * distance, self.linear_speed)
            
            # 각속도
            angle_diff = self.compute_angle()
            if abs(angle_diff) > self.angular_tolerance:
                cmd_vel.angular.z = min(self.kp_angular * angle_diff, self.angular_speed)
                cmd_vel.linear.x = 0.0  # 회전 중에는 직진하지 않음
            else:
                cmd_vel.angular.z = 0.0
                
            self.cmd_vel_pub.publish(cmd_vel)
            rate.sleep()

if __name__ == '__main__':
    try:
        planner = CustomPathPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass