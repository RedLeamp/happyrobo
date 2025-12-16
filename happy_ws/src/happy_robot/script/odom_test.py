#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
import math

class OdometryCalculator:
    def __init__(self):
        # Initialize node
        rospy.init_node('cmd_vel_to_odometry')

        # Subscribe to /cmd_vel
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

        # Publisher for odometry
        self.odom_pub = rospy.Publisher('/odom_py', Odometry, queue_size=10)

        # Initialize robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Set the update rate
        # self.rate = rospy.Rate(10)  # 10 Hz

        # Initialize last time
        self.last_time = rospy.Time.now()

    def cmd_vel_callback(self, msg):
        # Update the robot's state based on /cmd_vel
        # rospy.loginfo("Received a /cmd_vel message!")
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()

        # Extract linear and angular velocities
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        # Compute change in position and orientation
        delta_x = linear_velocity * math.cos(self.theta) * dt
        delta_y = linear_velocity * math.sin(self.theta) * dt
        delta_theta = angular_velocity * dt

        # Update state
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Normalize theta to [-pi, pi]
        # self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Publish the odometry message
        self.publish_odometry(current_time)

        # Update the last time
        self.last_time = current_time

    def publish_odometry(self, current_time):
        # Create Odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # Set position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Convert theta to quaternion
        quat = quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]

        # Set velocity
        odom.twist.twist.linear.x = 0.0  # Replace with real linear velocity if needed
        odom.twist.twist.angular.z = 0.0  # Replace with real angular velocity if needed

        # Publish odometry
        self.odom_pub.publish(odom)
        # rospy.loginfo("Odometry published!")
        rospy.loginfo(odom)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        odom_calculator = OdometryCalculator()
        odom_calculator.run()
    except rospy.ROSInterruptException:
        pass
