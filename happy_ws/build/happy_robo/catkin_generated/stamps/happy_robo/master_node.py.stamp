#! /usr/bin/env python3

from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
import rotating_calipers as rc
import numpy as np
import rospy
import math
import os
from geometry_msgs.msg import Pose, Point, Quaternion
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import tf
import tf2_geometry_msgs
from sensor_msgs.point_cloud2 import read_points, create_cloud
import PyKDL
import laser_geometry
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseAction, MoveBaseGoal
# from myactionlib import SimpleActionClient
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from std_srvs.srv import Empty
from threading import Condition
from dynamic_reconfigure.client import Client
import tf.transformations as tf_trans
import threading

from actionlib_msgs.msg import GoalStatus
from actionlib.action_client import ActionClient, CommState, get_name_of_constant

from message_filters import Subscriber, ApproximateTimeSynchronizer
from happy_robo.msg import AlignAndGoalPoseStamped

class SimpleGoalState:
    PENDING = 0
    ACTIVE = 1
    DONE = 2


SimpleGoalState.to_string = classmethod(get_name_of_constant)

class SimpleActionClient:

    def __init__(self, ns, ActionSpec):
        self.action_client = ActionClient(ns, ActionSpec)
        self.simple_state = SimpleGoalState.DONE
        self.gh = None
        self.done_condition = threading.Condition()

    def wait_for_server(self, timeout=rospy.Duration()):
        return self.action_client.wait_for_server(timeout)

    def send_goal(self, goal, done_cb=None, active_cb=None, feedback_cb=None):
        # destroys the old goal handle
        self.stop_tracking_goal()

        self.done_cb = done_cb
        self.active_cb = active_cb
        self.feedback_cb = feedback_cb

        self.simple_state = SimpleGoalState.PENDING
        self.gh = self.action_client.send_goal(goal, self._handle_transition, self._handle_feedback)

    def send_goal_and_wait(self, goal, execute_timeout=rospy.Duration(), preempt_timeout=rospy.Duration()):
        self.send_goal(goal)
        if not self.wait_for_result(execute_timeout):
            # preempt action
            rospy.logdebug("Canceling goal")
            self.cancel_goal()
            if self.wait_for_result(preempt_timeout):
                rospy.logdebug("Preempt finished within specified preempt_timeout [%.2f]", preempt_timeout.to_sec())
            else:
                rospy.logdebug("Preempt didn't finish specified preempt_timeout [%.2f]", preempt_timeout.to_sec())
        return self.get_state()

    def wait_for_result(self, timeout=rospy.Duration()):
        if not self.gh:
            rospy.logerr("Called wait_for_result when no goal exists")
            return False

        timeout_time = rospy.get_rostime() + timeout
        loop_period = rospy.Duration(0.1)
        with self.done_condition:
            while not rospy.is_shutdown():
                time_left = timeout_time - rospy.get_rostime()
                if timeout > rospy.Duration(0.0) and time_left <= rospy.Duration(0.0):
                    break

                if self.simple_state == SimpleGoalState.DONE:
                    break

                if time_left > loop_period or timeout == rospy.Duration():
                    time_left = loop_period

                self.done_condition.wait(time_left.to_sec())

        return self.simple_state == SimpleGoalState.DONE

    def get_result(self):
        if not self.gh:
            rospy.logerr("Called get_result when no goal is running")
            return None

        return self.gh.get_result()

    def get_state(self):
        if not self.gh:
            return GoalStatus.LOST
        status = self.gh.get_goal_status()

        if status == GoalStatus.RECALLING:
            status = GoalStatus.PENDING
        elif status == GoalStatus.PREEMPTING:
            status = GoalStatus.ACTIVE

        return status
    
    def get_goal_status_text(self):
        if not self.gh:
            rospy.logerr("Called get_goal_status_text when no goal is running")
            return "ERROR: Called get_goal_status_text when no goal is running"

        return self.gh.get_goal_status_text()
    
    def cancel_all_goals(self):
        self.action_client.cancel_all_goals()

    def cancel_goals_at_and_before_time(self, time):
        self.action_client.cancel_goals_at_and_before_time(time)

    def cancel_goal(self):
        if self.gh:
            self.gh.cancel()

    def stop_tracking_goal(self):
        self.gh = None

    def _handle_transition(self, gh):

        if gh != self.gh:
            rospy.logerr("Got a transition callback on a goal handle that we're not tracking")
            return

        comm_state = gh.get_comm_state()

        error_msg = "Received comm state %s when in simple state %s with SimpleActionClient in NS %s" % \
            (CommState.to_string(comm_state), SimpleGoalState.to_string(self.simple_state), rospy.resolve_name(self.action_client.ns))

        print("comm_state : ", CommState.to_string(comm_state))
        print("SimpleGoalState : ", SimpleGoalState.to_string(self.simple_state))
        if comm_state == CommState.ACTIVE:
            if self.simple_state == SimpleGoalState.PENDING:
                self._set_simple_state(SimpleGoalState.ACTIVE)
                if self.active_cb:
                    self.active_cb()
            elif self.simple_state == SimpleGoalState.DONE:
                rospy.logerr(error_msg)
        elif comm_state == CommState.RECALLING:
            if self.simple_state != SimpleGoalState.PENDING:
                rospy.logerr(error_msg)
        elif comm_state == CommState.PREEMPTING:
            if self.simple_state == SimpleGoalState.PENDING:
                self._set_simple_state(SimpleGoalState.ACTIVE)
                if self.active_cb:
                    self.active_cb()
            elif self.simple_state == SimpleGoalState.DONE:
                rospy.logerr(error_msg)
        elif comm_state == CommState.DONE:
            # if self.simple_state in [SimpleGoalState.PENDING, SimpleGoalState.ACTIVE]:
            if self.done_cb:
                self.done_cb(gh.get_goal_status(), gh.get_result())
            with self.done_condition:
                self._set_simple_state(SimpleGoalState.DONE)
                self.done_condition.notifyAll()

    def _handle_feedback(self, gh, feedback):
        if not self.gh:
            return
        if gh != self.gh:
            rospy.logerr("Got a feedback callback on a goal handle that we're not tracking. %s vs %s" %
                         (self.gh.comm_state_machine.action_goal.goal_id.id, gh.comm_state_machine.action_goal.goal_id.id))
            return
        if self.feedback_cb:
            self.feedback_cb(feedback)

    def _set_simple_state(self, state):
        self.simple_state = state


class MasterNode:
    def __init__(self):
        self.init_topic = '/api/sector'
        self.parking_complete_topic = '/api/park'
        self.target_parking_topic= "/target"
        self.obb_complete_topic = '/obb/goal'
        self.target_sector = 0
        self.projected_points = []
        self.archieve_points = [False, False]
        self.archieve_scan = [False, False]
        self.convex_hull_points = []
        self.rect_points = []
        self.goal_points = []
        self.get_tf_frontLeft_to_base = False
        self.get_tf_frontRight_to_base = False
        self.get_tf_frontLeftLeft_to_base = False
        self.get_tf_frontRightRight_to_base = False
        self.get_tf_base_to_map = False
        self.finish_cal = False
        self.init_sub = rospy.Subscriber(self.init_topic, Int32, self.init_callback)
        self.parking_complete_sub = rospy.Subscriber(self.parking_complete_topic, Bool, self.parking_complete_callback)
        self.obb_complete_sub = rospy.Subscriber(self.obb_complete_topic, AlignAndGoalPoseStamped, self.obb_complete_callback)
        self.obb_init_pub = rospy.Publisher('/obb/init', Bool, queue_size=10)
        self.parking_target_pub = rospy.Publisher(self.target_parking_topic, PolygonStamped, queue_size=10)
        self.clear_map_pub = rospy.Publisher('/clear_map', Bool, queue_size=10)
        self.control_mode_pub = rospy.Publisher('/control_mode', Bool, queue_size=10)
        self.nav_goal_pub = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size=1)
        self.nav_first_pose_pub = rospy.Publisher("/parking/first", PoseStamped, queue_size=1)
        self.nav_second_pose_pub = rospy.Publisher("/parking/second", PoseStamped, queue_size=1)
        self.nav_align_pose_pub = rospy.Publisher("/parking/align", PoseStamped, queue_size=1)
        self.nav_goal_pose_pub = rospy.Publisher("/parking/goal", PoseStamped, queue_size=1)
        self.final_goal_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.mode_pub = rospy.Publisher('/run_mode', Int32, queue_size=10)
        self.width = 2.5
        self.length = 5.7
        self.x_error = 0.0
        self.y_error = 0.0
        self.hori_alignment = True
        self.vert_alignment = True
        self.cirterion = np.array([1, 0])
        self.client = SimpleActionClient('move_base',MoveBaseAction)
        self.goal_cnt = 0
        self.final_goal = Pose()
        self.pose_error_goal_to_current = Pose()
        self.ref_frame = "map"
        self.ref_base_frame = "base_link"
        self.init_x = 0
        self.init_y = 0
        self.first_x = 8.5
        self.first_y = 0
        self.second_x = 8.5
        self.second_y = 10
        self.init_PolyGon = PolygonStamped()
        self.init_orientation = Quaternion(*(quaternion_from_euler(0, 0, 0, axes='sxyz')))
        self.init_pose = Pose(Point(self.init_x, self.init_y, 0), self.init_orientation)
        self.first_seq_pose = Pose(Point(self.first_x, self.first_y, 0), self.init_orientation)
        self.second_seq_pose = Pose(Point(self.second_x, self.second_y, 0), self.init_orientation)
        self.pose_seq = [self.first_seq_pose, self.second_seq_pose]
        self.frame_seq = [self.ref_frame, self.ref_frame]
        self.vertical_mode = [2.2, -2.2, 0.0, 0.0, 2.5, 0.0]
        self.horizontal_mode = [0, 0, 2.2, -2.2, 0.0, 2.5]
        self.mode_param = [True, False]
        self.prev_mode = True
        self.obb_seq_init = False
        self.init = False # Sector Decision API
        self.parking_complete = False # Parking Complete API
        self.obb_calculated = False
        self.obb_approved_signal = False
        self.obb_calculating = False
        self.obb_approved = False
        self.obb_condition = Condition()
        self.init_condition = Condition()
        self.parking_complete_condition = Condition()
        self.whole_seq = 8
        self.feedback_pose = Pose()
        self.cal_car_obb_seq = 2
        self.clear_map_seq = 3
        self.retrieve_seq = 4
        self.max_vel = 2.2
        self.test = True

    def initialize(self):

        rospy.loginfo("Initialize Master Node !")
        
        with self.init_condition:
            rospy.loginfo("Waiting for Init Signal....")
            while not self.init:  # self.calculate가 True가 될 때까지 대기
                self.init_condition.wait()
        rospy.loginfo("Nav Seq Init, wait for 1 seconds")
        rospy.sleep(1)
        self.goal_cnt = 0
        wait = self.client.wait_for_server(rospy.Duration(60.0))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")
        self.init_movebase_client()

    def set_lidar_mode(self, mode):
        msg = Int32()
        msg.data = mode
        self.mode_pub.publish(msg)
        rospy.loginfo("Published run_mode: %d (0: 2D, 1: 3D, 2: Dual)", mode)
        rospy.sleep(1)

    def init_callback(self, msg):
        with self.init_condition:
            self.target_sector = msg.data
            self.init = True
            self.publish_parking_target_polygon()
            rospy.loginfo(f"======= Got Sector Data : {self.target_sector} ======")
            self.init_condition.notify_all()

    def parking_complete_callback(self, msg):
        with self.parking_complete_condition:
            self.parking_complete = msg.data
            rospy.loginfo(f"======= Got Parking Complete Signal : {self.parking_complete} ======")
            self.parking_complete_condition.notify_all()

    def obb_complete_callback(self, msg):
        rospy.loginfo("✅ OBB Complete")
        with self.obb_condition:
            self.pose_seq = [self.first_seq_pose, self.second_seq_pose, msg.align_pose.pose, msg.goal_pose.pose, msg.align_pose.pose, self.second_seq_pose, self.first_seq_pose, self.init_pose]
            self.obb_calculated = True
            self.obb_condition.notify_all()

    def active_cb(self):
        rospy.loginfo("Goal pose "+str(self.goal_cnt+1)+" is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        self.feedback_pose = feedback.base_position.pose
        # rospy.loginfo("Feedback for goal pose "+str(self.goal_cnt)+" ERROR : " + str(self.feedback_pose))

    def done_cb(self, status, result):
    
        self.goal_cnt += 1

        if (self.goal_cnt == self.retrieve_seq) :
            rospy.loginfo("Wait For Parking Complete Signal.....")
            with self.parking_complete_condition:
                        while not self.parking_complete:  # self.parking_complete True가 될 때까지 대기
                            self.parking_complete_condition.wait()
        if status == 2:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request after it started executing, completed execution!")

        if status == 3:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached") 
            if self.goal_cnt < self.whole_seq:
                if self.goal_cnt == self.cal_car_obb_seq:
                    self.set_lidar_mode(2)
                    rospy.loginfo("✅ Convert Mode to 2D 3D Dual")
                    rospy.sleep(5)
                    self.obb_seq_init = True
                    rospy.loginfo("Waiting for new goals to be calculated...")
                    rospy.sleep(1)
                    with self.obb_condition:
                        self.obb_init_pub.publish(Bool(True))
                        rospy.loginfo("Send OBB Init Signal")
                        rospy.sleep(0.001)  
                        while not self.obb_calculated:  # self.calculate가 True가 될 때까지 대기
                            self.obb_condition.wait()
                elif self.goal_cnt == self.clear_map_seq:
                    self.set_marking(False)
                    self.set_shutdown_costmaps(True)
                    rospy.loginfo("Clearing Map")
                    rospy.sleep(1)
                elif self.goal_cnt == self.retrieve_seq:
                    self.set_lidar_mode(0)
                    rospy.loginfo("✅ Convert Mode to 2D Only")
                    rospy.sleep(5)
                    self.set_marking(True)
                    rospy.loginfo("Regenerating Map")
                    rospy.sleep(1)
                next_goal = MoveBaseGoal()
                next_goal.target_pose.header.frame_id = self.ref_frame
                next_goal.target_pose.header.stamp = rospy.Time.now()
                next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
                self.publish_pose(self.pose_seq[self.goal_cnt], self.ref_frame, self.nav_align_pose_pub)
                rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
                rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
                self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb) 
            else:
                rospy.loginfo("Final goal pose reached!")
                rospy.signal_shutdown("Final goal pose reached!")

        if status == 4:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached, aborting previous mission") 
            if self.goal_cnt == self.cal_car_obb_seq:
                self.obb_seq_init = True
                rospy.loginfo("Waiting for new goals to be calculated...")
                rospy.sleep(1)
                with self.obb_condition:
                    self.obb_init_pub.publish(Bool(True))
                    rospy.loginfo("Send OBB Init Signal")
                    rospy.sleep(0.001)
                    while not self.obb_calculated:  # self.calculate가 True가 될 때까지 대기
                        self.obb_condition.wait()
            elif self.goal_cnt == self.clear_map_seq:
                self.set_marking(False)
                self.set_shutdown_costmaps(True)
                rospy.loginfo("Clearing Map")
                rospy.sleep(1)
            elif self.goal_cnt == self.retrieve_seq:
                self.set_marking(True)
                rospy.loginfo("Regenerating Map")
                rospy.sleep(1)
            next_goal = MoveBaseGoal()
            next_goal.target_pose.header.frame_id = self.ref_frame
            next_goal.target_pose.header.stamp = rospy.Time.now()
            next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
            self.publish_pose(self.pose_seq[self.goal_cnt], self.ref_frame, self.nav_align_pose_pub)
            rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
            rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
            self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb) 
            # else:
            #     rospy.loginfo("Goal pose "+str(self.goal_cnt)+" was aborted by the Action Server")
            #     rospy.logwarn("Abort ROS for safety")
            #     rospy.signal_shutdown("Abort ROS for safety")
            #     return

        if status == 5:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" has been rejected by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" rejected, shutting down!")
            return

        if status == 8:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request before it started executing, successfully cancelled!")


    def init_movebase_client(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.ref_frame
        goal.target_pose.header.stamp = rospy.Time.now() 
        goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        self.publish_pose(self.pose_seq[self.goal_cnt], self.ref_frame, self.nav_align_pose_pub)
        rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
        rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        rospy.spin()

    def get_car_pose(self):
        sum_x = 0.0
        sum_y = 0.0
        for point in self.rect_points:
            sum_x += point[0]
            sum_y += point[1]
        center = [sum_x/4, sum_y/4]
        alignmment_pose_to_ref_frame, goal_pose_to_ref_frame = self.get_car_orientation(center)
        return alignmment_pose_to_ref_frame, goal_pose_to_ref_frame

    def publish_pose(self, pose, frame, publisher):
        poseStamped = PoseStamped()
        poseStamped.header.stamp = rospy.Time.now()
        poseStamped.pose = pose
        poseStamped.header.frame_id = frame
        publisher.publish(poseStamped)
        rospy.loginfo("Published Pose")

    def publish_parking_target_polygon(self):
        ros_publisher_points = Polygon()

        polygon_point = Point32(13, 9, 0)
        ros_publisher_points.points.append(polygon_point)

        polygon_point = Point32(17, 9, 0)
        ros_publisher_points.points.append(polygon_point)

        polygon_point = Point32(17, 11, 0)
        ros_publisher_points.points.append(polygon_point)

        polygon_point = Point32(13, 11, 0)
        ros_publisher_points.points.append(polygon_point)
        polygonStamped = PolygonStamped()
        polygonStamped.polygon = ros_publisher_points
        polygonStamped.header.frame_id = self.ref_frame
        polygonStamped.header.stamp = rospy.Time.now()
        self.parking_target_pub.publish(polygonStamped)
        rospy.loginfo("Published Target Parking Space")
        return

    
    def set_shutdown_costmaps(self, value):

        rospy.loginfo("============ Stop Publishing Front Scan ============")
        rospy.wait_for_service('/move_base/clear_costmaps')
        try:
            clear_costmaps_service = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
            response = clear_costmaps_service()
            rospy.loginfo("Costmap cleared successfully.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
        rospy.sleep(1)

    def set_marking(self, value):

        rospy.loginfo("============ Set Marking Value ============")
        
        clientLocalCostmapMark = Client("/move_base/global_costmap/obstacles_layer_mark", timeout=10)
        clientGlobalCostmapMark = Client("/move_base/local_costmap/obstacles_layer_mark", timeout=10)

        # costmap_2d의 dynamic_reconfigure 클라이언트 생성
        clientLocalCostmapNoMark = Client("/move_base/global_costmap/obstacles_layer_no_mark", timeout=10)
        clientGlobalCostmapNoMark = Client("/move_base/local_costmap/obstacles_layer_no_mark", timeout=10)

        # marking 값을 설정
        params = {"enabled": value}
        clientLocalCostmapMark.update_configuration(params)
        clientGlobalCostmapMark.update_configuration(params)
        rospy.sleep(1)

        # marking 값을 설정
        params = {"enabled": not value}
        clientLocalCostmapNoMark.update_configuration(params)
        clientGlobalCostmapNoMark.update_configuration(params)

        rospy.loginfo(f"✅ global costmap marking 값을 {value} 으로 변경했습니다.")
        rospy.loginfo(f"✅ local costmap marking 값을 {value} 으로 변경했습니다.")
        rospy.sleep(1)
    

if __name__ == '__main__':


    rospy.init_node("init_master_node")
    rospy.loginfo("init_master_node")
    try :
        masterNode = MasterNode()
        masterNode.initialize()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted.")
