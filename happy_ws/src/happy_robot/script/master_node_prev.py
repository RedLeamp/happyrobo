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
        self.obb_approval_topic = '/api/obb'
        self.frontLeft_scan_topic = "/cygbot/scan/2d/0"
        self.frontRight_scan_topic = "/cygbot/scan/2d/1"
        self.frontLeft_left_scan_topic = "/cygbot/scan/2d/4"
        self.frontRight_right_scan_topic = "/cygbot/scan/2d/5"
        self.frontLeft_points_topic = "/cygbot/point/3d/0"
        self.frontRight_points_topic = "/cygbot/point/3d/1"
        self.frontLeft_left_points_topic = "/cygbot/point/3d/4"
        self.frontRight_right_points_topic = "/cygbot/point/3d/5"
        self.obb_topic = "/obb"
        self.target_parking_topic= "/target"
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
        self.tf_transformListener = tf.TransformListener()
        self.init_sub = rospy.Subscriber(self.init_topic, Int32, self.init_callback)
        self.parking_complete_sub = rospy.Subscriber(self.parking_complete_topic, Bool, self.parking_complete_callback)
        self.obb_approval_sub = rospy.Subscriber(self.obb_approval_topic, Bool, self.obb_approval_callback)
        self.parking_target_pub = rospy.Publisher(self.target_parking_topic, PolygonStamped, queue_size=10)
        self.clear_map_pub = rospy.Publisher('/clear_map', Bool, queue_size=10)
        self.control_mode_pub = rospy.Publisher('/control_mode', Bool, queue_size=10)
        self.frontLeft_Scan_Sub = Subscriber(self.frontLeft_scan_topic, LaserScan)
        self.frontRight_Scan_Sub = Subscriber(self.frontRight_scan_topic, LaserScan)
        self.frontLeft_Left_Scan_Sub = Subscriber(self.frontLeft_left_scan_topic, LaserScan)
        self.frontRight_Right_Scan_Sub = Subscriber(self.frontRight_right_scan_topic, LaserScan)
        self.frontLeft_Points_Sub = Subscriber(self.frontLeft_points_topic, PointCloud2)
        self.frontRight_Points_Sub = Subscriber(self.frontRight_points_topic, PointCloud2)
        self.frontLeft_Left_Points_Sub = Subscriber(self.frontLeft_left_points_topic, PointCloud2)
        self.frontRight_Right_Points_Sub = Subscriber(self.frontRight_right_points_topic, PointCloud2)
        self.combined_points_pub = rospy.Publisher("/combined/points/3d", PointCloud2, queue_size=10)
        self.ts = ApproximateTimeSynchronizer(
            [
                Subscriber(self.frontLeft_scan_topic, LaserScan),
                Subscriber(self.frontRight_scan_topic, LaserScan),
                Subscriber(self.frontLeft_left_scan_topic, LaserScan),
                Subscriber(self.frontRight_right_scan_topic, LaserScan),
                Subscriber(self.frontLeft_points_topic, PointCloud2),
                Subscriber(self.frontRight_points_topic, PointCloud2),
                Subscriber(self.frontLeft_left_points_topic, PointCloud2),
                Subscriber(self.frontRight_right_points_topic, PointCloud2)
                # self.frontLeft_scan_sub,
                # self.frontRight_scan_sub,
                # self.frontLeft_left_scan_sub,
                # self.frontRight_right_scan_sub,
                # self.frontLeft_points_sub,
                # self.frontRight_points_sub,
                # self.frontLeft_left_points_sub,
                # self.frontRight_right_points_sub,
            ],
            queue_size=10,
            slop=1,
            allow_headerless=False
        )

        # 동기화 콜백 등록
        self.ts.registerCallback(self.synced_callback)
        self.obb_pub = rospy.Publisher(self.obb_topic, PolygonStamped, queue_size=10)
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
        self.obb_approval_condition = Condition()
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

    def obb_approval_callback(self, msg):
        with self.obb_approval_condition:
            rospy.loginfo("Get OBB Approval Signal")
            self.obb_approved_signal = True
            self.obb_approved = msg.data
            rospy.loginfo(f"======= Got OBB Approved Signal : {self.obb_approved} ======")
            self.obb_approval_condition.notify_all()

    def synced_callback(self, fl_scan, fr_scan, fll_scan, frr_scan, fl_points, fr_points, fll_points, frr_points):

        if (not self.obb_seq_init): return
        self.getPointToFrameTF(fl_scan, fr_scan, fll_scan, frr_scan, fl_points, fr_points, fll_points, frr_points)
        if (not self.obb_calculated and self.get_tf_frontLeft_to_base and self.get_tf_frontRight_to_base and self.get_tf_frontLeftLeft_to_base and self.get_tf_frontRightRight_to_base and self.get_tf_base_to_map):
            self.processing(fl_scan, fr_scan, fll_scan, frr_scan, fl_points, fr_points, fll_points, frr_points)
        else : rospy.loginfo("Wait for TF DATA")

    def getPointToFrameTF(self, fl_scan, fr_scan, fll_scan, frr_scan, fl_points, fr_points, fll_points, frr_points):
        if not (self.get_tf_frontLeft_to_base):
            self.tf_frontLeft_to_ref_base_frame, self.get_tf_frontLeft_to_base = self.lookupTransform(fl_scan.header.frame_id, self.ref_base_frame, fl_scan.header.stamp)
        if not (self.get_tf_frontRight_to_base):
            self.tf_frontRight_to_ref_base_frame, self.get_tf_frontRight_to_base = self.lookupTransform(fr_scan.header.frame_id, self.ref_base_frame, fr_scan.header.stamp)
        if not (self.get_tf_frontLeftLeft_to_base):
            self.tf_frontLeft_Left_to_ref_base_frame, self.get_tf_frontLeftLeft_to_base = self.lookupTransform(fll_scan.header.frame_id, self.ref_base_frame, fll_scan.header.stamp)
        if not (self.get_tf_frontRightRight_to_base):
            self.tf_frontRight_Right_to_ref_base_frame, self.get_tf_frontRightRight_to_base = self.lookupTransform(frr_scan.header.frame_id, self.ref_base_frame, frr_scan.header.stamp)
        if not (self.get_tf_base_to_map):
            self.tf_base_frame_to_ref_frame, self.get_tf_base_to_map = self.lookupTransform(self.ref_base_frame, self.ref_frame, frr_scan.header.stamp)


    def lookupTransform(self, source_frame_id, target_frame_id, stamp):
        # tar to src
        kTFTimeout_ms = 1000
        kMsToNs = 1000000
        kTFWait = rospy.Duration(0, 200*kMsToNs)
        try :
            self.tf_transformListener.waitForTransform(target_frame_id, source_frame_id, stamp + kTFWait, rospy.Duration(0, kTFTimeout_ms * kMsToNs))
            tf_frontLeft_to_frontRight = self.tf_transformListener.lookupTransform(target_frame_id, source_frame_id, stamp + kTFWait)
            return tf_frontLeft_to_frontRight, True
        except Exception as e:
            rospy.loginfo("Failed to Lookup Transform : " + str(e))
            return None, False

    def transform_point_to_ref(self, points, transform, header):
        transformed = self.do_transform_cloud(points, transform[0], transform[1], header)
        return transformed

    # PointStamped
    def do_transform_cloud(self, cloud, trans, rot, header):
        t_kdl = PyKDL.Frame(PyKDL.Rotation.Quaternion(rot[0], rot[1], rot[2], rot[3]),
                        PyKDL.Vector(trans[0], trans[1], trans[2]))
        points_out = []
        for p_in in read_points(cloud):
            p_out = t_kdl * PyKDL.Vector(p_in[0], p_in[1], p_in[2])
            points_out.append((p_out[0], p_out[1], p_out[2]) + p_in[3:])
        res = create_cloud(header, cloud.fields, points_out)
        return res
    
    def transform_to_kdl(self, trans, rot):
        return PyKDL.Frame(PyKDL.Rotation.Quaternion(rot[0], rot[1], rot[2], rot[3]), PyKDL.Vector(trans[0], trans[1], trans[2]))
    
    def do_transform_pose(self, trans, rot , pose, header):
        f = self.transform_to_kdl(trans, rot) * PyKDL.Frame(PyKDL.Rotation.Quaternion(pose.pose.orientation.x, pose.pose.orientation.y,
                                                                          pose.pose.orientation.z, pose.pose.orientation.w),
                                                PyKDL.Vector(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z))
        res = PoseStamped()
        res.pose.position.x = f[(0, 3)]
        res.pose.position.y = f[(1, 3)]
        res.pose.position.z = f[(2, 3)]
        (res.pose.orientation.x, res.pose.orientation.y, res.pose.orientation.z, res.pose.orientation.w) = f.M.GetQuaternion()
        res.header = header
        return res
    
    def scan_to_point(self, scan):
        converted_points = laser_geometry.LaserProjection().projectLaser(scan)
        converted_points.header.frame_id = scan.header.frame_id
        return converted_points
    
    def project_scan(self, scan, projected_points):
        for i in range(len(scan.ranges)):
            angle = scan.angle_min + i * scan.angle_increment
            if (scan.ranges[i] != float('inf')):
                projected_points.append(np.array([scan.ranges[i] * np.cos(angle), scan.ranges[i] * np.sin(angle), 0]))
        return projected_points
    
    def project_filtered_points(self, points, projected_points):
        test = pc2.read_points(points)
        cloud = np.array(list(test), dtype=np.float32)
        for point in cloud:
            if (point[2] >= -2.0): # Z -2.0m 이상만
                dist = np.sqrt(point[0]**2 + point[1]**2)
                if (dist <= 10.0 and dist > 0.1): # 10m 이내만
                    angle_deg = np.degrees(np.arctan2(point[1], point[0]))
                    if -90 <= angle_deg <= 90:  # Filter points within -90 to 90 degrees
                        projected_points.append([point[0]*100, point[1]*100])
        return projected_points
    
    def apply_convex_hull(self, points):
        self.convexHullResult = ConvexHull(np.array(points))
        for point in self.convexHullResult.points:
            self.convex_hull_points.append((point[0], point[1]))
        rospy.loginfo("Calculated Convex Hull")
        return self.convex_hull_points
    
    def apply_rotating_calipers(self, convex_hull_points):
        result, scale_up_rect_points = rc.smallest_rectangle(convex_hull_points, rc.compare_area)
        rospy.loginfo("\n\n\n====Calculated Rotating Calipers====\n\n\n")
        self.rect_points = [(point[0]/100, point[1]/100) for point in scale_up_rect_points]

    def merge_pointcloud(self, points1, points2, points3, points4, points5, points6, points7, points8):
        cloud1 = list(pc2.read_points(points1, field_names=("x", "y", "z"), skip_nans=True))
        cloud2 = list(pc2.read_points(points2, field_names=("x", "y", "z"), skip_nans=True))
        cloud3 = list(pc2.read_points(points3, field_names=("x", "y", "z"), skip_nans=True))
        cloud4 = list(pc2.read_points(points4, field_names=("x", "y", "z"), skip_nans=True))
        cloud5 = list(pc2.read_points(points5, field_names=("x", "y", "z"), skip_nans=True))
        cloud6 = list(pc2.read_points(points6, field_names=("x", "y", "z"), skip_nans=True))
        cloud7 = list(pc2.read_points(points7, field_names=("x", "y", "z"), skip_nans=True))
        cloud8 = list(pc2.read_points(points8, field_names=("x", "y", "z"), skip_nans=True))

        # 2. 모든 포인트 클라우드 데이터를 하나로 결합
        merged_points = cloud1 + cloud2 + cloud3 + cloud4 + cloud5 + cloud6 + cloud7 + cloud8

        # 3. numpy 배열로 변환
        merged_points_np = np.array(merged_points, dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32)])

        # 4. 새로운 PointCloud2 메시지 생성
        header = points1.header  # 첫 번째 클라우드의 헤더를 사용 (필요에 따라 수정 가능)
        merged_cloud = pc2.create_cloud_xyz32(header, merged_points_np)

        return merged_cloud

    
    def processing(self, fl_scan, fr_scan, fll_scan, frr_scan, fl_points, fr_points, fll_points, frr_points):

        rospy.loginfo("\n\n===== Init Processing =====\n\n")

        if (self.obb_calculating) : 
            rospy.loginfo("Waiting for OBB Approval Signal...")
            return
        self.obb_calculating = True

        converted_frontLeft_scan_to_Points = self.scan_to_point(fl_scan)
        self.transformed_frontLeft_Scan_to_Points = self.transform_point_to_ref(converted_frontLeft_scan_to_Points, self.tf_frontLeft_to_ref_base_frame, fl_scan.header)
        self.project_filtered_points(self.transformed_frontLeft_Scan_to_Points, self.projected_points)
    
        converted_frontRight_scan_to_Points = self.scan_to_point(fr_scan)
        self.transformed_frontRight_Scan_to_Points = self.transform_point_to_ref(converted_frontRight_scan_to_Points, self.tf_frontRight_to_ref_base_frame, fr_scan.header)
        self.project_filtered_points(self.transformed_frontRight_Scan_to_Points, self.projected_points)

        converted_frontLeft_Left_scan_to_Points = self.scan_to_point(fll_scan)
        self.transformed_frontLeft_Left_Scan_to_Points = self.transform_point_to_ref(converted_frontLeft_Left_scan_to_Points, self.tf_frontLeft_Left_to_ref_base_frame, fll_scan.header)
        self.project_filtered_points(self.transformed_frontLeft_Left_Scan_to_Points, self.projected_points)
    
        converted_frontRight_Right_scan_to_Points = self.scan_to_point(frr_scan)
        self.transformed_frontRight_Right_Scan_to_Points = self.transform_point_to_ref(converted_frontRight_Right_scan_to_Points, self.tf_frontRight_Right_to_ref_base_frame, frr_scan.header)
        self.project_filtered_points(self.transformed_frontRight_Right_Scan_to_Points, self.projected_points)

        self.transformed_frontLeft_points = self.transform_point_to_ref(fl_points, self.tf_frontLeft_to_ref_base_frame, fl_points.header)
        self.project_filtered_points(self.transformed_frontLeft_points, self.projected_points)

        self.transformed_frontRight_points = self.transform_point_to_ref(fr_points, self.tf_frontRight_to_ref_base_frame, fr_points.header)
        self.project_filtered_points(self.transformed_frontRight_points, self.projected_points)

        self.transformed_frontLeft_Left_points = self.transform_point_to_ref(fll_points, self.tf_frontLeft_Left_to_ref_base_frame, fll_points.header)
        self.project_filtered_points(self.transformed_frontLeft_Left_points, self.projected_points)

        self.transformed_frontRight_Right_points = self.transform_point_to_ref(frr_points, self.tf_frontRight_Right_to_ref_base_frame, frr_points.header)
        self.project_filtered_points(self.transformed_frontRight_Right_points, self.projected_points)

        self.merge_pointcloud2_msg = self.merge_pointcloud(self.transformed_frontLeft_Scan_to_Points, self.transformed_frontRight_Scan_to_Points, self.transformed_frontLeft_Left_Scan_to_Points, self.transformed_frontRight_Right_Scan_to_Points,
                                self.transformed_frontLeft_points, self.transformed_frontRight_points, self.transformed_frontLeft_Left_points, self.transformed_frontRight_Right_points)

        self.combined_points_pub.publish(self.merge_pointcloud2_msg)

        rospy.loginfo("\n\n===== apply_convex_hull =====\n\n")
        self.apply_convex_hull(self.projected_points)

        rospy.loginfo("\n\n===== apply_rotating_calipers =====\n\n")
        self.apply_rotating_calipers(self.convex_hull_points)

        rospy.loginfo("Processing Completed")
        alignmment_pose_to_ref_ftame, goal_pose_to_ref_frame = self.get_car_pose()

        rospy.loginfo("Publish Current Goal and estimated Polygon")
        self.publish_polygon()
        self.publish_pose(goal_pose_to_ref_frame.pose, self.ref_frame, self.nav_goal_pose_pub)

        with self.obb_approval_condition:
            rospy.loginfo("Waiting for OBB Approval Signal...")
            while not self.obb_approved_signal:  # self.obb_approved_signal True가 될 때까지 대기
                self.obb_approval_condition.wait()

        if (self.obb_approved) :
            rospy.loginfo("✅ OBB Ready to Go")
            with self.obb_condition:
                self.pose_seq = [self.first_seq_pose, self.second_seq_pose, alignmment_pose_to_ref_ftame.pose, goal_pose_to_ref_frame.pose, alignmment_pose_to_ref_ftame.pose, self.second_seq_pose, self.first_seq_pose, self.init_pose]
                self.whole_seq = len(self.pose_seq)
                rospy.sleep(1)
                self.obb_approved_signal = True
                self.obb_calculating = True
                self.obb_calculated = True
                self.obb_condition.notify_all()
        else :
            rospy.loginfo("❌ OBB not approved, calculates again")
            self.obb_approved_signal = False
            self.obb_calculating = False
            self.obb_calculated = False

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

    def l2_norm (self, vec):
        return np.sqrt(np.sum(vec**2))

    def get_inner_angle(self, vec1, vec2):
        cos_theta = np.inner(vec1, vec2) / (self.l2_norm(vec1) * self.l2_norm(vec2))
        return np.arccos(cos_theta)

    def get_car_orientation(self, center):
        angle_list = []
        vec_list = []
        for point in self.rect_points:
            vec = np.array([point[0] - center[0], point[1] - center[1]])
            vec_list.append(vec)
            angle_list.append(math.degrees(self.get_inner_angle(self.cirterion, vec)))
        asc_sorted_list = sorted(range(len(angle_list)), key=lambda i: angle_list[i])
        car_pose_vec =  np.sum(np.array([vec_list[asc_sorted_list[0]], vec_list[asc_sorted_list[1]]]), axis=0)
        rot_quat = Quaternion(*(quaternion_from_euler(0, 0, self.get_inner_angle(self.cirterion, car_pose_vec), axes='sxyz')))
        
        y_intercept = self.get_y_intercept(car_pose_vec, center)
        alignment_pose_vec = np.array([0, y_intercept])
        goal_pose_vec = np.array([center[0], center[1]])

        alignmment_pose_to_ref_ftame = self.transform_car_coord_to_ref_frame(alignment_pose_vec, rot_quat)
        goal_pose_to_ref_frame = self.transform_car_coord_to_ref_frame(goal_pose_vec, rot_quat)

        return alignmment_pose_to_ref_ftame, goal_pose_to_ref_frame
    
    def transform_car_coord_to_ref_frame(self, trans, rot):

        pose = PoseStamped()
        pose.header.frame_id = self.ref_frame
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = trans[0]
        pose.pose.position.y = trans[1]
        pose.pose.position.z = 0
        pose.pose.orientation = rot
        transformed_pose_to_ref_frame = self.do_transform_pose(self.tf_base_frame_to_ref_frame[0], self.tf_base_frame_to_ref_frame[1], pose, pose.header)
        return transformed_pose_to_ref_frame

    
    def get_y_intercept(self, vec, center):
        return -(vec[1] / vec[0]) * (center[0]) + (center[1])

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
        
    def publish_polygon(self):
        ros_publisher_points = Polygon()
        for point in self.rect_points:
            polygon_point = Point32(point[0], point[1], 0)
            ros_publisher_points.points.append(polygon_point)
        polygonStamped = PolygonStamped()
        polygonStamped.polygon = ros_publisher_points
        polygonStamped.header.frame_id = self.ref_base_frame
        polygonStamped.header.stamp = rospy.Time.now()
        self.obb_pub.publish(polygonStamped)
        rospy.loginfo("Published OBB")
        return
    
    def publish_transformed_polygon(self):
        ros_publisher_points = Polygon()
        for point in self.rect_points:
            polygon_point = Point32(point[0]+self.second_x, point[1]+self.second_y, 0)
            ros_publisher_points.points.append(polygon_point)
        polygonStamped = PolygonStamped()
        polygonStamped.polygon = ros_publisher_points
        polygonStamped.header.frame_id = self.ref_frame
        polygonStamped.header.stamp = rospy.Time.now()
        self.obb_pub.publish(polygonStamped)
        rospy.loginfo("Published OBB")
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
