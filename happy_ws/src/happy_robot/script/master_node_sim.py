#! /usr/bin/env python3

from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
import rotating_calipers as rc
import numpy as np
import rospy
import math
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


class OBBCalculator:
    def __init__(self):
        self.init_topic = '/api/sector'
        self.parking_complete_topic = '/api/sector'
        self.frontLeft_scan_topic = "/cygbot/scan/2d/0"
        self.frontRight_scan_topic = "/cygbot/scan/2d/1"
        self.frontLeft_points_topic = "/sigbot/point01"
        self.frontRight_points_topic = "/sigbot/point02"
        # self.frontScan_topic = "/front/scan"
        self.obb_topic = "/obb"
        self.target_parking_topic= "/target"
        self.target_sector = 0
        self.projected_points = []
        self.archieve_points = [False, False]
        self.archieve_scan = [False, False]
        self.convex_hull_points = []
        self.rect_points = []
        self.goal_points = []
        self.get_tf_1 = False
        self.get_tf_2 = False
        self.get_tf_3 = False
        self.get_tf_4 = False
        self.get_tf_5 = False
        self.tf_transformListener = tf.TransformListener()
        self.init_sub = rospy.Subscriber(self.init_topic, Int32, self.init_callback)
        self.parking_complete_sub = rospy.Subscriber(self.parking_complete_topic, Bool, self.parking_complete_callback)
        self.parking_target_pub = rospy.Publisher(self.target_parking_topic, PolygonStamped, queue_size=10)
        self.clear_map_pub = rospy.Publisher('/clear_map', Bool, queue_size=10)
        self.control_mode_pub = rospy.Publisher('/control_mode', Bool, queue_size=10)
        self.frontLeft_Scan_Sub = rospy.Subscriber(self.frontLeft_scan_topic, LaserScan, self.frontLeft_scan_callback)
        self.frontRight_Scan_Sub = rospy.Subscriber(self.frontRight_scan_topic, LaserScan, self.frontRight_scan_callback)
        self.frontLeft_Points_Sub = rospy.Subscriber(self.frontLeft_points_topic, PointCloud2, self.frontLeft_point_callback)
        self.frontRight_Points_Sub = rospy.Subscriber(self.frontRight_points_topic, PointCloud2, self.frontRight_point_callback)
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
        self.first_x = 10
        self.first_y = 0
        self.second_x = 10
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
        self.init = True # Sector Decision API
        self.parking_complete = False # Parking Complete API
        self.calculated = False
        self.calculate_condition = Condition()
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

    def frontLeft_scan_callback(self, msg):
        if (self.obb_seq_init) :
            self.frontLeft_scan = msg
            self.archieve_scan[0] = True
            # rospy.loginfo("[1-1] Got Front Left Scan")
            self.obb_seq_init = False

    def frontRight_scan_callback(self, msg):
        if (self.archieve_scan[0]) :
            self.frontRight_scan = msg
            self.archieve_scan[1] = True
            # rospy.loginfo("[1-2] Got Front Right Scan")
            self.archieve_scan[0] = False

    def frontLeft_point_callback(self, msg):
        if (self.archieve_scan[1]) :
            self.frontLeft_points = msg 
            self.archieve_points[0] = True
            # rospy.loginfo("[1-3] Got Front Left Point")
            self.archieve_scan[1] = False

    def frontRight_point_callback(self, msg):
        if (self.archieve_points[0]) :
            self.frontRight_points = msg
            self.archieve_points[1] = True
            # rospy.loginfo("[1-4] Got Front Right Point")
            if not (self.get_tf_1):
                self.tf_frontLeft_Scan_to_ref_base_frame, self.get_tf_1 = self.lookupTransform(self.frontLeft_scan.header.frame_id, self.ref_base_frame, self.frontLeft_scan.header.stamp)
            if not (self.get_tf_2):
                self.tf_frontRight_Scan_to_ref_base_frame, self.get_tf_2 = self.lookupTransform(self.frontRight_scan.header.frame_id, self.ref_base_frame, self.frontRight_scan.header.stamp)
            if not (self.get_tf_3):
                self.tf_frontLeft_Points_to_ref_base_frame, self.get_tf_3 = self.lookupTransform(self.frontLeft_points.header.frame_id, self.ref_base_frame, self.frontLeft_points.header.stamp)
            if not (self.get_tf_4):
                self.tf_frontRight_Points_to_ref_base_frame, self.get_tf_4 = self.lookupTransform(self.frontRight_points.header.frame_id, self.ref_base_frame, self.frontRight_points.header.stamp)
            if not (self.get_tf_5):
                self.tf_base_frame_to_ref_frame, self.get_tf_5 = self.lookupTransform(self.ref_base_frame, self.ref_frame, self.frontRight_points.header.stamp)
            if (self.get_tf_1 and self.get_tf_2 and self.get_tf_3 and self.get_tf_4):
                rospy.loginfo("Ready to Process")
                self.archieve_points[0] = False
                import threading
                threading.Thread(target=self.processing).start()
            else : rospy.loginfo("Wait for TF DATA")

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
            if (point[2] >= 0.00): # Z 0.01m 이상만
                if (np.sqrt(point[0]**2 + point[1]**2) <= 10.0): # 12m 이내만
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

    def update_dwa_params(self, is_vertical):
        if (not self.test) :
            try:
                # dynamic_reconfigure 클라이언트 생성
                client = Client("/move_base/DWAPlannerROS", timeout=10)
                if (self.prev_mode == is_vertical) : return
                if (self.prev_mode and not is_vertical) : # prev : vertical, current : horizontal
                    self.control_mode_pub.publish(Bool(data=False))
                    rospy.loginfo("Changed to Horizontal Mode")
                    mode_param = self.horizontal_mode
                elif (not self.prev_mode and is_vertical) : # prev : horizontal, current : vertical
                    self.control_mode_pub.publish(Bool(data=True))
                    rospy.loginfo("Changed to Vertical Mode")
                    mode_param = self.vertical_mode
                rospy.loginfo("Waiting for Mode Conversion..")
                rospy.sleep(1)
                self.prev_mode = is_vertical
                # 매개변수 업데이트
                params = {
                    "max_vel_x": mode_param[0],
                    "min_vel_x": mode_param[1],
                    "max_vel_y": mode_param[2],
                    "min_vel_y": mode_param[3],
                    "acc_lim_x": mode_param[4],
                    "acc_lim_y": mode_param[5]
                }
                # client.update_configuration(params)
                rospy.sleep(2)
                rospy.loginfo(f"Updated DWAPlannerROS parameters: max_vel_x={mode_param[0]}, min_vel_x={mode_param[1]}, max_vel_y={mode_param[2]}, min_vel_y={mode_param[3]}, acc_lim_x={mode_param[4]}, acc_lim_y={mode_param[5]}"),
            except Exception as e:
                rospy.logerr(f"Failed to update parameters: {e}")
                rospy.signal_shutdown("Failed to update parameters")
    
    def processing(self):
        if (not self.calculated) :
            rospy.loginfo("\n\n===== Init Processing =====\n\n")

            converted_frontLeft_scan_to_Points = self.scan_to_point(self.frontLeft_scan)
            self.transformed_frontLeft_Scan_to_Points = self.transform_point_to_ref(converted_frontLeft_scan_to_Points, self.tf_frontLeft_Scan_to_ref_base_frame, self.frontRight_points.header)
            # self.transformed_frontLeft_Scan_to_Points.header.frame_id = self.ref_frame
            self.project_filtered_points(self.transformed_frontLeft_Scan_to_Points, self.projected_points)
            # rospy.loginfo("[2-1] Project Front Left Scan")
       
            converted_frontRight_scan_to_Points = self.scan_to_point(self.frontRight_scan)
            self.transformed_frontRight_Scan_to_Points = self.transform_point_to_ref(converted_frontRight_scan_to_Points, self.tf_frontRight_Scan_to_ref_base_frame, self.frontRight_points.header)
            # self.transformed_frontRight_Scan_to_Points.header.frame_id = self.ref_frame
            self.project_filtered_points(self.transformed_frontRight_Scan_to_Points, self.projected_points)
            # rospy.loginfo("[2-2] Project Front Right Scan")

            self.transformed_frontLeft_points = self.transform_point_to_ref(self.frontLeft_points, self.tf_frontLeft_Points_to_ref_base_frame, self.frontRight_points.header)
            # self.transformed_frontLeft_points.header.frame_id = self.ref_frame
            self.project_filtered_points(self.transformed_frontLeft_points, self.projected_points)
            # rospy.loginfo("[2-3] Project Front Left Points")

            self.transformed_frontRight_points = self.transform_point_to_ref(self.frontRight_points, self.tf_frontRight_Points_to_ref_base_frame, self.frontRight_points.header)
            # self.transformed_frontRight_points.header.frame_id = self.ref_frame
            self.project_filtered_points(self.transformed_frontRight_points, self.projected_points)
            # rospy.loginfo("[2-4] Project Front Right Points")

            self.apply_convex_hull(self.projected_points)
            self.apply_rotating_calipers(self.convex_hull_points)

            rospy.loginfo("Processing Completed")

            self.calculated = False
            alignmment_pose_to_ref_ftame, goal_pose_to_ref_frame = self.get_car_pose()

            with self.calculate_condition:
                self.pose_seq = [self.first_seq_pose, self.second_seq_pose, alignmment_pose_to_ref_ftame.pose, goal_pose_to_ref_frame.pose, alignmment_pose_to_ref_ftame.pose, self.second_seq_pose, self.first_seq_pose, self.init_pose]
                self.mode_param = [True, False, False, True, True, False, False, True]
                assert len(self.pose_seq) == len(self.mode_param)
                self.whole_seq = len(self.pose_seq)
                self.publish_polygon()
                self.publish_pose(goal_pose_to_ref_frame.pose, self.ref_frame, self.nav_goal_pose_pub)
                rospy.sleep(1)
                self.calculated = True
                self.calculate_condition.notify_all()

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
                    self.obb_seq_init = True
                    rospy.loginfo("Waiting for new goals to be calculated...")
                    rospy.sleep(1)
                    with self.calculate_condition:
                        while not self.calculated:  # self.calculate가 True가 될 때까지 대기
                            self.calculate_condition.wait()
                elif self.goal_cnt == self.clear_map_seq:
                    self.set_marking(False)
                    self.clear_map_pub.publish(Bool(data=True))
                    self.set_shutdown_costmaps(True)
                    rospy.loginfo("Clearing Map")
                    rospy.sleep(1)
                elif self.goal_cnt == self.retrieve_seq:
                    self.set_marking(True)
                    self.clear_map_pub.publish(Bool(data=False))
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
            if self.goal_cnt == self.retrieve_seq:
                rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached, aborting previous mission") 
                self.set_marking(True)
                self.clear_map_pub.publish(Bool(data=False))
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
                rospy.loginfo("Goal pose "+str(self.goal_cnt)+" was aborted by the Action Server")
                rospy.logwarn("Abort ROS for safety")
                rospy.signal_shutdown("Abort ROS for safety")
                return

        if status == 5:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" has been rejected by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" rejected, shutting down!")
            return

        if status == 8:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request before it started executing, successfully cancelled!")


    def init_movebase_client(self):
        # self.update_dwa_params(self.mode_param[self.goal_cnt])
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
        
        # costmap_2d의 dynamic_reconfigure 클라이언트 생성
        clientLocalCostmap = Client("/move_base/global_costmap/obstacles_layer", timeout=10)
        clientGlobalCostmap = Client("/move_base/local_costmap/obstacles_layer", timeout=10)

        # marking 값을 설정
        params = {"enabled": value}
        clientLocalCostmap.update_configuration(params)
        clientGlobalCostmap.update_configuration(params)

        rospy.loginfo(f"✅ global costmap marking 값을 {value} 으로 변경했습니다.")
        rospy.loginfo(f"✅ local costmap marking 값을 {value} 으로 변경했습니다.")
        rospy.sleep(2)


if __name__ == '__main__':

    rospy.init_node("init_obb_calcultor_node")
    rospy.loginfo("init_obb_calcultor_node")

    try :
        oBBCalculator = OBBCalculator()
        oBBCalculator.initialize()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted.")
