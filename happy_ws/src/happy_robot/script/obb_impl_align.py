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
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import tf
from sensor_msgs.point_cloud2 import read_points, create_cloud
import PyKDL
import laser_geometry
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseAction, MoveBaseGoal
from myactionlib import SimpleActionClient
from tf.transformations import quaternion_from_euler
from std_srvs.srv import Empty
from threading import Condition
from dynamic_reconfigure.client import Client
import tf.transformations as tf_trans

class OBBCalculator:
    def __init__(self):
        self.init_topic = "/seq/obb/init"
        self.frontLeft_scan_topic = "/sigbot/scan01"
        self.frontRight_scan_topic = "/sigbot/scan02"
        self.frontLeft_points_topic = "/sigbot/point01"
        self.frontRight_points_topic = "/sigbot/point02"
        # self.frontScan_topic = "/front/scan"
        self.obb_topic = "/obb"
        self.projected_points = []
        self.archieve_points = [False, False]
        self.archieve_scan = [False, False]
        self.convex_hull_points = []
        self.rect_points = []
        self.goal_points = []
        self.get_tf_1 = False
        self.get_tf_2 = False
        self.get_tf_3 = False
        self.tf_transformListener = tf.TransformListener()
        self.init_sub = rospy.Subscriber(self.init_topic, Bool, self.init_callback)
        self.clear_map_pub = rospy.Publisher('/clear_map', Bool, queue_size=10)
        self.frontLeft_Scan_Sub = rospy.Subscriber(self.frontLeft_scan_topic, LaserScan, self.frontLeft_scan_callback)
        self.frontRight_Scan_Sub = rospy.Subscriber(self.frontRight_scan_topic, LaserScan, self.frontRight_scan_callback)
        self.frontLeft_Points_Sub = rospy.Subscriber(self.frontLeft_points_topic, PointCloud2, self.frontLeft_point_callback)
        self.frontRight_Points_Sub = rospy.Subscriber(self.frontRight_points_topic, PointCloud2, self.frontRight_point_callback)
        # self.frontScan_Sub = rospy.Subscriber(self.frontScan_topic, LaserScan, self.frontScan_callback)
        self.obb_pub = rospy.Publisher(self.obb_topic, PolygonStamped, queue_size=10)
        self.transformed1_point_pub = rospy.Publisher("/transformed_scan_1", PointCloud2, queue_size=10)
        self.transformed2_point_pub = rospy.Publisher("/transformed_scan_2", PointCloud2, queue_size=10)
        self.nav_goal_pub = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size=1)
        self.nav_first_pose_pub = rospy.Publisher("/parking/first", PoseStamped, queue_size=1)
        self.nav_second_pose_pub = rospy.Publisher("/parking/second", PoseStamped, queue_size=1)
        self.nav_align_pose_pub = rospy.Publisher("/parking/align", PoseStamped, queue_size=1)
        self.nav_goal_pose_pub = rospy.Publisher("/parking/goal", PoseStamped, queue_size=1)
        self.final_goal_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.frontScan_pub = rospy.Publisher("/front/scan/map", LaserScan, queue_size=1)
        self.pub_frontScan = True
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
        self.init_orientation = Quaternion(*(quaternion_from_euler(0, 0, 0, axes='sxyz')))
        self.init_pose = Pose(Point(self.init_x, self.init_y, 0), self.init_orientation)
        self.first_seq_pose = Pose(Point(self.first_x, self.first_y, 0), self.init_orientation)
        self.second_seq_pose = Pose(Point(self.second_x, self.second_y, 0), self.init_orientation)
        self.pose_seq = [self.first_seq_pose, self.second_seq_pose]
        self.frame_seq = [self.ref_frame, self.ref_frame]
        self.vertical_mode = [2.2, -2.2, 0.0, 0.0, 2.5, 0.0]
        self.horizontal_mode = [0, 0, 2.2, -2.2, 0.0, 2.5]
        self.mode_param = [self.vertical_mode, self.horizontal_mode]
        self.init = False
        self.calculated = False
        self.calculate_condition = Condition()
        self.whole_seq = 8
        self.feedback_pose = Pose()
        self.clear_map_seq = 4
        self.max_vel = 2.2
        self.cal_car_obb_seq = 2

    # def frontScan_callback(self, msg):
    #     if (self.pub_frontScan):
    #         self.frontScan_pub.publish(msg)
    #         print("Front Scan Callback")

    def initialize(self):

        rospy.loginfo("Nav Seq Init")
        self.goal_cnt = 0

        wait = self.client.wait_for_server(rospy.Duration(10.0))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")
        self.movebase_client()

        
    def init_callback(self, msg):
        self.init = msg.data
        if (self.init) : rospy.loginfo("Got OBB Calculator Init Signal")

    def frontLeft_scan_callback(self, msg):
        if (self.init) :
            self.frontLeft_scan = msg
            self.archieve_scan[0] = True
            # rospy.loginfo("[1-1] Got Front Left Scan")
            self.init = False

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
                self.tf_frontLeft_Scan_to_frontRight_Points, self.get_tf_1 = self.lookupTransform(self.frontLeft_scan, self.frontRight_points)
            if not (self.get_tf_2):
                self.tf_frontRight_Scan_to_frontRight_Points, self.get_tf_2 = self.lookupTransform(self.frontRight_scan, self.frontRight_points)
            if not (self.get_tf_3):
                self.tf_frontLeft_Points_to_frontRight_Points, self.get_tf_3 = self.lookupTransform(self.frontLeft_points, self.frontRight_points)
            if (self.get_tf_1 and self.get_tf_2 and self.get_tf_3):
                rospy.loginfo("Ready to Process")
                self.archieve_points[0] = False
                import threading
                threading.Thread(target=self.processing).start()
            else : rospy.loginfo("Wait for TF DATA")

    def lookupTransform(self, tar, src):
        # tar to src
        kTFTimeout_ms = 1000
        kMsToNs = 1000000
        kTFWait = rospy.Duration(0, 200*kMsToNs)
        try :
            self.tf_transformListener.waitForTransform(src.header.frame_id, tar.header.frame_id, src.header.stamp + kTFWait, rospy.Duration(0, kTFTimeout_ms * kMsToNs))
            tf_frontLeft_to_frontRight = self.tf_transformListener.lookupTransform(src.header.frame_id, tar.header.frame_id, src.header.stamp + kTFWait)
            # rospy.loginfo("Got Transform")
            return tf_frontLeft_to_frontRight, True
        except Exception as e:
            rospy.loginfo("Failed to Lookup Transform : " + e)
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
            # if (point[2] >= 0.01): # Z 0.01m 이상만
            if (np.sqrt(point[0]**2 + point[1]**2) <= 5.0): # 5m 이내만
                projected_points.append([point[0]*100, point[1]*100])
        return projected_points
    
    def apply_convex_hull(self, points):
        self.convexHullResult = ConvexHull(np.array(points))
        for point in self.convexHullResult.points:
            self.convex_hull_points.append((point[0], point[1]))
        # rospy.loginfo("Calculated Convex Hull")
        return self.convex_hull_points
    
    def apply_rotating_calipers(self, convex_hull_points):
        result, scale_up_rect_points = rc.smallest_rectangle(convex_hull_points, rc.compare_area)
        rospy.loginfo("\n\n\n====Calculated Rotating Calipers====\n\n\n")
        self.rect_points = [(point[0]/100, point[1]/100) for point in scale_up_rect_points]

    def update_dwa_params(self, mode_param):
        """
        DWAPlannerROS의 max_vel_x와 min_vel_x 값을 업데이트하는 함수.
        """
        try:
            # dynamic_reconfigure 클라이언트 생성
            client = Client("/move_base/DWAPlannerROS", timeout=10)
            # 매개변수 업데이트
            params = {
                "max_vel_x": mode_param[0],
                "min_vel_x": mode_param[1],
                "max_vel_y": mode_param[2],
                "min_vel_y": mode_param[3],
                "acc_lim_x": mode_param[4],
                "acc_lim_y": mode_param[5]
            }
            client.update_configuration(params)
            rospy.sleep(1)
            rospy.loginfo(f"Updated DWAPlannerROS parameters: max_vel_x={mode_param[0]}, min_vel_x={mode_param[1]}, max_vel_y={mode_param[2]}, min_vel_y={mode_param[3]}, acc_lim_x={mode_param[4]}, acc_lim_y={mode_param[5]}"),
        except Exception as e:
            rospy.logerr(f"Failed to update parameters: {e}")
    
    def processing(self):
        if (not self.calculated) :
            rospy.loginfo("\n\n===== Init Processing =====\n\n")

            converted_frontLeft_scan_to_Points = self.scan_to_point(self.frontLeft_scan)
            self.transformed_frontLeft_Scan_to_Points = self.transform_point_to_ref(converted_frontLeft_scan_to_Points, self.tf_frontLeft_Scan_to_frontRight_Points, self.frontRight_points.header)
            self.project_filtered_points(self.transformed_frontLeft_Scan_to_Points, self.projected_points)
            # rospy.loginfo("[2-1] Project Front Left Scan")
       
            converted_frontRight_scan_to_Points = self.scan_to_point(self.frontRight_scan)
            self.transformed_frontRight_Scan_to_Points = self.transform_point_to_ref(converted_frontRight_scan_to_Points, self.tf_frontRight_Scan_to_frontRight_Points, self.frontRight_points.header)
            self.project_filtered_points(self.transformed_frontRight_Scan_to_Points, self.projected_points)
            # rospy.loginfo("[2-2] Project Front Right Scan")

            self.transformed_frontLeft_points = self.transform_point_to_ref(self.frontLeft_points, self.tf_frontLeft_Points_to_frontRight_Points, self.frontRight_points.header)
            self.project_filtered_points(self.transformed_frontLeft_points, self.projected_points)
            # rospy.loginfo("[2-3] Project Front Left Points")

            self.project_filtered_points(self.frontRight_points, self.projected_points)
            # rospy.loginfo("[2-4] Project Front Right Points")

            self.apply_convex_hull(self.projected_points)
            self.apply_rotating_calipers(self.convex_hull_points)

            # rospy.loginfo("Processing Completed")
            self.calculated = False
            center, rot_quat, y_intercept = self.get_car_pose()
            # alignment_frame = "base_link"
            # goal_frame = "base_link"
            # alignment_pose = Pose(Point(0 + self.length/2, y_intercept- self.width/2, 0), rot_quat)
            # alignment_pose = Pose(Point(0, y_intercept, 0), rot_quat)
            # goal_pose = Pose(Point(center[0] + self.length/2, center[1] - self.width/2, 0), rot_quat)
            # goal_pose_final = Pose(Point(center[0] + self.length/2, center[1] - self.width/2 - y_intercept, 0), rot_quat)
            # self.pose_seq = [ alignment_pose, goal_pose_final ]

            # alignment_pose = Pose(Point(self.second_x + 0 - self.length/2, self.second_y + y_intercept + self.width/2, 0), rot_quat)
            # goal_pose = Pose(Point(self.second_x +center[0], self.second_y +center[1], 0), rot_quat)
            # alignment_pose = Pose(Point(self.second_x, self.second_y + y_intercept - self.width/2, 0), rot_quat)
            # goal_pose = Pose(Point(self.second_x + self.length/2 + center[0], self.second_y - self.width/2 + center[1], 0), rot_quat)
            offset_x = self.length/2
            offset_y = -self.width/2
            alignment_pose_coord = Pose(Point(self.second_x, self.second_y + y_intercept, 0), self.init_orientation)
            alignment_pose_ori = Pose(Point(self.second_x, self.second_y + y_intercept, 0), rot_quat)
            goal_pose = Pose(Point(center[0] + self.second_x, center[1] + self.second_y, 0), rot_quat)
            # goal_pose = Pose(Point(center[0] + offset_x + self.second_x, center[1] + offset_y + self.second_y, 0), rot_quat)



            # push_range = math.sqrt((center[0]+offset_x)**2 + (y_intercept-center[1]-offset_y)**2)
            # alignment_pose_coord_relative = Pose(Point(0, y_intercept, 0), self.init_orientation)
            # alignment_pose_ori_relative = Pose(Point(0, 0, 0), rot_quat)
            # goal_pose_relative = Pose(Point(push_range, 0, 0), self.init_orientation)
            # goal_pose_relative_backward = Pose(Point(-(push_range), 0, 0), self.init_orientation)
            # alignment_pose_ori_relative_backward = Pose(Point(0, 0, 0), rot_quat_restore)
            # alignment_pose_coord_relative_backward = Pose(Point(0, -y_intercept, 0), self.init_orientation)
            # goal_pose_relative_compensate = Pose(Point(-self.length/2, y_intercept + self.width/2, 0), rot_quat)
            # alignment_pose = Pose(Point(self.second_x, self.second_y + y_intercept, 0), rot_quat)
            # alignment_pose = Pose(Point(self.length/2, self.second_y + y_intercept, 0), rot_quat)
            # goal_pose = Pose(Point(self.second_x +center[0] + self.length/2, self.second_y +center[1] - self.width/2, 0), rot_quat)
            # goal_pose_final = Pose(Point(self.second_x + center[0], self.second_y + center[1] - y_intercept, 0), rot_quat)
            # self.pose_seq.append = [ alignment_pose, goal_pose ]
            with self.calculate_condition:
                # self.pose_seq = [self.first_seq_pose, self.second_seq_pose, alignment_pose, goal_pose]
                self.pose_seq = [self.first_seq_pose, self.second_seq_pose, alignment_pose_coord, alignment_pose_ori, goal_pose, alignment_pose_ori, alignment_pose_coord, self.second_seq_pose, self.first_seq_pose, self.init_pose]
                self.mode_param = [self.vertical_mode, self.horizontal_mode, self.horizontal_mode, self.horizontal_mode, self.vertical_mode, self.vertical_mode, self.horizontal_mode, self.horizontal_mode, self.horizontal_mode, self.vertical_mode]
                assert len(self.pose_seq) == len(self.mode_param)
                self.whole_seq = len(self.pose_seq)
                # self.pose_seq.append = alignment_pose
                # self.pose_seq.append = goal_pose
                # self.goal_cnt = 0
                self.publish_polygon()
                # self.publish_pose(alignment_pose_ori, self.ref_frame, self.nav_align_pose_pub)
                self.publish_pose(goal_pose, self.ref_frame, self.nav_goal_pose_pub)
                rospy.sleep(1)
                self.calculated = True
                self.calculate_condition.notify_all()

            # rospy.loginfo("Waiting for move_base action server...")
            # wait = self.client.wait_for_server(rospy.Duration(5.0))
            # if not wait:
            #     rospy.logerr("Action server not available!")
            #     rospy.signal_shutdown("Action server not available!")
            #     return
            # rospy.loginfo("Connected to move base server")
            # rospy.loginfo("Starting goals achievements ...")
            # self.movebase_client()

    def active_cb(self):
        rospy.loginfo("Goal pose "+str(self.goal_cnt+1)+" is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        # self.pose_error_goal_to_current = self.final_goal - feedback.base_position.pose
        self.feedback_pose = feedback.base_position.pose
        # rospy.loginfo("Feedback for goal pose "+str(self.goal_cnt)+" ERROR : " + str(self.feedback_pose))

    def done_cb(self, status, result):
        
        self.goal_cnt += 1
        if status == 2:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request after it started executing, completed execution!")

        if status == 3:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached") 
            if self.goal_cnt < self.whole_seq:
                if self.goal_cnt == self.cal_car_obb_seq:
                    self.init = True
                    # self.clear_map_pub.publish(Bool(data=True))
                    # self.set_shutdown_costmaps(True)
                    rospy.loginfo("Waiting for new goals to be calculated...")
                    with self.calculate_condition:
                        while not self.calculated:  # self.calculate가 True가 될 때까지 대기
                            self.calculate_condition.wait()
                elif self.goal_cnt == self.clear_map_seq:
                    self.clear_map_pub.publish(Bool(data=True))
                    self.set_shutdown_costmaps(True)
                    rospy.loginfo("Clearing Map")
                    rospy.sleep(1)
                elif self.goal_cnt == self.clear_map_seq + 1:
                    self.clear_map_pub.publish(Bool(data=False))
                    rospy.loginfo("Regenerating Map")
                    rospy.sleep(1)
                # self.update_dwa_params(self.mode_param[self.goal_cnt])
                next_goal = MoveBaseGoal()
                next_goal.target_pose.header.frame_id = self.ref_frame
                # next_goal.target_pose.header.frame_id = self.frame_seq[self.goal_cnt]
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
            if self.goal_cnt == self.clear_map_seq+1:
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


    def movebase_client(self):
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

    def get_transformation(self, source_pose, target_pose):
        """
        Calculate the transformation from source to target coordinate frames.

        Args:
            source_pose (Pose): The pose of the source frame.
            target_pose (Pose): The pose of the target frame.

        Returns:
            dict: A dictionary containing translation and rotation as the transformation.
        """
        # Extract position and orientation from source_pose
        source_position = [source_pose.position.x, source_pose.position.y, source_pose.position.z]
        source_orientation = [source_pose.orientation.x, source_pose.orientation.y, 
                            source_pose.orientation.z, source_pose.orientation.w]
        # Extract position and orientation from target_pose
        target_position = [target_pose.position.x, target_pose.position.y, target_pose.position.z]
        target_orientation = [target_pose.orientation.x, target_pose.orientation.y, 
                            target_pose.orientation.z, target_pose.orientation.w]
        
        euler_src = tf_trans.euler_from_quaternion(source_orientation)
        print("euler_src: ", euler_src)

        euler_tar = tf_trans.euler_from_quaternion(target_orientation)
        print("euler_tar: ", euler_tar)

        err_lin_x = target_pose.position.x - source_pose.position.x
        err_lin_y =  target_pose.position.y - source_pose.position.y
        err_ang_z = euler_tar[2] - euler_src[2]
        
        # source_rot_matrix = tf_trans.quaternion_matrix(source_orientation)
        # source_trans_matrix = tf_trans.translation_matrix(source_position)
        # source_tranform_matrix = np.matmul(source_rot_matrix, source_trans_matrix)
        # # source_tranform_matrix = source_rot_matrix.dot(source_trans_matrix)
        # source_translation = -source_rot_matrix[:3, :3].dot(source_position)

        # print("Source Rotation Matrix: ", source_rot_matrix)
        # print("Source Translation: ", source_trans_matrix)
        # # print("Source Transform: ", source_tranform_matrix)

        # # Transform target relative to source
        # relative_translation = source_rot_matrix[:3, :3].dot(target_position) + source_translation
        # relative_rotation = tf_trans.quaternion_multiply(
        #     tf_trans.quaternion_inverse(source_orientation), target_orientation
        # )

        # direct_translation = [target_pose.position.x - source_pose.position.x, target_pose.position.y - source_pose.position.y, target_pose.position.z - source_pose.position.z]

        # print("Source Pose: ", source_pose)
        # print("Target Pose: ", target_pose)
        # print("Direct Translation: ", direct_translation)
        # print("Relative Translation: ", relative_translation)
        # print("Relative Rotation: ", relative_rotation)
        # rospy.sleep(10)
        # Inverse of target_pose (transform target to origin)
        # target_rot_matrix = tf_trans.quaternion_matrix(target_orientation)
        # target_translation = -target_rot_matrix[:3, :3].dot(target_position)

        # # Combine transformations
        # relative_translation = target_translation
        # relative_rotation = tf_trans.quaternion_inverse(target_orientation)

        return err_lin_x, err_lin_y, err_ang_z
    
    def transform_point(self, err_lin_x, err_lin_y, err_ang_z, point_x, point_y):
        """
        Transform a point from the source coordinate frame to the target coordinate frame.

        Args:
            point (Point): The point in the source coordinate frame.
            transformation (dict): The transformation (translation and rotation) to the target frame.

        Returns:
            Point: The transformed point in the target coordinate frame.
        """

        # transformed_X = math.cos(err_ang_z) * point_x - math.sin(err_ang_z) * point_y + err_lin_x + self.length/2
        # transformed_y = math.sin(err_ang_z) * point_x + math.cos(err_ang_z) * point_y + err_lin_y - self.width/2

        transformed_X = point_x + err_lin_x + self.length/2
        transformed_y = point_y + err_lin_y - self.width/2

        return transformed_X, transformed_y
    
    def transform_rect_points(self):
        err_lin_x, err_lin_y, err_ang_z = self.get_transformation(self.feedback_pose, self.second_seq_pose)
        transformed_rect_points = []
        for point in self.rect_points:
            transformed_X, transformed_y = self.transform_point(err_lin_x, err_lin_y, err_ang_z, point[0], point[1])
            transformed_rect_points.append([transformed_X, transformed_y])
        self.rect_points = transformed_rect_points

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
        rot = Quaternion(*(quaternion_from_euler(0, 0, self.get_inner_angle(self.cirterion, car_pose_vec), axes='sxyz')))
        y_intercept = self.get_y_intercept(car_pose_vec, center)
        return rot, y_intercept
    
    def get_y_intercept(self, vec, center):
        # return -(vec[1] / vec[0]) * (center[0] + self.length/2) + (center[1]- self.width/2)
        return -(vec[1] / vec[0]) * (center[0]) + (center[1])

    def get_car_pose(self):
        self.transform_rect_points()
        sum_x = 0.0
        sum_y = 0.0
        for point in self.rect_points:
            sum_x += point[0]
            sum_y += point[1]
        center = [sum_x/4, sum_y/4]
        rot_quat, y_intercept = self.get_car_orientation(center)
        return center, rot_quat, y_intercept
        
    def publish_final_pose(self, goal_pose, goal_frame):
        poseStamped = PoseStamped()
        # poseStamped.header.seq = 0
        poseStamped.header.stamp = rospy.Time.now()
        poseStamped.pose = goal_pose
        poseStamped.header.frame_id = goal_frame
        # poseStamped.header.frame_id = 'sigbot_base_02'
        # poseStamped.pose.position.x = center[0]
        # poseStamped.pose.position.y = center[1]
        # poseStamped.pose.position.z = 0.0
        # poseStamped.pose.orientation = rot_quat
        self.nav_goal_pose_pub.publish(poseStamped)
        rospy.loginfo("Published Goal")

    def publish_pose(self, pose, frame, publisher):
        poseStamped = PoseStamped()
        poseStamped.header.stamp = rospy.Time.now()
        poseStamped.pose = pose
        poseStamped.header.frame_id = frame
        publisher.publish(poseStamped)
        rospy.loginfo("Published Pose")

    def publish_alignment_pose(self, alignment_pose, alignment_frame):
        poseStamped = PoseStamped()
        poseStamped.header.stamp = rospy.Time.now()
        poseStamped.pose = alignment_pose
        poseStamped.header.frame_id = alignment_frame
        self.nav_align_pose_pub.publish(poseStamped)
        rospy.loginfo("Published Alignment Pose")

        
    def publish_polygon(self):
        ros_publisher_points = Polygon()
        for point in self.rect_points:
            # polygon_point = Point32(point[0]+self.length/2+self.second_x, point[1]-self.width/2+self.second_y, 0)
            polygon_point = Point32(point[0]+self.second_x, point[1]+self.second_y, 0)
            ros_publisher_points.points.append(polygon_point)
        polygonStamped = PolygonStamped()
        polygonStamped.polygon = ros_publisher_points
        polygonStamped.header.frame_id = self.ref_frame
        polygonStamped.header.stamp = rospy.Time.now()
        self.obb_pub.publish(polygonStamped)
        rospy.loginfo("Published OBB")
        return
    
    def publish_alignment_goal(self, alignment_pose, alignment_frame):
        msg = MoveBaseActionGoal()
        msg.header.seq = 0
        # msg.header.stamp.secs = 0
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = ''
        # msg.goal_id.stamp = rospy.Time.now()
        msg.goal_id.id = ''
        # msg.goal.target_pose.header.seq = 0
        msg.goal.target_pose.header.stamp = rospy.Time.now()
        msg.goal.target_pose.header.frame_id = alignment_frame
        # msg.goal.target_pose.header.frame_id = 'base_link'
        # msg.goal.target_pose.pose.position.x = sum_x - self.length/2
        # msg.goal.target_pose.pose.position.y = sum_y - self.width/2
        msg.goal.target_pose.pose = alignment_pose
        # msg.goal.target_pose.pose.position.x = 0.0
        # # msg.goal.target_pose.pose.position.x = sum_x
        # msg.goal.target_pose.pose.position.y = y_intercept
        # msg.goal.target_pose.pose.position.z = 0.0
        # msg.goal.target_pose.pose.orientation = rot_quat
        self.nav_goal_pub.publish(msg)
        return
    
    def publish_final_goal(self, goal_pose, goal_frame):
        msg = MoveBaseActionGoal()
        msg.header.seq = 0
        # msg.header.stamp.secs = 0
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = ''
        # msg.goal_id.stamp = rospy.Time.now()
        msg.goal_id.id = ''
        # msg.goal.target_pose.header.seq = 0
        msg.goal.target_pose.header.stamp = rospy.Time.now()
        msg.goal.target_pose.pose = goal_pose
        msg.goal.target_pose.header.frame_id = goal_frame
        # msg.goal.target_pose.header.frame_id = 'sigbot_base_02'
        # msg.goal.target_pose.pose.position.x = sum_x - self.length/2
        # msg.goal.target_pose.pose.position.y = sum_y - self.width/2
        # msg.goal.target_pose.pose.position.x = center[0]
        # msg.goal.target_pose.pose.position.y = center[1]
        # msg.goal.target_pose.pose.position.z = 0.0
        # msg.goal.target_pose.pose.orientation = rot_quat
        self.final_goal = msg.goal.target_pose.pose
        self.nav_goal_pub.publish(msg)
        return
    
    def set_shutdown_costmaps(self, value):
        # param_name = "/move_base/shutdown_costmaps"  # 파라미터 이름
        # if rospy.has_param(param_name):
        #     rospy.set_param(param_name, value)
        #     rospy.loginfo(f"Set {param_name} to {value}")
        # else:
        #     rospy.logwarn(f"Parameter {param_name} does not exist!")

        rospy.loginfo("============ Stop Publishing Front Scan ============")
        rospy.wait_for_service('/move_base/clear_costmaps')
        try:
            clear_costmaps_service = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
            response = clear_costmaps_service()
            rospy.loginfo("Costmap cleared successfully.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
        
        # param_name = "/move_base/shutdown_costmaps"  # 파라미터 이름
        # if rospy.has_param(param_name):
        #     rospy.set_param(param_name, value)
        #     rospy.loginfo(f"Set {param_name} to {value}")
        # else:
        #     rospy.logwarn(f"Parameter {param_name} does not exist!")
        rospy.sleep(1)


if __name__ == '__main__':

    rospy.init_node("init_obb_calcultor_node")
    rospy.loginfo("init_obb_calcultor_node")

    try :
        oBBCalculator = OBBCalculator()
        oBBCalculator.initialize()
        # while not rospy.is_shutdown():
        #     rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted.")
