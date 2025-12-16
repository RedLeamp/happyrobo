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
from std_msgs.msg import Header
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
from happy_robo.msg import AlignAndGoalPoseStamped
from sklearn.cluster import DBSCAN
from message_filters import Subscriber, ApproximateTimeSynchronizer

class OBBCalculator:
    def __init__(self):
        self.obb_approval_topic = '/api/obb'
        self.obb_init_topic = '/obb/init'
        self.frontLeft_scan_topic = "/cygbot/scan/2d/0"
        self.frontRight_scan_topic = "/cygbot/scan/2d/1"
        self.frontLeft_left_scan_topic = "/cygbot/scan/2d/4"
        self.frontRight_right_scan_topic = "/cygbot/scan/2d/5"
        self.frontLeft_points_topic = "/cygbot/point/3d/0"
        self.frontRight_points_topic = "/cygbot/point/3d/1"
        self.frontLeft_left_points_topic = "/cygbot/point/3d/4"
        self.frontRight_right_points_topic = "/cygbot/point/3d/5"
        self.obb_topic = "/obb"
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
        self.obb_approval_sub = rospy.Subscriber(self.obb_approval_topic, Bool, self.obb_approval_callback)
        self.obb_init_sub = rospy.Subscriber(self.obb_init_topic, Bool, self.obb_init_callback)
        self.frontLeft_Scan_Sub = Subscriber(self.frontLeft_scan_topic, LaserScan)
        self.frontRight_Scan_Sub = Subscriber(self.frontRight_scan_topic, LaserScan)
        self.frontLeft_Left_Scan_Sub = Subscriber(self.frontLeft_left_scan_topic, LaserScan)
        self.frontRight_Right_Scan_Sub = Subscriber(self.frontRight_right_scan_topic, LaserScan)
        self.frontLeft_Points_Sub = Subscriber(self.frontLeft_points_topic, PointCloud2)
        self.frontRight_Points_Sub = Subscriber(self.frontRight_points_topic, PointCloud2)
        self.frontLeft_Left_Points_Sub = Subscriber(self.frontLeft_left_points_topic, PointCloud2)
        self.frontRight_Right_Points_Sub = Subscriber(self.frontRight_right_points_topic, PointCloud2)
        self.combined_points_pub = rospy.Publisher("/combined/points/3d", PointCloud2, queue_size=10)
        self.convex_hull_pub = rospy.Publisher("/obb/convex/3d", PointCloud2, queue_size=10)
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
        self.nav_align_pose_pub = rospy.Publisher("/parking/align", PoseStamped, queue_size=1)
        self.nav_goal_pose_pub = rospy.Publisher("/parking/goal", PoseStamped, queue_size=1)
        self.nav_obb_center_pub = rospy.Publisher("/obb/center", PoseStamped, queue_size=1)
        self.nav_obb_goal_pub = rospy.Publisher("/obb/goal", AlignAndGoalPoseStamped, queue_size=1)
        self.width = 2.5
        self.length = 5.7
        self.x_error = 0.0
        self.y_error = 0.0
        self.hori_alignment = True
        self.vert_alignment = True
        self.cirterion = np.array([1, 0])
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

    def obb_init_callback(self, msg):
        rospy.loginfo("Get OBB Init Signal")
        self.obb_seq_init = msg.data
        rospy.loginfo(f"======= Got OBB Init Signal : {self.obb_seq_init} ======")

    def obb_approval_callback(self, msg):
        with self.obb_approval_condition:
            rospy.loginfo("Get OBB Approval Signal")
            self.obb_approved_signal = True
            self.obb_approved = msg.data
            rospy.loginfo(f"======= Got OBB Approved Signal : {self.obb_approved} ======")
            self.obb_approval_condition.notify_all()

    def synced_callback(self, fl_scan, fr_scan, fll_scan, frr_scan, fl_points, fr_points, fll_points, frr_points):

        if (not self.obb_seq_init): 
            rospy.loginfo("Passed OBB Calculation..")
            return
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
                        projected_points.append([point[0], point[1]])
        return projected_points
    
    def apply_convex_hull(self, points):
        # merge_pointcloud2_msg = self.pub_obb_raw_points(points)
        # self.combined_points_pub.publish(merge_pointcloud2_msg)
        # self.convex_hull_points = points
        self.convexHullResult = ConvexHull(np.array(points), qhull_options="Qt QbB")
        for idx in self.convexHullResult.vertices:
            self.convex_hull_points.append((points[idx][0], points[idx][1]))
        rospy.loginfo("Calculated Convex Hull")
        merge_pointcloud2_msg = self.pub_obb_raw_points(self.convex_hull_points)
        self.convex_hull_pub.publish(merge_pointcloud2_msg)
    
    def compute_edge_based_axis(self, convex_hull_points):
        points = np.array(convex_hull_points)
        n = len(points)
        if n < 3:
            rospy.logwarn("포인트 수가 부족하여 변 기반 주축 계산 불가")
            return np.array([1.0, 0.0]), np.mean(points, axis=0)

        # 변 길이와 방향 계산
        edges = []
        for i in range(n):
            p1 = points[i]
            p2 = points[(i + 1) % n]
            vector = p2 - p1
            length = np.linalg.norm(vector)
            if length > 0:
                direction = vector / length
                edges.append((length, direction))

        # 가장 긴 변 선택
        edges.sort(key=lambda x: x[0], reverse=True)  # 길이 기준 내림차순 정렬
        principal_axis = edges[0][1]  # 가장 긴 변의 방향

        # 평행한 변들로 방향 보정 (선택적)
        parallel_directions = [direction for length, direction in edges[:4] if np.abs(np.dot(direction, principal_axis)) > 0.95]
        if parallel_directions:
            principal_axis = np.mean(parallel_directions, axis=0)
            principal_axis /= np.linalg.norm(principal_axis)

        centroid = np.mean(points, axis=0)
        return principal_axis, centroid
    
    def compute_pca(self, points):
        """포인트 클라우드의 주축을 PCA로 계산합니다."""
        points_np = np.array(points)
        if len(points_np) < 3:  # 최소 포인트 수 확인
            rospy.logwarn("포인트 수가 부족하여 PCA 계산 불가")
            return np.array([1.0, 0.0]), np.mean(points_np, axis=0)
        
        clustering = DBSCAN(eps=0.1, min_samples=3).fit(points_np)  # eps는 미터 단위, 조정 필요
        labels = clustering.labels_
        valid_points = points_np[labels != -1]  # 노이즈(-1) 제외

        if len(valid_points) < 3:
            rospy.logwarn("유효한 포인트 부족, 기본 주축 반환")
            return np.array([1.0, 0.0]), np.mean(points_np, axis=0)
        
        # 포인트 중심화
        centroid = np.mean(valid_points, axis=0)
        centered_points = valid_points - centroid
        # 공분산 행렬 계산
        cov_matrix = np.cov(centered_points.T)
        # 고유값 및 고유벡터 계산
        eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)
        # 고유값 기준 내림차순 정렬
        idx = np.argsort(eigenvalues)[::-1]
        eigenvectors = eigenvectors[:, idx]
        # 첫 번째 고유벡터가 주축(가장 긴 축)
        principal_axis = eigenvectors[:, 0]

        return principal_axis, centroid
    
    def compute_ransac_principal_axis(self, points, max_iterations=1000, distance_threshold=0.05, min_consensus=0.6):
        """포인트 클라우드의 주축을 RANSAC으로 계산합니다."""
        points_np = np.array(points)
        if len(points_np) < 3:  # 최소 포인트 수 확인
            rospy.logwarn("포인트 수가 부족하여 RANSAC 계산 불가")
            return np.array([1.0, 0.0]), np.mean(points_np, axis=0)

        # DBSCAN으로 노이즈 제거
        clustering = DBSCAN(eps=0.1, min_samples=3).fit(points_np)
        labels = clustering.labels_
        valid_points = points_np[labels != -1]  # 노이즈(-1) 제외

        if len(valid_points) < 3:
            rospy.logwarn("유효한 포인트 부족, 기본 주축 반환")
            return np.array([1.0, 0.0]), np.mean(points_np, axis=0)

        best_axis = np.array([1.0, 0.0])
        best_centroid = np.mean(valid_points, axis=0)
        max_inliers = 0
        n_points = len(valid_points)

        for _ in range(max_iterations):
            # 무작위로 두 포인트 선택
            idx = np.random.choice(n_points, 2, replace=False)
            p1, p2 = valid_points[idx]

            # 선분 방향 벡터 계산
            direction = p2 - p1
            norm = np.linalg.norm(direction)
            if norm < 1e-6:  # 방향 벡터가 너무 작으면 스킵
                continue
            direction = direction / norm

            # 모든 포인트와 선분 사이의 거리 계산
            inliers = 0
            for p in valid_points:
                # 포인트에서 선분으로의 수직 거리
                t = np.dot(p - p1, direction)
                projection = p1 + t * direction
                distance = np.linalg.norm(p - projection)
                if distance < distance_threshold:
                    inliers += 1

            # 더 많은 인라이어를 가진 선분 저장
            if inliers > max_inliers and inliers >= min_consensus * n_points:
                max_inliers = inliers
                best_axis = direction
                # 인라이어 포인트들의 중심을 새로운 중심점으로
                inlier_points = []
                for p in valid_points:
                    t = np.dot(p - p1, direction)
                    projection = p1 + t * direction
                    if np.linalg.norm(p - projection) < distance_threshold:
                        inlier_points.append(p)
                if inlier_points:
                    best_centroid = np.mean(np.array(inlier_points), axis=0)

        if max_inliers == 0:
            rospy.logwarn("적합한 선분을 찾지 못함, 기본 주축 반환")
            return np.array([1.0, 0.0]), np.mean(valid_points, axis=0)

        return best_axis, best_centroid
    
    def pub_center_obb (self,principal_axis,centroid):
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = self.ref_base_frame
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.position = Point(x=centroid[0], y=centroid[1], z=0.0)

        ref_vector = np.array([1.0, 0.0])
        cos_theta = np.dot(principal_axis, ref_vector) / (np.linalg.norm(principal_axis) * np.linalg.norm(ref_vector))
        theta = np.arccos(np.clip(cos_theta, -1.0, 1.0))
        if principal_axis[1] < 0:
            theta = -theta

        # z축 회전 쿼터니언
        quat = quaternion_from_euler(0, 0, theta)
        pose_msg.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

        self.nav_obb_center_pub.publish(pose_msg)

    def apply_rotating_calipers(self, convex_hull_points):
        """convex hull 포인트에서 principal_axis를 한 변으로 가지는 OBB를 계산합니다."""
        # 주축과 중심점 계산
        self.principal_axis, self.centroid = self.compute_ransac_principal_axis(convex_hull_points)
        rospy.loginfo(f"주축: {self.principal_axis}, 중심: {self.centroid}")
        self.pub_center_obb(self.principal_axis, self.centroid)

        # 주축 정규화
        principal_axis = self.principal_axis / np.linalg.norm(self.principal_axis)

        # 주축에 수직인 벡터 계산 (2D의 경우)
        perpendicular_axis = np.array([-principal_axis[1], principal_axis[0]])

        # 포인트를 중심점 기준으로 이동
        points_np = np.array(convex_hull_points)
        centered_points = points_np - self.centroid

        # 주축과 수직 방향으로 포인트 투영
        proj_along_axis = centered_points @ principal_axis
        proj_along_perp = centered_points @ perpendicular_axis

        # 투영된 포인트의 최소/최대 값 계산
        min_along_axis = np.min(proj_along_axis)
        max_along_axis = np.max(proj_along_axis)
        min_along_perp = np.min(proj_along_perp)
        max_along_perp = np.max(proj_along_perp)

        # OBB의 4개 모서리 계산
        aabb_corners = np.array([
            [min_along_axis, min_along_perp],  # 주축 및 수직 방향 최소
            [max_along_axis, min_along_perp],  # 주축 최대, 수직 최소
            [max_along_axis, max_along_perp],  # 주축 및 수직 최대
            [min_along_axis, max_along_perp]   # 주축 최소, 수직 최대
        ])

        # OBB 모서리를 글로벌 좌표계로 변환
        # 각 모서리는 주축과 수직 벡터의 선형 결합으로 표현
        global_corners = (
            aabb_corners[:, 0].reshape(-1, 1) * principal_axis +
            aabb_corners[:, 1].reshape(-1, 1) * perpendicular_axis
        ) + self.centroid

        # OBB 포인트 저장
        self.rect_points = [(point[0], point[1]) for point in global_corners]
        rospy.loginfo(f"OBB 모서리 계산 완료: {self.rect_points}")
    
    def dep_apply_rotating_calipers(self, convex_hull_points):
        
        self.principal_axis, self.centroid = self.compute_ransac_principal_axis(convex_hull_points)
        rospy.loginfo(f"주축: {self.principal_axis}, 중심: {self.centroid}")
        self.pub_center_obb(self.principal_axis, self.centroid)

        ref_vector = np.array([1.0, 0.0])
        cos_theta = np.dot(self.principal_axis, ref_vector) / (np.linalg.norm(self.principal_axis) * np.linalg.norm(ref_vector))
        theta = np.arccos(np.clip(cos_theta, -1.0, 1.0))
        # 각도가 올바른 사분면에 있는지 확인
        if self.principal_axis[1] < 0:
            theta = -theta

        # 포인트를 주축에 정렬하기 위한 회전 행렬
        rotation_matrix = np.array([
            [np.cos(theta), np.sin(theta)],
            [-np.sin(theta), np.cos(theta)]
        ])

        # 포인트를 객체 좌표계로 변환 (주축과 정렬)
        points_np = np.array(convex_hull_points)
        centered_points = points_np - self.centroid
        aligned_points = centered_points @ rotation_matrix

        # 객체 좌표계에서 AABB 계산
        min_coords = np.min(aligned_points, axis=0)
        max_coords = np.max(aligned_points, axis=0)
        # AABB 모서리 정의
        aabb_corners = np.array([
            [min_coords[0], min_coords[1]],  # 좌측 하단
            [max_coords[0], min_coords[1]],  # 우측 하단
            [max_coords[0], max_coords[1]],  # 우측 상단
            [min_coords[0], max_coords[1]]   # 좌측 상단
        ])

        # AABB 모서리를 글로벌 좌표계로 역변환
        inverse_rotation = np.array([
            [np.cos(-theta), np.sin(-theta)],
            [-np.sin(-theta), np.cos(-theta)]
        ])
        global_corners = (aabb_corners @ inverse_rotation) + self.centroid

        # OBB 포인트 저장
        self.rect_points = [(point[0], point[1]) for point in global_corners]
        rospy.loginfo(f"OBB 모서리 계산 완료: {self.rect_points}")


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

        merged_points = [(point[0], point[1], 0.0) for point in merged_points]

        # 3. numpy 배열로 변환
        merged_points_np = np.array(merged_points, dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32)])

        # 4. 새로운 PointCloud2 메시지 생성
        header = points1.header  # 첫 번째 클라우드의 헤더를 사용 (필요에 따라 수정 가능)
        merged_cloud = pc2.create_cloud_xyz32(header, merged_points_np)

        return merged_cloud
    
    def pub_obb_raw_points(self,projected_points):
        """projected_points를 Z=0인 PointCloud2 메시지로 변환."""
        # projected_points를 numpy 배열로 변환 (Z=0 추가)
        points_np = np.array(
            [(point[0], point[1], 0.0) for point in projected_points],
            dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32)]
        )

        # PointCloud2 메시지 생성
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.ref_base_frame  # 원하는 프레임 ID로 설정
        cloud_msg = pc2.create_cloud_xyz32(header, points_np)
        return cloud_msg

    
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

        rospy.loginfo("\n\n===== apply_convex_hull =====\n\n")
        self.apply_convex_hull(self.projected_points)

        rospy.loginfo("\n\n===== apply_rotating_calipers =====\n\n")
        self.apply_rotating_calipers(self.convex_hull_points)

        rospy.loginfo("Processing Completed")
        alignmment_pose_to_ref_ftame, goal_pose_to_ref_frame = self.get_car_pose()

        rospy.loginfo("Publish Current Goal and estimated Polygon")
        self.publish_polygon()
        nav_goal_pose = self.publish_pose(goal_pose_to_ref_frame.pose, self.ref_frame, self.nav_goal_pose_pub)
        nav_align_pose = self.publish_pose(alignmment_pose_to_ref_ftame.pose, self.ref_frame, self.nav_align_pose_pub)

        with self.obb_approval_condition:
            rospy.loginfo("Waiting for OBB Approval Signal...")
            while not self.obb_approved_signal:  # self.obb_approved_signal True가 될 때까지 대기
                self.obb_approval_condition.wait()

        if (self.obb_approved) :
            rospy.loginfo("✅ OBB Ready to Go")
            self.pose_seq = [self.first_seq_pose, self.second_seq_pose, alignmment_pose_to_ref_ftame.pose, goal_pose_to_ref_frame.pose, alignmment_pose_to_ref_ftame.pose, self.second_seq_pose, self.first_seq_pose, self.init_pose]
            self.whole_seq = len(self.pose_seq)
            obb_goal_pose = AlignAndGoalPoseStamped()
            obb_goal_pose.align_pose = nav_align_pose
            obb_goal_pose.goal_pose = nav_goal_pose
            self.nav_obb_goal_pub.publish(obb_goal_pose)
            rospy.loginfo("✅ OBB Nav Goal Published")
            rospy.sleep(1)
            self.obb_approved_signal = True
            self.obb_calculating = True
            self.obb_calculated = True
            self.obb_seq_init = False
        else :
            rospy.loginfo("❌ OBB not approved, calculates again")
            self.obb_approved_signal = False
            self.obb_calculating = False
            self.obb_calculated = False

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

    # def get_car_orientation(self, center):

    #     # x축([1, 0]) 대비 각도 계산
    #     ref_vector = np.array([1.0, 0.0])
    #     cos_theta = np.dot(self.principal_axis, ref_vector) / (np.linalg.norm(self.principal_axis) * np.linalg.norm(ref_vector))
    #     theta = np.arccos(np.clip(cos_theta, -1.0, 1.0))
    #     if self.principal_axis[1] < 0:
    #         theta = -theta
    #     rot_quat = Quaternion(*(quaternion_from_euler(0, 0, theta, axes='sxyz')))
    #     if abs(self.principal_axis[0]) > 1e-6:  # 0으로 나누기 방지
    #         y_intercept = -(self.principal_axis[1] / self.principal_axis[0]) * center[0] + center[1]
    #     else:
    #         y_intercept = center[1]

    #     alignment_pose_vec = np.array([0, y_intercept])
    #     goal_pose_vec = np.array([center[0], center[1]])

    #     alignmment_pose_to_ref_ftame = self.transform_car_coord_to_ref_frame(alignment_pose_vec, rot_quat)
    #     goal_pose_to_ref_frame = self.transform_car_coord_to_ref_frame(goal_pose_vec, rot_quat)

    #     return alignmment_pose_to_ref_ftame, goal_pose_to_ref_frame
    
    def transform_car_coord_to_ref_frame(self, trans, rot):

        pose = PoseStamped()
        pose.header.frame_id = self.ref_frame
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = trans[0]
        pose.pose.position.y = trans[1]
        pose.pose.position.z = 0
        pose.pose.orientation = rot
        # return pose
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
        return poseStamped
        
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
    

if __name__ == '__main__':


    rospy.init_node("init_obb_calculator_node")
    rospy.loginfo("init_obb_calculator_node")
    try :
        oBBCalculator = OBBCalculator()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted.")
