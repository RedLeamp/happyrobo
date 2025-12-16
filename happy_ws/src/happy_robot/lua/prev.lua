-- include "map_builder.lua"
-- include "trajectory_builder.lua"

-- options = {} -- 옵션 테이블 초기화

-- -- 기본 설정
-- options.tracking_frame = "base_link" -- 로봇의 기본 프레임
-- options.published_frame = "odom" -- 변환을 게시할 프레임
-- options.num_laser_scans = 0 -- 2D 레이저 스캔 비활성화
-- options.num_point_clouds = 1 -- 포인트 클라우드 데이터 활성화
-- options.use_odometry = true -- 오도메트리 데이터 사용
-- options.use_imu_data = true -- IMU 데이터 사용
-- options.num_subdivisions_per_laser_scan = 1

-- -- 센서 토픽 설정
-- options.point_cloud_topic = "/merged/points" -- 포인트 클라우드 토픽
-- options.imu_topic = "/imu/data" -- IMU 토픽
-- options.odometry_topic = "/odom" -- 오도메트리 토픽

-- -- 2D SLAM을 위한 Trajectory Builder 설정
-- options.trajectory_builder = {}
-- options.trajectory_builder.trajectory_builder_2d = {
--   min_range = 0.2, -- 포인트 클라우드 데이터의 최소 범위
--   max_range = 30.0, -- 포인트 클라우드 데이터의 최대 범위
--   min_z = 0.0, -- 포인트 클라우드 필터링의 최소 Z 값
--   max_z = 10.0, -- 포인트 클라우드 필터링의 최대 Z 값
--   missing_data_ray_length = 5.0, -- 누락된 데이터에 대한 광선 길이
--   num_accumulated_range_data = 1, -- 누적된 범위 데이터 수
--   voxel_filter_size = 0.1, -- 포인트 클라우드 다운샘플링을 위한 복셀 필터 크기
--   use_imu_data = true, -- IMU 데이터로 중력 정렬 사용
--   imu_gravity_time_constant = 10.0, -- IMU 중력 정렬을 위한 시간 상수
--   use_online_correlative_scan_matching = true, -- 실시간 상관 스캔 매칭 사용
--   real_time_correlative_scan_matcher = {
--     linear_search_window = 0.1, -- 스캔 매칭을 위한 선형 검색 창
--     angular_search_window = math.rad(20.0), -- 스캔 매칭을 위한 각도 검색 창 (20도)
--     translation_delta_cost_weight = 1e-1, -- 이동 델타 비용 가중치
--     rotation_delta_cost_weight = 1e-1, -- 회전 델타 비용 가중치
--   },
--   ceres_scan_matcher = {
--     occupied_space_weight = 10.0, -- 스캔 매칭에서 점유 공간의 가중치
--     translation_weight = 10, -- 이동에 대한 가중치
--     rotation_weight = 40, -- 회전에 대한 가중치
--     ceres_solver_options = {
--       max_num_iterations = 50, -- Ceres 솔버의 최대 반복 횟수
--     },
--   },
--   motion_filter = {
--     max_time_seconds = 5.0, -- 포즈 간 최대 시간 간격
--     max_distance_meters = 0.2, -- 포즈 간 최대 이동 거리
--     max_angle_radians = 0.0175, -- 포즈 간 최대 회전 각도 (약 1도)
--   },
--   submaps = {
--     num_range_data = 100, -- 서브맵당 범위 데이터 삽입 횟수
--     grid_options_2d = {
--       grid_type = "PROBABILITY_GRID", -- 서브맵 그리드 유형
--       resolution = 0.05, -- 서브맵 그리드 해상도 (미터 단위)
--     },
--   },
-- }

-- -- Pose Graph 설정 (Swerve Drive 모션 모델 조정용)
-- POSE_GRAPH = {}
-- POSE_GRAPH.optimization_problem = {}
-- POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
-- POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5

-- return options


-- include "map_builder.lua"
-- include "trajectory_builder.lua"

-- options = {
--   map_builder = MAP_BUILDER,
--   trajectory_builder = TRAJECTORY_BUILDER,
  
--   map_frame = "map",
--   tracking_frame = "base_link",
--   published_frame = "base_link",
--   odom_frame = "odom",
  
--   provide_odom_frame = true,
--   publish_frame_projected_to_2d = false,
  
--   use_odometry = false,
--   use_nav_sat = false,
--   use_landmarks = false,
  
--   use_laser_scan = false,
--   use_multi_echo_laser_scan = false,
--   use_point_cloud2 = true,

--   num_laser_scans = 0,
--   num_multi_echo_laser_scans = 0,
--   num_subdivisions_per_laser_scan = 1,
--   num_point_clouds = 1,

--   lookup_transform_timeout_sec = 0.2,
--   submap_publish_period_sec = 0.3,
--   pose_publish_period_sec = 5e-3,
--   trajectory_publish_period_sec = 30e-3,
-- }

-- MAP_BUILDER.use_trajectory_builder_2d = true

-- TRAJECTORY_BUILDER_2D.use_imu_data = false
-- TRAJECTORY_BUILDER_2D.min_range = 0.1
-- TRAJECTORY_BUILDER_2D.max_range = 30.0
-- TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.0
-- TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05

-- return options




include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_odometry = true,
--   use_imu_data = true,    
  use_nav_sat = false,
  use_landmarks = false,
--   use_laser_scan = true,
--   use_multi_echo_laser_scan = false,
--   use_point_cloud2 = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- TRAJECTORY_BUILDER_2D.use_imu_data = false
-- TRAJECTORY_BUILDER_2D.min_range = 0.1
-- TRAJECTORY_BUILDER_2D.max_range = 30.0
-- TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.0
-- TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05
-- TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 10

return options
