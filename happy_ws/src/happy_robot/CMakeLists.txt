cmake_minimum_required(VERSION 2.8.3)
project(happy_robo)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")

# bugfix for g++-Warning
#   "<command-line>:0:0: warning: missing whitespace after the macro name"
remove_definitions(-DDISABLE_LIBUSB-1.0)

# generate_dynamic_reconfigure_options(cfg/laserscan_multi_merger.cfg cfg/laserscan_virtualizer.cfg)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/WoorobotSDK)

find_package(catkin REQUIRED COMPONENTS
  laser_geometry
  tf
  tf2
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  dynamic_reconfigure
  cartographer_ros
  cartographer_ros_msgs
  pcl_ros
  message_generation
)

add_message_files(
  FILES
  AlignAndGoalPoseStamped.msg
)

add_service_files(
  FILES
  TriggerWithCommand.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_runtime
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.10 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

# include directories
include_directories(
	include
	${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/WoorobotSDK/include
)

# link directories
link_directories(
	include
)

catkin_install_python(PROGRAMS
  # klp_detection/klp_detection.py klp_detection/klp_detection_pub.py
  script/master_node.py
  script/obb_calculator.py
  script/mode_converter.py
  script/myactionlib.py
  script/map_regenerater.py
  script/server_fastapi.py
  script/server_ws.py
  script/record_rosbag.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(conversion src/conversion.cpp)
target_link_libraries(conversion ${catkin_LIBRARIES})

add_library(motor_controller_lib src/motor_controller.cpp)
target_link_libraries(motor_controller_lib PRIVATE woorobot_sdk ${catkin_LIBRARIES})
target_include_directories(motor_controller_lib PUBLIC include)

add_executable(vel_converter_sdk src/vel_converter_sdk.cpp)
target_link_libraries(vel_converter_sdk PRIVATE motor_controller_lib ${catkin_LIBRARIES})

# add_executable(sim_vel_converter src/sim_vel_converter.cpp)
# target_link_libraries(sim_vel_converter ${catkin_LIBRARIES})

# add_executable(sim_odom src/sim_odom.cpp)
# target_link_libraries(sim_odom ${catkin_LIBRARIES})

add_executable(dynamic_map_publisher src/dynamic_map_publisher.cpp)
target_link_libraries(dynamic_map_publisher ${catkin_LIBRARIES})

add_executable(imu_receiver src/imu_receiver.cpp)
target_link_libraries(imu_receiver ${catkin_LIBRARIES})

add_executable(async_scan_merger_real_2d src/async_scan_merger_real_2d.cpp)
target_link_libraries(async_scan_merger_real_2d ${catkin_LIBRARIES})

add_executable(odom_publisher src/odom_publisher.cpp)
target_link_libraries(odom_publisher ${catkin_LIBRARIES})

add_executable(filter_pcd src/filter_pcd.cpp)
target_link_libraries(filter_pcd ${catkin_LIBRARIES})
