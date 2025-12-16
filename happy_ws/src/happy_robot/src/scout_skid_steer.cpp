// /*
//  * scout_skid_steer.cpp
//  *
//  * Created on: Mar 25, 2020 22:54
//  * Description:
//  *
//  * Copyright (c) 2019 Ruixiang Du (rdu)
//  */

// #include "scout_gazebo/scout_skid_steer.hpp"

// #include <geometry_msgs/Twist.h>
// #include <std_msgs/Float64.h>

// namespace wescore {
// ScoutSkidSteer::ScoutSkidSteer(ros::NodeHandle *nh, std::string robot_name)
//     : nh_(nh), robot_name_(robot_name) {
//   motor_fr_topic_ = robot_name_ + "/scout_motor_fr_controller/command";
//   motor_fl_topic_ = robot_name_ + "/scout_motor_fl_controller/command";
//   motor_rl_topic_ = robot_name_ + "/scout_motor_rl_controller/command";
//   motor_rr_topic_ = robot_name_ + "/scout_motor_rr_controller/command";
//   //
//   arm_pelvis_topic_ = robot_name_ + "/arm_controller_pelvis/command";
//   arm_hip_topic_ = robot_name_ + "/arm_controller_hip/command";
//   arm_elbow_topic_ = robot_name_ + "/arm_controller_elbow/command";
//   arm_wrist_topic_ = robot_name_ + "/arm_controller_wrist/command";
//   cmd_topic_ = robot_name_ + "/cmd_vel";
// }

// void ScoutSkidSteer::SetupSubscription() {
//   // command subscriber
//   cmd_sub_ = nh_->subscribe<geometry_msgs::Twist>(
//       cmd_topic_, 9, &ScoutSkidSteer::TwistCmdCallback, this);

//   // motor command publisher
//   motor_fr_pub_ = nh_->advertise<std_msgs::Float64>(motor_fr_topic_, 50);
//   motor_fl_pub_ = nh_->advertise<std_msgs::Float64>(motor_fl_topic_, 50);
//   motor_rl_pub_ = nh_->advertise<std_msgs::Float64>(motor_rl_topic_, 50);
//   motor_rr_pub_ = nh_->advertise<std_msgs::Float64>(motor_rr_topic_, 50);
//   //
//   arm_pelvis_pub_ = nh_->advertise<std_msgs::Float64>(arm_pelvis_topic_, 50);
//   arm_hip_pub_ = nh_->advertise<std_msgs::Float64>(arm_hip_topic_, 50);
//   arm_elbow_pub_ = nh_->advertise<std_msgs::Float64>(arm_elbow_topic_, 50);
//   arm_wrist_pub_ = nh_->advertise<std_msgs::Float64>(arm_wrist_topic_, 50);
// }

// void ScoutSkidSteer::TwistCmdCallback(
//     const geometry_msgs::Twist::ConstPtr &msg) {
//   std_msgs::Float64 motor_cmd[8];

//   double driving_vel = msg->linear.x;
//   double steering_vel = msg->angular.z;

//   double wrist_vel = msg->angular.x;
//   double pelvis_vel = msg->angular.y;

//   double elbow_vel = msg->linear.z;
//   double hip_vel =  msg->linear.y;


//   double left_side_velocity =
//       (driving_vel - steering_vel * SCOUT_WHEELBASE) / SCOUT_WHEEL_RADIUS;
//   double right_side_velocity =
//       (driving_vel + steering_vel * SCOUT_WHEELBASE) / SCOUT_WHEEL_RADIUS;

//   motor_cmd[0].data = right_side_velocity;
//   motor_cmd[1].data = -left_side_velocity;
//   motor_cmd[2].data = -left_side_velocity;
//   motor_cmd[3].data = right_side_velocity;

//   motor_cmd[4].data = pelvis_vel;
//   motor_cmd[5].data = hip_vel;
//   motor_cmd[6].data = elbow_vel;
//   motor_cmd[7].data = wrist_vel;

//   motor_fr_pub_.publish(motor_cmd[0]);
//   motor_fl_pub_.publish(motor_cmd[1]);
//   motor_rl_pub_.publish(motor_cmd[2]);
//   motor_rr_pub_.publish(motor_cmd[3]);
//   arm_pelvis_pub_.publish(motor_cmd[4]);
//   arm_hip_pub_.publish(motor_cmd[5]);
//   arm_elbow_pub_.publish(motor_cmd[6]);
//   arm_wrist_pub_.publish(motor_cmd[7]);
// }

// }  // namespace wescore