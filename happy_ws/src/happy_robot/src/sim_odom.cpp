#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <vector>
#include <stdint.h>
#include <cmath>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <serial/serial.h>
#include <string>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_broadcaster.h>
#include <dynamic_reconfigure/client.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <std_srvs/Trigger.h>
#include <fcntl.h>      // 파일 제어
#include <termios.h>    // 시리얼 통신 설정
#include <unistd.h>     // POSIX 함수
#include <errno.h>      // 에러 처리
#include <sensor_msgs/JointState.h>
// #include <signal.h>

const double PI = 3.141592653589793;
bool wait_for_steer = false;

class MotorStateReader {
    public:
        MotorStateReader(ros::NodeHandle& nh) {
            joint_state_sub_ = nh.subscribe(
                "/swerve_drive_robot/joint_states", 10,
                &MotorStateReader::jointStateCallback, this);
            
            odom_pub_ = nh.advertise<nav_msgs::Odometry>("/odom/raw", 10);
            
            motor_names_ = {
                "front_left_wheel_joint",
                "front_right_wheel_joint",
                "rear_left_wheel_joint",
                "rear_right_wheel_joint",
                "front_left_steering_joint",
                "front_right_steering_joint",
                "rear_left_steering_joint",
                "rear_right_steering_joint"
            };

            x_ = 0.0;
            y_ = 0.0;
            theta_ = 0.0;
            vx_ = 0.0;
            vy_ = 0.0;
            vtheta_ = 0.0;
            is_first_msg_ = true;
            wait_for_steer = false;
        }

        void outputMotorValuesLoop() {
            ros::spin();
        }

        void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {

            // ROS_WARN("jointStateCallback");
            
            std::map<std::string, double> joint_values;
            std::vector<float> joint_motor_values(8, 0.0);
            for (size_t i = 0; i < msg->name.size(); ++i) {
                if (std::find(motor_names_.begin(), motor_names_.end(), msg->name[i]) != motor_names_.end()) {
                    if (msg->name[i].find("steering_joint") != std::string::npos) {
                        joint_values[msg->name[i]] = msg->position[i];
                    } else if (msg->name[i].find("wheel_joint") != std::string::npos) {
                        joint_values[msg->name[i]] = msg->velocity[i];
                    }
                }
            }
            
            for (size_t i = 0; i < motor_names_.size(); ++i) {
                if (joint_values.find(motor_names_[i]) != joint_values.end()) {
                    joint_motor_values[i] = joint_values[motor_names_[i]];
                } else {
                    ROS_WARN("Motor %s data not found, using default 0.0", motor_names_[i].c_str());
                    joint_motor_values[i] = 0.0;
                    ros::shutdown();
                }
            }
            updateOdometry(joint_motor_values);
        }

        void updateOdometry(std::vector<float> wheel_motor_values) {

            std::lock_guard<std::mutex> lock(odom_mutex_);

            if (wheel_motor_values.size() != 8) return;

            ros::Time current_time = ros::Time::now();

            if (is_first_msg_) {
                last_time_ = current_time;
                is_first_msg_ = false;
                return;
            }

            double dt = (current_time - last_time_).toSec();
            if (dt < 1e-6 || dt > 10.0) {
                ROS_WARN("[SIM odom] Invalid dt: %f", dt);
                return;
            }

            double vx_sum = 0.0, vy_sum = 0.0, vtheta_sum = 0.0;

            std::vector<double> wheel_vx_vec(4, 0.0);
            std::vector<double> wheel_vy_vec(4, 0.0);

            for (int i = 0; i < 4; i++) {

                double vel = wheel_motor_values[i] * wheel_vertical_coords[i];  // 컨트롤러에서 이미 방향 반영됨
                double steer = (wheel_motor_values[i + 4]) * steer_coords[i];  // 조향 각도 (도 단위)

                // double wheel_rpm = abs(wheel_motor_values[i]) > 0.001 ? wheel_motor_values[i] * wheel_vertical_coords[i] : 0; 
                // double wheel_degree = abs(wheel_motor_values[i+4]) > 0.001 ? wheel_motor_values[i+4] * steer_coords[i] : 0;

                double wheel_linear_vel = vel * WHEEL_RADIUS ;
                double wheel_vx = wheel_linear_vel * cos(steer);
                double wheel_vy = wheel_linear_vel * sin(steer);

                wheel_vx_vec[i] = wheel_vx;
                wheel_vy_vec[i] = wheel_vy;

                vx_sum += wheel_vx;
                vy_sum += wheel_vy;

                double wx = WHEEL_POSITIONS[i].second;  // y좌표 (좌/우)
                double wy = WHEEL_POSITIONS[i].first;   // x좌표 (앞/뒤)
                // vtheta_sum += -wheel_vx / wx;
            }

            vx_ = vx_sum / 4.0;
            vy_ = vy_sum / 4.0;

            double omega_sum = 0.0;
            const double epsilon = 1e-5;
            int count = 0;

            for (int i=0; i<4; i++) {
                double x = WHEEL_POSITIONS[i].first;
                double y = WHEEL_POSITIONS[i].second;
                // 만약 y가 충분히 크다면 (foros::Duration(0.0005).sleep();rward 식 사용)
                if (fabs(y) > epsilon) {
                    double omega_est = (vx_ - wheel_vx_vec[i]) / y;
                    omega_sum += omega_est;
                    count++;
                }
                // 만약 x가 충분히 크다면 (다른 식 사용)
                if (fabs(x) > epsilon) {
                    double omega_est = (wheel_vy_vec[i] - vy_) / x;
                    omega_sum += omega_est;
                    count++;
                }
            }
            double wz = (count > 0) ? (omega_sum / count) : 0.0;
            vtheta_ = wz;
            // ROS_INFO("updateOdometry \nlinear x : %.2f\nlinear y %.2f\nangular z : %.2f\n", vx_, vy_, vtheta_);

            x_ += (vx_ * cos(theta_) - vy_ * sin(theta_)) * dt;
            y_ += (vx_ * sin(theta_) + vy_ * cos(theta_)) * dt;
            theta_ += vtheta_ * dt;
            theta_ = fmod(theta_ + M_PI, 2 * M_PI) - M_PI;  // -π ~ π 정규화

            last_time_ = current_time;

            publishOdometry(current_time);
        }

        void publishOdometry(const ros::Time& time) {
            nav_msgs::Odometry odom;
            odom.header.stamp = time;
            odom.header.frame_id = "odom";
            odom.child_frame_id = "base_link";

            // 위치
            odom.pose.pose.position.x = x_;
            odom.pose.pose.position.y = y_;
            odom.pose.pose.position.z = 0.0;
            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta_);
            odom.pose.pose.orientation = odom_quat;

            // 속도
            odom.twist.twist.linear.x = vx_;
            odom.twist.twist.linear.y = vy_;
            odom.twist.twist.angular.z = vtheta_;

            // 공분산 (간단히 고정값 사용)
            odom.pose.covariance = {0.01, 0, 0, 0, 0, 0,
                                    0, 0.01, 0, 0, 0, 0,
                                    0, 0, 0.01, 0, 0, 0,
                                    0, 0, 0, 0.01, 0, 0,
                                    0, 0, 0, 0, 0.01, 0,
                                    0, 0, 0, 0, 0, 0.03};
            odom.twist.covariance = odom.pose.covariance;
            odom_pub_.publish(odom);
        }
    
    private:
        ros::Subscriber joint_state_sub_;
        std::vector<std::string> motor_names_;
        std::mutex odom_mutex_;
        ros::Publisher odom_pub_;
        tf2_ros::TransformBroadcaster tf_broadcaster_;
        const std::string odom_frame = "odom"; 
        const std::string base_frame = "base_link"; 
        ros::Time last_time_;
        bool is_first_msg_ = true;
        double x_, y_, theta_;        // 위치와 방향
        double vx_, vy_, vtheta_;
                // for mini
        // double WHEELBASE = 0.57;
        // double TRACK_WIDTH = 0.34;
        // for real
        double WHEELBASE = 5.65;
        double TRACK_WIDTH = 2.35;
        double WHEEL_RADIUS = 0.102;
        const std::vector<std::pair<double, double>> WHEEL_POSITIONS = {
            {WHEELBASE/2, TRACK_WIDTH/2},   // Front-left
            {WHEELBASE/2, -TRACK_WIDTH/2},  // Front-right
            {-WHEELBASE/2, TRACK_WIDTH/2},  // Rear-left
            {-WHEELBASE/2, -TRACK_WIDTH/2}  // Rear-right
        };
        bool MODE_VERTICAL;
        bool MODE_STOP;
        bool dwa_update_ = false;
        bool steer_angle_filter_ = true;
        uint8_t u8RxNumberOfData = 0;
        int LED_PIN = 11;
        double max_lin_speed = 0.5;
        double max_ang_speed = 0.5;
        double max_outer_ang_degree = 60;
        double max_inner_ang_degrees = 60;
        float steer_angle_limit = 20;
        std::vector<int> steer_coords = {-1, -1, -1, -1};
        std::vector<int> wheel_vertical_coords = {-1, 1, -1, 1};
        std::vector<double> vertical_mode_ = {max_lin_speed, -max_lin_speed, 0.0, 0.0, max_ang_speed, 0.0};
        std::vector<double> horizontal_mode_ = {0.0, 0.0, max_lin_speed, -max_lin_speed, 0.0, max_ang_speed};
        serial::Serial ser;
        std::mutex vel_mutex;
        std::mutex odom_mutex;
        std::condition_variable cv;
        ros::ServiceServer ping_service;
        ros::ServiceServer brake_service;
        ros::ServiceServer motor_init_service;
        ros::ServiceServer tank_service;
        std::vector<ros::Publisher> drive_pubs_;
        std::vector<ros::Publisher> steer_pubs_;
        std::vector<float> lastest_wheel_motor_values = {0, 0, 0, 0, 0, 0, 0, 0};
        std::vector<float> lastest_wheel_motor_inputs = {0, 0, 0, 0, 0, 0, 0, 0};
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "odom_node");
    ROS_INFO("Robot twist controller is running...");
	try {
        ros::NodeHandle nh;
        MotorStateReader reader(nh);
        std::thread loop_thread(&MotorStateReader::outputMotorValuesLoop, &reader);
        if (loop_thread.joinable()) loop_thread.join();
        else {
            ROS_ERROR_STREAM("loop_thread not joinable ");
            ros::shutdown();
        }
    }
    catch (const std::exception& e) {
        ROS_ERROR_STREAM("std::exception: " << e.what());
        ros::shutdown();
        // return 1;
    }
    catch (...) {
        ROS_ERROR_STREAM("Unknown Exception.");
        ros::shutdown();
        // return 1;
    }
    ROS_INFO("Shutdown !");
	return 0;
}