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

class VelConverter {
    
    public:
        explicit VelConverter(ros::NodeHandle& n) : nh_(n), MODE_VERTICAL(true), MODE_STOP(false) {
            // Topic names.
            const std::string vel_topic = "/cmd_vel";
            const std::string control_mode_topic = "/control_mode";
            const std::string stop_mode_topic = "/stop_mode";
            cmd_vel_sub = nh_.subscribe(vel_topic, 10, &VelConverter::smoothSwerveDriveControllerCallback, this);
            // cmd_vel_sub = nh_.subscribe(vel_topic, 10, &VelConverter::twistCallback, this);
            
            x_ = 0.0;
            y_ = 0.0;
            theta_ = 0.0;
            vx_ = 0.0;
            vy_ = 0.0;
            vtheta_ = 0.0;
            is_first_msg_ = true;
            wait_for_steer = false;

            // 생성자에서 퍼블리셔 초기화
            drive_pubs_ = {
                nh_.advertise<std_msgs::Float64>("/swerve_drive_robot/front_left_wheel_joint_controller/command", 1),
                nh_.advertise<std_msgs::Float64>("/swerve_drive_robot/front_right_wheel_joint_controller/command", 1),
                nh_.advertise<std_msgs::Float64>("/swerve_drive_robot/rear_left_wheel_joint_controller/command", 1),
                nh_.advertise<std_msgs::Float64>("/swerve_drive_robot/rear_right_wheel_joint_controller/command", 1)
            };
            steer_pubs_ = {
                nh_.advertise<std_msgs::Float64>("/swerve_drive_robot/front_left_steering_joint_controller/command", 1),
                nh_.advertise<std_msgs::Float64>("/swerve_drive_robot/front_right_steering_joint_controller/command", 1),
                nh_.advertise<std_msgs::Float64>("/swerve_drive_robot/rear_left_steering_joint_controller/command", 1),
                nh_.advertise<std_msgs::Float64>("/swerve_drive_robot/rear_right_steering_joint_controller/command", 1)
            };
        }

        void smoothSwerveDriveControllerCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        
            std::vector<uint8_t> control_params;
        
            double vx = msg->linear.x;
            double vy = msg->linear.y;
            double wz = msg->angular.z;
        
            std::stringstream ss;

            std::vector<float> wheel_motor_inputs(8, 0.0);

            std::vector<double> drive_motor_inputs(4, 0.0);
            std::vector<double> steer_motor_inputs(4, 0.0);
        
            // 각 바퀴의 속도와 각도 계산
            for (int i = 0; i < 4; i++) {
                double x = WHEEL_POSITIONS[i].first;
                double y = WHEEL_POSITIONS[i].second;
        
                // 바퀴 속도 성분 계산
                double wheel_vx = vx - wz * y;
                double wheel_vy = vy + wz * x;
        
                // 속도 크기와 각도 계산
                double speed = sqrt(wheel_vx * wheel_vx + wheel_vy * wheel_vy);
                double angle_rad = atan2(wheel_vy, wheel_vx);
                double angle_deg = angle_rad * 180.0 / M_PI * steer_coords[i];
        
                // RPM 계산
                double rpm = (speed / (2.0 * M_PI * WHEEL_RADIUS)) * 60.0;
        
                // 각도 최적화: 90° 이상 회전 시 방향 반전
                if (fabs(angle_deg) > 90.0) {
                    // ROS_INFO("reverse");
                    angle_deg = angle_deg - 180.0;  // 각도를 반대 방향으로
                    rpm = -rpm;                     // RPM 부호 반전
                    if (angle_deg > 180.0) angle_deg -= 360.0;  // 각도 정규화
                    if (angle_deg < -180.0) angle_deg += 360.0;
                } else if (fabs(angle_deg + 90) < 1e-6 && vx == 0 && wz ==0) {
                    // ROS_INFO("angle reverse");
                    angle_deg = -angle_deg;
                    rpm = -rpm;
                }
    
                // 모터 방향 조정 (오른쪽 바퀴)
                rpm = rpm * wheel_vertical_coords[i];

                // 디버깅 출력
                // ss << "ID : " << i+1 << ", RPM : " << rpm << "\n";
                // ss << "ID : " << i+5 << ", degree : " << angle_deg << "\n";
                wheel_motor_inputs[i] = rpm;
                wheel_motor_inputs[i+4] = angle_deg;
            }

            std::vector<float> steer_angle_diff = std::vector<float>(4, 0.0);

            for (int idx =4; idx<8; idx++) {
                steer_angle_diff.push_back(abs(lastest_wheel_motor_inputs[idx] - wheel_motor_inputs[idx]));
            }
            float max_diff = *std::max_element(steer_angle_diff.begin(), steer_angle_diff.end());
            if (max_diff >= steer_angle_limit) {

                // ROS_WARN("max_diff overs steer_angle_limit");

                for (int i = 0; i < 4; i++) {
                    std_msgs::Float64 drive_msg;
                    // drive_msg.data = drive_motor_inputs[i]; // RPM -> rad/s
                    drive_msg.data = 0;
                    drive_pubs_[i].publish(drive_msg);
                
                    std_msgs::Float64 steer_msg;
                    // steer_msg.data = steer_motor_inputs[i]; // degree -> radians
                    steer_msg.data = wheel_motor_inputs[i + 4] * M_PI / 180.0;
                    steer_pubs_[i].publish(steer_msg);

                }

                ros::Duration(0.8).sleep();

                for (int i = 0; i < 4; i++) {
                    std_msgs::Float64 drive_msg;
                    // drive_msg.data = drive_motor_inputs[i]; // RPM -> rad/s
                    drive_msg.data = wheel_motor_inputs[i] * (2 * M_PI / 60.0);
                    drive_pubs_[i].publish(drive_msg);
        
                }
     
            } else {

                // ROS_INFO("max_diff lowers steer_angle_limit");

                for (int i = 0; i < 4; i++) {
                    std_msgs::Float64 drive_msg;
                    // drive_msg.data = drive_motor_inputs[i]; // RPM -> rad/s
                    drive_msg.data = wheel_motor_inputs[i] * (2 * M_PI / 60.0);
                    drive_pubs_[i].publish(drive_msg);
                
                    std_msgs::Float64 steer_msg;
                    // steer_msg.data = steer_motor_inputs[i]; // degree -> radians
                    steer_msg.data = wheel_motor_inputs[i + 4] * M_PI / 180.0;
                    steer_pubs_[i].publish(steer_msg);
                }
            }

            {
                lastest_wheel_motor_inputs = {};
                lastest_wheel_motor_inputs.resize(8);
                for (int idx=0; idx<lastest_wheel_motor_inputs.size(); idx++) {
                    lastest_wheel_motor_inputs[idx] = wheel_motor_inputs[idx];
                }
            }


            // updateOdometry(to_odom);
        
            // ROS_INFO("smoothSwerveDriveCallback : \n%s", ss.str().c_str());
        }

        void twistCallback(const geometry_msgs::Twist::ConstPtr& msg) {

            double linear_x = msg->linear.x;
            double linear_y = msg->linear.y;
            double angular_z = msg->angular.z;

            double this_track_width = TRACK_WIDTH;
            double this_wheel_base = WHEELBASE;
            double linear_vel = linear_x;

            double R, curvature;

            if (fabs(angular_z) < 1e-6) {
                R = 1e6;  // angular_z가 0이면 직진 (무한대에 가까운 값)
                curvature = 0.0;
            } else {
                curvature = angular_z / linear_vel;
                R = linear_vel / angular_z;
            }

            // 조향각 계산
            double theta_fl_rad = atan((this_wheel_base / 2.0) / (R - this_track_width / 2.0));
            double theta_fr_rad = atan((this_wheel_base / 2.0) / (R + this_track_width / 2.0));
            double theta_rl_rad = -theta_fl_rad;
            double theta_rr_rad = -theta_fr_rad;

            double theta_fl = theta_fl_rad * 180.0 / M_PI;
            double theta_fr = theta_fr_rad * 180.0 / M_PI;
            double theta_rl = theta_rl_rad * 180.0 / M_PI;
            double theta_rr = theta_rr_rad * 180.0 / M_PI;

            // 바퀴 속도 계산
            double v_fl = linear_vel * ((R - this_track_width / 2.0) / R);
            double v_fr = linear_vel * ((R + this_track_width / 2.0) / R);
            double v_rl = v_fl;
            double v_rr = v_fr;

            if (steer_angle_filter_) {
                // outer angle < inner_angle
                double outer_angle = theta_fl >= theta_fr ? theta_fr : theta_fl;
                double inner_angle = theta_fl >= theta_fr ? theta_fl : theta_fr;

                // outer_angle = abs(outer_angle) > max_outer_ang_degree ? std::copysign(max_outer_ang_degree, outer_angle) : outer_angle;
                // inner_angle = abs(inner_angle) > max_inner_ang_degrees ? std::copysign(max_inner_ang_degrees, inner_angle) : inner_angle;

                theta_fl = theta_fl >= theta_fr ? inner_angle : outer_angle;
                theta_fr = theta_fr >= theta_fl ? inner_angle : outer_angle;

                theta_rl = -theta_fl; 
                theta_rr = -theta_fr;
            }

            if (fabs(angular_z) > 0.01 && fabs(linear_vel) < 0.01) {
                double turn_angle = 66.13;
                double R_wheel = sqrt(pow(this_wheel_base / 2.0, 2) + pow(this_track_width / 2.0, 2));
                double turn_speed = angular_z * R_wheel;

                theta_fl = turn_angle;
                theta_fr = -turn_angle;
                theta_rl = -turn_angle;
                theta_rr = turn_angle;

                theta_fl = -theta_fl;
                theta_fr = -theta_fr;
                theta_rl = -theta_rl;
                theta_rr = -theta_rr;

                v_fl = turn_speed;
                v_fr = -turn_speed;
                v_rl = turn_speed;
                v_rr = -turn_speed;
            }

            std::vector<double> steer_ang_vector = {theta_fl, theta_fr, theta_rl, theta_rr};
            std::vector<double> wheel_vel_vector = {v_fl, v_fr, v_rl, v_rr};

            std::vector<uint8_t> control_params;

            for (size_t i = 0; i < 4; i++) {

                double velocity = wheel_vel_vector[i];

                velocity = wheel_vertical_coords[i] * velocity;
                // velocity = abs(velocity) > max_lin_speed ? std::copysign(max_lin_speed, velocity) : velocity;

                double rpm_double = (velocity / (2 * PI * WHEEL_RADIUS)) * 60; // m/s -> RPM
                int32_t rpm = (MODE_STOP) ? 0 : static_cast<int32_t>(rpm_double);
                std::cout << "ID : " << i+1 <<", RPM : " << rpm << std::endl;
                std_msgs::Float64 drive_msg;
                drive_msg.data = rpm * (2 * M_PI / 60.0);
                drive_pubs_[i].publish(drive_msg);
            }

            for (size_t i = 0; i < 4; i++) {

                double steer_angle = steer_ang_vector[i] * steer_coords[i];
                std::cout << "ID : " << i+5 <<", Degree : " << steer_angle << std::endl;
                std_msgs::Float64 steer_msg;
                steer_msg.data = steer_angle * M_PI / 180.0;
                steer_pubs_[i].publish(steer_msg);
            }
        }

        void updateOdometry(std::vector<float> wheel_motor_values) {

            if (wheel_motor_values.size() != 8) return;

            ros::Time current_time = ros::Time::now();

            if (is_first_msg_) {
                last_time_ = current_time;
                is_first_msg_ = false;
                return;
            }

            double dt = (current_time - last_time_).toSec();
            if (dt < 1e-6 || dt > 1.0) {
                ROS_WARN("Invalid dt: %f", dt);
                // return;
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
            ROS_INFO("updateOdometry \nlinear x : %.2f\nlinear y %.2f\nangular z : %.2f\n", vx_, vy_, vtheta_);

            x_ += (vx_ * cos(theta_) - vy_ * sin(theta_)) * dt;
            y_ += (vx_ * sin(theta_) + vy_ * cos(theta_)) * dt;
            theta_ += vtheta_ * dt;
            theta_ = fmod(theta_ + M_PI, 2 * M_PI) - M_PI;  // -π ~ π 정규화

            last_time_ = current_time;
        }

        void subscribeControlThread() {
            ros::spin();
        }

        ~VelConverter() {
        }

    private:
        ros::NodeHandle& nh_;
        ros::Subscriber cmd_vel_sub;
        ros::Subscriber mode_control_sub;
        ros::Subscriber mode_stop_sub;
        ros::Publisher odom_pub_;
        tf2_ros::TransformBroadcaster tf_broadcaster_;
        const std::string odom_frame = "odom"; 
        const std::string base_frame = "base_link"; 
        ros::Time last_time_;
        bool is_first_msg_ = true;
        double x_, y_, theta_;        // 위치와 방향
        double vx_, vy_, vtheta_;
        float steer_angle_limit = 360;
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
        std::vector<int> steer_coords = {-1, -1, -1, -1};
        std::vector<int> wheel_vertical_coords = {-1, 1, -1, 1};
        std::vector<uint8_t> zero_rpm_motor_control = {0, 0, 0, 0};
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
	ros::init(argc, argv, "robot_twist_controller");
    ROS_INFO("Robot twist controller is running...");
	try {
        ros::NodeHandle nh;
        VelConverter velConverter(nh);
        std::thread subscribe_thread(&VelConverter::subscribeControlThread, &velConverter);
        if (subscribe_thread.joinable()) subscribe_thread.join();
        else {
            ROS_ERROR_STREAM("subscribe_thread not joinable ");
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