#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <vector>
#include <stdint.h>
#include <cmath>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
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
#include <happy_robo/TriggerWithCommand.h>
#include "motor_controller.h"

const double PI = 3.141592653589793;
bool wait_for_steer = false;
bool handeling_service = false;
bool handeling_rx = false;

class VelConverter {
    
    public:
        explicit VelConverter(ros::NodeHandle& n) : nh_(n), MODE_VERTICAL(true), MODE_STOP(false) {
            // Topic names.
            const std::string vel_topic = "/cmd_vel";
            const std::string control_mode_topic = "/control_mode";
            const std::string stop_mode_topic = "/stop_mode";
            cmd_vel_sub = nh_.subscribe(vel_topic, 10, &VelConverter::smoothSwerveDriveControllerCallback, this);
            mode_stop_sub = nh_.subscribe(stop_mode_topic, 1, &VelConverter::stopModeCallback, this);
            odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom/raw", 10);
            ping_service = nh_.advertiseService("ping_service", &VelConverter::handlePing, this);
            brake_service = nh_.advertiseService("brake_service", &VelConverter::handleBrake, this);
            motor_init_service = nh_.advertiseService("motor_init_service", &VelConverter::handleMotorInit, this);
            tank_service = nh_.advertiseService("tank_service", &VelConverter::handleTank, this);
            x_ = 0.0;
            y_ = 0.0;
            theta_ = 0.0;
            vx_ = 0.0;
            vy_ = 0.0;
            vtheta_ = 0.0;
            is_first_msg_ = true;
            wait_for_steer = false;
            try {
                motor_controller = new MotorController(serial_port, speed, motor_number);
            }
            catch (const std::exception& e) {
                ROS_ERROR_STREAM("std::exception from init Serial Connector: " << e.what());
                ros::shutdown();
                // return 1;
            }
            catch (...) {
                ROS_ERROR_STREAM("Unknown Exception.");
                ros::shutdown();
                // return 1;
            }
        }

        void smoothSwerveDriveControllerCallback(const geometry_msgs::Twist::ConstPtr& msg) {
            const double MAX_LINEAR_SPEED = 0.5;  // 최대 선속도 (m/s)
            const double MAX_ANGULAR_SPEED = 0.5;  // 최대 각속도 (rad/s)

            std::vector<uint8_t> control_params;
        
            double vx = msg->linear.x;
            double vy = msg->linear.y;
            double wz = msg->angular.z;
        
            // 속도 제한
            vx = std::max(-MAX_LINEAR_SPEED, std::min(MAX_LINEAR_SPEED, vx));
            vy = std::max(-MAX_LINEAR_SPEED, std::min(MAX_LINEAR_SPEED, vy));
            wz = std::max(-MAX_ANGULAR_SPEED, std::min(MAX_ANGULAR_SPEED, wz));
        
            std::stringstream ss;

            ss << "Lin X : " << vx << ", Lin Y : " << vy << ", Ang z : "  << wz << "\n";

            std::vector<float> wheel_motor_inputs(8, 0.0);
            std::vector<float> wheel_motor_inputs_origin(8, 0.0);

            std::vector<float> drive_motor_inputs(4);
            std::vector<float> steer_motor_inputs(4);
        
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
                    ROS_INFO("reverse");
                    angle_deg = angle_deg - 180.0;  // 각도를 반대 방향으로
                    rpm = -rpm;                     // RPM 부호 반전
                    if (angle_deg > 180.0) angle_deg -= 360.0;  // 각도 정규화
                    if (angle_deg < -180.0) angle_deg += 360.0;
                } else if (angle_deg == -90.0 && vx == 0 && wz ==0) {
                    //// For Demo
                    ROS_INFO("angle reverse");
                    angle_deg = -angle_deg;
                    rpm = -rpm;
                }
    
                // 모터 방향 조정 (오른쪽 바퀴)
                rpm = rpm * wheel_coords[i];

                // 디버깅 출력
                ss << "ID : " << i+1 << ", RPM : " << rpm << "\n";
                ss << "ID : " << i+5 << ", degree : " << angle_deg << "\n";
                wheel_motor_inputs_origin[i] = rpm * wheel_coords[i];
                wheel_motor_inputs_origin[i+4] = angle_deg * steer_coords[i];

                wheel_motor_inputs[i] = rpm;
                wheel_motor_inputs[i+4] = angle_deg;

                drive_motor_inputs[i] = static_cast<float> (rpm);
                steer_motor_inputs[i] = static_cast<float> (angle_deg);
            }
            int jdx = 0;
            for (auto motor_input : wheel_motor_inputs_origin) {
                if (motor_input > 75) {
                    ROS_ERROR("%d motor_input overs 75 : %.3f", jdx, motor_input);
                }
                jdx++;
            }
            {
                std::unique_lock<std::mutex> lock(odom_mutex);
                lastest_wheel_motor_inputs = {};
                lastest_wheel_motor_inputs.resize(8);
                for (int idx=0; idx<lastest_wheel_motor_inputs.size(); idx++) {
                    lastest_wheel_motor_inputs[idx] = wheel_motor_inputs[idx];
                }
            }
            ROS_INFO("smoothSwerveDriveCallback : \n%s", ss.str().c_str());
            if (separate_mode_) {
                bool AmIWait = false;
                {
                    std::unique_lock<std::mutex> lock(odom_mutex);
                    std::vector<float> steer_angle_diff = std::vector<float>(4, 0.0);
                    for (int idx =4; idx<8; idx++) {
                        steer_angle_diff.push_back(abs(lastest_wheel_motor_inputs[idx] - lastest_wheel_motor_values[idx]));
                    }
                    float max_diff = *std::max_element(steer_angle_diff.begin(), steer_angle_diff.end());
                    if (max_diff >= steer_angle_limit) {
                        ROS_ERROR("max_diff overs steer_angle_limit");
                        wait_for_steer = true;
                    }
                    else {
                        ROS_INFO("max_diff lower than steer_angle_limit");
                        wait_for_steer = false;
                    }
                    AmIWait = wait_for_steer;
                }
                if (!AmIWait) {
                    for (int idx=0; idx<drive_motor_inputs.size(); idx++) {
                        motor_controller->setRPM(idx+1, drive_motor_inputs[idx]);
                    }
                    for (int idx=0; idx<steer_motor_inputs.size(); idx++) {
                       motor_controller->setDegree(idx+drive_motor_inputs.size()+1, steer_motor_inputs[idx]);
                    }     
                } else {
                    for (int idx=1; idx<5; idx++) {
                        motor_controller->setRPM(idx, 0);
                    }
                    ROS_INFO("RPM -> 0 Waiting for 3 sec...");
                    ros::Duration(3).sleep();
                    for (int idx=0; idx<steer_motor_inputs.size(); idx++) {
                       motor_controller->setDegree(idx+drive_motor_inputs.size()+1, steer_motor_inputs[idx]);
                    }    
                    ROS_INFO("Degree to target, Waiting for 3 sec...");
                    ros::Duration(3).sleep();
                    for (int idx=0; idx<drive_motor_inputs.size(); idx++) {
                        motor_controller->setRPM(idx+1, drive_motor_inputs[idx]);
                    } 
                } 
            } else {
                for (int idx=0; idx<drive_motor_inputs.size(); idx++) {
                    motor_controller->setRPM(idx+1, drive_motor_inputs[idx]);
                }
                for (int idx=0; idx<steer_motor_inputs.size(); idx++) {
                    motor_controller->setDegree(idx+drive_motor_inputs.size()+1, steer_motor_inputs[idx]);
                }   
            }
        }

        bool handlePing(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
            {
                std::unique_lock<std::mutex> lock(service_mutex);
                handeling_service = true;

                ROS_INFO("Ping Service has been called.");

                auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(1000);

                if (service_cv.wait_until(lock, deadline, [] { return !handeling_rx; })) {
                                std::vector<bool> ping_results(motor_number, false);
                    std::stringstream result_ss;

                    for (int idx = 0; idx < motor_number; idx++) {
                        bool ping_result = motor_controller->pingTest(idx+1);
                        ping_results[idx] = ping_result;
                        result_ss << (ping_result ? "True" : "False");
                        if (idx < motor_number - 1) result_ss << ",";
                        ROS_INFO("Ping result for motor ID %d: %s", idx + 1, ping_result ? "True" : "False");
                    }

                    res.message = result_ss.str();
                    res.success = true; // 서비스 호출 자체는 성공
                } else {
                    ROS_WARN("Timeout reached while waiting for service");
                    res.message = "False,False,False,False,False,False,False,False";
                    res.success = false;
                }

                handeling_service = false;
                return true;
            }
        }

        bool handleBrake(happy_robo::TriggerWithCommand::Request& req, happy_robo::TriggerWithCommand::Response& res) {
            {
                std::unique_lock<std::mutex> lock(service_mutex);
                handeling_service = true;

                ROS_INFO("handleBrake Service has been called.");

                auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(1000);

                if (service_cv.wait_until(lock, deadline, [] { return !handeling_rx; })) {
                    int command = req.command;  // 커스텀 메시지의 command 필드에서 int32 값 읽기
                    if (command >= 0 && command < brake_control.size()) {
                        if (command == 2) {
                            ROS_INFO("Outter Brake needs Motor Stop");
                            motor_controller->setMotorBrake();
                            ros::Duration(1).sleep();
                        } else if (command == 3) {
                            ROS_INFO("Outter Brake Before");
                            motor_controller->setOutterBrakeBefore();
                            ros::Duration(1).sleep();
                        }
                        motor_controller->setMotorCmd(command);
                        ros::Duration(1).sleep();
                        res.message = "True";
                        res.success = true;
                    } else {
                        ROS_ERROR("Invalid command: %d", command);
                        res.message = "False";
                        res.success = false;
                    }
                } else {
                    ROS_WARN("Timeout reached while waiting for service");
                    res.message = "False";
                    res.success = false;
                }

                handeling_service = false;
                return true;
            }
        }

        bool handleMotorInit(happy_robo::TriggerWithCommand::Request& req, happy_robo::TriggerWithCommand::Response& res) {

            {
                std::unique_lock<std::mutex> lock(service_mutex);
                handeling_service = true;

                ROS_INFO("handleMotorInit Service has been called.");

                auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(1000);

                if (service_cv.wait_until(lock, deadline, [] { return !handeling_rx; })) {
                    int command = req.command;  // 0~8 (Flutter에서 1~9로 보낸 값에서 -1 처리됨)
                    ROS_INFO("Motor Init Service has been called with command: %d", command);

                    if (command >= 0 && command < motor_number) {
                        if (command == 0) {
                            ROS_INFO("Init Setup needs outter non brake");
                            ROS_INFO("Outter Brake Before");
                            motor_controller->setOutterBrakeBefore();
                            ros::Duration(1).sleep();
                            motor_controller->setMotorCmd(3);
                            ros::Duration(1).sleep();
                        }
                        motor_controller->initializeMotor(command+1);
                        ROS_INFO("Motor Init response for ID : %d", command);
                        res.message = "True";
                        res.success = true;
                        ros::Duration(0.05).sleep();
                    } else {
                        motor_controller->inputMotorParams(0);
                        ROS_ERROR("Invalid command inputMotorParams : %d", command);
                        res.message = "False";
                        res.success = false;
                    }
                } else {
                    ROS_WARN("Timeout reached while waiting for service");
                    res.message = "False";
                    res.success = false;
                }

                handeling_service = false;
                return true;
            }
        }

        bool handleTank(happy_robo::TriggerWithCommand::Request& req, happy_robo::TriggerWithCommand::Response& res) {
            int command = req.command;  // 0: tank_up, 1: tank_down
            double degree = req.degree;
            ROS_INFO("Tank Service has been called with command: %d, degree: %.2f", command, degree);
            res.message = "False";
            res.success = false;
        
            // if (ser.isOpen()) {
            //     if (command == 0 || command == 1) {
            //         std::vector<uint8_t> control_params;
            //         // inverse angle
            //         double tank_degree = command == 0 ? 0.0 : degree;

            //         InputDegree( static_cast<int32_t>(9), tank_degree, control_params);
            //         std::cout << "ID : " << 9 <<", Degree : " << tank_degree << ", ";

            //         ser.write(control_params);
            //         ros::Duration(0.05).sleep();
            //         if (ser.available()) {
            //             std::vector<uint8_t> response(ser.available());
            //             ser.read(response, response.size());
            //             std::stringstream ss;
            //             for (uint8_t byte : response) {
            //                 ss << std::hex << std::uppercase << (int)byte << " ";
            //             }
            //             ROS_INFO("Tank response for command %d: %s", command, ss.str().c_str());
            //             res.message = "True";
            //             res.success = true;
            //         } else {
            //             ROS_WARN("No response for command %d", command);
            //             res.message = "True";
            //             res.success = true;
            //         }
            //     } else {
            //         ROS_ERROR("Invalid command: %d", command);
            //         res.message = "False";
            //         res.success = false;
            //     }
            // } else {
            //     res.message = "False";
            //     res.success = false;
            //     ROS_ERROR("Serial port is not open!");
            // }
        
            return true;
        }

        void outputMotorValuesLoop() {
            ros::Rate rate(20); // 10Hz로 실행
            while (ros::ok()) {
                bool _handeling_service = false;
                {
                    std::unique_lock<std::mutex> lock(service_mutex);
                    _handeling_service = handeling_service;
                    handeling_rx = true;
                    std::cout << "_handeling_service : " << _handeling_service << std::endl;
                }
                if (!_handeling_service) {
                    ros::Time start_time = ros::Time::now();
                    outputMotorValuesSlow();
                    double whole_time = (ros::Time::now() - start_time).toSec();
                    ROS_INFO("HZ: %.1f, time: %.3f sec", 1/whole_time, whole_time);
                }
                {
                    std::unique_lock<std::mutex> lock(service_mutex);
                    handeling_rx = false;
                }
                service_cv.notify_all();
                rate.sleep();
            }
        }

        void outputMotorValuesSlow() {

            std::vector<float> wheel_motor_values;

            ros::Time start_time = ros::Time::now();

            int succeed = 0;

            for (int idx=1; idx<9; idx++) {

                ros::Time interval = ros::Time::now();
                ros::Duration timeout(0.1); // 0.1초 타임아웃
                bool received = false;

                float value = (idx < 5) ? motor_controller->getRPM(idx) : motor_controller->getDegree(idx);
                wheel_motor_values.push_back(value);
                received = true;
                succeed++;

                // while (ros::ok() && !received && (ros::Time::now() - interval) < timeout) {
                    
                // }
            
                // if (!received) {
                //     ROS_WARN("No response received at ID: %d (timeout after 0.1s)", idx);
                //     return;
                // }

                double wait_time = (ros::Time::now() - interval).toSec();
                // ROS_INFO("ID: %d, Wait time: %.6f sec processing", idx, wait_time);
            }

            if (succeed == motor_number) {
                // ROS_INFO("Received: %s", ss.str().c_str());
                // double whole_time = (ros::Time::now() - start_time).toSec();
                // ROS_INFO("HZ: %.1f, time: %.3f sec", 1/whole_time, whole_time);
                {
                    std::unique_lock<std::mutex> lock(odom_mutex);
                    lastest_wheel_motor_values = {};
                    int idx = 1;
                    for (auto value : wheel_motor_values) {
                        lastest_wheel_motor_values.push_back(value);
                        ROS_INFO("%d : %f", idx, value);
                        idx++;
                    }
                    std::vector<float> steer_angle_diff = std::vector<float>(4, 0.0);
                    for (int idx =4; idx<8; idx++) {
                        steer_angle_diff.push_back(abs(lastest_wheel_motor_inputs[idx] - lastest_wheel_motor_values[idx]));
                    }
                    float max_diff = *std::max_element(steer_angle_diff.begin(), steer_angle_diff.end());
                    if (max_diff >= steer_angle_limit) {
                        // ROS_INFO("max_diff overs steer_angle_limit");
                        wait_for_steer = true;
                    }
                    else {
                        // ROS_INFO("max_diff lower than steer_angle_limit");
                        wait_for_steer = false;
                    }
                    cv.notify_all();
                }
                updateOdometry(wheel_motor_values);
                std::stringstream ss;
                for (float value : wheel_motor_values) {
                    ss << value << " ";
                }
            } else {
                ROS_WARN("Invalid Odom Data");
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
            if (dt < 1e-6) {
                ROS_WARN("[Odom] Invalid dt: %f", dt);
                return;
            }

            double vx_sum = 0.0, vy_sum = 0.0, vtheta_sum = 0.0;

            std::vector<double> wheel_vx_vec(4, 0.0);
            std::vector<double> wheel_vy_vec(4, 0.0);

            for (int i = 0; i < 4; i++) {

                double wheel_rpm = wheel_motor_values[i];  // 컨트롤러에서 이미 방향 반영됨
                double wheel_degree = wheel_motor_values[i + 4];  // 조향 각도 (도 단위)

                // double wheel_rpm = abs(wheel_motor_values[i]) > 0.001 ? wheel_motor_values[i] * wheel_coords[i] : 0; 
                // double wheel_degree = abs(wheel_motor_values[i+4]) > 0.001 ? wheel_motor_values[i+4] * steer_coords[i] : 0;

                double wheel_linear_vel = wheel_rpm * WHEEL_RADIUS * (2.0 * M_PI / 60.0);
                double wheel_vx = wheel_linear_vel * cos(wheel_degree * M_PI / 180.0);
                double wheel_vy = wheel_linear_vel * sin(wheel_degree  * M_PI / 180.0);

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

            x_ += (vx_ * cos(theta_) - vy_ * sin(theta_)) * dt;
            y_ += (vx_ * sin(theta_) + vy_ * cos(theta_)) * dt;
            theta_ += vtheta_ * dt;
            theta_ = fmod(theta_ + M_PI, 2 * M_PI) - M_PI;  // -π ~ π 정규화

            last_time_ = current_time;

            ROS_INFO("updateOdometry fps(hz) : %.3f \nglobal x: %.3f\nglobal y : %.3f\nglobal_thera : %.3f\nlinear x : %.2f\nlinear y %.2f\nangular z : %.2f\n", 1/dt, x_, y_, theta_, vx_, vy_, vtheta_);

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

        void stopModeCallback(const std_msgs::Bool::ConstPtr& msg) {
            ROS_INFO("Got Stop Mode MSG : %s", msg->data);
            MODE_STOP = msg->data;
            if (MODE_STOP) {
                ROS_INFO("✅ Stop Mode Enabled.");
            } else {
                ROS_INFO("✅ Stop Mode Disabled.");
            }
        }

        void subscribeControlThread() {
            ros::spin();
        }

        ~VelConverter() {
            delete motor_controller;
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
        // double WHEELBASE = 5.65;
        // double TRACK_WIDTH = 2.35;
        double WHEELBASE = 1.135;
        double TRACK_WIDTH = 0.8;
        double WHEEL_RADIUS = 0.102;
        const std::vector<std::pair<double, double>> WHEEL_POSITIONS = {
            {WHEELBASE/2, TRACK_WIDTH/2},   // Front-left
            {WHEELBASE/2, -TRACK_WIDTH/2},  // Front-right
            {-WHEELBASE/2, TRACK_WIDTH/2},  // Rear-left
            {-WHEELBASE/2, -TRACK_WIDTH/2}  // Rear-right
        };
        bool MODE_VERTICAL;
        bool MODE_STOP;
        bool steer_angle_filter_ = true;
        bool separate_mode_ = false;
        double prevVx = 0.0;
        double prevVy = 0.0;
        double prevWz = 0.0;
        double max_lin_speed = 0.5;
        double max_ang_speed = 0.5;
        float steer_angle_limit = 30;
        std::vector<int> steer_coords = {-1, -1, -1, -1};
        std::vector<int> wheel_coords = {1, -1, 1, -1};
        std::vector<uint8_t> zero_rpm_motor_control;
        std::vector<double> vertical_mode_ = {max_lin_speed, -max_lin_speed, 0.0, 0.0, max_ang_speed, 0.0};
        std::vector<double> horizontal_mode_ = {0.0, 0.0, max_lin_speed, -max_lin_speed, 0.0, max_ang_speed};
        std::string serial_port = "/dev/ttyUSB0";
        unsigned int speed = 1000000;
        // unsigned int speed = 9600;
        int motor_number = 8;
        MotorController* motor_controller;
        std::mutex odom_mutex;
        std::mutex service_mutex;
        std::condition_variable cv;
        std::condition_variable service_cv;
        ros::ServiceServer ping_service;
        ros::ServiceServer brake_service;
        ros::ServiceServer motor_init_service;
        ros::ServiceServer tank_service;
        std::vector<float> lastest_wheel_motor_values = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        std::vector<float> lastest_wheel_motor_inputs = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        std::vector<uint8_t> motor_brake = {
            0xFF, 0xFF, 0xFE, 0x00, 0x01, 0x5E, 0x03, 0x00, 0x75, 0x00
        };
        std::vector<uint8_t> motor_non_brake = {
            0xFF, 0xFF, 0xFE, 0x00, 0x01, 0x5E, 0x03, 0x01, 0xB4, 0xC0
        };
        std::vector<uint8_t> outter_brake = {
            0xFF, 0xFF, 0xFE, 0x00, 0x01, 0x5F, 0x03, 0x00, 0x24, 0xC0
        };
        std::vector<uint8_t> outter_non_brake_before = {
            0xFF, 0xFF, 0xFE, 0x00, 0x01, 0x5F, 0x03, 0x64, 0x25 ,0x2B
        };
        std::vector<uint8_t> outter_non_brake = {
            // FA -> C8, 96, 64, 32, 0 (250, 200, 150, 100, 50, 0)
            // 0xFF, 0xFF, 0xFE, 0x00, 0x01, 0x5F, 0x03, 0x64, 0x25 ,0x2B
            // 0xFF, 0xFF, 0xFE, 0x00, 0x01, 0x5F, 0x03, 0xc8, 0x25 ,0x56 
            // 0xFF, 0xFF, 0xFE, 0x00, 0x01, 0x5F, 0x03, 0x96, 0xA4 ,0xAE // 24V
            // 0xFF, 0xFF, 0xFE, 0x00, 0x01, 0x5F, 0x03, 0x78, 0x24 ,0xE2
            // 0xFF, 0xFF, 0xFE, 0x00, 0x01, 0x5F, 0x03, 0x20, 0x25 ,0x18
            0xFF, 0xFF, 0xFE, 0x00, 0x01, 0x5F, 0x03, 0x64, 0x25 ,0x2B // 48V
            // 0xFF, 0xFF, 0xFE, 0x00, 0x01, 0x5F, 0x03, 0x50, 0x24, 0xFC
        };
        std::vector<uint8_t> reset_pose = {
            0xFF, 0xFF, 0xFE, 0x00, 0x01, 0x00, 0x03, 0x01, 0xD5, 0x12
        };
        std::vector<std::vector<uint8_t>> brake_control = {
            motor_brake,
            motor_non_brake,
            outter_brake,
            outter_non_brake,
            reset_pose
        };
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "robot_twist_controller");
    ROS_INFO("Robot twist controller is running...");
	try {
        ros::NodeHandle nh;
        VelConverter velConverter(nh);
        std::thread loop_thread(&VelConverter::outputMotorValuesLoop, &velConverter);
        std::thread subscribe_thread(&VelConverter::subscribeControlThread, &velConverter);
        if (loop_thread.joinable()) loop_thread.join();
        else {
            ROS_ERROR_STREAM("loop_thread not joinable ");
            ros::shutdown();
        }
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