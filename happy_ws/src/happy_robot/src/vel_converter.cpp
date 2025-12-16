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
#include "MCS_SDK.h"
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
#include <JetsonGPIO.h>
#include <happy_robo/TriggerWithCommand.h>
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
            {
                zero_rpm_motor_control = {};
                for (int idx=1; idx<5; idx++) {
                    InputRPM(static_cast<int32_t>(idx), 0, zero_rpm_motor_control);
                }
                ROS_INFO("Create Zero RPM Control Vector");
            }
            try {
                // GPIO::setmode(GPIO::BOARD);
                // GPIO::setup(11, GPIO::OUT, GPIO::HIGH);
                // GPIO::output(11, GPIO::LOW);
                
                // ser.setPort("/dev/ttyTHS0");
                ser.setPort(serial_port);
                ser.setBaudrate(1000000);
                serial::Timeout to = serial::Timeout::simpleTimeout(1000);
                ser.setTimeout(to);
                ser.open();
                if (!ser.isOpen()) {
                    ROS_ERROR("Failed to open serial port.");     
                    ros::shutdown();
                }
                ROS_INFO("Serial port opened.");
            }
            catch (const serial::IOException& e) {
                ROS_ERROR_STREAM("serial::IOException from init: " << e.what());
                ros::shutdown();
            }
            catch (const std::exception& e) {
                ROS_ERROR_STREAM("std::exception from init: " << e.what());
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
            const double MAX_LINEAR_SPEED = 0.4;  // 최대 선속도 (m/s)
            const double MAX_ANGULAR_SPEED = 0.1;  // 최대 각속도 (rad/s)
        
            std::vector<uint8_t> control_params;
        
            double vx = msg->linear.x;
            double vy = msg->linear.y;
            double wz = msg->angular.z;
        
            // 속도 제한
            vx = std::max(-MAX_LINEAR_SPEED, std::min(MAX_LINEAR_SPEED, vx));
            vy = std::max(-MAX_LINEAR_SPEED, std::min(MAX_LINEAR_SPEED, vy));
            wz = std::max(-MAX_ANGULAR_SPEED, std::min(MAX_ANGULAR_SPEED, wz));
        
            std::stringstream ss;

            std::vector<float> wheel_motor_inputs(8, 0.0);
            std::vector<float> wheel_motor_inputs_origin(8, 0.0);

            std::vector<uint8_t> drive_motor_inputs(4);
            std::vector<uint8_t> steer_motor_inputs(4);
        
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

                InputRPM(static_cast<int32_t>(i+1), rpm, drive_motor_inputs);
                InputDegree(static_cast<int32_t>(i+5), angle_deg, steer_motor_inputs);
            }
            int jdx = 0;
            for (auto motor_input : wheel_motor_inputs_origin) {
                if (motor_input > 75) {
                    ROS_ERROR("%d motor_input overs 75 : %d", jdx, motor_input);
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
                    for (auto input : drive_motor_inputs) {
                        control_params.push_back(input);
                    }
                    for (auto input : steer_motor_inputs) {
                        control_params.push_back(input);
                    }
                    if (ser.isOpen()) {
                        ser.write(control_params);
                        ROS_INFO("Drive Data sent");
                    } else {
                        ROS_ERROR("Serial port is not open!");
                    }        
                } else {
                    if (zero_rpm_motor_control.size() != 4) {
                        zero_rpm_motor_control = {};
                        for (int idx=1; idx<5; idx++) {
                            InputRPM(static_cast<int32_t>(idx), 0, zero_rpm_motor_control);
                        }
                         ROS_INFO("Create Zero RPM Control Vector");
                    }
                    if (ser.isOpen()) {
                        ser.write(zero_rpm_motor_control);
                        ROS_INFO("zero rpm Data sent");
                    } else {
                        ROS_ERROR("Serial port is not open!");
                    }  
                    // for (auto input : zero_rpm_motor_control) {
                    //     control_params.push_back(input);
                    // }
                    ros::Duration(3).sleep();
                    for (auto input : steer_motor_inputs) {
                        control_params.push_back(input);
                    }
                    if (ser.isOpen()) {
                        ser.write(control_params);
                        ROS_INFO("Steer Data sent");
                    } else {
                        ROS_ERROR("Serial port is not open!");
                    }  
                    ros::Duration(3).sleep();
                    // {
                    //     std::unique_lock<std::mutex> lock(odom_mutex);
                    //     cv.wait(lock, [] {return !wait_for_steer;});
                    //     ROS_INFO("Ready to Drive");
                    // }
                    if (ser.isOpen()) {
                        ser.write(drive_motor_inputs);
                        ROS_INFO("Drive Data sent");
                    } else {
                        ROS_ERROR("Serial port is not open!");
                    }  
                } 
            } else {
                for (auto input : drive_motor_inputs) {
                    control_params.push_back(input);
                }
                for (auto input : steer_motor_inputs) {
                    control_params.push_back(input);
                }
               if (ser.isOpen()) {
                    ser.write(control_params);
                    ROS_INFO("Drive Data sent");
                } else {
                    ROS_ERROR("Serial port is not open!");
                }   
            }
        
            ROS_INFO("smoothSwerveDriveCallback : \n%s", ss.str().c_str());
        }

        void twistCallback(const geometry_msgs::Twist::ConstPtr& msg) {

            if (WHEEL_POSITIONS.size() != wheel_coords.size()) {
                ROS_ERROR("Wheel positions and wheel coordinates size mismatch!");
                return;
            }
            // cmdVelSwerveCallback(msg);

            double linear_x = msg->linear.x;
            double linear_y = msg->linear.y;
            double angular_z = msg->angular.z;

            double this_track_width = this->MODE_VERTICAL ? TRACK_WIDTH : WHEELBASE;
            double this_wheel_base = this->MODE_VERTICAL ? WHEELBASE : TRACK_WIDTH;
            double linear_vel = this->MODE_VERTICAL ? linear_x : linear_y;

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
                double turn_angle = 66.17;
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

                velocity = wheel_coords[i] * velocity;
                // velocity = abs(velocity) > max_lin_speed ? std::copysign(max_lin_speed, velocity) : velocity;

                double rpm_double = (velocity / (2 * PI * WHEEL_RADIUS)) * 60; // m/s -> RPM
                int32_t rpm = (MODE_STOP) ? 0 : static_cast<int32_t>(rpm_double);
                std::cout << "ID : " << i+1 <<", RPM : " << rpm << std::endl;
                InputRPM( static_cast<int32_t>(i+1), rpm, control_params);
            }

            for (size_t i = 0; i < 4; i++) {

                double steer_angle = steer_ang_vector[i] * steer_coords[i];
                std::cout << "ID : " << i+5 <<", Degree : " << steer_angle << std::endl;
                InputDegree( static_cast<int32_t>(i+5), steer_angle, control_params);
            }

            std::stringstream ss;
            for (uint8_t byte : control_params) {
                ss << std::hex << std::uppercase << (int)byte << " ";
            }
            // ROS_INFO("send: %s", ss.str().c_str());
            if (ser.isOpen()) {
                // GPIO::output(11, GPIO::HIGH);
                // ros::Duration(0.0001).sleep();
                ser.write(control_params);
                // ros::Duration(0.0001).sleep();
                // GPIO::output(11, GPIO::LOW);
                // ros::Duration(0.0005).sleep();
                // std::cout << std::endl;
                ROS_INFO("Data sent");
            } else {
                ROS_ERROR("Serial port is not open!");
            }
        }

        void setSpeed () {
            std::vector<uint8_t> control_params;
            InputRPM(4, 10, control_params);
            if (ser.isOpen()) {
                ser.write(control_params);
                std::cout << std::endl;
                ROS_INFO("Data sent");
            } else {
                ROS_ERROR("Serial port is not open!");
            }
        }

        void InputRPM(int32_t Id, int32_t Rpm, std::vector<uint8_t>& control_params) {//0.1RPM
            // static volatile uint8_t Buffer[16] = {0,};
            uint8_t Buffer[16] = {0,};
            int32_t Rpm10 = 0;
            uint8_t Rpm1 = 0;
            uint8_t Rpm2 = 0;
            uint8_t Rpm3 = 0;
            uint8_t RpmDir = 0;
            Rpm10 = (Rpm>0) ? Rpm*10 : -Rpm*10;
            // if(Rpm>0) Rpm10 = (int32_t)((float)Rpm * 10.0f);
            // else Rpm10 = -(int32_t)((float)Rpm * 10.0f);
            
            Rpm1 |= (Rpm10 >>16) & 0xFF;
            Rpm2 |= (Rpm10 >>8) & 0xFF;
            Rpm3 |= Rpm10 & 0xFF;
            if(Rpm > 0) RpmDir = 1;
            else if(Rpm < 0) RpmDir = 2;
            if(Rpm == 0) RpmDir = 0;

            Buffer[0] = Id; //ID
            Buffer[1] = Rpm1; //Data1
            Buffer[2] = Rpm2; //Data1
            Buffer[3] = Rpm3; //Data2
            Buffer[4] = RpmDir; //Data2
            u8RxNumberOfData = 5; // catkin build happy_robo --cmake-args -DCMAKE_BUILD_TYPE=Release
            MCS.MotorProtocolTxMODE(Buffer,u8RxNumberOfData, 0x01, 0x58, control_params);    
        }

        void InputDegree(int32_t Id, double Degree, std::vector<uint8_t>& control_params){//0.1 Degree
            uint8_t Buffer[16] = {0,};
            int32_t Degree10 = 0;
            uint8_t Degree1 = 0;
            uint8_t Degree2 = 0;
            uint8_t Degree3 = 0;
            uint8_t DegreeDir = 0;
            Degree10 = Degree > 0 ? (int32_t)((double)Degree * 10.0) : -(int32_t)((double)Degree * 10.0);
            // else  Degree10 = -(int32_t)((double)Degree * 10.0);
            Degree1 |= (Degree10 >>16) & 0xFF;
            Degree2 |= (Degree10 >>8) & 0xFF;
            Degree3 |= Degree10 & 0xFF;
            if(Degree > 0) DegreeDir = 1;
            else if(Degree < 0) DegreeDir = 2;
            if(Degree == 0) DegreeDir = 0;

            Buffer[0] = Id; //ID
            Buffer[1] = Degree1; //Data1
            Buffer[2] = Degree2; //Data1
            Buffer[3] = Degree3; //Data2
            Buffer[4] = DegreeDir; //Data2
            u8RxNumberOfData = 5; // Size = ID + Data1 + Data2 + ··· + DataN
            MCS.MotorProtocolTxMODE(Buffer,u8RxNumberOfData, 0x01, 0x58, control_params);    
        }

        void send_ping() {

            bool isBreak = false;
            while (ros::ok()) {
                if (ser.isOpen()) {
                    GPIO::output(11, GPIO::HIGH);
                    // usleep(10);  // 0.00001초
                    ros::Duration(0.0001).sleep();
                    ser.write(ping);
                    ros::Duration(0.0001).sleep();
                    // usleep(10);
                    GPIO::output(11, GPIO::LOW);
                    ros::Duration(0.0005).sleep();
                } else {
                    ROS_ERROR("Serial port is not open!");
                }

                if (ser.available()) {
                    std::stringstream ss;
                    // for (uint8_t byte : response) {
                    //         ss << std::hex << std::uppercase << (int)byte << " ";
                    //     }
                    std::vector<uint8_t> response;
                    response.resize(ser.available());
                    ser.read(response, response.size());
                    for (uint8_t byte : response) {
                        ss << std::hex << std::uppercase << (int)byte << " ";
                    }
                    ROS_INFO("%s", ss.str().c_str());
                    // isBreak = true;
                } else {
                    ROS_WARN("No response received.");
                }
            }
        }
        bool handlePing(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

            ROS_INFO("Ping Service has been called.");
        
            if (ser.isOpen()) {
                std::vector<bool> ping_test(ping_all.size(), false);
                // 8개의 ping 테스트 수행
                for (int i = 0; i < ping_all.size(); i++) {
                    ser.write(ping_all[i]);
                    ros::Duration(0.001).sleep();
        
                    if (ser.available()) {
                        std::vector<uint8_t> response(ser.available());
                        ser.read(response, response.size());
                        std::stringstream ss;
                        for (uint8_t byte : response) {
                            ss << std::hex << std::uppercase << (int)byte << " ";
                        }
                        ROS_INFO("Ping response for ID%d: %s", i + 1, ss.str().c_str());
                        ping_test[i] = true;  // 응답이 있으면 true
                    } else {
                        ping_test[i] = false; // 응답이 없으면 false
                        ROS_WARN("No response for ID%d", i + 1);
                    }
                }
                std::stringstream result_ss;
                for (int i = 0; i < ping_all.size(); i++) {
                    result_ss << (ping_test[i] ? "True" : "False");
                    if (i < ping_all.size()-1) result_ss << ",";
                }
                res.message = result_ss.str();
                res.success = true; // 서비스 호출 자체는 성공
            } else {
                // 시리얼 포트가 열리지 않은 경우 모두 False로 설정
                res.message = "False,False,False,False,False,False,False,False";
                res.success = false;
                ROS_ERROR("Serial port is not open!");
            }
            return true; // 서비스 호출 성공
        }

        bool handleBrake(happy_robo::TriggerWithCommand::Request& req, happy_robo::TriggerWithCommand::Response& res) {
            int command = req.command;  // 커스텀 메시지의 command 필드에서 int32 값 읽기
            ROS_INFO("Brake Service has been called with command: %d", command);
            if (ser.isOpen()) {
                if (command >= 0 && command < brake_control.size()) {
                    if (command == 2) {
                        ROS_INFO("Outter Brake needs Motor Stop");
                        ser.write(brake_control[0]);
                        ros::Duration(0.1).sleep();
                    } else if (command == 3) {
                        ROS_INFO("Outter Brake Before");
                        ser.write(outter_non_brake_before);
                        ros::Duration(1).sleep();
                    }
                    ser.write(brake_control[command]);
                    ros::Duration(0.1).sleep();
                    if (ser.available()) {
                        std::vector<uint8_t> response(ser.available());
                        ser.read(response, response.size());
                        std::stringstream ss;
                        for (uint8_t byte : response) {
                            ss << std::hex << std::uppercase << (int)byte << " ";
                        }
                        ROS_INFO("Brake response for command %d: %s", command, ss.str().c_str());
                        res.message = "True";
                        res.success = true;
                    } else {
                        ROS_WARN("No response for command %d", command);
                        res.message = "True";
                        res.success = true;
                    }
                } else {
                    ROS_ERROR("Invalid command: %d", command);
                    res.message = "False";
                    res.success = false;
                }
            } else {
                res.message = "False";
                res.success = false;
                ROS_ERROR("Serial port is not open!");
            }
            return true;
        }

        bool handleMotorInit(happy_robo::TriggerWithCommand::Request& req, happy_robo::TriggerWithCommand::Response& res) {
            int command = req.command;  // 0~8 (Flutter에서 1~9로 보낸 값에서 -1 처리됨)
            ROS_INFO("Motor Init Service has been called with command: %d", command);
        
            if (ser.isOpen()) {
                if (command >= 0 && command < motor_init_all.size()) {
                    if (command == 0) {
                        ROS_INFO("Init Setup needs outter non brake");
                        ROS_INFO("Outter Brake Before");
                        ser.write(outter_non_brake_before);
                        ros::Duration(1).sleep();
                        ser.write(brake_control[3]);
                        ros::Duration(1).sleep();
                    } 
                    ser.write(motor_init_all[command]);
                    ros::Duration(0.05).sleep();
                    if (ser.available()) {
                        std::vector<uint8_t> response(ser.available());
                        ser.read(response, response.size());
                        std::stringstream ss;
                        for (uint8_t byte : response) {
                            ss << std::hex << std::uppercase << (int)byte << " ";
                        }
                        ROS_INFO("Motor Init response for command %d: %s", command, ss.str().c_str());
                        res.message = "True";
                        res.success = true;
                    } else {
                        ROS_WARN("No response for command %d", command);
                        res.message = "True";
                        res.success = true;
                    }
                } else if (command == motor_init_all.size()) {
                    ros::Duration(0.05).sleep();
                    for (std::vector<uint8_t> motor_param : motor_param_all) {
                        ser.write(motor_param);
                        ros::Duration(0.05).sleep();
                        ROS_INFO("Send Motor Param");
                    }
                    ROS_INFO("Motor Param response for command");
                    res.message = "True";
                    res.success = true;
                } else {
                    ROS_ERROR("Invalid command: %d", command);
                    res.message = "False";
                    res.success = false;
                }
            } else {
                res.message = "False";
                res.success = false;
                ROS_ERROR("Serial port is not open!");
            }
            return true;
        }

        bool handleTank(happy_robo::TriggerWithCommand::Request& req, happy_robo::TriggerWithCommand::Response& res) {
            int command = req.command;  // 0: tank_up, 1: tank_down
            double degree = req.degree;
            ROS_INFO("Tank Service has been called with command: %d, degree: %.2f", command, degree);
        
            if (ser.isOpen()) {
                if (command == 0 || command == 1) {
                    std::vector<uint8_t> control_params;
                    // inverse angle
                    double tank_degree = command == 0 ? 0.0 : degree;

                    InputDegree( static_cast<int32_t>(9), tank_degree, control_params);
                    std::cout << "ID : " << 9 <<", Degree : " << tank_degree << ", ";

                    ser.write(control_params);
                    ros::Duration(0.05).sleep();
                    if (ser.available()) {
                        std::vector<uint8_t> response(ser.available());
                        ser.read(response, response.size());
                        std::stringstream ss;
                        for (uint8_t byte : response) {
                            ss << std::hex << std::uppercase << (int)byte << " ";
                        }
                        ROS_INFO("Tank response for command %d: %s", command, ss.str().c_str());
                        res.message = "True";
                        res.success = true;
                    } else {
                        ROS_WARN("No response for command %d", command);
                        res.message = "True";
                        res.success = true;
                    }
                } else {
                    ROS_ERROR("Invalid command: %d", command);
                    res.message = "False";
                    res.success = false;
                }
            } else {
                res.message = "False";
                res.success = false;
                ROS_ERROR("Serial port is not open!");
            }
        
            return true;
        }

        void outputMotorValuesLoop() {
            ros::Rate rate(30); // 10Hz로 실행
            while (ros::ok()) {
                ros::Time start_time = ros::Time::now();
                outputMotorValuesSlow();
                double whole_time = (ros::Time::now() - start_time).toSec();
                ROS_INFO("HZ: %.1f, time: %.3f sec", 1/whole_time, whole_time);
                rate.sleep();
            }
        }

        void outputMotorValuesSlow() {

            std::vector<float> wheel_motor_values;

            ros::Time start_time = ros::Time::now();

            int succeed = 0;

            for (int idx=1; idx<9; idx++) {
                std::vector<uint8_t> wheel_motor_speeds;
                uint8_t Buffer[16] = {0,};
                Buffer[0] = (uint8_t) idx; //ID
                Buffer[1] = idx < 5  ? 0x03 : 0x04; //데이터 개수
                u8RxNumberOfData = 2; // Size = ID + Data1 + Data2 + ··· + DataN
                MCS.MotorProtocolTxMODE(Buffer,u8RxNumberOfData, 0x02, idx < 5  ? 0x6A : 0x66, wheel_motor_speeds); //0x66 위치

                if (ser.isOpen()) {
                    ser.write(wheel_motor_speeds);
                } else {
                    ROS_ERROR("Serial port is not open!");
                    break;
                }

                // ros::Duration(0.1).sleep();

                ros::Time interval = ros::Time::now();
                ros::Duration timeout(0.1); // 0.1초 타임아웃
                bool received = false;

                while (ros::ok() && !received && (ros::Time::now() - interval) < timeout) {
                    if (ser.available()) {
                        std::vector<uint8_t> response;
                        response.resize(ser.available());
                        ser.read(response, response.size());
                        // std::stringstream ss;
                        // for (uint8_t byte : response) {
                        //     ss << std::hex << std::uppercase << (int)byte << " ";
                        // }
                        // ROS_INFO("%s", ss.str().c_str());
                        for (uint8_t byte : response) {
                            MCS.MotorDataRX(byte);
                            MCS.DeviceToMotorCONTROL();
                        }
                        // ROS_INFO("values %f", (idx < 5) ? MCS.fMotorRpm : MCS.fMotorDegree);
                        float value = (idx < 5) ? (MCS.fMotorRpm * wheel_coords[idx-1]): (MCS.fMotorDegree * steer_coords[idx-1]);
                        wheel_motor_values.push_back(value);
                        received = true;
                        succeed++;
                    }
                }

                // ros::Duration(0.1).sleep();
            
                if (!received) {
                    ROS_WARN("No response received at ID: %d (timeout after 0.1s)", idx);
                    return;
                }

                double wait_time = (ros::Time::now() - interval).toSec();
                ROS_INFO("ID: %d, Wait time: %.6f sec processing", idx, wait_time);
            }

            if (succeed == 8) {
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
            ROS_INFO("updateOdometry : %.3f \nlinear x : %.2f\nlinear y %.2f\nangular z : %.2f\n", 1/dt, vx_, vy_, vtheta_);

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

        void outputMotorValues() {

            // gpiod_line_set_value(line, 0);

            std::vector<float> wheel_motor_values;

            ros::Time start_time = ros::Time::now();

            int succeed = 0;

            for (int idx=1; idx<9; idx++) {
                std::vector<uint8_t> wheel_motor_speeds;
                uint8_t Buffer[16] = {0,};
                Buffer[0] = (uint8_t) idx; //ID
                Buffer[1] = idx < 5  ? 0x03 : 0x04; //데이터 개수
                u8RxNumberOfData = 2; // Size = ID + Data1 + Data2 + ··· + DataN
                MCS.MotorProtocolTxMODE(Buffer,u8RxNumberOfData, 0x02, idx < 5  ? 0x6A : 0x66, wheel_motor_speeds); //0x66 위치

                if (ser.isOpen()) {
                    GPIO::output(11, GPIO::HIGH);
                    ros::Duration(0.0001).sleep();
                    ser.write(wheel_motor_speeds);
                    // ser.write(ping);
                    ros::Duration(0.0001).sleep();
                    // usleep(10);
                    GPIO::output(11, GPIO::LOW);
                    ros::Duration(0.0005).sleep();
                    // GPIO::output(11, GPIO::HIGH);
                    // // usleep(10);  // 0.00001초
                    // ros::Duration(0.00001).sleep();
                    // ser.write(wheel_motor_speeds);
                    // ros::Duration(0.00001).sleep();
                    // // usleep(10);
                    // GPIO::output(11, GPIO::LOW);
                    // ros::Duration(0.00001).sleep();
                } else {
                    ROS_ERROR("Serial port is not open!");
                }

                if (ser.available()) {
                    std::vector<uint8_t> response;
                    response.resize(ser.available());
                    ser.read(response, response.size());
                    for (uint8_t byte : response) {
                        MCS.MotorDataRX(byte);
                        MCS.DeviceToMotorCONTROL();
                    }
                    float value = (idx < 5) ? MCS.fMotorRpm : MCS.fMotorDegree;
                    wheel_motor_values.push_back(value);
                    succeed++;
                    // received = true;
                } else {
                    ROS_WARN("No response ");
                    break;
                }

                // usleep(500);

                //  ros::Duration(0.001).sleep();

                // ros::Time interval = ros::Time::now();
                // ros::Duration timeout(0.1); // 0.1초 타임아웃
                // bool received = false;

                // while (ros::ok() && !received && (ros::Time::now() - interval) < timeout) {
                //     if (ser.available()) {
                //         std::vector<uint8_t> response;
                //         response.resize(ser.available());
                //         ser.read(response, response.size());
                //         for (uint8_t byte : response) {
                //             MCS.MotorDataRX(byte);
                //             MCS.DeviceToMotorCONTROL();
                //         }
                //         float value = (idx < 5) ? MCS.fMotorRpm : MCS.fMotorDegree;
                //         wheel_motor_values.push_back(value);
                //         received = true;
                //     }
                //     // CPU 사용을 줄이기 위해 약간의 대기 (1ms)
                //     // ros::Duration(0.0001).sleep();
                // }

                // double wait_time = (ros::Time::now() - interval).toSec();
                // ROS_INFO("ID: %d, Wait time: %.6f sec processing", idx, wait_time);
            

                // if (!received) {
                //     ROS_WARN("No response received at ID: %d (timeout after 0.1s)", idx);
                //     return;
                // }
            }

            if (succeed == 8) {
                updateOdometry(wheel_motor_values);
                std::stringstream ss;
                for (float value : wheel_motor_values) {
                    ss << value << " ";
                }
                // ROS_INFO("Received: %s", ss.str().c_str());
                double whole_time = (ros::Time::now() - start_time).toSec();
                ROS_INFO("HZ: %.4f, Whole time: %.4f sec, %s", 1/whole_time, whole_time, ss.str().c_str());
            } else {
                ROS_WARN("No response ");
            }
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

        void send_serial_data(std::vector<uint8_t> data) {
            if (ser.isOpen()) {
                ser.write(data);
                std::cout << std::endl;
                ROS_INFO("Data sent");
            } else {
                ROS_ERROR("Serial port is not open!");
            }
            // 데이터 수신
            // while (ros::ok()) {
            //     if (ser.available()) {
            //         std::string incoming_data = ser.readline();
            //         ROS_INFO("Received data: %s", incoming_data.c_str());
            //     }
            //     ros::spinOnce();  // ROS 콜백을 처리
            // }  // 시리얼 포트 닫기
            // return;
        }

        void subscribeControlThread() {
            ros::spin();
        }

        ~VelConverter() {
            if (ser.isOpen()) {
                ser.close();
                GPIO::cleanup();
                ROS_INFO("GPIO and Serial port cleaned up");
            }
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
        bool steer_angle_filter_ = true;
        bool separate_mode_ = true;
        uint8_t u8RxNumberOfData = 0;
        McsSdk MCS;
        struct gpiod_chip *chip;
        struct gpiod_line *line;
        int LED_PIN = 11;
        double prevVx = 0.0;
        double prevVy = 0.0;
        double prevWz = 0.0;
        double max_lin_speed = 0.5;
        double max_ang_speed = 0.5;
        double max_outer_ang_degree = 60;
        double max_inner_ang_degrees = 60;
        float steer_angle_limit = 20;
        std::vector<int> steer_coords = {-1, -1, -1, -1};
        std::vector<int> wheel_coords = {-1, 1, -1, 1};
        std::vector<uint8_t> zero_rpm_motor_control;
        std::vector<double> vertical_mode_ = {max_lin_speed, -max_lin_speed, 0.0, 0.0, max_ang_speed, 0.0};
        std::vector<double> horizontal_mode_ = {0.0, 0.0, max_lin_speed, -max_lin_speed, 0.0, max_ang_speed};
        std::string serial_port = "/dev/ttyUSB0";
        serial::Serial ser;
        std::mutex odom_mutex;
        std::condition_variable cv;
        ros::ServiceServer ping_service;
        ros::ServiceServer brake_service;
        ros::ServiceServer motor_init_service;
        ros::ServiceServer tank_service;
        std::vector<float> lastest_wheel_motor_values = {0, 0, 0, 0, 0, 0, 0, 0};
        std::vector<float> lastest_wheel_motor_inputs = {0, 0, 0, 0, 0, 0, 0, 0};
        std::vector<uint8_t> ping = {0xFF, 0xFF, 0xFE, 0x01, 0x02, 0x01, 0x03, 0x01, 0xB9, 0x56};
        std::vector<std::vector<uint8_t>> ping_all = {
            {0xFF, 0xFF, 0xFE, 0x01, 0x02, 0x01, 0x03, 0x01, 0xB9, 0x56},
            {0xFF, 0xFF, 0xFE, 0x02, 0x02, 0x01, 0x03, 0x01, 0xFD, 0x56},
            {0xFF, 0xFF, 0xFE, 0x03, 0x02, 0x01, 0x03, 0x01, 0xC0, 0x96},
            {0xFF, 0xFF, 0xFE, 0x04, 0x02, 0x01, 0x03, 0x01, 0x75, 0x56},
            {0xFF, 0xFF, 0xFE, 0x05, 0x02, 0x01, 0x03, 0x01, 0x48, 0x96},
            {0xFF, 0xFF, 0xFE, 0x06, 0x02, 0x01, 0x03, 0x01, 0x0c, 0x96},
            {0xFF, 0xFF, 0xFE, 0x07, 0x02, 0x01, 0x03, 0x01, 0x31, 0x56},
            {0xFF, 0xFF, 0xFE, 0x08, 0x02, 0x01, 0x03, 0x01, 0x65, 0x57},
            {0xFF, 0xFF, 0xFE, 0x09, 0x02, 0x01, 0x03, 0x01, 0x58, 0x97}
        };
        std::vector<std::vector<uint8_t>> motor_init_all = {
            {0xFF, 0xFF, 0xFE, 0x01, 0x01, 0x18, 0x03, 0x06, 0x29, 0x17},
            {0xFF, 0xFF, 0xFE, 0x02, 0x01, 0x18, 0x03, 0x06, 0x6D, 0x17},
            {0xFF, 0xFF, 0xFE, 0x03, 0x01, 0x18, 0x03, 0x06, 0x50, 0xD7},
            {0xFF, 0xFF, 0xFE, 0x04, 0x01, 0x18, 0x03, 0x06, 0xE5, 0x17},
            {0xFF, 0xFF, 0xFE, 0x05, 0x01, 0x18, 0x03, 0x06, 0xD8, 0xD7},
            {0xFF, 0xFF, 0xFE, 0x06, 0x01, 0x18, 0x03, 0x06, 0x9C, 0xD7},
            {0xFF, 0xFF, 0xFE, 0x07, 0x01, 0x18, 0x03, 0x06, 0xA1, 0x17},
            {0xFF, 0xFF, 0xFE, 0x08, 0x01, 0x18, 0x03, 0x06, 0xF5, 0x16},
            {0xFF, 0xFF, 0xFE, 0x09, 0x01, 0x18, 0x03, 0x06, 0xC8, 0xD6}
        };

        std::vector<uint8_t> motor_1_vel = {0xFF, 0xFE, 0x01, 0x01, 0x54, 0x03, 0x31, 0xBD, 0x19};
        std::vector<uint8_t> motor_2_vel = {0xFF, 0xFE, 0x02, 0x01, 0x54, 0x03, 0x31, 0xF9, 0x19};
        std::vector<uint8_t> motor_3_vel = {0xFF, 0xFE, 0x03, 0x01, 0x54, 0x03, 0x31, 0xC4, 0xD9};
        std::vector<uint8_t> motor_4_vel = {0xFF, 0xFE, 0x04, 0x01, 0x54, 0x03, 0x31, 0x71, 0x19};

        std::vector<uint8_t> motor_5_pos = {0xFF, 0xFE, 0x05, 0x01, 0x54, 0x03, 0x71, 0x4D, 0x29};
        std::vector<uint8_t> motor_6_pos = {0xFF, 0xFE, 0x06, 0x01, 0x54, 0x03, 0x71, 0x09, 0x29};
        std::vector<uint8_t> motor_7_pos = {0xFF, 0xFE, 0x07, 0x01, 0x54, 0x03, 0x71, 0x34, 0xE9};
        std::vector<uint8_t> motor_8_pos = {0xFF, 0xFE, 0x08, 0x01, 0x54, 0x03, 0x71, 0x60, 0xE8};

        std::vector<uint8_t> motor_steer_acc_10 = {0xFF, 0xFE, 0x00, 0x01, 0x50, 0x06, 0x00, 0x0A, 0x00, 0x0A, 0x39, 0x05};
        std::vector<uint8_t> motor_steer_sub_1000 = {0xFF, 0xFE, 0x00, 0x01, 0x55, 0x05, 0x00, 0x03, 0xE8, 0xCC, 0x63};

        std::vector<uint8_t> motor_driv_acc_10 = {0xFF, 0xFE, 0x00, 0x01, 0x50, 0x06, 0x00, 0x0A, 0x00, 0x0A, 0x39, 0x05};
        std::vector<uint8_t> motor_driv_sub_1000 = {0xFF, 0xFE, 0x00, 0x01, 0x55, 0x05, 0x00, 0x03, 0xE8, 0xCC, 0x63};

        std::vector<std::vector<uint8_t>> motor_param_all = {

            {0xFF, 0xFF, 0xFE, 0x01, 0x01, 0x54, 0x03, 0x74, 0x68, 0xE5},
            {0xFF, 0xFF, 0xFE, 0x02, 0x01, 0x54, 0x03, 0x74, 0x2C, 0xE5},
            {0xFF, 0xFF, 0xFE, 0x03, 0x01, 0x54, 0x03, 0x74, 0x11, 0x25},
            {0xFF, 0xFF, 0xFE, 0x04, 0x01, 0x54, 0x03, 0x74, 0xA4, 0xE5},

            {0xFF, 0xFF, 0xFE, 0x05, 0x01, 0x54, 0x03, 0xF4, 0x98, 0x85},
            {0xFF, 0xFF, 0xFE, 0x06, 0x01, 0x54, 0x03, 0xF4, 0xDC, 0x85},
            {0xFF, 0xFF, 0xFE, 0x07, 0x01, 0x54, 0x03, 0xF4, 0xE1, 0x45},
            {0xFF, 0xFF, 0xFE, 0x08, 0x01, 0x54, 0x03, 0xF4, 0xB5, 0x44},

            {0xFF, 0xFF, 0xFE, 0x09, 0x01, 0x54, 0x03, 0xF4, 0x88, 0x84},

            {0xFF, 0xFF, 0xFE, 0x00, 0x01, 0x50, 0x06, 0x07, 0xD0, 0x07, 0xD0, 0xDF, 0xD5}, // motor_1_4_acc_dcc_1000_hex
            // {0xFF, 0xFF, 0xFE, 0x00, 0x01, 0x55, 0x05, 0x00, 0x03, 0xE8, 0x83, 0x67},        // motor_1_4_sub_1000
            {0xFF, 0xFF, 0xFE, 0x00, 0x01, 0x58, 0x05, 0x00, 0x00, 0x64, 0xAF, 0xF3},        // motor_1_4_target_100
            // {0xFF, 0xFF, 0xFE, 0x00, 0x01, 0x50, 0x06, 0x00, 0x0A, 0x00, 0x0A, 0x7C, 0xF1}, // motor_5_8_acc_dcc_10_hex
            // {0xFF, 0xFF, 0xFE, 0x00, 0x01, 0x55, 0x05, 0x00, 0x03, 0xE8, 0x83, 0x67},         // motor_5_8_sub_100
            // {0xFF, 0xFF, 0xFE, 0x00, 0x01, 0x55, 0x05, 0x00, 0x00, 0x64, 0x82, 0x32},         // motor_5_8_sub_100
            {0xFF, 0xFF, 0xFE, 0x00, 0x01, 0x55, 0x05, 0x00, 0x01, 0xF4, 0x83, 0xCE},
            {0xFF, 0xFF, 0xFE, 0x09, 0x01, 0x55, 0x05, 0x00, 0x00, 0x64, 0x1B, 0x32}
            // {0xFF, 0xFF, 0xFE, 0x09, 0x01, 0x58, 0x05, 0x00, 0x00, 0x64, 0x36, 0xF3}
        };

        // std::vector<uint8_t> motor_steer_brake = {
        //     // 바뀜
        //     0xFF, 0xFF, 0xFE, 0x05, 0x01, 0x5E, 0x03, 0x00, 0x5D, 0x0F, 
        //     0xFF, 0xFF, 0xFE, 0x06, 0x01, 0x5E, 0x03, 0x00, 0x19, 0x0F, 
        //     0xFF, 0xFF, 0xFE, 0x07, 0x01, 0x5E, 0x03, 0x00, 0x24, 0xCF, 
        //     0xFF, 0xFF, 0xFE, 0x08, 0x01, 0x5E, 0x03, 0x00, 0x70, 0xCE
        // };
        // std::vector<uint8_t> motor_steer_non_brake = {
        //     // 바뀜
        //     0xFF, 0xFF, 0xFE, 0x05, 0x01, 0x5E, 0x03, 0x01, 0x9C, 0xCF, 
        //     0xFF, 0xFF, 0xFE, 0x06, 0x01, 0x5E, 0x03, 0x01, 0xD8, 0xCF, 
        //     0xFF, 0xFF, 0xFE, 0x07, 0x01, 0x5E, 0x03, 0x01, 0xE5, 0x0F, 
        //     0xFF, 0xFF, 0xFE, 0x08, 0x01, 0x5E, 0x03, 0x01, 0xB1, 0x0E
        // };
        // std::vector<uint8_t> motor_driv_brake = {
        //     // 바뀜
        //     0xFF, 0xFF, 0xFE, 0x01, 0x01, 0x5E, 0x03, 0x00, 0xAC, 0xCF, 
        //     0xFF, 0xFF, 0xFE, 0x02, 0x01, 0x5E, 0x03, 0x00, 0xE8, 0xCF, 
        //     0xFF, 0xFF, 0xFE, 0x03, 0x01, 0x5E, 0x03, 0x00, 0xD5, 0x0F, 
        //     0xFF, 0xFF, 0xFE, 0x04, 0x01, 0x5E, 0x03, 0x00, 0x60, 0xCF
        // };
        // std::vector<uint8_t> motor_driv_non_brake = {
        //     // 바뀜
        //     0xFF, 0xFF, 0xFE, 0x01, 0x01, 0x5E, 0x03, 0x01, 0x89, 0x00, 
        //     0xFF, 0xFF, 0xFE, 0x02, 0x01, 0x5E, 0x03, 0x01, 0x29, 0x0F, 
        //     0xFF, 0xFF, 0xFE, 0x03, 0x01, 0x5E, 0x03, 0x01, 0x14, 0xCF, 
        //     0xFF, 0xFsubscriberNode0xFE, 0x00, 0x01, 0x5E, 0x03, 0x00, 0x75, 0x00
        // };
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
            0xFF, 0xFF, 0xFE, 0x00, 0x01, 0x5F, 0x03, 0x20, 0x25 ,0x18
            // 0xFF, 0xFF, 0xFE, 0x00, 0x01, 0x5F, 0x03, 0x64, 0x25 ,0x2B // 48V
            // 0xFF, 0xFF, 0xFE, 0x00, 0x01, 0x5F, 0x03, 0x50, 0x24, 0xFC
        };
        std::vector<uint8_t> reset_pose = {
            0xFF, 0xFF, 0xFE, 0x00, 0x01, 0x00, 0x03, 0x01, 0xD5, 0x12
        };
        // fa //250
        // c8 //200
        // 96 //150
        // 64 //100
        // 32 //50
        // 0  //0

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
    catch (const serial::IOException& e) {
        ROS_ERROR_STREAM("serial::IOException: " << e.what());
        ros::shutdown();
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