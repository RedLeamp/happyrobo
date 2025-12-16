#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <vector>
#include <cmath>

class JointStatePublisher {
public:
    JointStatePublisher() {
        // ROS 노드 초기화
        ros::NodeHandle nh;
        
        // 퍼블리셔 설정
        joint_state_pub_ = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

        // 조인트 이름 설정
        joint_names_ = {"front_left_wheel", "rear_left_wheel", "front_right_wheel", "rear_right_wheel",
                        "front_left_steering", "rear_left_steering", "front_right_steering", "rear_right_steering"};

        // 초기 값 설정 (시뮬레이션용, RPM과 Degree 단위)
        velocities_ = std::vector<double>(4, 0.0);  // 주행 모터 속도 (RPM)
        positions_ = std::vector<double>(4, 0.0);   // 조향 모터 각도 (Degree)
    }

    void publishJointStates() {
        ros::Rate rate(50);  // 50Hz로 발행
        while (ros::ok()) {
            sensor_msgs::JointState joint_state;
            joint_state.header.stamp = ros::Time::now();
            joint_state.name = joint_names_;

            // 시뮬레이션 데이터 생성 및 단위 변환
            updateEncoderValues();

            // 주행 모터 속도와 조향 모터 각도 설정
            joint_state.velocity.resize(8);
            joint_state.position.resize(8);

            // 주행 모터 (RPM -> rad/s 변환)
            for (int i = 0; i < 4; ++i) {
                // RPM -> rad/s: RPM * 2π / 60
                joint_state.velocity[i] = velocities_[i] * 2.0 * M_PI / 60.0;
                joint_state.position[i] = 0.0;  // 주행 모터는 continuous, 위치 없음
            }

            // 조향 모터 (Degree -> Radian 변환)
            for (int i = 0; i < 4; ++i) {
                joint_state.velocity[i + 4] = 0.0;  // 조향 모터 속도 (고정 가정)
                // Degree -> Radian: Degree * π / 180
                joint_state.position[i + 4] = positions_[i] * M_PI / 180.0;
            }

            // 발행
            joint_state_pub_.publish(joint_state);
            rate.sleep();
        }
    }

private:
    void updateEncoderValues() {
        // 시뮬레이션용: RPM과 Degree 단위로 데이터 생성 (실제 하드웨어에서는 엔코더 값으로 대체)
        // velocities_[0] = readEncoderRPM("front_left_wheel");    // RPM
        // velocities_[1] = readEncoderRPM("rear_left_wheel");
        // velocities_[2] = readEncoderRPM("front_right_wheel");
        // velocities_[3] = readEncoderRPM("rear_right_wheel");

        // positions_[0] = readEncoderDegree("front_left_steering");  // Degree
        // positions_[1] = readEncoderDegree("rear_left_steering");
        // positions_[2] = readEncoderDegree("front_right_steering");
        // positions_[3] = readEncoderDegree("rear_right_steering");
        static double t = 0.0;
        t += 0.02;  // 50Hz -> 0.02초 간격

        // 주행 속도 (RPM 단위)
        velocities_[0] = 60.0 * sin(t);  // front_left (예: ±60 RPM)
        velocities_[1] = 60.0 * sin(t);  // rear_left
        velocities_[2] = 60.0 * sin(t);  // front_right
        velocities_[3] = 60.0 * sin(t);  // rear_right

        // 조향 각도 (Degree 단위)
        positions_[0] = 30.0 * sin(t);   // front_left (예: ±30°)
        positions_[1] = 30.0 * cos(t);   // rear_left
        positions_[2] = 30.0 * -sin(t);  // front_right
        positions_[3] = 30.0 * -cos(t);  // rear_right
    }

    ros::Publisher joint_state_pub_;
    std::vector<std::string> joint_names_;
    std::vector<double> velocities_;  // 주행 모터 속도 (RPM)
    std::vector<double> positions_;   // 조향 모터 각도 (Degree)
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "joint_state_publisher");
    JointStatePublisher publisher;
    publisher.publishJointStates();
    return 0;
}