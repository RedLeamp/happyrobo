#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <cmath>

class SwerveOdometry {
private:
    double x_;           // x 위치 (m)
    double y_;           // y 위치 (m)
    double theta_;       // 방향 (rad)
    double vx_;          // 로컬 x 속도 (m/s)
    double vy_;          // 로컬 y 속도 (m/s)
    double omega_;       // 각속도 (rad/s)
    ros::Time last_time_; // 이전 시간
    bool is_first_msg_;   // 첫 메시지 여부
    ros::Subscriber sub_; // cmd_vel 구독자
    ros::Publisher pub_;  // odom 발행자
    tf::TransformBroadcaster tf_broadcaster_; // TF 브로드캐스터

public:
    SwerveOdometry() {
        // 초기화
        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;
        vx_ = 0.0;       // 초기 속도 0
        vy_ = 0.0;
        omega_ = 0.0;
        is_first_msg_ = true;

        // ROS 노드 핸들 및 Subscriber/Publisher 설정
        ros::NodeHandle nh;
        sub_ = nh.subscribe("/cmd_vel", 10, &SwerveOdometry::cmdVelCallback, this);
        pub_ = nh.advertise<nav_msgs::Odometry>("/odom/raw", 10);
    }

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        // 속도 업데이트
        vx_ = msg->linear.x;
        vy_ = msg->linear.y;
        omega_ = msg->angular.z;

        // 첫 메시지라면 시간 초기화
        if (is_first_msg_) {
            last_time_ = ros::Time::now();
            is_first_msg_ = false;
        }
    }

    void updateAndPublishOdometry() {
        ros::Rate rate(20.0); // 20Hz 주기 설정

        while (ros::ok()) {
            ros::Time current_time = ros::Time::now();

            // 첫 메시지가 아직 안 왔으면 스킵
            if (is_first_msg_) {
                last_time_ = current_time;
                rate.sleep();
                continue;
            }

            // 시간 차이 계산
            double dt = (current_time - last_time_).toSec();

            // dt가 너무 작으면 무시
            if (dt < 1e-6) {
                rate.sleep();
                ros::spinOnce();
                continue;
            }

            // 글로벌 속도 계산
            double vx_global = vx_ * std::cos(theta_) - vy_ * std::sin(theta_);
            double vy_global = vx_ * std::sin(theta_) + vy_ * std::cos(theta_);

            // 위치와 방향 갱신
            x_ += vx_global * dt;
            y_ += vy_global * dt;
            theta_ += omega_ * dt;

            // 방향 정규화 (-pi ~ pi)
            theta_ = std::atan2(std::sin(theta_), std::cos(theta_));

            // Quaternion 생성
            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta_);

            // Odometry 메시지 작성
            nav_msgs::Odometry odom_msg;
            odom_msg.header.stamp = current_time;
            odom_msg.header.frame_id = "odom";
            odom_msg.child_frame_id = "base_link";

            // 위치 설정
            odom_msg.pose.pose.position.x = x_;
            odom_msg.pose.pose.position.y = y_;
            odom_msg.pose.pose.position.z = 0.0;
            odom_msg.pose.pose.orientation = odom_quat;

            // 속도 설정 (로컬 좌표계 기준)
            odom_msg.twist.twist.linear.x = vx_;
            odom_msg.twist.twist.linear.y = vy_;
            odom_msg.twist.twist.angular.z = omega_;

            // 공분산 (임의 값)
            odom_msg.pose.covariance[0] = 0.1;  // x
            odom_msg.pose.covariance[7] = 0.1;  // y
            odom_msg.pose.covariance[35] = 0.1; // yaw

            // 메시지 발행
            pub_.publish(odom_msg);

            // TF 변환 발행
            // geometry_msgs::TransformStamped odom_trans;
            // odom_trans.header.stamp = current_time;
            // odom_trans.header.frame_id = "odom";
            // odom_trans.child_frame_id = "base_link";
            // odom_trans.transform.translation.x = x_;
            // odom_trans.transform.translation.y = y_;
            // odom_trans.transform.translation.z = 0.0;
            // odom_trans.transform.rotation = odom_quat;
            // tf_broadcaster_.sendTransform(odom_trans);

            // 로그 출력
            ROS_INFO("dt: %.4f, Position: (%.2f, %.2f), Orientation: %.2f", dt, x_, y_, theta_);

            // 이전 시간 업데이트
            last_time_ = current_time;

            // 20Hz 주기 유지
            rate.sleep();
            ros::spinOnce(); // 콜백 처리
        }
    }
};

int main(int argc, char** argv) {
    // ROS 노드 초기화
    ros::init(argc, argv, "swerve_odometry");
    ros::NodeHandle nh;

    // SwerveOdometry 객체 생성
    SwerveOdometry odom;

    // 오도메트리 발행 시작
    odom.updateAndPublishOdometry();

    return 0;
}