#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define MPU9250_ADDR 0x68  // I2C address of the MPU9250
#define PWR_MGMT_1 0x6B    // Power management register
#define ACCEL_XOUT_H 0x3B  // Accelerometer data registers
#define GYRO_XOUT_H 0x43   // Gyroscope data registers
#define WHO_AM_I 0x75      // WHO_AM_I register
#define ACCEL_CONFIG 0x1C  // Accelerometer configuration register
#define GYRO_CONFIG 0x1B   // Gyroscope configuration register
#define SMPLRT_DIV 0x19    // Sample rate divider

class IMUReceiver {
private:
    int i2c_file;
    ros::Publisher imu_pub;
    ros::Publisher imu_correct_pub;
    bool calibration_done = false;
    int calibration_samples = 100;
    int sample_count = 0;
    double roll_offset = 0.0, pitch_offset = 0.0;

    void writeRegister(uint8_t reg, uint8_t value) {
        uint8_t data[2] = {reg, value};
        if (write(i2c_file, data, 2) != 2) {
            std::cerr << "Failed to write to the I2C bus." << std::endl;
        }
    }

    // Function to read data from a specific register
    int16_t readRegister16(int reg) {
        uint8_t buffer[2];
        if (write(i2c_file, &reg, 1) != 1) {
            std::cerr << "Failed to write to the I2C bus." << std::endl;
            return -1;
        }
        if (read(i2c_file, buffer, 2) != 2) {
            std::cerr << "Failed to read from the I2C bus." << std::endl;
            return -1;
        }
        return (int16_t)((buffer[0] << 8) | buffer[1]);
    }

public:
    IMUReceiver(const char *i2c_dev, ros::NodeHandle &nh) {
        // Open I2C device
        i2c_file = open(i2c_dev, O_RDWR);
        if (i2c_file < 0) {
            std::cerr << "Failed to open the I2C bus." << std::endl;
            exit(1);
        }

        // Set I2C address
        if (ioctl(i2c_file, I2C_SLAVE, MPU9250_ADDR) < 0) {
            std::cerr << "Failed to acquire bus access or talk to slave." << std::endl;
            exit(1);
        }

                // Wake up the MPU9250
        writeRegister(PWR_MGMT_1, 0x00);  // Wake up device (clears sleep mode)

        // Configure accelerometer and gyroscope for higher sampling rate
        writeRegister(ACCEL_CONFIG, 0x00);  // ±2g for accelerometer
        writeRegister(GYRO_CONFIG, 0x00);   // ±250°/s for gyroscope

        // Set the sample rate divider to get higher sampling rate
        // writeRegister(SMPLRT_DIV, 0x00);   // 1 kHz sampling rate (default)
        writeRegister(SMPLRT_DIV, 19);   // 50 Hz sampling rate (default)

        // Initialize ROS publisher
        imu_pub = nh.advertise<sensor_msgs::Imu>("/imu/data_raw", 10);
        imu_correct_pub = nh.advertise<sensor_msgs::Imu>("/imu/data/correct", 10);
    }

    ~IMUReceiver() {
        close(i2c_file);
    }

    void imuCalibration(const sensor_msgs::Imu& msg) {
        // if (calibration_done) return;

        double ax = msg.linear_acceleration.x;
        double ay = msg.linear_acceleration.y;
        double az = msg.linear_acceleration.z;

        // 중력 벡터 기반으로 Roll, Pitch 계산
        double roll = atan2(ay, az);
        double pitch = atan2(-ax, sqrt(ay * ay + az * az));

        // 오프셋 누적
        roll_offset += roll;
        pitch_offset += pitch;
        sample_count++;

        // 충분한 샘플 수집 후 평균값 계산
        if (sample_count >= calibration_samples) {
            roll_offset /= calibration_samples;
            pitch_offset /= calibration_samples;
            calibration_done = true;

            ROS_INFO("IMU Calibration Completed!");
            ROS_INFO("Roll Offset: %f degrees", roll_offset * 180.0 / M_PI);
            ROS_INFO("Pitch Offset: %f degrees", pitch_offset * 180.0 / M_PI);
        }
    }

    void readIMUData() {
        // Read accelerometer data
        int16_t accel_x = readRegister16(ACCEL_XOUT_H);
        int16_t accel_y = readRegister16(ACCEL_XOUT_H + 2);
        int16_t accel_z = readRegister16(ACCEL_XOUT_H + 4);

        // Read gyroscope data
        int16_t gyro_x = readRegister16(GYRO_XOUT_H);
        int16_t gyro_y = readRegister16(GYRO_XOUT_H + 2);
        int16_t gyro_z = readRegister16(GYRO_XOUT_H + 4);

        // Convert raw data to G's and degrees per second
        double accel_x_g = accel_x / 16383.7;  // Assuming full scale of ±2g (2.0/32767.5)
        double accel_y_g = accel_y / 16383.7;
        double accel_z_g = accel_z / 16383.7;

        double gyro_x_dps = gyro_x / 131.07;  // Assuming full scale of ±250°/s (250.0f/32767.5)
        double gyro_y_dps = gyro_y / 131.07;
        double gyro_z_dps = gyro_z / 131.07;

        tf2::Quaternion quat;
        quat.setRPY(0, 0, 0);
        quat.normalize();

        // Create and populate IMU message
        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = "imu_link";

        // Fill accelerometer data
        imu_msg.linear_acceleration.x = accel_x_g * 9.81;  // Convert to m/s^2
        imu_msg.linear_acceleration.y = accel_y_g * 9.81;
        imu_msg.linear_acceleration.z = accel_z_g * 9.81;

        // Fill gyroscope data
        // imu_msg.angular_velocity.x = gyro_x_dps * (M_PI / 180.0);  // Convert to rad/s
        // imu_msg.angular_velocity.y = gyro_y_dps * (M_PI / 180.0);
        // imu_msg.angular_velocity.z = gyro_z_dps * (M_PI / 180.0);
        imu_msg.angular_velocity.x = (gyro_x_dps * (M_PI / 180.0)) < 0.05 ? 0.0 : gyro_x_dps * (M_PI / 180.0);  // Convert to rad/s
        imu_msg.angular_velocity.y = (gyro_y_dps * (M_PI / 180.0)) < 0.05 ? 0.0 : gyro_y_dps * (M_PI / 180.0);
        imu_msg.angular_velocity.z = (gyro_z_dps * (M_PI / 180.0)) < 0.05 ? 0.0 : gyro_z_dps * (M_PI / 180.0);

        imu_msg.orientation.x = quat.getX();
        imu_msg.orientation.y = quat.getY();
        imu_msg.orientation.z = quat.getZ();
        imu_msg.orientation.w = quat.getW();

        addCovarianceToImuMsg(imu_msg);

        imu_pub.publish(imu_msg);
    }

    void addCovarianceToImuMsg(sensor_msgs::Imu& imu_msg) {
        // 1. Orientation Covariance (Roll, Pitch, Yaw)
        // Orientation은 필터링된 값이므로 추정 공분산 사용 (예: 0.01 rad^2)
        constexpr double orientation_variance = 0.01; // 약 0.57도 (rad^2 단위)
        imu_msg.orientation_covariance[0] = orientation_variance; // Roll
        imu_msg.orientation_covariance[4] = orientation_variance; // Pitch
        imu_msg.orientation_covariance[8] = orientation_variance; // Yaw
        // 비대각 요소는 0으로 유지 (상관관계 없음 가정)
        imu_msg.orientation_covariance[1] = 0.0;
        imu_msg.orientation_covariance[2] = 0.0;
        imu_msg.orientation_covariance[3] = 0.0;
        imu_msg.orientation_covariance[5] = 0.0;
        imu_msg.orientation_covariance[6] = 0.0;
        imu_msg.orientation_covariance[7] = 0.0;

        // 2. Angular Velocity Covariance (Gyroscope)
        // MPU9250 데이터시트: RMS 노이즈 0.01°/s/√Hz -> rad/s 단위로 변환 (0.0001745 rad/s)
        constexpr double gyro_noise = 0.0001745; // rad/s
        constexpr double angular_velocity_variance = gyro_noise * gyro_noise; // 약 3.04e-8 rad^2/s^2
        imu_msg.angular_velocity_covariance[0] = angular_velocity_variance; // X
        imu_msg.angular_velocity_covariance[4] = angular_velocity_variance; // Y
        imu_msg.angular_velocity_covariance[8] = angular_velocity_variance; // Z
        // 비대각 요소는 0으로 유지
        imu_msg.angular_velocity_covariance[1] = 0.0;
        imu_msg.angular_velocity_covariance[2] = 0.0;
        imu_msg.angular_velocity_covariance[3] = 0.0;
        imu_msg.angular_velocity_covariance[5] = 0.0;
        imu_msg.angular_velocity_covariance[6] = 0.0;
        imu_msg.angular_velocity_covariance[7] = 0.0;

        // 3. Linear Acceleration Covariance (Accelerometer)
        // MPU9250 데이터시트: RMS 노이즈 300 μg/√Hz -> m/s^2 단위로 변환 (0.002943 m/s^2)
        constexpr double accel_noise = 0.002943; // m/s^2
        constexpr double linear_acceleration_variance = accel_noise * accel_noise; // 약 8.66e-6 m^2/s^4
        imu_msg.linear_acceleration_covariance[0] = linear_acceleration_variance; // X
        imu_msg.linear_acceleration_covariance[4] = linear_acceleration_variance; // Y
        imu_msg.linear_acceleration_covariance[8] = linear_acceleration_variance; // Z
        // 비대각 요소는 0으로 유지
        imu_msg.linear_acceleration_covariance[1] = 0.0;
        imu_msg.linear_acceleration_covariance[2] = 0.0;
        imu_msg.linear_acceleration_covariance[3] = 0.0;
        imu_msg.linear_acceleration_covariance[5] = 0.0;
        imu_msg.linear_acceleration_covariance[6] = 0.0;
        imu_msg.linear_acceleration_covariance[7] = 0.0;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "imuReceiver_node");
    ROS_INFO("imuReceiver_node");
    ros::NodeHandle nh;

    const char *i2c_device = "/dev/i2c-7";  // Default I2C bus on Jetson Orin Nano
    IMUReceiver imuReceiver(i2c_device, nh);

    ros::Rate rate(50);  // 10 Hz
    while (ros::ok()) {
        imuReceiver.readIMUData();
        rate.sleep();
    }

    return 0;
}
