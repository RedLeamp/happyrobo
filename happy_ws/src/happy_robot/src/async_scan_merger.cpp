#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "laser_geometry/laser_geometry.h"
#include <geometry_msgs/PointStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <thread>
#include <chrono>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <mutex>

class LaserScanMerger
{
public:
    LaserScanMerger(ros::NodeHandle& nh) : nh(nh) {
        // LaserScan 구독 (message_filters 사용)
        scan01_sub_.subscribe(nh, "/sigbot/scan01", 1);
        scan02_sub_.subscribe(nh, "/sigbot/scan02", 1);
        scan03_sub_.subscribe(nh, "/sigbot/scan03", 1);
        scan04_sub_.subscribe(nh, "/sigbot/scan04", 1);

        // ApproximateTimeSynchronizer 설정 (4개의 데이터를 동기화)
        // SyncPolicy 값	의미	장점	단점
        // 5	약 333ms 유지	최신 데이터 반영, 지연 최소화	동기화 실패 확률 높음
        // 10	약 666ms 유지	동기화 성공률 적절	약간의 지연 가능
        // 15	약 1초 유지	동기화 성공률 높음	지연 가능성 증가
        // 20	약 1.33초 유지	매우 높은 동기화율	지연이 커질 수 있음

        sync_.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), scan01_sub_, scan02_sub_, scan03_sub_, scan04_sub_));
        sync_->registerCallback(boost::bind(&LaserScanMerger::scanCallback, this, _1, _2, _3, _4));

        merged_scan_pub_ = nh.advertise<sensor_msgs::LaserScan>("/merged/scan", 1);
        merged_point_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/merged/pointClouds", 1);
        transformed_point_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/transformed_point", 1);
        scan01_pub_ = nh.advertise<sensor_msgs::LaserScan>("/test/01", 1);
        scan02_pub_ = nh.advertise<sensor_msgs::LaserScan>("/test/02", 1);
        scan03_pub_ = nh.advertise<sensor_msgs::LaserScan>("/test/03", 1);
        scan04_pub_ = nh.advertise<sensor_msgs::LaserScan>("/test/04", 1);
        tf_2_to_1_is_known_ = false;
        tf_3_to_1_is_known_ = false;
        tf_4_to_1_is_known_ = false;
    }

protected:

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan01, 
                      const sensor_msgs::LaserScan::ConstPtr &scan02, 
                      const sensor_msgs::LaserScan::ConstPtr &scan03, 
                      const sensor_msgs::LaserScan::ConstPtr &scan04) {
        // scan01을 기준으로 변환
        std::string target_frame = scan01->header.frame_id;
        sensor_msgs::LaserScan scan = *scan01;  // 기준 scan01 데이터 복사

        // get TF between scan02, scan03, scan04 and scan01
        if (!tf_2_to_1_is_known_) lookupTransform(*scan02, *scan01, tf_scan_2_to_scan_1_, tf_2_to_1_is_known_);
        if (!tf_3_to_1_is_known_) lookupTransform(*scan03, *scan01, tf_scan_3_to_scan_1_, tf_3_to_1_is_known_);
        if (!tf_4_to_1_is_known_) lookupTransform(*scan04, *scan01, tf_scan_4_to_scan_1_, tf_4_to_1_is_known_);

        if (!tf_2_to_1_is_known_ && !tf_3_to_1_is_known_ && !tf_4_to_1_is_known_) return;

        // ROS_INFO("Transforms are Ready!");

        sensor_msgs::PointCloud2 MergedCloud = laserScanToPointCloud2(scan);
        // scan02, scan03, scan04를 변환하여 병합
        sensor_msgs::PointCloud2 cloud21 = mergeTwoPointClouds(MergedCloud, *scan02, tf_scan_2_to_scan_1_);
        sensor_msgs::PointCloud2 cloud321 = mergeTwoPointClouds(cloud21, *scan03, tf_scan_3_to_scan_1_);
        sensor_msgs::PointCloud2 cloud4321 = mergeTwoPointClouds(cloud321, *scan04, tf_scan_4_to_scan_1_);

        merged_point_pub_.publish(cloud4321); // 병합된 데이터 발행

        sensor_msgs::LaserScan merged_scan;

        pointCloud2ToLaserScan(cloud4321, merged_scan);

        merged_scan_pub_.publish(merged_scan); // 병합된 데이터 발행
    }

    void lookupTransform(sensor_msgs::LaserScan source_scan, sensor_msgs::LaserScan target_scan, tf::StampedTransform& tf, bool& tf_is_known_) {

        ROS_INFO_STREAM("Waiting for transform between src, tar frames : " << target_scan.header.frame_id << ", " << source_scan.header.frame_id);
        // src frame -> tar frame
        if (!tf_is_known_) {
            constexpr size_t kTFTimeout_ms = 1000;
            constexpr size_t kMsToNs = 1000000;

            try {
                ros::Duration kTFWait = ros::Duration(0, 200*kMsToNs);
                tf_listener_.waitForTransform(target_scan.header.frame_id, source_scan.header.frame_id,
                                            target_scan.header.stamp + kTFWait,
                                            ros::Duration(0, kTFTimeout_ms * kMsToNs));
                tf_listener_.lookupTransform(target_scan.header.frame_id, source_scan.header.frame_id,
                                            target_scan.header.stamp + kTFWait, tf);
            }
            catch ( tf::TransformException &ex ) {
                ROS_ERROR_STREAM("Error while looking up transform between scan 1 and scan 2 : " << ex.what());
                tf_is_known_ = false;
                return;
            }
            ROS_INFO_STREAM("Transform found !");
            tf_is_known_ = true;
        }
        return;
    }

    sensor_msgs::PointCloud2 laserScanToPointCloud2(sensor_msgs::LaserScan & scan)
    {
        static laser_geometry::LaserProjection projector;
        sensor_msgs::PointCloud2 pc2_dst;
        projector.projectLaser(scan, pc2_dst,-1,laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Distance);
        pc2_dst.header.frame_id = scan.header.frame_id;
        return pc2_dst;

        // // PointCloud2 메시지 생성
        // sensor_msgs::PointCloud2 cloud;
        // cloud.header = scan.header;
        // cloud.height = 1;  // 1D 포인트 클라우드
        // cloud.width = scan.ranges.size();  // ranges 크기만큼 포인트 개수 설정

        // // PointCloud2의 필드 정의
        // pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        // pcl_cloud.width = scan.ranges.size();
        // pcl_cloud.height = 1;

        // // x, y, z 필드 설정
        // pcl_cloud.points.resize(pcl_cloud.width * pcl_cloud.height);

        // // PointCloud2의 데이터 포인터 설정
        // sensor_msgs::PointCloud2Modifier pcd_modifier(cloud);
        // pcd_modifier.setPointCloud2FieldsByString(1, "xyz"); // x, y, z 필드

        // // 각 레이의 거리 데이터를 이용해 x, y, z 계산
        // for (size_t i = 0; i < scan.ranges.size(); ++i)
        // {
        //     float range = scan.ranges[i];

        //     // 유효한 범위 내에서만 포인트 생성
        //     if (range >= scan.range_min && range <= scan.range_max)
        //     {
        //         float angle = scan.angle_min + i * scan.angle_increment;

        //         pcl_cloud.points[i].x = range * cos(angle);
        //         pcl_cloud.points[i].y = range * sin(angle);
        //         pcl_cloud.points[i].z = 0.0;  // 2D 레이이므로 Z값은 0으로 설정
        //     }
        //     else
        //     {
        //         pcl_cloud.points[i].x = std::numeric_limits<float>::quiet_NaN();
        //         pcl_cloud.points[i].y = std::numeric_limits<float>::quiet_NaN();
        //         pcl_cloud.points[i].z = std::numeric_limits<float>::quiet_NaN();
        //     }
        // }

        // // PointCloud2에 pcl_cloud의 포인트 데이터 복사
        // pcl::toPCLPointCloud2(pcl_cloud, cloud);

        // return cloud;
    }

    void pointCloud2ToLaserScan(sensor_msgs::PointCloud2& cloud, sensor_msgs::LaserScan& scan)
    {
        // LaserScan 메시지 생성
        scan.header = cloud.header;

        // PointCloud2에서 pcl::PointCloud<pcl::PointXYZ>로 변환
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(cloud, pcl_pc2);
        // pcl_conversions::toPCL(cloud, pcl_cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
        // pcl::fromPCLPointCloud2(cloud, pcl_cloud);
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud = *temp_cloud;

        // LaserScan의 기본 필드 설정
        scan.angle_min = -M_PI;  // 예시: -90도
        scan.angle_max = M_PI;   // 예시: +90도
        // scan.angle_increment = 0.75*M_PI / 360;  // 예시: 1도씩 증가
        // scan.angle_increment = 0.00982; // 360'/640
        scan.angle_increment = 2*M_PI/640; // 360'/640
        scan.scan_time = 1.0 / 15.0;  // 1회 스캔하는 데 걸리는 시간
        scan.range_min = 0.01;  // 최소 범위
        scan.range_max = 100.0; // 최대 범위
        // scan.time = 1.0;

        // 레이의 개수 계산 (각 포인트마다 하나의 레이 계산)
        // scan.ranges.resize(pcl_cloud.points.size());
        uint32_t ranges_size = std::ceil((scan.angle_max - scan.angle_min) / scan.angle_increment);
        scan.ranges.assign(ranges_size, scan.range_max + 1.0);
        // scan.ranges.assign((scan.range_max - scan.range_min) / scan.angle_increment);

        // LaserScan 데이터를 계산하여 채우기
        for (size_t i = 0; i < pcl_cloud.points.size(); ++i)
        {
            // 각 포인트의 x, y, z 값을 가져와 레이의 각도 및 범위 계산
            float x = pcl_cloud.points[i].x;
            float y = pcl_cloud.points[i].y;
            float z = pcl_cloud.points[i].z;

            // 레이의 각도 계산 (x, y를 기반으로)
            double angle = atan2(y, x);

            // 레이의 범위 계산 (점 (x, y)까지의 거리)
            double range = sqrt(x * x + y * y);

            // 범위가 유효한 경우에만 데이터 저장
            if (range >= scan.range_min && range <= scan.range_max)
            {
                int index = (angle - scan.angle_min) / scan.angle_increment;

                if (index >= 0 && index < scan.ranges.size())
                {
                    scan.ranges[index] = range;
                }
            } else {
                std::cout << "range is out of range" << std::endl;
            }
        }
    }

    sensor_msgs::PointCloud2 mergeTwoPointClouds(sensor_msgs::PointCloud2 &MergedCloud, 
                                                sensor_msgs::LaserScan scan, 
                                                tf::StampedTransform& tf) {

        sensor_msgs::PointCloud2 scanCloud = laserScanToPointCloud2(scan);

        sensor_msgs::PointCloud2 scan_in_frame1;
        pcl_ros::transformPointCloud(MergedCloud.header.frame_id, tf, scanCloud, scan_in_frame1);

        // transformed_point_pub_.publish(scan_in_frame1); // 변환된 데이터 발행

        // PCL 포인트 클라우드 선언
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud1, pcl_cloud2, merged_cloud;

        // PointCloud2 메시지를 PCL 포인트 클라우드로 변환
        pcl::fromROSMsg(MergedCloud, pcl_cloud1);
        pcl::fromROSMsg(scan_in_frame1, pcl_cloud2);

        // 두 개의 포인트 클라우드를 병합
        merged_cloud = pcl_cloud1;  // 첫 번째 포인트 클라우드 복사
        merged_cloud += pcl_cloud2; // 두 번째 포인트 클라우드 추가

        // 병합된 포인트 클라우드를 다시 PointCloud2 메시지로 변환
        sensor_msgs::PointCloud2 merged_msg;
        pcl::toROSMsg(merged_cloud, merged_msg);

        // 헤더 설정
        merged_msg.header = MergedCloud.header;  // 첫 번째 메시지의 헤더를 사용

        return merged_msg;

        // MergedCloud.width += scan_in_frame1.width;
        // uint64_t OriginalSize = MergedCloud.data.size();
        // MergedCloud.data.resize(MergedCloud.data.size() + scan_in_frame1.data.size());
        // std::copy(scan_in_frame1.data.begin(),scan_in_frame1.data.end(), MergedCloud.data.begin() + OriginalSize);
        
        // return MergedCloud;
    }

private:
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan, sensor_msgs::LaserScan, sensor_msgs::LaserScan> SyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    message_filters::Subscriber<sensor_msgs::LaserScan> scan01_sub_;
    message_filters::Subscriber<sensor_msgs::LaserScan> scan02_sub_;
    message_filters::Subscriber<sensor_msgs::LaserScan> scan03_sub_;
    message_filters::Subscriber<sensor_msgs::LaserScan> scan04_sub_;
    
    ros::NodeHandle nh;
    ros::Publisher merged_scan_pub_;
    // tf2_ros::Buffer tf_buffer_;
    tf::TransformListener tf_listener_;
    // tf2_ros::TransformListener tf_listener_;
    tf::StampedTransform tf_scan_2_to_scan_1_;
    tf::StampedTransform tf_scan_3_to_scan_1_;
    tf::StampedTransform tf_scan_4_to_scan_1_;
    bool tf_2_to_1_is_known_;
    bool tf_3_to_1_is_known_;
    bool tf_4_to_1_is_known_;

    ros::Publisher scan01_pub_;
    ros::Publisher scan02_pub_;
    ros::Publisher scan03_pub_;
    ros::Publisher scan04_pub_;
    ros::Publisher merged_point_pub_;
    ros::Publisher transformed_point_pub_;

    // sensor_msgs::LaserScan generateEmptyOutputScan(sensor_msgs::LaserScan reference_scan) const {
    //     const double kIncrementUpsampling = 1. / kResolutionUpsampling;
    //     // Create a new scan
    //     sensor_msgs::LaserScan combined_scan;
    //     combined_scan.header = reference_scan.header;
    //     combined_scan.header.stamp = reference_scan.header.stamp;
    //     combined_scan.angle_increment = reference_scan.angle_increment * kIncrementUpsampling;
    //     combined_scan.time_increment = reference_scan.time_increment * kIncrementUpsampling;
    //     combined_scan.scan_time = reference_scan.scan_time;
    //     combined_scan.range_min = reference_scan.range_min;
    //     combined_scan.range_max = reference_scan.range_max;
    //     combined_scan.angle_min = reference_scan.angle_min; // start with the same angle
    //     combined_scan.ranges.resize(2*M_PI / combined_scan.angle_increment, 0.);
    //     combined_scan.intensities.resize(combined_scan.ranges.size(), 0.);
    //     combined_scan.angle_max = reference_scan.angle_min +
    //     combined_scan.angle_increment * ( combined_scan.ranges.size() - 1 ); // sh. be full circle
    //     return combined_scan;
    // }

    // void mergeScan2 (sensor_msgs::LaserScan &merged_scan, 
    //                const sensor_msgs::LaserScan::ConstPtr &scan, 
    //                const std::string &target_frame) {
    //     sensor_msgs::LaserScan combined_scan = generateEmptyOutputScan(msg->header.stamp);
    //     // Convert the point cloud to frame 1
    //     sensor_msgs::PointCloud2 scan2_in_frame1;
    //     pcl_ros::transformPointCloud(reference_scan1_.header.frame_id, tf_scan_1_to_scan_2_,
    //         *msg, scan2_in_frame1);
    //     // Convert the point cloud to scan values (angle, distance).
    //     sensor_msgs::PointCloud scan2_in_frame1_xyzi;
    //     sensor_msgs::convertPointCloud2ToPointCloud(scan2_in_frame1, scan2_in_frame1_xyzi);
    //     // Find intensity values if desired
    //     bool found_intensities = false;
    //     std::vector<float> scan2_intensities;
    //     try {
    //     for ( size_t i = 0; i < scan2_in_frame1_xyzi.channels.size(); i++ ) {
    //         if ( scan2_in_frame1_xyzi.channels.at(i).name == "intensity" ) {
    //         scan2_intensities = scan2_in_frame1_xyzi.channels.at(i).values;
    //         if ( scan2_intensities.size() == scan2_in_frame1_xyzi.points.size() ) {
    //             found_intensities = true;
    //             break;
    //         }
    //         }
    //     }
    //     } catch (const std::exception& e) {
    //     }

    //     // Fill in values from scan 2
    //     // Scan 2 angles
    //     for ( size_t i = 0; i < scan2_in_frame1_xyzi.points.size();  i++ ) {
    //     geometry_msgs::Point32 p = scan2_in_frame1_xyzi.points.at(i);
    //     float angle = atan2(p.y, p.x);
    //     float range = sqrt(p.y * p.y + p.x * p.x);
    //     // find the index in combined_scan corresponding to that angle.
    //     float relative_angle = angle - combined_scan.angle_min;
    //     // constrain relative angle to [0, 2pi[
    //     if ( abs(relative_angle) >= 2*M_PI ) {
    //         relative_angle = fmod(relative_angle, ( 2*M_PI ));
    //     }
    //     if ( relative_angle < 0 && combined_scan.angle_increment > 0 ) {
    //         relative_angle += 2*M_PI;
    //     }
    //     if ( relative_angle > 0 && combined_scan.angle_increment < 0 ) {
    //         relative_angle -= 2*M_PI;
    //     }
    //     CHECK( ( relative_angle / combined_scan.angle_increment )  >= 0 );
    //     size_t index = round(relative_angle / combined_scan.angle_increment);
    //     if ( index == combined_scan.ranges.size() ) {
    //         index = 0;
    //     }
    //     VLOG(2) << "angle: " << angle;
    //     VLOG(2) << "min angle: " << combined_scan.angle_min;
    //     VLOG(2) << "max angle: " << combined_scan.angle_max;
    //     VLOG(2) << "angle inc: " << combined_scan.angle_increment;
    //     VLOG(2) << "rel angle: " << relative_angle;
    //     VLOG(2) << "index: " << index;
    //     combined_scan.ranges.at(index) = range;
    //     if ( found_intensities ) {
    //         combined_scan.intensities.at(index) = scan2_intensities.at(i);
    //     }
    //     }
    //     // publish result.
    //     latest_published_scan_ = combined_scan;
    //     latest_published_scan_is_set_ = true;
    //     combined_scan_pub_.publish(combined_scan);
    // }

    // void mergeScan(sensor_msgs::LaserScan &merged_scan, 
    //                const sensor_msgs::LaserScan::ConstPtr &scan, 
    //                const std::string &target_frame)
    // {
    //     std::string source_frame = scan->header.frame_id;

    //     geometry_msgs::TransformStamped transform;
    //     try
    //     {
    //         transform = tf_buffer_.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(0.1));
    //     }
    //     catch (tf2::TransformException &ex)
    //     {
    //         ROS_WARN("%s", ex.what());
    //         return;
    //     }

    //      // scan01의 ranges 크기를 기준으로 scan02, scan03, scan04의 ranges 크기를 맞추기
    //     size_t num_ranges = merged_scan.ranges.size();
    //     size_t scan_size = scan->ranges.size();

    //     // scan의 ranges가 merged_scan의 크기와 다를 경우 보간법을 사용하여 크기 맞추기
    //     if (scan_size != num_ranges)
    //     {
    //         // 보간법 또는 리샘플링 코드 추가 (간단히 가장 가까운 인덱스값으로 맞추는 방법 예시)
    //         for (size_t i = 0; i < num_ranges; i++)
    //         {
    //             // 각도 계산: (각도는 angle_min + i * angle_increment)
    //             float angle = merged_scan.angle_min + i * merged_scan.angle_increment;
    //             size_t scan_index = static_cast<size_t>((angle - scan->angle_min) / scan->angle_increment);
    //             if (scan_index < scan_size)
    //             {
    //                 // 가까운 인덱스의 값을 복사
    //                 merged_scan.ranges[i] = scan->ranges[scan_index];
    //             }
    //         }
    //     }

    //     for (size_t i = 0; i < scan->ranges.size(); i++)
    //     {
    //         if (std::isinf(scan->ranges[i]) || std::isnan(scan->ranges[i]))
    //             continue;

    //         float angle = scan->angle_min + i * scan->angle_increment;
    //         float range = scan->ranges[i];

    //         // LaserScan 데이터를 변환하여 기준 좌표계로 이동
    //         geometry_msgs::PointStamped source_point, transformed_point;
    //         source_point.header.frame_id = source_frame;
    //         source_point.point.x = range * cos(angle);
    //         source_point.point.y = range * sin(angle);
    //         source_point.point.z = 0.0;

    //         try
    //         {
    //             tf2::doTransform(source_point, transformed_point, transform);
    //         }
    //         catch (tf2::TransformException &ex)
    //         {
    //             ROS_WARN("%s", ex.what());
    //             continue;
    //         }

    //         // 변환된 점을 scan01의 LaserScan 인덱스에 반영
    //         float transformed_range = sqrt(pow(transformed_point.point.x, 2) + pow(transformed_point.point.y, 2));
    //         float transformed_angle = atan2(transformed_point.point.y, transformed_point.point.x);

    //         if (transformed_angle >= merged_scan.angle_min && transformed_angle <= merged_scan.angle_max)
    //         {
    //             int index = (transformed_angle - merged_scan.angle_min) / merged_scan.angle_increment;
    //             if (index >= 0 && index < merged_scan.ranges.size())
    //             {
    //                 if (merged_scan.ranges[index] == 0 || transformed_range < merged_scan.ranges[index])
    //                 {
    //                     merged_scan.ranges[index] = transformed_range;
    //                 }
    //             }
    //         }
    //     }
    // }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_scan_merger");
    ros::NodeHandle nh;
    // LaserScanMerger pthread_mutexattr_getprioceiling(nh);
    LaserScanMerger merger(nh);
    try {
        // ros::MultiThreadedSpinner spinner(2); // Necessary to allow concurrent callbacks.
        // spinner.spin();
        ros::spin();
    }
    catch (const std::exception& e) {
        ROS_ERROR_STREAM("Exception: " << e.what());
        return 1;
    }
    catch (...) {
        ROS_ERROR_STREAM("Unknown Exception.");
        return 1;
    }

	return 0;
    // LaserScanMerger merger(nh);
    // ros::spin();
    // return 0;
}
