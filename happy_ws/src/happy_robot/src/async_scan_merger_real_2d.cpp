#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "laser_geometry/laser_geometry.h"
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
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
        // scan00_sub_.subscribe(nh, "/cygbot/scan/2d/0", 1);
        scan01_sub_.subscribe(nh, "/cygbot/scan/2d/1", 1);
        scan02_sub_.subscribe(nh, "/cygbot/scan/2d/2", 1);
        scan03_sub_.subscribe(nh, "/cygbot/scan/2d/3", 1);
        scan04_sub_.subscribe(nh, "/cygbot/scan/2d/4", 1);
        // scan05_sub_.subscribe(nh, "/cygbot/scan/2d/5", 1);
        scan06_sub_.subscribe(nh, "/cygbot/scan/2d/6", 1);
        scan07_sub_.subscribe(nh, "/cygbot/scan/2d/7", 1);

        // ApproximateTimeSynchronizer 설정 (4개의 데이터를 동기화)
        // SyncPolicy 값	의미	장점	단점
        // 5	약 333ms 유지	최신 데이터 반영, 지연 최소화	동기화 실패 확률 높음
        // 10	약 666ms 유지	동기화 성공률 적절	약간의 지연 가능
        // 15	약 1초 유지	동기화 성공률 높음	지연 가능성 증가
        // 20	약 1.33초 유지	매우 높은 동기화율	지연이 커질 수 있음

        // sync_.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), scan00_sub_, scan01_sub_, scan02_sub_, scan03_sub_, scan04_sub_, scan05_sub_, scan06_sub_, scan07_sub_));
        // sync_->registerCallback(boost::bind(&LaserScanMerger::scanCallback, this, _1, _2, _3, _4, _5, _6, _7, _8));

        sync_.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), scan01_sub_, scan02_sub_, scan03_sub_, scan04_sub_, scan06_sub_, scan07_sub_));
        sync_->registerCallback(boost::bind(&LaserScanMerger::scanCallback, this, _1, _2, _3, _4, _5, _6));

        merged_scan_pub_ = nh.advertise<sensor_msgs::LaserScan>("/merged/scan", 10);
        merged_point_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/merged/points", 10);
        transformed_point_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/transformed_point", 10);
        tf_1_to_base_is_known_ = false;
        tf_2_to_base_is_known_ = false;
        tf_3_to_base_is_known_ = false;
        tf_4_to_base_is_known_ = false;
        tf_5_to_base_is_known_ = false;
        tf_6_to_base_is_known_ = false;
        tf_7_to_base_is_known_ = false;
        base_frame = "base_link";
    }

protected:

    void scanCallback(
                    //   const sensor_msgs::LaserScan::ConstPtr &scan00, 
                      const sensor_msgs::LaserScan::ConstPtr &scan01, 
                      const sensor_msgs::LaserScan::ConstPtr &scan02, 
                      const sensor_msgs::LaserScan::ConstPtr &scan03, 
                      const sensor_msgs::LaserScan::ConstPtr &scan04,
                    //   const sensor_msgs::LaserScan::ConstPtr &scan05,
                      const sensor_msgs::LaserScan::ConstPtr &scan06,
                      const sensor_msgs::LaserScan::ConstPtr &scan07
                      ) {

        double init = ros::Time::now().toSec();

        ROS_INFO("Got MSG !");

        // get TF between scan02, scan03, scan04 and scan01
        // if (!tf_0_to_base_is_known_) lookupTransform(*scan00, base_frame, tf_scan_0_to_base_, tf_0_to_base_is_known_);
        if (!tf_1_to_base_is_known_) lookupTransform(*scan01, base_frame, tf_scan_1_to_base_, tf_1_to_base_is_known_);
        if (!tf_2_to_base_is_known_) lookupTransform(*scan02, base_frame, tf_scan_2_to_base_, tf_2_to_base_is_known_);
        if (!tf_3_to_base_is_known_) lookupTransform(*scan03, base_frame, tf_scan_3_to_base_, tf_3_to_base_is_known_);
        if (!tf_4_to_base_is_known_) lookupTransform(*scan04, base_frame, tf_scan_4_to_base_, tf_4_to_base_is_known_);
        // if (!tf_5_to_base_is_known_) lookupTransform(*scan05, base_frame, tf_scan_5_to_base_, tf_5_to_base_is_known_);
        if (!tf_6_to_base_is_known_) lookupTransform(*scan06, base_frame, tf_scan_6_to_base_, tf_6_to_base_is_known_);
        if (!tf_7_to_base_is_known_) lookupTransform(*scan07, base_frame, tf_scan_7_to_base_, tf_7_to_base_is_known_);

        // if (!tf_0_to_base_is_known_ && !tf_1_to_base_is_known_ && !tf_2_to_base_is_known_ && !tf_3_to_base_is_known_ && !tf_4_to_base_is_known_ && !tf_5_to_base_is_known_ && !tf_6_to_base_is_known_ && !tf_7_to_base_is_known_) return;
        if (!tf_1_to_base_is_known_ && !tf_2_to_base_is_known_ && !tf_3_to_base_is_known_ && !tf_4_to_base_is_known_ && !tf_6_to_base_is_known_ && !tf_7_to_base_is_known_) return;


        // scan02, scan03, scan04를 변환하여 병합
        // sensor_msgs::PointCloud2 mergedPoints = laserScanToPointCloud2(*scan00);
        // transformPointCloudToTargerFrame(mergedPoints, tf_scan_0_to_base_, base_frame);

        sensor_msgs::PointCloud2 mergedPoints = laserScanToPointCloud2(*scan01);
        transformPointCloudToTargerFrame(mergedPoints, tf_scan_1_to_base_, base_frame);

        // sensor_msgs::PointCloud2 scan01_point = laserScanToPointCloud2(*scan01);
        // transformPointCloudToTargerFrame(scan01_point, tf_scan_1_to_base_, base_frame);
        // mergeTwoPointClouds(mergedPoints, scan01_point);

        sensor_msgs::PointCloud2 scan02_point = laserScanToPointCloud2(*scan02);
        transformPointCloudToTargerFrame(scan02_point, tf_scan_2_to_base_, base_frame);
        mergeTwoPointClouds(mergedPoints, scan02_point);

        sensor_msgs::PointCloud2 scan03_point = laserScanToPointCloud2(*scan03);
        transformPointCloudToTargerFrame(scan03_point, tf_scan_3_to_base_, base_frame);
        mergeTwoPointClouds(mergedPoints, scan03_point);

        //      sensor_msgs::PointCloud2 mergedPoints = laserScanToPointCloud2(*scan04);
        // transformPointCloudToTargerFrame(mergedPoints, tf_scan_4_to_base_, base_frame);



        sensor_msgs::PointCloud2 scan04_point = laserScanToPointCloud2(*scan04);
        transformPointCloudToTargerFrame(scan04_point, tf_scan_4_to_base_, base_frame);
        mergeTwoPointClouds(mergedPoints, scan04_point);

        // sensor_msgs::PointCloud2 scan05_point = laserScanToPointCloud2(*scan05);
        // transformPointCloudToTargerFrame(scan05_point, tf_scan_5_to_base_, base_frame);
        // mergeTwoPointClouds(mergedPoints, scan05_point);

        sensor_msgs::PointCloud2 scan06_point = laserScanToPointCloud2(*scan06);
        transformPointCloudToTargerFrame(scan06_point, tf_scan_6_to_base_, base_frame);
        mergeTwoPointClouds(mergedPoints, scan06_point);

        sensor_msgs::PointCloud2 scan07_point = laserScanToPointCloud2(*scan07);
        transformPointCloudToTargerFrame(scan07_point, tf_scan_7_to_base_, base_frame);
        mergeTwoPointClouds(mergedPoints, scan07_point);

        merged_point_pub_.publish(mergedPoints); // 병합된 데이터 발행

        sensor_msgs::LaserScan mergedScan;

        pointCloud2ToLaserScan(mergedPoints, mergedScan);

        merged_scan_pub_.publish(mergedScan); // 병합된 데이터 발행

        // std::vector<double> timeStamps = {scan00->header.stamp.toSec(), scan01->header.stamp.toSec(), scan02->header.stamp.toSec(), scan03->header.stamp.toSec(), scan04->header.stamp.toSec(), scan05->header.stamp.toSec(), scan06->header.stamp.toSec(), scan07->header.stamp.toSec()};
        std::vector<double> timeStamps = {scan01->header.stamp.toSec(), scan02->header.stamp.toSec(), scan03->header.stamp.toSec(), scan04->header.stamp.toSec(), scan06->header.stamp.toSec(), scan07->header.stamp.toSec()};
        double firstIn = *std::min_element(timeStamps.begin(), timeStamps.end());
        double current = ros::Time::now().toSec();
        ROS_INFO_STREAM("Time Diff : " << current - firstIn <<"\nCal Time : " << current - init);
    }

    void lookupTransform(sensor_msgs::LaserScan source_points, std::string target_frame, tf::StampedTransform& tf, bool& tf_is_known_) {

        ROS_INFO_STREAM("Waiting for transform between src, tar frames : " << target_frame << ", " << source_points.header.frame_id);
        // src frame -> tar frame
        if (!tf_is_known_) {
            constexpr size_t kTFTimeout_ms = 1000;
            constexpr size_t kMsToNs = 1000000;

            try {
                ros::Duration kTFWait = ros::Duration(0, 200*kMsToNs);
                tf_listener_.waitForTransform(target_frame, source_points.header.frame_id, source_points.header.stamp + kTFWait, ros::Duration(0, kTFTimeout_ms * kMsToNs));
                tf_listener_.lookupTransform(target_frame, source_points.header.frame_id, source_points.header.stamp + kTFWait, tf);
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

    sensor_msgs::PointCloud2 laserScanToPointCloud2(const sensor_msgs::LaserScan scan)
    {
        static laser_geometry::LaserProjection projector;
        sensor_msgs::PointCloud2 pc2_raw;
        projector.projectLaser(scan, pc2_raw, -1, laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Distance);

        // Convert to PCL for filtering
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(pc2_raw, *pcl_cloud);

        // Filter out points with distance >= 7.0
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_filtered(new pcl::PointCloud<pcl::PointXYZI>());
        for (const auto& pt : pcl_cloud->points)
        {
            float distance = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
            if (distance <= 7.0)
            {
                pcl_filtered->points.push_back(pt);
            }
        }

        // Convert back to ROS message
        sensor_msgs::PointCloud2 pc2_filtered;
        pcl::toROSMsg(*pcl_filtered, pc2_filtered);
        pc2_filtered.header = scan.header;

        return pc2_filtered;
        // static laser_geometry::LaserProjection projector;
        // sensor_msgs::PointCloud2 pc2_dst;
        // projector.projectLaser(scan, pc2_dst,-1,laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Distance);
        // pc2_dst.header.frame_id = scan.header.frame_id;
        // return pc2_dst;
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
        scan.angle_increment = 2*M_PI/pcl_cloud.points.size(); // 360'/640
        scan.scan_time = 1.0 / 15.0;  // 1회 스캔하는 데 걸리는 시간
        scan.range_min = 0.01;  // 최소 범위
        scan.range_max = 100.0; // 최대 범위
        // scan.time = 1.0;

        // ROS_INFO_STREAM("cloud.size : " << pcl_cloud.points.size());

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

    void transformPointCloudToTargerFrame(sensor_msgs::PointCloud2& sourceCloud, tf::StampedTransform& tf, std::string target_frame) {

        // pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        // pcl::fromROSMsg(sourceCloud, pcl_cloud);

        // // 변환 적용
        // pcl::PointCloud<pcl::PointXYZ> pcl_cloud_transformed;
        // Eigen::Matrix4f eigen_transform;
        // pcl_ros::transformAsMatrix(tf, eigen_transform);
        // pcl::transformPointCloud(pcl_cloud, pcl_cloud_transformed, eigen_transform);

        // sensor_msgs::PointCloud2 output_cloud;
        // // 변환된 PCL PointCloud를 ROS 메시지로 변환
        // pcl::toROSMsg(pcl_cloud_transformed, output_cloud);
        // output_cloud.header.frame_id = target_frame;  // 변환된 프레임 ID 설정
        // output_cloud.header.stamp = sourceCloud.header.stamp;
        // sensor_msgs::PointCloud2 transformedCloud;
        // tf2::doTransform(sourceCloud, sourceCloud, tf);
        pcl_ros::transformPointCloud(target_frame, tf, sourceCloud, sourceCloud);
        sourceCloud.header.frame_id = target_frame;
        // return output_cloud;
    }

    void mergeTwoPointClouds(sensor_msgs::PointCloud2 &MergedCloud, 
                                                sensor_msgs::PointCloud2 sourceCloud) {

        // PCL 포인트 클라우드 선언
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud1, pcl_cloud2, merged_cloud;
        pcl::fromROSMsg(MergedCloud, pcl_cloud1);
        pcl::fromROSMsg(sourceCloud, pcl_cloud2);

            // 변환된 포인트 클라우드에서 NaN, Inf 값이 있는지 점검
        for (const auto& point : pcl_cloud1) {
            if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) {
                ROS_WARN("NaN point detected in pcl_cloud1");
            }
        }

        for (const auto& point : pcl_cloud2) {
            if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) {
                ROS_WARN("NaN point detected in pcl_cloud2");
            }
        }

        // 두 개의 포인트 클라우드를 병합
        merged_cloud = pcl_cloud1;  // 첫 번째 포인트 클라우드 복사
        merged_cloud += pcl_cloud2; // 두 번째 포인트 클라우드 추가

        // 병합된 포인트 클라우드를 다시 PointCloud2 메시지로 변환
        pcl::toROSMsg(merged_cloud, MergedCloud);
        // // 헤더 설정
        // merged_msg.header = MergedCloud.header;  // 첫 번째 메시지의 헤더를 사용
        // return merged_msg;
    }  

    sensor_msgs::PointCloud2 mergeTwoPointClouds2(sensor_msgs::PointCloud2 &MergedCloud, 
                                                sensor_msgs::PointCloud2 sourceCloud) {

        // PCL 포인트 클라우드 선언
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud1, pcl_cloud2, merged_cloud;
        pcl::fromROSMsg(MergedCloud, pcl_cloud1);
        pcl::fromROSMsg(sourceCloud, pcl_cloud2);

            // 변환된 포인트 클라우드에서 NaN, Inf 값이 있는지 점검
        for (const auto& point : pcl_cloud1) {
            if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) {
                ROS_WARN("NaN point detected in pcl_cloud1");
            }
        }

        for (const auto& point : pcl_cloud2) {
            if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) {
                ROS_WARN("NaN point detected in pcl_cloud2");
            }
        }

        // 두 개의 포인트 클라우드를 병합
        merged_cloud = pcl_cloud1;  // 첫 번째 포인트 클라우드 복사
        merged_cloud += pcl_cloud2; // 두 번째 포인트 클라우드 추가

        // 병합된 포인트 클라우드를 다시 PointCloud2 메시지로 변환
        sensor_msgs::PointCloud2 merged_msg;
        pcl::toROSMsg(merged_cloud, merged_msg);
        // 헤더 설정
        merged_msg.header = MergedCloud.header;  // 첫 번째 메시지의 헤더를 사용
        return merged_msg;
    }

private:
    // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan, sensor_msgs::LaserScan, sensor_msgs::LaserScan, sensor_msgs::LaserScan, sensor_msgs::LaserScan, sensor_msgs::LaserScan, sensor_msgs::LaserScan> SyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan, sensor_msgs::LaserScan, sensor_msgs::LaserScan, sensor_msgs::LaserScan, sensor_msgs::LaserScan> SyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    message_filters::Subscriber<sensor_msgs::LaserScan> scan00_sub_;
    message_filters::Subscriber<sensor_msgs::LaserScan> scan01_sub_;
    message_filters::Subscriber<sensor_msgs::LaserScan> scan02_sub_;
    message_filters::Subscriber<sensor_msgs::LaserScan> scan03_sub_;
    message_filters::Subscriber<sensor_msgs::LaserScan> scan04_sub_;
    message_filters::Subscriber<sensor_msgs::LaserScan> scan05_sub_;
    message_filters::Subscriber<sensor_msgs::LaserScan> scan06_sub_;
    message_filters::Subscriber<sensor_msgs::LaserScan> scan07_sub_;
    
    std::string base_frame;
    ros::NodeHandle nh;
    ros::Publisher merged_scan_pub_;
    // tf2_ros::Buffer tf_buffer_;
    tf::TransformListener tf_listener_;
    // tf2_ros::TransformListener tf_listener_;
    tf::StampedTransform tf_scan_0_to_base_;
    tf::StampedTransform tf_scan_1_to_base_;
    tf::StampedTransform tf_scan_2_to_base_;
    tf::StampedTransform tf_scan_3_to_base_;
    tf::StampedTransform tf_scan_4_to_base_;
    tf::StampedTransform tf_scan_5_to_base_;
    tf::StampedTransform tf_scan_6_to_base_;
    tf::StampedTransform tf_scan_7_to_base_;
    bool tf_0_to_base_is_known_;
    bool tf_1_to_base_is_known_;
    bool tf_2_to_base_is_known_;
    bool tf_3_to_base_is_known_;
    bool tf_4_to_base_is_known_;
    bool tf_5_to_base_is_known_;
    bool tf_6_to_base_is_known_;
    bool tf_7_to_base_is_known_;

    ros::Publisher scan01_pub_;
    ros::Publisher scan02_pub_;
    ros::Publisher scan03_pub_;
    ros::Publisher scan04_pub_;
    ros::Publisher merged_point_pub_;
    ros::Publisher transformed_point_pub_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_scan_merger_2d_only");
    ros::NodeHandle nh;
    // LaserScanMerger merger(nh);
    try {
        ROS_INFO("laser_scan_merger_2d_only!");
        ros::AsyncSpinner spinner(2);
        spinner.start();
        LaserScanMerger pthread_mutexattr_getprioceiling(nh);
        ros::waitForShutdown();
        // ros::MultiThreadedSpinner spinner(2); // Necessary to allow concurrent callbacks.
        // spinner.spin();
        // ros::spin();
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
}
