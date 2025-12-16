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
        scan01_sub_.subscribe(nh, "/cygbot/point/2d/0", 1);
        scan02_sub_.subscribe(nh, "/cygbot/point/2d/1", 1);
        scan03_sub_.subscribe(nh, "/cygbot/point/2d/2", 1);
        scan04_sub_.subscribe(nh, "/cygbot/point/2d/3", 1);

        pointcloud01_sub_.subscribe(nh, "/cygbot/point/3d/0", 1);
        pointcloud02_sub_.subscribe(nh, "/cygbot/point/3d/1", 1);
        pointcloud03_sub_.subscribe(nh, "/cygbot/point/3d/2", 1);
        pointcloud04_sub_.subscribe(nh, "/cygbot/point/3d/3", 1);

        // ApproximateTimeSynchronizer 설정 (4개의 데이터를 동기화)
        // SyncPolicy 값	의미	장점	단점
        // 5	약 333ms 유지	최신 데이터 반영, 지연 최소화	동기화 실패 확률 높음
        // 10	약 666ms 유지	동기화 성공률 적절	약간의 지연 가능
        // 15	약 1초 유지	동기화 성공률 높음	지연 가능성 증가
        // 20	약 1.33초 유지	매우 높은 동기화율	지연이 커질 수 있음

        sync_.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), scan01_sub_, scan02_sub_, scan03_sub_, scan04_sub_, pointcloud01_sub_, pointcloud02_sub_, pointcloud03_sub_, pointcloud04_sub_));
        sync_->registerCallback(boost::bind(&LaserScanMerger::scanCallback, this, _1, _2, _3, _4, _5, _6, _7, _8));

        merged_scan_pub_ = nh.advertise<sensor_msgs::LaserScan>("/merged/scan", 1);
        merged_point_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/merged/pointClouds", 1);
        transformed_point_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/transformed_point", 1);
        tf_2_to_1_is_known_ = false;
        tf_3_to_1_is_known_ = false;
        tf_4_to_1_is_known_ = false;
    }

protected:

    void scanCallback(const sensor_msgs::PointCloud2::ConstPtr &scan01, 
                      const sensor_msgs::PointCloud2::ConstPtr &scan02, 
                      const sensor_msgs::PointCloud2::ConstPtr &scan03, 
                      const sensor_msgs::PointCloud2::ConstPtr &scan04,
                      const sensor_msgs::PointCloud2::ConstPtr &point01,
                      const sensor_msgs::PointCloud2::ConstPtr &point02,
                      const sensor_msgs::PointCloud2::ConstPtr &point03,
                      const sensor_msgs::PointCloud2::ConstPtr &point04) {

        // scan01을 기준으로 변환
        std::string target_frame = scan01->header.frame_id;
        sensor_msgs::PointCloud2 MergedScan = *scan01;  // 기준 scan01 데이터 복사
        sensor_msgs::PointCloud2 MergedCloud = *point01;  // 기준 scan01 데이터 복사

        // get TF between scan02, scan03, scan04 and scan01
        if (!tf_2_to_1_is_known_) lookupTransform(*scan02, *scan01, tf_scan_2_to_scan_1_, tf_2_to_1_is_known_);
        if (!tf_3_to_1_is_known_) lookupTransform(*scan03, *scan01, tf_scan_3_to_scan_1_, tf_3_to_1_is_known_);
        if (!tf_4_to_1_is_known_) lookupTransform(*scan04, *scan01, tf_scan_4_to_scan_1_, tf_4_to_1_is_known_);

        if (!tf_2_to_1_is_known_ && !tf_3_to_1_is_known_ && !tf_4_to_1_is_known_) return;

        // ROS_INFO("Transforms are Ready!");

        // scan02, scan03, scan04를 변환하여 병합
        sensor_msgs::PointCloud2 scan_2_1 = mergeTwoPointClouds(MergedScan, *scan02, tf_scan_2_to_scan_1_, true);
        sensor_msgs::PointCloud2 scan_3_2_1 = mergeTwoPointClouds(scan_2_1, *scan03, tf_scan_3_to_scan_1_, true);
        sensor_msgs::PointCloud2 scan_4_3_2_1 = mergeTwoPointClouds(scan_3_2_1, *scan04, tf_scan_4_to_scan_1_, true);

        sensor_msgs::LaserScan merged_scan;

        pointCloud2ToLaserScan(scan_4_3_2_1, merged_scan);

        merged_scan_pub_.publish(merged_scan); // 병합된 데이터 발행

        sensor_msgs::PointCloud2 cloud_2_1 = mergeTwoPointClouds(MergedCloud, *point02, tf_scan_2_to_scan_1_, true);
        sensor_msgs::PointCloud2 cloud_3_2_1 = mergeTwoPointClouds(cloud_2_1, *point03, tf_scan_3_to_scan_1_, true);
        sensor_msgs::PointCloud2 cloud_4_3_2_1 = mergeTwoPointClouds(cloud_3_2_1, *point04, tf_scan_4_to_scan_1_, true);

        sensor_msgs::PointCloud2 wholeMergedCloud = mergeTwoPointClouds(cloud_4_3_2_1, scan_4_3_2_1, tf_scan_4_to_scan_1_, false);

        merged_point_pub_.publish(wholeMergedCloud); // 병합된 데이터 발행
        std::vector<double> timeStamps = {scan01->header.stamp.toSec(), scan02->header.stamp.toSec(), scan03->header.stamp.toSec(), scan04->header.stamp.toSec(), point01->header.stamp.toSec(), point02->header.stamp.toSec(), point03->header.stamp.toSec(), point04->header.stamp.toSec()};
        double firstIn = *std::min_element(timeStamps.begin(), timeStamps.end());
        double current = ros::Time::now().toSec();
        ROS_INFO_STREAM("Time Diff : " << current - firstIn);
    }

    void lookupTransform(sensor_msgs::PointCloud2 source_scan, sensor_msgs::PointCloud2 target_scan, tf::StampedTransform& tf, bool& tf_is_known_) {

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

    sensor_msgs::PointCloud2 mergeTwoPointClouds(sensor_msgs::PointCloud2 &MergedCloud, 
                                                sensor_msgs::PointCloud2 sourceCloud, 
                                                tf::StampedTransform& tf,
                                                bool will_transform) {

        // PCL 포인트 클라우드 선언
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud1, pcl_cloud2, merged_cloud;

        if (will_transform) {
            sensor_msgs::PointCloud2 pointCloud_in_frame1;
            pcl_ros::transformPointCloud(MergedCloud.header.frame_id, tf, sourceCloud, pointCloud_in_frame1);
            pcl::fromROSMsg(MergedCloud, pcl_cloud1);
            pcl::fromROSMsg(pointCloud_in_frame1, pcl_cloud2);
        } else {
            pcl::fromROSMsg(MergedCloud, pcl_cloud1);
            pcl::fromROSMsg(sourceCloud, pcl_cloud2);
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
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> SyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    message_filters::Subscriber<sensor_msgs::PointCloud2> scan01_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> scan02_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> scan03_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> scan04_sub_;

    message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud01_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud02_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud03_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud04_sub_;
    
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
