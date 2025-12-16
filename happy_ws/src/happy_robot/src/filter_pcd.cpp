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

class FilterPCD
{
public:
    FilterPCD(ros::NodeHandle& nh) : nh_(nh) {
        scan_sub_ = nh.subscribe("/cygbot/scan/2d/1", 10, &FilterPCD::scanCallback, this);
        scan_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/filtered/scan", 10);
    }

protected:

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan) {

        double init = ros::Time::now().toSec();

        sensor_msgs::PointCloud2 scan_point = laserScanToPointCloud2(*scan);

        scan_pub_.publish(scan_point);
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
            if (distance < 7.0)
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

private:
    ros::NodeHandle& nh_;
    ros::Subscriber scan_sub_;
    ros::Publisher scan_pub_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scan_filter");
    ros::NodeHandle nh;
    // FilterPCD merger(nh);
    try {
        ros::AsyncSpinner spinner(2);
        spinner.start();
        FilterPCD pthread_mutexattr_getprioceiling(nh);
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
