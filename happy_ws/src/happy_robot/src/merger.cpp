#include <thread>
#include <chrono>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <mutex>

#include <ros/console.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_ros/transforms.h>

class Merger {
    
    public:
        explicit Merger(ros::NodeHandle& n) : nh_(n) {
            // Topic names.
            const std::string scan1Topic = "/sigbot/scan01"
            const std::string scan2Topic = "/sigbot/scan02"
            const std::string scan3Topic = "/sigbot/scan03"
            const std::string scan4Topic = "/sigbot/scan04"
            const std::string point1Topic = "/sigbot/point01"
            const std::string point2Topic = "/sigbot/point02"
            const std::string point3Topic = "/sigbot/point03"
            const std::string point4Topic = "/sigbot/point04"
            const std::string mergedScanTopic = "/combined/scan";
            tf_2_to_1_is_known_ = false;
            tf_3_to_1_is_known_ = false;
            tf_4_to_1_is_known_ = false;
            reference_scan1_is_set_ = false;
            reference_scan2_is_set_ = false;
            reference_scan3_is_set_ = false;
            reference_scan4_is_set_ = false;
            latest_published_scan_is_set_ = false;
            scan_1_sub_ = nh_.subscribe(scan1Topic, 1000, &Merger::scan1Callback, this);
            scan_2_sub_ = nh_.subscribe(scan2Topic, 1000, &Merger::scan2Callback, this);
            scan_3_sub_ = nh_.subscribe(scan3Topic, 1000, &Merger::scan3Callback, this);
            scan_4_sub_ = nh_.subscribe(scan4Topic, 1000, &Merger::scan4Callback, this);
            combined_laserscan_pub_ = nh_.advertise<sensor_msgs::LaserScan>(mergedScanTopic, 1);
        }
        ~Merger() {}

    protected:

        void scan1Callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
            // On first run, only set reference scan.
            if ( !reference_scan1_is_set_ && !reference_scan2_is_set_ && !reference_scan3_is_set_ && !reference_scan4_is_set_) {
                reference_scan1_ = *msg;
                reference_scan1_is_set_ = true;
                // ROS_INFO("[1] Set scan as reference scan for sensor 1.");
                return;
            } else {
                // ROS_INFO("[1] Waiting scan for other sensor.");
                return;
            }
        }

        void scan2Callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
            
            // On first run, only set reference scan.
            if ( reference_scan1_is_set_  && !reference_scan2_is_set_ && !reference_scan3_is_set_ && !reference_scan4_is_set_) {
                reference_scan2_ = *msg;
                reference_scan2_is_set_ = true;
                if (tf_2_to_1_is_known_) lookupTransform(reference_scan2_, reference_scan1_ ,tf_scan_2_to_scan_1_, tf_2_to_1_is_known_);
                // ROS_INFO("[2] Set scan as reference scan for sensor 2.");
            } else {
                // ROS_INFO("[2] Waiting scan for other sensor.");
                return;
            }
        }

        void scan3Callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
            
            // On first run, only set reference scan.
            if ( reference_scan1_is_set_  && reference_scan2_is_set_ && !reference_scan3_is_set_ && !reference_scan4_is_set_) {
                reference_scan3_ = *msg;
                reference_scan3_is_set_ = true;
                if (tf_3_to_1_is_known_) lookupTransform(reference_scan3_, reference_scan1_ ,tf_scan_3_to_scan_1_, tf_3_to_1_is_known_);
                // ROS_INFO("[3] Set scan as reference scan for sensor 3.");
            } else {
                // ROS_INFO("[3] Waiting scan for other sensor.");
                return;
            }
        }

        void scan4Callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
            
            // On first run, only set reference scan.
            if ( reference_scan1_is_set_  && reference_scan2_is_set_ && reference_scan3_is_set_ && !reference_scan4_is_set_) {
                reference_scan4_ = *msg;
                reference_scan4_is_set_ = true;
                if(tf_4_to_1_is_known_) lookupTransform(reference_scan4_, reference_scan1_ ,tf_scan_4_to_scan_1, tf_4_to_1_is_known_);
                // ROS_INFO("[4] Set scan as reference scan for sensor 4.");
            } else {
                // ROS_INFO("[4] Waiting scan for other sensor.");
                return;
            }
            sensor_msgs::PointCloud2 MergedCloud = mergePointCloudsToOne();
            // publish result.
            combined_pointcloud_pub_.publish(MergedCloud);
            ROS_INFO("[Succeed] Pub combined PointCloud");
        }

        void lookupTransform(sensor_msgs::LaserScan source_scan, sensor_msgs::LaserScan target_scan, tf::StampedTransform& tf, bool& tf_is_known_) {

            // src frame -> tar frame
            if (!tf_is_known_) {
                constexpr size_t kTFTimeout_ms = 1000;
                constexpr size_t kMsToNs = 1000000;

                ROS_INFO_STREAM("Waiting for transform between src, tar frames : " << target_scan.header.frame_id << ", " << source_scan.header.frame_id);
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

        sensor_msgs::LaserScan mergeLaserScanToOne() {

            sensor_msgs::LaserScan merged_scan = reference_scan1_;

        }

        sensor_msgs::PointCloud2 mergePointCloudsToOne() {
            sensor_msgs::PointCloud2 MergedCloud = reference_scan1_;

            sensor_msgs::PointCloud2 scan2_in_frame1;
            pcl_ros::transformPointCloud(reference_scan1_.header.frame_id, tf_scan_2_to_scan_1_, reference_scan2_, scan2_in_frame1);
            MergedCloud.width += scan2_in_frame1.width;
            uint64_t OriginalSize1 = MergedCloud.data.size();
            MergedCloud.data.resize(MergedCloud.data.size() + scan2_in_frame1.data.size());
            std::copy(scan2_in_frame1.data.begin(),scan2_in_frame1.data.end(), MergedCloud.data.begin() + OriginalSize1);

            sensor_msgs::PointCloud2 scan3_in_frame1;
            pcl_ros::transformPointCloud(reference_scan1_.header.frame_id, tf_scan_3_to_scan_1_, reference_scan3_, scan3_in_frame1);
            MergedCloud.width += scan3_in_frame1.width;
            uint64_t OriginalSize2 = MergedCloud.data.size();
            MergedCloud.data.resize(MergedCloud.data.size() + scan3_in_frame1.data.size());
            std::copy(scan3_in_frame1.data.begin(),scan3_in_frame1.data.end(), MergedCloud.data.begin() + OriginalSize2);

            sensor_msgs::PointCloud2 scan4_in_frame1;
            pcl_ros::transformPointCloud(reference_scan1_.header.frame_id, tf_scan_4_to_scan_1_, reference_scan4_, scan4_in_frame1);
            MergedCloud.width += scan4_in_frame1.width;
            uint64_t OriginalSize3 = MergedCloud.data.size();
            MergedCloud.data.resize(MergedCloud.data.size() + scan4_in_frame1.data.size());
            std::copy(scan4_in_frame1.data.begin(),scan4_in_frame1.data.end(), MergedCloud.data.begin() + OriginalSize3);
            
            reference_scan1_is_set_ = false;
            reference_scan2_is_set_ = false;
            reference_scan3_is_set_ = false;
            reference_scan4_is_set_ = false;

            return MergedCloud;
        }
    

        void point1Callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
            // On first run, only set reference scan.
            if ( !reference_scan1_is_set_ && !reference_scan2_is_set_ && !reference_scan3_is_set_ && !reference_scan4_is_set_) {
                reference_scan1_ = *msg;
                reference_scan1_is_set_ = true;
                // ROS_INFO("[1] Set scan as reference scan for sensor 1.");
                return;
            } else {
                // ROS_INFO("[1] Waiting scan for other sensor.");
                return;
            }
        }

        void point2Callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
            
            // On first run, only set reference scan.
            if ( reference_scan1_is_set_  && !reference_scan2_is_set_ && !reference_scan3_is_set_ && !reference_scan4_is_set_) {
                reference_scan2_ = *msg;
                reference_scan2_is_set_ = true;
                getTFScanTo1(reference_scan2_, tf_scan_2_to_scan_1_, tf_2_to_1_is_known_);
                // ROS_INFO("[2] Set scan as reference scan for sensor 2.");
            } else {
                // ROS_INFO("[2] Waiting scan for other sensor.");
                return;
            }
        }

        void point3Callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
            
            // On first run, only set reference scan.
            if ( reference_scan1_is_set_  && reference_scan2_is_set_ && !reference_scan3_is_set_ && !reference_scan4_is_set_) {
                reference_scan3_ = *msg;
                reference_scan3_is_set_ = true;
                getTFScanTo1(reference_scan3_, tf_scan_3_to_scan_1_, tf_3_to_1_is_known_);
                // ROS_INFO("[3] Set scan as reference scan for sensor 3.");
            } else {
                // ROS_INFO("[3] Waiting scan for other sensor.");
                return;
            }
        }

        void point4Callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
            
            // On first run, only set reference scan.
            if ( reference_scan1_is_set_  && reference_scan2_is_set_ && reference_scan3_is_set_ && !reference_scan4_is_set_) {
                reference_scan4_ = *msg;
                reference_scan4_is_set_ = true;
                getTFScanTo1(reference_scan4_, tf_scan_4_to_scan_1_, tf_4_to_1_is_known_);
                // ROS_INFO("[4] Set scan as reference scan for sensor 4.");
            } else {
                // ROS_INFO("[4] Waiting scan for other sensor.");
                return;
            }
            sensor_msgs::PointCloud2 MergedCloud = mergePointCloudsToOne();
            // publish result.
            combined_pointcloud_pub_.publish(MergedCloud);
            ROS_INFO("[Succeed] Pub combined PointCloud");
        }

    private:
        // ROS
        ros::NodeHandle& nh_;
        ros::Subscriber scan_1_sub_;
        ros::Subscriber scan_2_sub_;
        ros::Subscriber scan_3_sub_;
        ros::Subscriber scan_4_sub_;
        ros::Publisher combined_laserscan_pub_;
        // State
        tf::StampedTransform tf_scan_2_to_scan_1_;
        tf::StampedTransform tf_scan_3_to_scan_1_;
        tf::StampedTransform tf_scan_4_to_scan_1_;
        tf::TransformListener tf_listener_;
        bool tf_2_to_1_is_known_;
        bool tf_3_to_1_is_known_;
        bool tf_4_to_1_is_known_;
        sensor_msgs::LaserScan reference_scan1_;
        bool reference_scan1_is_set_;
        sensor_msgs::LaserScan reference_scan2_;
        bool reference_scan2_is_set_;
        sensor_msgs::LaserScan reference_scan3_;
        bool reference_scan3_is_set_;
        sensor_msgs::LaserScan reference_scan4_;
        bool reference_scan4_is_set_;
        sensor_msgs::LaserScan latest_published_scan_;
        bool latest_published_scan_is_set_;
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "multi_pointcloud_merger");
    ros::NodeHandle nh;
	Merger pthread_mutexattr_getprioceiling(nh);

	try {
//     ros::MultiThreadedSpinner spinner(2); // Necessary to allow concurrent callbacks.
//     spinner.spin();
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
}