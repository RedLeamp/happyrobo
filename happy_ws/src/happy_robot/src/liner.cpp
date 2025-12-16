#include <ros/ros.h>
#include <pcl-1.10/pcl/point_cloud.h>
#include <geometry_msgs/Pose2D.h>


class Liner {

    public:
        explicit Liner(ros::NodeHandle& n) : nh_(n) {
            // Topic names.
            const std::string frontLeft_scan_topic = "/sigbot/scan01";
            const std::string frontRight_scan_topic = "/sigbot/scan02";
            const std::string frontLeft_point_topic = "/sigbot/point01";
            const std::string frontRight_point_topic = "/sigbot/point02";
            const std::string finalGoal_2D_pose = "/final/goal";
            tf_2_to_1_is_known_ = false;
            tf_3_to_1_is_known_ = false;
            tf_4_to_1_is_known_ = false;
            reference_scan1_is_set_ = false;
            reference_scan2_is_set_ = false;
            reference_scan3_is_set_ = false;
            reference_scan4_is_set_ = false;
            latest_published_scan_is_set_ = false;
            frontLeft_Scan_Sub_ = nh_.subscribe(frontLeft_scan_topic, 1000, &Liner::frontLeft_scan_callback, this);
            frontRight_Scan_Sub = nh_.subscribe(frontRight_scan_topic, 1000, &Liner::frontLeft_scan_callback, this);
            frontLeft_Point_Sub_ = nh_.subscribe(frontLeft_point_topic, 1000, &Liner::frontLeft_scan_callback, this);
            frontRight_Point_Sub = nh_.subscribe(frontRight_point_topic, 1000, &Liner::frontLeft_scan_callback, this);
            combined_pointcloud_pub_ = nh_.advertise<geometry_msgs::Pose2D>(finalGoal_2D_pose, 1);
        }
}

