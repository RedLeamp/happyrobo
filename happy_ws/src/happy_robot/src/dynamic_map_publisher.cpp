#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Bool.h>

class MapModifier {
public:
    MapModifier() : clear_obstacles_(false), latest_map_(nullptr) {
        // 구독할 토픽 설정
        map_sub_ = nh_.subscribe("/map", 1, &MapModifier::mapCallback, this);
        clear_sub_ = nh_.subscribe("/clear_map", 1, &MapModifier::clearCallback, this);

        // 퍼블리시할 토픽 설정
        map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/modified_map", 1);
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        // 가장 최근에 구독한 맵 데이터를 저장
        latest_map_ = boost::make_shared<nav_msgs::OccupancyGrid>(*msg);

        nav_msgs::OccupancyGrid modified_map = *msg;

        if (clear_obstacles_) {
            ROS_INFO("Clear obstacles mode enabled. Publishing modified map...");
            publishModifiedMap();
        } else {
            ROS_INFO("Clear obstacles mode disabled. Original map will not be modified.");
            map_pub_.publish(modified_map);
        }

        // 수정된 맵 퍼블리시
        // map_pub_.publish(modified_map);
        ROS_INFO("Map data received, modified, and published.");
    }

    void clearCallback(const std_msgs::Bool::ConstPtr& msg) {

        ROS_INFO("GETTING CLEAR MAP MSG");
        // /clear_map 토픽의 값에 따라 장애물 제거 여부 설정
        clear_obstacles_ = msg->data;

        if (clear_obstacles_) {
            ROS_INFO("Clear obstacles mode enabled. Publishing modified map...");
            publishModifiedMap();
        } else {
            map_pub_.publish(latest_map_);
            ROS_INFO("Clear obstacles mode disabled. Original map will not be modified.");
        }
    }

    void deepCopyOccupancyGrid(const nav_msgs::OccupancyGrid& source, nav_msgs::OccupancyGrid& destination) {
        // 헤더 복사
        destination.header = source.header;

        // 메타데이터 복사
        destination.info.map_load_time = source.info.map_load_time;
        destination.info.resolution = source.info.resolution;
        destination.info.width = source.info.width;
        destination.info.height = source.info.height;
        destination.info.origin = source.info.origin;

        // 데이터 복사 (std::vector의 깊은 복사)
        destination.data = source.data;  // std::vector는 복사 시 깊은 복사가 수행됨
    }

    void publishModifiedMap() {
        if (!latest_map_) {
            ROS_WARN("No map data received yet. Cannot publish modified map.");
            return;
        }

        nav_msgs::OccupancyGrid modified_map;

        deepCopyOccupancyGrid(*latest_map_, modified_map);

        // 맵 데이터 수정 (장애물 제거)
        for (int i = 0; i < modified_map.data.size(); ++i) {
            // if (modified_map.data[i] > 0) {  // 장애물로 표시된 셀
                // modified_map.data[i] = 0;   // FREE로 변경
            // }
            if (modified_map.data[i] != 0) {  // 장애물로 표시된 셀
                modified_map.data[i] = 0;   // FREE로 변경
            }
        }

        // 수정된 맵 퍼블리시
        map_pub_.publish(modified_map);
        ROS_INFO("Modified map published.");
    }

    void publishLatestMap() {
        if (!latest_map_) {
            ROS_WARN("No map data received yet. Cannot publish modified map.");
            return;
        }
        nav_msgs::OccupancyGrid modified_map;
        deepCopyOccupancyGrid(*latest_map_, modified_map);
        // 수정된 맵 퍼블리시
        map_pub_.publish(modified_map);
        ROS_INFO("Lastest map published.");
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber map_sub_;
    ros::Subscriber clear_sub_;
    ros::Publisher map_pub_;
    nav_msgs::OccupancyGrid::Ptr latest_map_; // 가장 최근에 받은 맵 데이터 포인터
    bool clear_obstacles_;                    // 장애물 제거 여부
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_modifier");

    ROS_INFO("Starting Map Modifier Node...");
    MapModifier modifier;

    ros::Rate loop_rate(0.5); // 10Hz
    
    while (ros::ok()) {
        ros::spinOnce();
        modifier.publishLatestMap();
        loop_rate.sleep();
    }
    return 0;
}
