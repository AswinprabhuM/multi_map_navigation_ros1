#ifndef MULTI_MAP_NAVIGATOR_MAP_MANAGER_HPP
#define MULTI_MAP_NAVIGATOR_MAP_MANAGER_HPP

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class MapManager {
public:
    MapManager(ros::NodeHandle& nh, const std::string& map_directory);

    bool switchMap(const std::string& map_name);
    void setInitialPose(double x, double y, double yaw);

private:
    ros::NodeHandle nh_;
    ros::Publisher initial_pose_pub_;
    std::string map_dir_;
};

#endif // MULTI_MAP_NAVIGATOR_MAP_MANAGER_HPP

