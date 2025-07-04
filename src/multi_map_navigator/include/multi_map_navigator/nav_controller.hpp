#ifndef MULTI_MAP_NAVIGATOR_NAV_CONTROLLER_HPP
#define MULTI_MAP_NAVIGATOR_NAV_CONTROLLER_HPP

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include "multi_map_navigator/db_interface.hpp"
#include "multi_map_navigator/map_manager.hpp"

class NavController {
public:
    NavController(ros::NodeHandle& nh, const std::string& db_path, const std::string& map_dir, const std::string& world_dir);

    bool handleNavigation(const std::string& target_map, const geometry_msgs::PoseStamped& target_pose);

private:
    bool sendGoal(const geometry_msgs::PoseStamped& goal);
    std::string current_map_;
    ros::NodeHandle nh_;
    DBInterface db_;
    MapManager map_mgr_;
};

#endif // MULTI_MAP_NAVIGATOR_NAV_CONTROLLER_HPP

