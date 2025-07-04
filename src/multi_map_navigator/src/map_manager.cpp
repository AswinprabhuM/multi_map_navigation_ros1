#include "multi_map_navigator/map_manager.hpp"
#include <tf/tf.h>
#include <sstream>
#include <cstdlib>

MapManager::MapManager(ros::NodeHandle& nh, const std::string& map_directory)
    : nh_(nh), map_dir_(map_directory) {

    initial_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, true);
}

bool MapManager::switchMap(const std::string& map_name) {
    // Kill current map_server
    ROS_INFO_STREAM("Switching map to: " << map_name);
    int ret = system("rosnode kill /map_server");
    if (ret != 0) {
        ROS_WARN("Failed to kill existing map_server (maybe not running). Continuing...");
    }

    // Build map server launch command
    std::string map_path = map_dir_ + "/" + map_name + ".yaml";
    std::string cmd = "rosrun map_server map_server " + map_path + " &";

    ROS_INFO_STREAM("Launching new map: " << map_path);
    ret = system(cmd.c_str());
    if (ret != 0) {
        ROS_ERROR("Failed to launch new map_server.");
        return false;
    }

    ros::Duration(1.0).sleep();  // Give time for map_server to come up
    return true;
}

void MapManager::setInitialPose(double x, double y, double yaw) {
    geometry_msgs::PoseWithCovarianceStamped init_pose;
    init_pose.header.stamp = ros::Time::now();
    init_pose.header.frame_id = "map";

    init_pose.pose.pose.position.x = x;
    init_pose.pose.pose.position.y = y;

    tf::Quaternion q;
    q.setRPY(0, 0, yaw);
    init_pose.pose.pose.orientation.x = q.x();
    init_pose.pose.pose.orientation.y = q.y();
    init_pose.pose.pose.orientation.z = q.z();
    init_pose.pose.pose.orientation.w = q.w();

    init_pose.pose.covariance[0] = 0.25;
    init_pose.pose.covariance[7] = 0.25;
    init_pose.pose.covariance[35] = 0.06853891945200942;

    initial_pose_pub_.publish(init_pose);
    ROS_INFO("Published /initialpose.");
}

