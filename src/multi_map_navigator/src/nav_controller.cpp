#include "multi_map_navigator/nav_controller.hpp"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <cstdlib>
#include <sstream>
#include <tf/tf.h>
#include <ros/package.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

NavController::NavController(ros::NodeHandle& nh,
                             const std::string& db_path,
                             const std::string& map_dir,
                             const std::string& world_dir)
  : nh_(nh), db_(db_path), map_mgr_(nh_, map_dir), current_map_("room_a") {}

bool NavController::handleNavigation(const std::string& target_map, const geometry_msgs::PoseStamped& target_pose) {
    if (target_map == current_map_) {
        ROS_INFO("Same map — sending goal directly.");
        return sendGoal(target_pose);
    }

    // Step 1: Get wormhole
    Wormhole wh;
    if (!db_.getWormhole(current_map_, target_map, wh)) {
        ROS_ERROR("No wormhole found between %s and %s", current_map_.c_str(), target_map.c_str());
        return false;
    }

    // Step 2: Navigate to wormhole
    geometry_msgs::PoseStamped wh_pose;
    wh_pose.header.frame_id = "map";
    wh_pose.header.stamp = ros::Time::now();
    wh_pose.pose.position.x = wh.source_x;
    wh_pose.pose.position.y = wh.source_y;
    wh_pose.pose.orientation = tf::createQuaternionMsgFromYaw(wh.source_yaw);

    ROS_INFO("Navigating to wormhole at (%.2f, %.2f)...", wh.source_x, wh.source_y);
    if (!sendGoal(wh_pose)) {
        ROS_WARN("Failed to reach wormhole.");
        return false;
    }

    // Step 3: Kill current Gazebo and navigation stack
    ROS_INFO("Killing current simulation and nav stack...");
    system("rosnode kill /gazebo /gazebo_gui /map_server /amcl /move_base");
    system("pkill -9 gzserver");
    system("pkill -9 gzclient");
    ros::Duration(2.0).sleep();

    // Step 4: Ensure gzserver has stopped
    bool gz_killed = false;
    for (int i = 0; i < 10; ++i) {
        if (system("pgrep gzserver > /dev/null") != 0) {
            gz_killed = true;
            break;
        }
        ros::Duration(0.5).sleep();
    }
    if (!gz_killed) {
        ROS_ERROR("gzserver still running — aborting map switch.");
        return false;
    }

    // Step 5: Launch new world and nav stack
    std::string pkg_path = ros::package::getPath("multi_map_navigator");
    std::string map_file = pkg_path + "/maps/" + target_map + ".yaml";
    std::string world_file = pkg_path + "/world/" + target_map + ".world";

    ROS_INFO("Launching new Gazebo world...");
    std::stringstream world_cmd;
    world_cmd << "roslaunch multi_map_navigator my_world_loader.launch "
              << "world_file:=" << world_file << " "
              << "x_pos:=" << wh.target_x << " "
              << "y_pos:=" << wh.target_y << " "
              << "yaw:="   << wh.target_yaw << " "
              << "__name:=room_world_loader &";
    system(world_cmd.str().c_str());
    ros::Duration(5.0).sleep();

    ROS_INFO("Launching new navigation stack...");
    std::stringstream nav_cmd;
    nav_cmd << "roslaunch multi_map_navigator my_navigation.launch "
            << "map_file:=" << map_file << " open_rviz:=false __name:=room_nav_stack &";
    system(nav_cmd.str().c_str());
    ros::Duration(5.0).sleep();

    // Step 6: Set initial pose (optional if spawn is correct, keep for robustness)
    ROS_INFO("Setting initial pose in new map...");
    map_mgr_.setInitialPose(wh.target_x, wh.target_y, wh.target_yaw);
    ros::Duration(1.0).sleep();

    // Step 7: Wait for move_base
    ROS_INFO("Waiting for move_base to become available...");
    MoveBaseClient ac("move_base", true);
    if (!ac.waitForServer(ros::Duration(20.0))) {
        ROS_ERROR("move_base not available in new map.");
        return false;
    }

    // Step 8: Send final goal
    ROS_INFO("Sending final goal in map: %s", target_map.c_str());
    bool result = sendGoal(target_pose);
    if (result) current_map_ = target_map;

    return result;
}

bool NavController::sendGoal(const geometry_msgs::PoseStamped& goal_pose) {
    MoveBaseClient ac("move_base", true);

    ROS_INFO("Waiting for move_base...");
    if (!ac.waitForServer(ros::Duration(20.0))) {
        ROS_ERROR("Timeout: move_base action server not available.");
        return false;
    }

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose = goal_pose;

    ROS_INFO("Sending goal to move_base...");
    ac.sendGoal(goal);

    bool finished = ac.waitForResult(ros::Duration(60.0));
    if (!finished) {
        ROS_WARN("Navigation timeout.");
        return false;
    }

    auto state = ac.getState();
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Reached goal.");
        return true;
    } else {
        ROS_WARN("Failed to reach goal: %s", state.toString().c_str());
        return false;
    }
}

