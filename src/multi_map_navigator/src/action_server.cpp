#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/server/simple_action_server.h>
#include <multi_map_navigator/NavigateBetweenMapsAction.h>
#include "multi_map_navigator/nav_controller.hpp"

class NavigateActionServer {
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<multi_map_navigator::NavigateBetweenMapsAction> as_;
  std::string action_name_;
  NavController nav_ctrl_;

public:
  NavigateActionServer(const std::string& name,
                       const std::string& db_path,
                       const std::string& map_dir,
                       const std::string& world_dir)
    : as_(nh_, name, boost::bind(&NavigateActionServer::executeCB, this, _1), false),
      action_name_(name),
      nav_ctrl_(nh_, db_path, map_dir, world_dir)
  {
    as_.start();
    ROS_INFO("NavigateBetweenMaps Action Server started.");
  }

  void executeCB(const multi_map_navigator::NavigateBetweenMapsGoalConstPtr &goal) {
    multi_map_navigator::NavigateBetweenMapsResult result;

    // Optional feedback publishing
    multi_map_navigator::NavigateBetweenMapsFeedback feedback_msg;
    feedback_msg.feedback = "Received goal. Executing...";
    as_.publishFeedback(feedback_msg);

    // Main logic call
    bool success = nav_ctrl_.handleNavigation(goal->target_map, goal->target_pose);
    result.success = success;

    if (success)
      as_.setSucceeded(result);
    else
      as_.setAborted(result);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "navigate_between_maps_server");

  ros::NodeHandle nh("~");

  // Get paths from ROS params or hardcode for now
  std::string db_path = ros::package::getPath("multi_map_navigator") + "/my_wormholes.db";
  std::string map_dir = ros::package::getPath("multi_map_navigator") + "/maps";
  std::string world_dir = ros::package::getPath("multi_map_navigator") + "/world";

  NavigateActionServer server("navigate_between_maps", db_path, map_dir, world_dir);
  ros::spin();
  return 0;
}

