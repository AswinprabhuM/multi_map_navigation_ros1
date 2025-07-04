#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/client/simple_action_client.h>
#include <multi_map_navigator/NavigateBetweenMapsAction.h>
#include <cstdlib>

void feedbackCb(const multi_map_navigator::NavigateBetweenMapsFeedbackConstPtr& feedback) {
  ROS_INFO("Server Feedback: %s", feedback->feedback.c_str());
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "navigate_between_maps_client");
  ros::NodeHandle nh;

  // Get package path
  std::string pkg_path = ros::package::getPath("multi_map_navigator");
  std::string map_path = pkg_path + "/maps/room_a.yaml";
  std::string world_path = pkg_path + "/world/room_a.world";

  // STEP 1: Launch world (Gazebo + TurtleBot3)
  ROS_INFO("Launching my_world_loader.launch...");
  std::string world_launch_cmd =
    "roslaunch multi_map_navigator my_world_loader.launch "
    "world_file:=" + world_path + " &";
  system(world_launch_cmd.c_str());

  ros::Duration(10.0).sleep();  // Let Gazebo initialize

  // STEP 2: Launch navigation stack
  ROS_INFO("Launching my_navigation.launch...");
  std::string nav_launch_cmd =
    "roslaunch multi_map_navigator my_navigation.launch "
    "map_file:=" + map_path + " &";
  system(nav_launch_cmd.c_str());

  ros::Duration(8.0).sleep();  // Let AMCL + move_base initialize

  // STEP 3: Connect to action server
  actionlib::SimpleActionClient<multi_map_navigator::NavigateBetweenMapsAction> ac("navigate_between_maps", true);

  ROS_INFO("Waiting for action server to start...");
  ac.waitForServer();
  ROS_INFO("Action server started.");

  // STEP 4: Build goal
  multi_map_navigator::NavigateBetweenMapsGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = -3.300855;
  goal.target_pose.pose.position.y = 2.168595;
  goal.target_pose.pose.orientation.w = 1.0;
  goal.target_map = "room_b";

  // STEP 5: Send goal
  ROS_INFO("Sending goal to map: %s", goal.target_map.c_str());
  ac.sendGoal(goal,
              actionlib::SimpleActionClient<multi_map_navigator::NavigateBetweenMapsAction>::SimpleDoneCallback(),
              actionlib::SimpleActionClient<multi_map_navigator::NavigateBetweenMapsAction>::SimpleActiveCallback(),
              &feedbackCb);

  bool finished = ac.waitForResult(ros::Duration(90.0));

  if (finished) {
    auto state = ac.getState();
    ROS_INFO("Result: %s", state.toString().c_str());
    ROS_INFO("Success: %d", ac.getResult()->success);
  } else {
    ROS_WARN("Timeout: did not complete navigation.");
  }

  return 0;
}

