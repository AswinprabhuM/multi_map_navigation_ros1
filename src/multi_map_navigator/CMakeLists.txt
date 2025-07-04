cmake_minimum_required(VERSION 3.0.2)
project(multi_map_navigator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  map_server
  move_base_msgs
  actionlib
  actionlib_msgs
  message_generation
)

find_package(SQLite3 REQUIRED)

find_package(roslib REQUIRED)

###################################
## Declare messages and actions ##
###################################

add_action_files(
  DIRECTORY action
  FILES NavigateBetweenMaps.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

###################################
## Declare catkin_package        ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES nav_controller db_interface map_manager
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs move_base_msgs map_server actionlib actionlib_msgs message_runtime
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${roslib_INCLUDE_DIRS}
)

add_library(map_manager
  src/map_manager.cpp
)

target_link_libraries(map_manager
  ${catkin_LIBRARIES}
)

add_library(db_interface
  src/db_interface.cpp
)

target_link_libraries(db_interface
  ${catkin_LIBRARIES}
  sqlite3
)

add_library(nav_controller
  src/nav_controller.cpp
)

target_link_libraries(nav_controller
  ${catkin_LIBRARIES}
  map_manager
  db_interface
)

add_executable(navigate_between_maps_client src/action_client.cpp)
target_link_libraries(navigate_between_maps_client ${catkin_LIBRARIES} ${roslib_LIBRARIES})
add_dependencies(navigate_between_maps_client ${PROJECT_NAME}_generate_messages_cpp)

add_executable(navigate_between_maps_server src/action_server.cpp)
target_link_libraries(navigate_between_maps_server
  ${catkin_LIBRARIES}
  map_manager
  db_interface
  nav_controller
  ${roslib_LIBRARIES} 
)
add_dependencies(navigate_between_maps_server ${PROJECT_NAME}_generate_messages_cpp)

