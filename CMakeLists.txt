cmake_minimum_required(VERSION 2.8.3)
project(slam_project_buggy)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  turtlesim
)


################################################
## Declare ROS messages, services and actions ##
################################################


## Generate messages in the 'msg' folder
add_message_files(
   FILES
   LaserMsgDistance.msg
   AllSidesDistance.msg
)

find_package(
  catkin REQUIRED 
  genmsg 
  actionlib_msgs 
  actionlib
)


add_action_files(
  DIRECTORY action FILES setDirectionRobot.action
)


## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   setDirectionRobot.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
)


#add_library(global_planner_lib src/createPath.cpp)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(makeMap src/makeMap.cpp)
target_link_libraries(makeMap ${catkin_LIBRARIES})
add_dependencies(makeMap slam_project_buggy_generate_messages_cpp)
