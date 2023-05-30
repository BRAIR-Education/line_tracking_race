cmake_minimum_required(VERSION 3.0.2)
project(line_tracking_race)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  stds_msgs
)

catkin_package()

# Uncomment if you want to use include directory
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)