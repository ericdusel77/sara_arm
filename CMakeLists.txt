cmake_minimum_required(VERSION 3.0.2)
project(sara_arm)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    pcl_msgs
    pcl_ros
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
  	sensor_msgs
    tf2
    tf2_ros
    kinova_msgs
    moveit_visual_tools
)


add_service_files(
  FILES
  image_to_cloud_point.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs  # Or other packages containing msgs
  sensor_msgs
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


# add_library(pcp_sara src/pcp_sara.cpp)

# add_library(sara_arm src/sara_arm.cpp)

add_executable(pcp_sara src/pcp_sara.cpp)
target_link_libraries(pcp_sara ${catkin_LIBRARIES})

add_executable(sara_arm src/sara_arm.cpp)
target_link_libraries(sara_arm ${catkin_LIBRARIES})