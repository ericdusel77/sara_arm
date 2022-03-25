cmake_minimum_required(VERSION 3.0.2)
project(sara_arm)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    point_cloud_proc
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    # kinova_driver
)


add_service_files(
  FILES
  image_to_cloud_point.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs  # Or other packages containing msgs
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(pcp_sara src/pcp_sara.cpp)

# add_library(sara_arm src/sara_arm.cpp)

add_executable(ground_plane_pub src/ground_plane_pub.cpp)
target_link_libraries(ground_plane_pub ${catkin_LIBRARIES})

add_executable(cloud_cycle_points src/cloud_cycle_points.cpp)
target_link_libraries(cloud_cycle_points ${catkin_LIBRARIES})

add_executable(pcp_service src/pcp_service.cpp)
target_link_libraries(pcp_service ${catkin_LIBRARIES} pcp_sara)

add_executable(sara_arm_sub src/sara_arm_sub.cpp)
target_link_libraries(sara_arm_sub ${catkin_LIBRARIES})