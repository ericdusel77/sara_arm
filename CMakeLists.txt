cmake_minimum_required(VERSION 3.0.2)
project(sara_arm)

set(CATKIN_PACKAGE_DEPENDENCIES
    actionlib
    actionlib_msgs
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
    tf2_eigen
    ocs2_ros_interfaces
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(PCL REQUIRED)

add_service_files(
  FILES
  image_to_cloud_point.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs  # Or other packages containing msgs
  sensor_msgs
)
###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  # DEPENDS
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# add_library(pcp_sara src/pcp_sara.cpp)

# add_library(sara_arm src/sara_arm.cpp)

add_executable(pcp_sara src/pcp_sara.cpp)
target_link_libraries(pcp_sara ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(sara_arm src/sara_arm.cpp)
target_link_libraries(sara_arm ${catkin_LIBRARIES})

add_executable(ocs2_sara src/ocs2_sara.cpp)
target_link_libraries(ocs2_sara ${catkin_LIBRARIES})

