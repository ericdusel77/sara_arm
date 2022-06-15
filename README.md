# SARA-ARM: Shared Autonomous Reach Augmentation through Assistive Robotic Manipulation

This code is meant to be used for assistive robotic applications in order to automate routine tasks using motion planing algorithms, with computer vision for determing task and goal from user input on image.

This code has been tested with ROS Noetic on Ubuntu 20.04 on a Kinova Jaco Gen 1.  The pointcloud and display image were produced by a RealSense D455.

## Prerequisites

[kinova-ros](https://github.com/Kinovarobotics/kinova-ros/tree/noetic-devel)

## Running the code

- With arm connected, launch arm
`roslaunch kinova_bringup kinova_robot.launch`

- Launch MoveIt config
`roslaunch j2n6s300_moveit_config j2n6s300_demo.launch`

- Launch RealSense node and GUI
`roslaunch sara_arm image_selector.launch`

- Run SARA ARM 
`rosrun sara_arm sara_arm joint_states:=j2n6s300/joint_states`

