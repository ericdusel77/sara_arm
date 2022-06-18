# SARA-ARM: Shared Autonomous Reach Augmentation through Assistive Robotic Manipulation

This code is meant to be used for assistive robotic applications in order to automate routine tasks using motion planing algorithms, with computer vision for determing task and goal from user input on image.

This code has been tested with ROS Noetic on Ubuntu 20.04 on a Kinova Jaco Gen 1.  The pointcloud and display image were produced by a RealSense D455.

## Prerequisites

[kinova-ros](https://github.com/Kinovarobotics/kinova-ros/tree/noetic-devel): ROS interface for the Kinova Robotics JACO, JACO2 and MICO robotic manipulator arms. The stack is developed above the Kinova C++ API functions, which communicate with the DSP inside robot base.

- `git clone https://github.com/Kinovarobotics/kinova-ros.git` 

- `git checkout noetic-devel` 

- `sudo cp kinova_driver/udev/10-kinova-arm.rules /etc/udev/rules.d/`: Copy udev rules to use USB connection to arm


[MoveIt!](https://moveit.ros.org/install/): Needed for building kinova-ros. Also use motion planning interface for SARA.

- `sudo apt install ros-noetic-moveit`

- [MoveIt! Visual Tools](https://github.com/Kinovarobotics/kinova-ros/tree/noetic-devel) `sudo apt-get install ros-noetic-moveit-visual-tools`

[Trac-IK](https://bitbucket.org/traclabs/trac_ik/src/master/): Inverse Kinematics solver used by motion planners.

- `sudo apt-get install ros-noetic-trac-ik`

`pip install screeninfo`: Used for matching GUI to monitor size

For camera, only [RealSense](https://github.com/IntelRealSense/realsense-ros) has been tested. Azure Kinect compatibility in development

- `sudo apt-get install ros-noetic-realsense2-camera`

## Running the code

- With arm connected, launch arm
`roslaunch kinova_bringup kinova_robot.launch`

- Launch MoveIt config
`roslaunch j2n6s300_moveit_config j2n6s300_demo.launch`

- Launch RealSense node and GUI
`roslaunch sara_arm image_selector.launch`

- Run SARA ARM 
`rosrun sara_arm sara_arm joint_states:=j2n6s300/joint_states`



