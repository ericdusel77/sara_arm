#ifndef SARA_ARM_H
#define SARA_ARM_H

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <geometric_shapes/solid_primitive_dims.h>
#include <tf/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <kinova_msgs/SetFingersPositionAction.h>

#include <moveit/kinematic_constraints/utils.h>

#include <moveit/robot_model_loader/robot_model_loader.h>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

class SaraArm
{
public:
    SaraArm(ros::NodeHandle &nh);
    ~SaraArm();
    std::vector<double> home;

private:
    ros::NodeHandle nh_;

    moveit::planning_interface::MoveGroupInterface* group_;
    
    ros::Subscriber cloud_input_sub_;

    void cloudInputCbk(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void evaluate_plan(moveit::planning_interface::MoveGroupInterface &group);
    bool result_;
    
    ros::Publisher marker_pub;
    ros::Publisher array_pub;
    ros::Subscriber sub_joint_;
    // update current state and pose
    std::string pause_;
    boost::mutex mutex_state_;
    boost::mutex mutex_pose_;
    sensor_msgs::JointState current_state_;
    geometry_msgs::PoseStamped current_pose_;

    void get_current_state(const sensor_msgs::JointStateConstPtr &msg);
    std::string robot_type_;
    bool robot_connected_;
    actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction>* finger_client_;
    bool gripper_action(double gripper_rad);
};



#endif // SARA_ARM_H
