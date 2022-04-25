#ifndef SARA_ARM_H
#define SARA_ARM_H

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <geometric_shapes/solid_primitive_dims.h>
#include <tf/transform_broadcaster.h>
#include <kinova_msgs/SetFingersPositionAction.h>

class SaraArm
{
public:
    SaraArm(ros::NodeHandle &nh);
    ~SaraArm();

private:
    ros::NodeHandle nh_;

    moveit::planning_interface::MoveGroupInterface* group_;
    
    ros::Subscriber cloud_input_sub_;

    void cloudInputCbk(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void evaluate_plan(moveit::planning_interface::MoveGroupInterface &group);
    bool result_;
    
    ros::Publisher marker_pub;

    std::string robot_type_;
    bool robot_connected_;
    actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction>* finger_client_;
    bool gripper_action(double gripper_rad);
};



#endif // SARA_ARM_H
