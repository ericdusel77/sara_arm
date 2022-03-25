#ifndef SARA_ARM_H
#define SARA_ARM_H

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>

class SaraArm
{
public:
    SaraArm(ros::NodeHandle &nh);
    ~SaraArm();

private:
    ros::NodeHandle nh_;

    moveit::planning_interface::MoveGroupInterface* group_;

    // bool my_pick();
    void cloudInputCbk(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void evaluate_plan(moveit::planning_interface::MoveGroupInterface &group);
    ros::Subscriber cloud_input_sub_;
};



#endif // SARA_ARM_H
