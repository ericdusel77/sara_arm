#include <sara_arm.h>

SaraArm::SaraArm(ros::NodeHandle &nh):
    nh_(nh)
{
    ros::NodeHandle pn("~");
    group_ = new moveit::planning_interface::MoveGroupInterface("arm");
    cloud_input_sub_ = nh_.subscribe("click_input", 1, &SaraArm::cloudInputCbk, this);
}


SaraArm::~SaraArm()
{
    delete group_;
}


void SaraArm::evaluate_plan(moveit::planning_interface::MoveGroupInterface &group)
{
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool result = (group.plan(my_plan) == moveit_msgs::MoveItErrorCodes::SUCCESS);
    if (result)
    {
        group.execute(my_plan);
    }
    ros::WallDuration(1.0).sleep();
}


void SaraArm::cloudInputCbk(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    ros::AsyncSpinner spinner(1);
    spinner.start();

    geometry_msgs::Pose target_pose;
    target_pose = msg->pose;
    group_->setPoseTarget(target_pose);
    evaluate_plan(*group_);
}