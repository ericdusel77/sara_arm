#include <sara_arm.h>

const double FINGER_MAX = 6400;

SaraArm::SaraArm(ros::NodeHandle &nh):
    nh_(nh)
{
    ros::NodeHandle pn("~");
    group_ = new moveit::planning_interface::MoveGroupInterface("arm");

    cloud_input_sub_ = nh_.subscribe("cloud_in_pose", 1, &SaraArm::cloudInputCbk, this);

    marker_pub = nh_.advertise<geometry_msgs::PoseStamped>("cloud_in_pose2", 10);

    robot_type_ = "j2n6s300";
    robot_connected_ = true;
    tf::Pose Home;
    Home.setOrigin(tf::Vector3(0.212322831154, -0.257197618484, 0.509646713734));
    // Home.setRotation(kinova::EulerXYZ2Quaternion(1.63771402836,1.11316478252, 0.134094119072));
    Home.setRotation(tf::Quaternion(0.68463, -0.22436, 0.68808, 0.086576));
    geometry_msgs::Pose start_pose; // start from Home pose of j2n6
    tf::poseTFToMsg(Home, start_pose);
    group_->setPoseTarget(start_pose);
    evaluate_plan(*group_);

    finger_client_ = new actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction>
            ("/" + robot_type_ + "_driver/fingers_action/finger_positions", false);
    while(robot_connected_ && !finger_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the finger action server to come up");
    }

    gripper_action(FINGER_MAX); // partially close

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

    tf::Pose button;
    tf::poseMsgToTF(msg->pose, button);
    tf::Vector3 button_xyz = button.getOrigin();
    tf::Quaternion q = button.getRotation();
    tf::Vector3 r_col = button.getBasis().getColumn(2);

    tf::Pose pre_button;
    float offset = 0.1;
    tf::Vector3 pre_button_xyz(-r_col.getX()*offset + button_xyz[0], -r_col.getY()*offset + button_xyz[1], -r_col.getZ()*offset + button_xyz[2]);
    pre_button.setOrigin(pre_button_xyz);
    pre_button.setRotation(q);

    geometry_msgs::PoseStamped pre_button_msg;
    tf::poseTFToMsg(pre_button, pre_button_msg.pose);
    pre_button_msg.header = msg->header;
    marker_pub.publish(pre_button_msg);

    tf::Pose Home;
    Home.setOrigin(tf::Vector3(0.212322831154, -0.257197618484, 0.509646713734));
    // Home.setRotation(kinova::EulerXYZ2Quaternion(1.63771402836,1.11316478252, 0.134094119072));
    Home.setRotation(tf::Quaternion(0.68463, -0.22436, 0.68808, 0.086576));
    geometry_msgs::Pose start_pose; // start from Home pose of j2n6
    tf::poseTFToMsg(Home, start_pose);

    std::vector<geometry_msgs::Pose> waypoints;

    // // waypoints.push_back(start_pose);  // up and out

    // waypoints.push_back(pre_button_msg.pose);  // up and out
    
    waypoints.push_back(msg->pose);  // down and right (back to start)

    // // waypoints.push_back(pre_button_msg.pose);  // up and out

    waypoints.push_back(start_pose);  // up and out
    // We want the cartesian path to be interpolated at a resolution of 1 cm
    // which is why we will specify 0.01 as the max step in cartesian
    // translation. 
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = group_->computeCartesianPath(waypoints,
                                                    0.01,  // eef_step
                                                    10.0,   // jump_threshold
                                                    trajectory);

    if (fraction == 1){
        group_->execute(trajectory);
    }
    
}

/**
 * @brief PickPlace::gripper_action
 * @param gripper_rad close for 6400 and open for 0.0
 * @return true is gripper motion reaches the goal
 */
bool SaraArm::gripper_action(double finger_turn)
{

    kinova_msgs::SetFingersPositionGoal goal;
    goal.fingers.finger1 = finger_turn;
    goal.fingers.finger2 = goal.fingers.finger1;
    goal.fingers.finger3 = goal.fingers.finger1;
    finger_client_->sendGoal(goal);

    if (finger_client_->waitForResult(ros::Duration(5.0)))
    {
        finger_client_->getResult();
        return true;
    }
    else
    {
        finger_client_->cancelAllGoals();
        ROS_WARN_STREAM("The gripper action timed-out");
        return false;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sara_arm_sub");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    SaraArm sa(n);

    ros::waitForShutdown();
    return 0;
}
