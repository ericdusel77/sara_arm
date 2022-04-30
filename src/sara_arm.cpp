#include <sara_arm.h>

const double FINGER_MAX = 6400;

SaraArm::SaraArm(ros::NodeHandle &nh):
    nh_(nh)
{
    ros::NodeHandle pn("~");

    group_ = new moveit::planning_interface::MoveGroupInterface("arm");

    sub_joint_ = nh_.subscribe<sensor_msgs::JointState>("/j2n6s300_driver/out/joint_state", 1, &SaraArm::get_current_state, this);

    cloud_input_sub_ = nh_.subscribe("button_pose", 1, &SaraArm::cloudInputCbk, this);

    marker_pub = nh_.advertise<geometry_msgs::PoseStamped>("pre_button_pose", 10);
    array_pub = nh_.advertise<geometry_msgs::PoseArray>("trajectory", 10);

    robot_type_ = "j2n6s300";
    robot_connected_ = true;
    home = {87*M_PI/180, 192*M_PI/180, 301*M_PI/180, 62*M_PI/180, 74*M_PI/180, -11*M_PI/180};

    group_->setJointValueTarget(home);

    evaluate_plan(*group_);

    finger_client_ = new actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction>
            ("/" + robot_type_ + "_driver/fingers_action/finger_positions", false);
    while(robot_connected_ && !finger_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the finger action server to come up");
    }

    gripper_action(FINGER_MAX); 

}


SaraArm::~SaraArm()
{
    delete group_;
}


void SaraArm::get_current_state(const sensor_msgs::JointStateConstPtr &msg)
{
    boost::mutex::scoped_lock lock(mutex_state_);
    current_state_ = *msg;
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
    //TURN BUTTON POSE MSG INTO TF TO GET ROTATION AND TRANSLATION
    tf::Pose button;
    tf::poseMsgToTF(msg->pose, button);
    tf::Vector3 button_xyz = button.getOrigin();
    tf::Quaternion q = button.getRotation();
    tf::Vector3 r_col = button.getBasis().getColumn(2);

    if (button_xyz[0] < 0){
        std::cout<<"Found incorrect pose. Try pressing a different point."<<std::endl;
        return;
    }

    // PRE BUTTON POSE FROM BUTTON POSE WITH OFFSET
    tf::Pose pre_button;
    float offset = 0.05;
    tf::Vector3 pre_button_xyz(-r_col.getX()*offset + button_xyz[0], -r_col.getY()*offset + button_xyz[1], -r_col.getZ()*offset + button_xyz[2]);
    pre_button.setOrigin(pre_button_xyz);
    pre_button.setRotation(q);

    geometry_msgs::PoseStamped pre_button_msg;
    tf::poseTFToMsg(pre_button, pre_button_msg.pose);
    pre_button_msg.header = msg->header;
    marker_pub.publish(pre_button_msg);

    // HOME POSE
    tf::Pose Home;
    Home.setOrigin(tf::Vector3(0.212322831154, -0.257197618484, 0.509646713734));
    // Home.setRotation(kinova::EulerXYZ2Quaternion(1.63771402836,1.11316478252, 0.134094119072));
    Home.setRotation(tf::Quaternion(0.68463, -0.22436, 0.68808, 0.086576));
    geometry_msgs::Pose start_pose; // start from Home pose of j2n6
    tf::poseTFToMsg(Home, start_pose);

    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(group_->getRobotModel()));
    robot_state->setToDefaultValues();
    const moveit::core::JointModelGroup* joint_model_group = group_->getRobotModel()->getJointModelGroup("arm");


    // const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
    // moveit::core::jointStateToRobotState(current_state_, *robot_state);

    // planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(group_->getRobotModel()));

    // planning_scene->setCurrentState(*robot_state);

    // // We will now construct a loader to load a planner, by name.
    // // Note that we are using the ROS pluginlib library here.
    // boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
    // planning_interface::PlannerManagerPtr planner_instance;
    // std::string planner_plugin_name = "ompl_interface/OMPLPlanner";

    // // We will get the name of planning plugin we want to load
    // // from the ROS parameter server, and then load the planner
    // // making sure to catch all exceptions.
    // try
    // {
    //     planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
    //         "moveit_core", "planning_interface::PlannerManager"));
    // }
    // catch (pluginlib::PluginlibException& ex)
    // {
    //     ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
    // }
    // try
    // {
    //     planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
    //     if (!planner_instance->initialize(robot_state->getRobotModel(), nh_.getNamespace()))
    //     ROS_FATAL_STREAM("Could not initialize planner instance");
    //     ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
    // }
    // catch (pluginlib::PluginlibException& ex)
    // {
    //     const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
    //     std::stringstream ss;
    //     for (const auto& cls : classes)
    //     ss << cls << " ";
    //     ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
    //                                                         << "Available plugins: " << ss.str());
    // }

    // planning_interface::MotionPlanRequest req;
    // planning_interface::MotionPlanResponse res;

    // // A tolerance of 0.01 m is specified in position
    // // and 0.01 radians in orientation
    // std::vector<double> tolerance_pose(3, 0.01);
    // std::vector<double> tolerance_angle = {0.01,0.01,1.0};

    // // We will create the request as a constraint using a helper function available
    // // from the
    // // `kinematic_constraints`_
    // // package.
    // //
    // // .. _kinematic_constraints:
    // //     http://docs.ros.org/noetic/api/moveit_core/html/cpp/namespacekinematic__constraints.html#a88becba14be9ced36fefc7980271e132
    // moveit_msgs::Constraints pose_goal =
    //     kinematic_constraints::constructGoalConstraints("j2n6s300_end_effector", *msg, tolerance_pose, tolerance_angle);

    // req.group_name = "arm";
    // req.goal_constraints.push_back(pose_goal);

    // // We now construct a planning context that encapsulate the scene,
    // // the request and the response. We call the planner using this
    // // planning context
    // planning_interface::PlanningContextPtr context =
    //     planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
    // context->solve(res);
    // if (res.error_code_.val != res.error_code_.SUCCESS)
    // {
    //     ROS_ERROR("Could not compute plan successfully");
    //     return;
    // }

    // moveit_msgs::RobotTrajectory trajectory2;
    // res.trajectory_->getRobotTrajectoryMsg(trajectory2);
    // trajectory_msgs::JointTrajectory j = trajectory2.joint_trajectory;
    // std::cout<<j.points.back()<<std::endl;

    // CREATE TRAJECTORY
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(pre_button_msg.pose);  // PRE BUTTON
    waypoints.push_back(msg->pose);  // GOAL (BUTTON)
    waypoints.push_back(pre_button_msg.pose);  // PRE BUTTON
    waypoints.push_back(start_pose);  // HOME
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = group_->computeCartesianPath(waypoints,
                                                    0.01,  // eef_step - path to be interpolated at this resolution
                                                    10.0,   // jump_threshold
                                                    trajectory);


    // std::cout<<j.points.back()<<std::endl;
    std::vector<trajectory_msgs::JointTrajectoryPoint> j = trajectory.joint_trajectory.points;
    geometry_msgs::PoseArray poseArray;
    poseArray.header.frame_id = "world";
    for (int i=0;i<j.size();i++) {
        std::vector<double> joint_values = j[i].positions;
        robot_state->setJointGroupPositions(joint_model_group, joint_values);
        const Eigen::Isometry3d& end_effector_state = robot_state->getGlobalLinkTransform(group_->getEndEffectorLink());

        geometry_msgs::Pose p = tf2::toMsg(end_effector_state);
        poseArray.poses.push_back(p);
    }
    array_pub.publish(poseArray);

    // EXECUTE TRAJECTORY IF FULL PATH IS FOUND
    if (fraction == 1){
        ROS_INFO_STREAM("Press 'y'' to start motion plan ...");
        std::cin >> pause_;
        if (pause_ == "y" || pause_ == "Y"){
            group_->execute(trajectory);
        }
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
