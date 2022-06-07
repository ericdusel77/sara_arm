#include <sara_arm.h>

const double FINGER_MAX = 6400;

SaraArm::SaraArm(ros::NodeHandle &nh):
    nh_(nh)
{
    ros::NodeHandle pn("~");

    group_ = new moveit::planning_interface::MoveGroupInterface("arm");

    sub_joint_ = nh_.subscribe<sensor_msgs::JointState>("/j2n6s300_driver/out/joint_state", 1, &SaraArm::get_current_state, this);

    cloud_input_sub_ = nh_.subscribe("button_pose", 1, &SaraArm::cloudInputCbk, this);

    marker_pub = nh_.advertise<geometry_msgs::PoseStamped>("approach_pose", 10);
    array_pub = nh_.advertise<geometry_msgs::PoseArray>("trajectory", 10);

    robot_type_ = "j2n6s300";
    robot_connected_ = true;
    home = {279*M_PI/180, 170*M_PI/180, 49*M_PI/180, 227*M_PI/180, 93*M_PI/180, 80*M_PI/180};

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

    if (button_xyz[0] < 0){
        std::cout<<"Found incorrect pose. Try pressing a different point."<<std::endl;
        return;
    }

    ///////////////////////
    // GET POSE FROM API
    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(group_->getRobotModel()));
    const moveit::core::JointModelGroup* joint_model_group = group_->getRobotModel()->getJointModelGroup("arm");

    // GET HOME POSE
    robot_state->setJointGroupPositions(joint_model_group, home);
    const Eigen::Isometry3d& state_home = robot_state->getGlobalLinkTransform(group_->getEndEffectorLink());
    geometry_msgs::Pose home_pose = tf2::toMsg(state_home);

    // Set robot state to current state
    moveit::core::jointStateToRobotState(current_state_, *robot_state);

    // CREATE PLANNING SCENE USED FOR REQUESTING TRAJECTORY
    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(group_->getRobotModel()));
    planning_scene->setCurrentState(*robot_state);

    // We will now construct a loader to load a planner, by name.
    // Note that we are using the ROS pluginlib library here.
    boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
    planning_interface::PlannerManagerPtr planner_instance;
    std::string planner_plugin_name = "ompl_interface/OMPLPlanner";

    // We will get the name of planning plugin we want to load
    // from the ROS parameter server, and then load the planner
    // making sure to catch all exceptions.
    try
    {
        planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
            "moveit_core", "planning_interface::PlannerManager"));
    }
    catch (pluginlib::PluginlibException& ex)
    {
        ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
    }
    try
    {
        planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
        if (!planner_instance->initialize(robot_state->getRobotModel(), nh_.getNamespace()))
        ROS_FATAL_STREAM("Could not initialize planner instance");
        ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
    }
    catch (pluginlib::PluginlibException& ex)
    {
        const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
        std::stringstream ss;
        for (const auto& cls : classes)
        ss << cls << " ";
        ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                                                            << "Available plugins: " << ss.str());
    }

    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;

    // ADD GOAL CONSTRAINT (HIGHER ROTATION TOLERANCE ON GOAL Z-AXIS)
    std::vector<double> tolerance_pose(3, 0.01);
    std::vector<double> tolerance_angle = {0.01,0.01,M_PI/2};
    moveit_msgs::Constraints constraint_goal =
        kinematic_constraints::constructGoalConstraints(group_->getEndEffectorLink(), *msg, tolerance_pose, tolerance_angle);
    std::cout<<group_->getEndEffectorLink()<<std::endl;
    req.group_name = "arm";
    req.goal_constraints.push_back(constraint_goal);

    // We now construct a planning context that encapsulate the scene,
    // the request and the response. We call the planner using this
    // planning context
    planning_interface::PlanningContextPtr context =
        planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
    context->solve(res);
    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
        ROS_ERROR("Could not compute plan successfully");
        return;
    }

    moveit_msgs::RobotTrajectory trajectory_temp;
    res.trajectory_->getRobotTrajectoryMsg(trajectory_temp);

    std::vector<trajectory_msgs::JointTrajectoryPoint> j = trajectory_temp.joint_trajectory.points;

    // GET LAST POINT OF CONSTRAINED TRAJECTORY
    std::vector<double> joint_values = j.back().positions;
    robot_state->setJointGroupPositions(joint_model_group, joint_values);
    const Eigen::Isometry3d& state_goal = robot_state->getGlobalLinkTransform(group_->getEndEffectorLink());
    geometry_msgs::Pose c_goal = tf2::toMsg(state_goal);

    // CONVERT CONSTRAINED POSE TO BUTTON
    // tf::poseMsgToTF(c_goal, button);

    // USE FOR GETTING PRE-BUTTON POSE
    tf::Quaternion q = button.getRotation();
    tf::Vector3 r_col = button.getBasis().getColumn(2);
    
    // PRE BUTTON POSE FROM BUTTON POSE WITH OFFSET
    tf::Pose approach;
    float offset = 0.05;
    tf::Vector3 approach_xyz(-r_col.getX()*offset + button_xyz[0], -r_col.getY()*offset + button_xyz[1], -r_col.getZ()*offset + button_xyz[2]);
    approach.setOrigin(approach_xyz);
    approach.setRotation(q);

    // CONVERT TF TO POSE MSG
    geometry_msgs::PoseStamped approach_msg;
    tf::poseTFToMsg(approach, approach_msg.pose);
    approach_msg.header = msg->header;
    marker_pub.publish(approach_msg);

    // CREATE TRAJECTORY
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(approach_msg.pose);  // PRE BUTTON
    waypoints.push_back(msg->pose);  // GOAL (BUTTON)
    // waypoints.push_back(c_goal);  // GOAL (BUTTON)
    waypoints.push_back(approach_msg.pose);  // PRE BUTTON
    waypoints.push_back(home_pose);  // HOME
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = group_->computeCartesianPath(waypoints,
                                                    0.01,  // eef_step - path to be interpolated at this resolution
                                                    10.0,   // jump_threshold
                                                    trajectory);



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
