#include <pcp_sara.h>

PcpSara::PcpSara(ros::NodeHandle n) :
        nh_(n), cloud_transformed_(new CloudT){

    // General parameters
    point_cloud_topic_ = "/camera/depth/color/points";
    fixed_frame_ = "world";

    // Subscriber 
    point_cloud_sub_ = nh_.subscribe(point_cloud_topic_, 10, &PcpSara::pointCloudCb, this);
    
}

void PcpSara::pointCloudCb(const sensor_msgs::PointCloud2ConstPtr &msg) {
    boost::mutex::scoped_lock lock(pc_mutex_);
    cloud_raw_ros_ = *msg;
    pc_received_ = true;
}

bool PcpSara::transformPointCloud() {
    while (ros::ok()){
        if(pc_received_)
            break;
        else
            ros::Duration(0.1).sleep();
            ROS_INFO("Waiting for point cloud");
    }

    boost::mutex::scoped_lock lock(pc_mutex_);

    cloud_transformed_->clear();

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener listener(tfBuffer);
    std::string target_frame = cloud_raw_ros_.header.frame_id;

    tfBuffer.canTransform(fixed_frame_, target_frame, ros::Time(0), ros::Duration(2.0));
    // geometry_msgs::TransformStamped transformStamped;
    // tf2::Transform cloud_transform;

    try {
        // transformStamped = tfBuffer.lookupTransform(fixed_frame_, target_frame, ros::Time(0));
        // tf2::Stamped<tf2::Transform> transform;
        // tf2::fromMsg(transformStamped, transform);
        // cloud_transform.setOrigin(transform.getOrigin());
        // cloud_transform.setRotation(transform.getRotation());

        CloudT cloud_in;
        auto time = ros::Time(0);
        tfBuffer.canTransform(fixed_frame_, target_frame, time, ros::Duration(2.0));
        pcl::fromROSMsg(cloud_raw_ros_, cloud_in);
        pcl_ros::transformPointCloud(fixed_frame_, time, cloud_in, target_frame, *cloud_transformed_, tfBuffer);

        // pcl::fromROSMsg(cloud_transformed, *cloud_transformed_);

        std::cout << "PCP: point cloud is transformed!" << std::endl;
        return true;

    }
    catch (tf2::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        return false;
    }


}

bool PcpSara::get3DPose(int col, int row, geometry_msgs::PoseStamped &pose) {

    if (!transformPointCloud()) {
        std::cout << "PCP: couldn't transform point cloud!" << std::endl;
        return false;
    }
    std::cout << "CLOUD TRANSFORMED.....MAKE IT COMPATIBEL" << std::endl;
    pcl_conversions::fromPCL(cloud_transformed_->header, pose.header);

    // FOR ORIENTATION
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::copyPointCloud(*cloud_transformed_, *cloud_xyz);
    std::cout << "CLOUD COPIED.....FIND NORMALS" << std::endl;
    // COMPUTE POINT NORMALS
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    CloudNT::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

    tree->setInputCloud(cloud_xyz);
    ne.setInputCloud(cloud_xyz);
    ne.setSearchMethod(tree);
    ne.setKSearch(20);
    ne.compute(*normals);
    std::cout << "NORMALS COMPUTED......BEGINNING QUEST FOR POSE" << std::endl;
    if (pcl::isFinite(cloud_transformed_->at(col, row))) {

        pose.pose.position.x = cloud_transformed_->at(col, row).x;
        pose.pose.position.y = cloud_transformed_->at(col, row).y;
        pose.pose.position.z = cloud_transformed_->at(col, row).z;

        // EXTRACT NORMAL
        tf2::Vector3 normal;
        normal.setX(normals->at(col, row).normal_x);
        normal.setY(normals->at(col, row).normal_y);
        normal.setZ(normals->at(col, row).normal_z);
        // CONVERT TO QUATERNION
        tf2::Quaternion qq(normal,1);
        pose.pose.orientation.x = qq.x();
        pose.pose.orientation.y = qq.y();
        pose.pose.orientation.z = qq.z();
        pose.pose.orientation.w = qq.w();
        return true;
    } else {
        std::cout << "PCP: The 3D point is not valid!" << std::endl;
        return false;
    }

}

bool PcpSara::get3DPoseSrv(sara_arm::image_to_cloud_point::Request &req, sara_arm::image_to_cloud_point::Response &res) 
{
    get3DPose(req.col, req.row, res.cloud_in_pose);

    res.success = true;
    return true;
}

void PcpSara::getNormalEstimation()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::copyPointCloud(*cloud_transformed_, *cloud_xyz);

    // Compute point normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    CloudNT::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

    tree->setInputCloud(cloud_xyz);
    ne.setInputCloud(cloud_xyz);
    ne.setSearchMethod(tree);
    ne.setKSearch(20);
    ne.compute(*normals);
}

int main(int argc, char** argv)
{
    // INITIALIZE ROS
    ros::init(argc, argv, "pcp_service");
    
    // INITIALIZE THE MAIN ROS NODE HANDLE
    ros::NodeHandle nh;
    
    PcpSara pcp(nh);
  
    // SERVICE
    ros::ServiceServer image_to_cloud_point_srv = nh.advertiseService("image_to_cloud_point", &PcpSara::get3DPoseSrv, &pcp);

    ros::spin();

}