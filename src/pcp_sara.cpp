#include <pcp_sara.h>

tf::Quaternion EulerZYZ_to_Quaternion(double tz1, double ty, double tz2)
{
    tf::Quaternion q;
    tf::Matrix3x3 rot;
    tf::Matrix3x3 rot_temp;
    rot.setIdentity();

    rot_temp.setEulerYPR(tz1, 0.0, 0.0);
    rot *= rot_temp;
    rot_temp.setEulerYPR(0.0, -ty, 0.0);
    rot *= rot_temp;
    rot_temp.setEulerYPR(tz2, 0.0, 0.0);
    rot *= rot_temp;
    rot.getRotation(q);
    return q;
}


PcpSara::PcpSara(ros::NodeHandle n) :
        nh_(n), cloud_transformed_(new CloudT){

    // General parameters
    point_cloud_topic_ = "/camera/depth/color/points";
    fixed_frame_ = "world";

    // Subscriber 
    point_cloud_sub_ = nh_.subscribe(point_cloud_topic_, 10, &PcpSara::pointCloudCb, this);
    point_cloud_pub_ = n.advertise<sensor_msgs::PointCloud2>("chatter", 10);
    plane_cloud_pub_ = n.advertise<sensor_msgs::PointCloud2>("chatter2", 10);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
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

    pcl_conversions::fromPCL(cloud_transformed_->header, pose.header);

    if (pcl::isFinite(cloud_transformed_->at(col, row))) {
        pose.pose.position.x = cloud_transformed_->at(col, row).x;
        pose.pose.position.y = cloud_transformed_->at(col, row).y;
        pose.pose.position.z = cloud_transformed_->at(col, row).z;

        CloudT newcloud;
        CloudT plane_cloud;
        newcloud.header.frame_id = "world";
        plane_cloud.header.frame_id = "world";

        pcl::NormalEstimation<PointT, PointNT> ne;
        pcl::Indices indx = {};

        int size = 10;
        int row_place = row-size;
        for (int t = 0; t < 2*size; t++){
            int place = row_place*640+col-size;
            for (int i = 0; i < size*2; i++){
                if (cloud_transformed_->points[place + i].x > 0){
                    indx.push_back(place + i);
                    newcloud.push_back(cloud_transformed_->points[place + i]);
                }   
            }
            row_place++;
        }

        Eigen::Vector4f plane_parameters;
        float curvature;
        ne.computePointNormal(*cloud_transformed_, indx, plane_parameters, curvature);
        
        float a = -1*(plane_parameters[0]/plane_parameters[2]);
        float b = -1*(plane_parameters[1]/plane_parameters[2]);
        float c = -1*(plane_parameters[3]/plane_parameters[2]);

        for (float x = -10; x < 10; x++){
            for (float y = -10; y < 10; y++){
                PointT temp_p;
                temp_p.x = cloud_transformed_->at(col, row).x+(x/50);
                temp_p.y = cloud_transformed_->at(col, row).y+(y/50);
                temp_p.z = a*(cloud_transformed_->at(col, row).x+(x/50)) + b*(cloud_transformed_->at(col, row).y+(y/50)) + c;
                plane_cloud.push_back(temp_p);
            }
        }

        point_cloud_pub_.publish(newcloud);
        plane_cloud_pub_.publish(plane_cloud);

        // EXTRACT NORMAL
        tf2::Vector3 normal;
        normal.setX(plane_parameters[0]);
        normal.setY(plane_parameters[1]);
        normal.setZ(plane_parameters[2]);

        double phi = atan2(normal.getY(),normal.getX());
        double theta = asin(normal.getZ()/normal.length());
        
        if (abs(phi) > M_PI/2){
            phi = phi + M_PI;
            theta = -theta;
        }
    
        tf::Quaternion qq;
        qq = EulerZYZ_to_Quaternion(phi, theta, 0.0);

        pose.pose.orientation.x = qq.x();
        pose.pose.orientation.y = qq.y();
        pose.pose.orientation.z = qq.z();
        pose.pose.orientation.w = qq.w();

        visualization_msgs::Marker points, line_strip, line_list;
        line_strip.header.frame_id = "world";
        line_strip.header.stamp = ros::Time::now();
        line_strip.ns = "points_and_lines";
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.pose.orientation.w = 1.0;

        line_strip.id = 1;

        line_strip.type = visualization_msgs::Marker::LINE_STRIP;

        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        line_strip.scale.x = 0.1;

        // Line strip is blue
        line_strip.color.b = 1.0;
        line_strip.color.a = 1.0;

        // Create the vertices for the points and lines
        for (uint32_t i = 0; i < 2; ++i)
        {
            geometry_msgs::Point p;
            p.x = i*normal.getX();
            p.y = i*normal.getY();
            p.z = i*normal.getZ();

            line_strip.points.push_back(p);

        }

        marker_pub.publish(line_strip);

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