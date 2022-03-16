#include "pcp_sara.h"

int main(int argc, char** argv)
{
    // INITIALIZE ROS
    ros::init(argc, argv, "pcp_service");
    
    // INITIALIZE THE MAIN ROS NODE HANDLE
    ros::NodeHandle nh;
    // ros::Publisher point_3d_pub = nh.advertise<geometry_msgs::Point>("point_cloud/selected_goal", 10);

    std::string pkg_path = ros::package::getPath("sara_arm");
    std::string config_path = pkg_path + "/config/pcp_params.yaml";
    
    PcpSara pcp(nh, false, config_path);
  
    // SERVICE
    ros::ServiceServer image_to_cloud_point_srv = nh.advertiseService("image_to_cloud_point", &PcpSara::get3DPointSrv, &pcp);

    ros::spin();
    //ros::waitForShutdown();
}