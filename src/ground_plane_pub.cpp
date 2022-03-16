#include "ros/ros.h"
#include <point_cloud_proc/point_cloud_proc.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ground_plane_pub");
  ros::NodeHandle nh;

  ros::Publisher plane_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("plane_cloud", 10);
  std::string pkg_path = ros::package::getPath("sara_arm");
  std::string config_path = pkg_path + "/config/pcp_params.yaml";
  
  PointCloudProc pcp(nh, false, config_path);

  sensor_msgs::PointCloud2 cloud_projected;
  
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  coefficients->values.resize (4);
  coefficients->values[0] = coefficients->values[1] = 0;
  coefficients->values[2] = 1.0;
  coefficients->values[3] = 0;

  ros::Publisher ground_proj_pub = nh.advertise<sensor_msgs::PointCloud2>("ground_proj_plane", 10);

  while (ros::ok())
  {
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::Duration(1.0).sleep();

    point_cloud_proc::Plane plane;
    pcp.segmentSinglePlane(plane, 'z');

    plane_cloud_pub.publish(plane.cloud);

    // coefficients->values[0] = plane.coef[0];
    // coefficients->values[1] = plane.coef[1];
    // coefficients->values[2] = plane.coef[2];
    // coefficients->values[3] = plane.coef[3];

    pcp.projectPointCloudToPlane(plane.cloud, cloud_projected, coefficients);

    ground_proj_pub.publish(cloud_projected);

  }

  return 0;
}