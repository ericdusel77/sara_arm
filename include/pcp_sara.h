#ifndef PCP_SARA_H
#define PCP_SARA_H

// ROS
#include <ros/ros.h>
#include <ros/package.h>

// PCL
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common.h>
#include <pcl/common/utils.h>
#include "sara_arm/image_to_cloud_point.h"
#include <visualization_msgs/Marker.h>

#include <std_msgs/UInt8.h>

class PcpSara {
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::Normal PointNT;
    typedef pcl::PointCloud<PointT> CloudT;
    typedef pcl::PointCloud<PointNT> CloudNT;
    
    public:
        PcpSara(ros::NodeHandle n);

        void pointCloudCb(const sensor_msgs::PointCloud2ConstPtr &msg);

        bool transformPointCloud();

        bool get3DPose(int col, int row, geometry_msgs::PoseStamped &point);
        bool get3DPoseSrv(sara_arm::image_to_cloud_point::Request &req, sara_arm::image_to_cloud_point::Response &res) ;

    private:
        // cb + transform
        bool pc_received_ = false;
        sensor_msgs::PointCloud2 cloud_raw_ros_;
        boost::mutex pc_mutex_;

        int plan_type_;
        int task_; // 1 == press button, 2 == card
        ros::Subscriber plan_type_sub_;
        void planTypeCbk(const std_msgs::UInt8::ConstPtr& msg);

        // params 
        std::string point_cloud_topic_, fixed_frame_;

        // for pcl processing
        CloudT::Ptr cloud_transformed_;

        ros::NodeHandle nh_;
        ros::Subscriber point_cloud_sub_;
};

#endif //PCP_SARA_H
