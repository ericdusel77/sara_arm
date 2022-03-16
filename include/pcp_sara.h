#ifndef PCP_SARA_H
#define PCP_SARA_H

#include "ros/ros.h"
#include <point_cloud_proc/point_cloud_proc.h>
#include "sara_arm/image_to_cloud_point.h"

class PcpSara: public PointCloudProc {
    public:
        using PointCloudProc::PointCloudProc;

        bool get3DPointSrv(sara_arm::image_to_cloud_point::Request &req, sara_arm::image_to_cloud_point::Response &res) ;
};

#endif