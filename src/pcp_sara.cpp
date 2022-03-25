#include "pcp_sara.h"

bool PcpSara::get3DPointSrv(sara_arm::image_to_cloud_point::Request &req, sara_arm::image_to_cloud_point::Response &res) 

{
    geometry_msgs::PointStamped cloud_input;
    get3DPoint(req.col, req.row, cloud_input);
    geometry_msgs::PoseStamped cloud_out;
    cloud_out.header = cloud_input.header;
    cloud_out.pose.position = cloud_input.point;
    res.cloud_input = cloud_out;
    res.success = true;
    return true;
}
