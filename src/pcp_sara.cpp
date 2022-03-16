#include "pcp_sara.h"

bool PcpSara::get3DPointSrv(sara_arm::image_to_cloud_point::Request &req, sara_arm::image_to_cloud_point::Response &res) 

{
    geometry_msgs::PointStamped cloud_input;
    get3DPoint(req.col, req.row, cloud_input);
    res.cloud_input = cloud_input;
    res.success = true;
    return true;
}
