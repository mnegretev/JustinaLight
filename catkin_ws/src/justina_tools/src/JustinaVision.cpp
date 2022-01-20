#include "justina_tools/JustinaVision.h"

bool JustinaVision::is_node_set = false;
//Services for getting point cloud
ros::ServiceClient JustinaVision::cltGetRgbdWrtKinect;
ros::ServiceClient JustinaVision::cltGetRgbdWrtRobot;
//Sevices for line finding
ros::ServiceClient JustinaVision::cltFindLines;

bool JustinaVision::setNodeHandle(ros::NodeHandle* nh)
{
    if(JustinaVision::is_node_set)
        return true;
    if(nh == 0)
        return false;

    std::cout << "JustinaVision.->Setting ros node..." << std::endl;
    //Services for getting point cloud
    JustinaVision::cltGetRgbdWrtKinect = nh->serviceClient<hardware_msgs::GetRgbd>("/hardware/kinect/get_rgbd_wrt_kinect");
    JustinaVision::cltGetRgbdWrtRobot  = nh->serviceClient<hardware_msgs::GetRgbd>("/hardware/kinect/get_rgbd_wrt_robot");
    //Sevices for line finding
    JustinaVision::cltFindLines = nh->serviceClient<vision_msgs::FindLines>("/vision/line_finder/find_lines_ransac");

    JustinaVision::is_node_set = true;
    return true;
}


//Methods for line finding
bool JustinaVision::findLine(float& x1, float& y1, float& z1, float& x2, float& y2, float& z2)
{
    std::cout << "JustinaVision.->Trying to find a straight line." << std::endl;
    hardware_msgs::GetRgbd srvGetCloud;
    if(!JustinaVision::cltGetRgbdWrtRobot.call(srvGetCloud))
    {
        std::cout << "JustinaVision.->Cannot find line: cannot get point cloud :'(" << std::endl;
        return false;
    }

    vision_msgs::FindLines srvFindLines;//It really finds only one line
    srvFindLines.request.point_cloud = srvGetCloud.response.point_cloud;

    if(!JustinaVision::cltFindLines.call(srvFindLines) || srvFindLines.response.lines.size() < 2)
    {
        std::cout << "JustinaVision.->Cannot find lines. " << std::endl;
        return false;
    }

    x1 = srvFindLines.response.lines[0].x;
    y1 = srvFindLines.response.lines[0].y;
    z1 = srvFindLines.response.lines[0].z;
    x2 = srvFindLines.response.lines[1].x;
    y2 = srvFindLines.response.lines[1].y;
    z2 = srvFindLines.response.lines[1].z;

    return true;
}
