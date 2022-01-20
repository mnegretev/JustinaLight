#include "ros/ros.h"
#include "pcl_ros/transforms.h"
#include "vision_msgs/FindLines.h"
#include "justina_tools/JustinaTools.h"
#include "ObjExtractor.hpp"

tf::TransformListener* tf_listener;
bool debug_mode = false;

bool callback_find_lines(vision_msgs::FindLines::Request& req, vision_msgs::FindLines::Response& resp)
{
    std::cout << "ObjReco.->Executing srvFindLines (Yisus Version)" << std::endl;
    if(req.point_cloud.header.frame_id != "base_link")
    {
        std::cout << "ObjReco.->Transforming point cloud to robot reference" << std::endl;
        pcl_ros::transformPointCloud("base_link", req.point_cloud, req.point_cloud, *tf_listener);
    }
    cv::Mat bgrImg;
    cv::Mat xyzCloud;
    JustinaTools::PointCloud2Msg_ToCvMat(req.point_cloud, bgrImg, xyzCloud);
    ObjExtractor::DebugMode = debug_mode;
    cv::Vec4i pointsLine = ObjExtractor::GetLine( xyzCloud );
    if( pointsLine == cv::Vec4i(0,0,0,0) )
    {
        std::cout << "ObjReco.->Line not Detected" << std::endl;
	return false;
    }
    cv::Point3f iniLine = xyzCloud.at<cv::Vec3f>( cv::Point(pointsLine[0], pointsLine[1]) );
    cv::Point3f endLine = xyzCloud.at<cv::Vec3f>( cv::Point(pointsLine[2], pointsLine[3]) );
    geometry_msgs::Point p1;
    p1.x = iniLine.x;
    p1.y = iniLine.y;
    p1.z = iniLine.z;
    geometry_msgs::Point p2;
    p2.x = endLine.x;
    p2.y = endLine.y;
    p2.z = endLine.z;
    resp.lines.push_back(p1);
    resp.lines.push_back(p2);
    cv::line(bgrImg,cv::Point(pointsLine[0], pointsLine[1]),cv::Point(pointsLine[2], pointsLine[3]),cv::Scalar(0,255,0),3,8);
    cv::imshow("Found Line", bgrImg );
    cv::waitKey(10);
    std::cout << "ObjReco.->Line found:" << std::endl;
    std::cout << "ObjReco.->  p1=" << iniLine << std::endl;
    std::cout << "ObjReco.->  p2=" << endLine << std::endl;
    return true;
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING OBJECT RECOGNIZER BY MR. YISUS" << std::endl;
    ros::init(argc, argv, "obj_reco_node");
    ros::NodeHandle n;
    ros::ServiceServer srvFindLines = n.advertiseService("/vision/line_finder/find_lines_ransac", callback_find_lines);
    ros::Rate loop(30);
    tf_listener = new tf::TransformListener();

    if(ros::param::has("~debug"))
        ros::param::get("~debug", debug_mode);

    while(ros::ok())
    {
        cv::waitKey(10);
        ros::spinOnce();
        loop.sleep();
    }
}
