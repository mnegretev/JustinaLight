#include "ros/ros.h"
#include "pcl_ros/transforms.h"
#include "vision_msgs/FindLines.h"
#include "vision_msgs/RecognizeObjects.h"
#include "vision_msgs/VisionObject.h"
#include "vision_msgs/TrainObject.h"
#include "visualization_msgs/MarkerArray.h"
#include "justina_tools/JustinaTools.h"
#include "ObjExtractor.hpp"
#include "ObjRecognizer.hpp"

tf::TransformListener* tf_listener;
bool debug_mode = false;
ObjRecognizer object_recognizer;

visualization_msgs::MarkerArray get_detected_objs_markers(std::vector<DetectedObject>& objs, std::vector<std::string> ids)
{
    visualization_msgs::MarkerArray mrks;
    for(size_t i=0; i< objs.size(); i++)
    {
        visualization_msgs::Marker m;
        m.header.frame_id = "base_link";
	m.header.stamp = ros::Time::now();
        m.ns = "recog_objects";
        m.id = i;
        m.type = visualization_msgs::Marker::SPHERE;
	m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = objs[i].centroid.x;
        m.pose.position.y = objs[i].centroid.y;
	m.pose.position.z = objs[i].centroid.z;
        m.scale.x = 0.05;
        m.scale.y = 0.05;
	m.scale.z = 0.05;
        m.color.r = 1.0;
        m.color.g = 0.0;
        m.color.b = 0.0;
        m.color.a = 0.70;
        m.lifetime = ros::Duration(20.0, 20.0);
        mrks.markers.push_back(m);
        m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        m.text = ids[i];
        m.scale.z = 0.05;
    }
    return mrks;
}

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
        cv::imshow("Found Line", bgrImg );
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
    std::cout << "ObjReco.->Line found:" << std::endl;
    std::cout << "ObjReco.->  p1=" << iniLine << std::endl;
    std::cout << "ObjReco.->  p2=" << endLine << std::endl;
    return true;
}

bool callback_recog_objs(vision_msgs::RecognizeObjects::Request& req, vision_msgs::RecognizeObjects::Response& resp)
{
    std::cout << "ObjReco.->Recognizing objects by Jebug's method..." << std::endl;
    if(req.point_cloud.header.frame_id != "base_link")
    {
        std::cout << "ObjReco.->Transforming point cloud to robot reference" << std::endl;
        pcl_ros::transformPointCloud("base_link", req.point_cloud, req.point_cloud, *tf_listener);
    }
    cv::Mat bgrImg;
    cv::Mat xyzCloud;
    JustinaTools::PointCloud2Msg_ToCvMat(req.point_cloud, bgrImg, xyzCloud);
    ObjExtractor::DebugMode = debug_mode;
    std::vector<DetectedObject> detected_objs = ObjExtractor::GetObjectsInHorizontalPlanes(xyzCloud);
    std::vector<std::string>    objs_names;
    for(int i=0; i < detected_objs.size(); i++)
    {
        std::string obj_name = object_recognizer.RecognizeObject(detected_objs[i], bgrImg);
        objs_names.push_back(obj_name);
        vision_msgs::VisionObject obj;
        obj.header.frame_id = "base_link";
        obj.header.stamp    = ros::Time::now();
        obj.id              = obj_name == "" ? "unknown" : obj_name;
        obj.confidence      = 0.5; //A temporal value, since ObjRecognizer does not return a confidence value
        obj.pose.position.x = detected_objs[i].centroid.x;
        obj.pose.position.y = detected_objs[i].centroid.y;
        obj.pose.position.z = detected_objs[i].centroid.z;
        obj.pose.orientation.w = 1.0;
        resp.recog_objects.push_back(obj);
        cv::rectangle(bgrImg, detected_objs[i].boundBox, cv::Scalar(0,0,255));
        cv::putText(bgrImg, obj_name, detected_objs[i].boundBox.tl(), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,0,255));
    }
    std::cout << "ObjReco.->Detected " << detected_objs.size() << " objects." << std::endl;
    cv::imshow("Recognized Objects", bgrImg);
    return resp.recog_objects.size() > 0;
}

bool callback_train_object(vision_msgs::TrainObject::Request& req, vision_msgs::TrainObject::Response& resp)
{
    std::cout << "ObjReco.->Training object " << req.name << " Jebusly..." << std::endl;
    if( req.name == "" )
    {
     	std::cout << "ObjReco.->ERROR!: objects must have a name to be trained" << std::endl;
        return false;
    }
    if(req.point_cloud.header.frame_id != "base_link")
    {
        std::cout << "ObjReco.->Transforming point cloud to robot reference" << std::endl;
        pcl_ros::transformPointCloud("base_link", req.point_cloud, req.point_cloud, *tf_listener);
    }
    cv::Mat bgrImg;
    cv::Mat xyzCloud;
    JustinaTools::PointCloud2Msg_ToCvMat(req.point_cloud, bgrImg, xyzCloud);
    ObjExtractor::DebugMode = debug_mode;
    std::vector<DetectedObject> detected_objs = ObjExtractor::GetObjectsInHorizontalPlanes(xyzCloud);
    std::sort(detected_objs.begin(), detected_objs.end(), DetectedObject::CompareByEuclidean);
    if(detected_objs.size() == 0)
    {
        std::cout << "ObjReco.->Cannot train object " << req.name << ". No objects detected on plane" << std::endl;
        return false;
    }
    object_recognizer.TrainObject(detected_objs[0], bgrImg, req.name);
    //cv::bitwise_and(bgrImg, detected_objs[0].oriMask, bgrImg);
    cv::imshow("Trained Object", bgrImg);
    std::cout << "ObjReco.->Object " << req.name << " trained succesfully in a Jebusly manner" << std::endl;
    return true;
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING OBJECT RECOGNIZER BY MR. YISUS" << std::endl;
    ros::init(argc, argv, "obj_reco_node");
    ros::NodeHandle n;
    ros::ServiceServer srvFindLines = n.advertiseService("/vision/line_finder/find_lines_ransac", callback_find_lines);
    ros::ServiceServer srvRecogObjs = n.advertiseService("/vision/obj_reco/recognize_objects", callback_recog_objs);
    ros::ServiceServer srvTrainObj  = n.advertiseService("/vision/obj_reco/train_object", callback_train_object);
    ros::Rate loop(30);
    tf_listener = new tf::TransformListener();


    std::string training_dir = ros::package::getPath("obj_reco") + std::string("/training_dir");
    if(ros::param::has("~debug"))
        ros::param::get("~debug", debug_mode);
    if(ros::param::has("~training_dir"))
        ros::param::get("~training_dir", training_dir);
    object_recognizer = ObjRecognizer(18);
    object_recognizer.TrainingDir = training_dir;
    if(!object_recognizer.LoadTrainingDir(training_dir))
        return -1;

    while(ros::ok())
    {
        cv::waitKey(10);
        ros::spinOnce();
        loop.sleep();
    }
}
