#include "ros/ros.h"
#include "vision_msgs/FindLines.h"
#include "vision_msgs/RecognizeObjects.h"
#include "vision_msgs/RecognizeObject.h"
#include "vision_msgs/VisionObject.h"
#include "vision_msgs/TrainObject.h"
#include "visualization_msgs/MarkerArray.h"
#include "PlaneExtractor.h"
#include "Utils.h"

tf::TransformListener* tf_listener;

visualization_msgs::MarkerArray get_detected_objs_markers()
{

}

bool callback_find_lines(vision_msgs::FindLines::Request& req, vision_msgs::FindLines::Response& resp)
{
    std::cout << "ObjReco.->Executing srvFindLines (Jebusian method)." << std::endl;
    cv::Mat img, cloud;
    Utils::transform_cloud_wrt_base(req.point_cloud, img, cloud, tf_listener);
    cv::Mat normals = PlaneExtractor::get_horizontal_normals(cloud);
    PlaneExtractor::find_lines(normals);
    return true;
}

bool callback_recog_objs(vision_msgs::RecognizeObjects::Request& req, vision_msgs::RecognizeObjects::Response& resp)
{
    std::cout << "ObjReco.->Recognizing objects by Jebug's method." << std::endl;
    cv::Mat img, cloud;
    //transform_cloud_wrt_base(req.point_cloud, img, cloud);
    return false;
}

bool callback_recog_obj(vision_msgs::RecognizeObject::Request& req, vision_msgs::RecognizeObject::Response& resp)
{
    std::cout << "ObjReco.->Trying to recognize " << req.name << " by Jebug's method." << std::endl;
    cv::Mat img, cloud;
    //transform_cloud_wrt_base(req.point_cloud, img, cloud);
}

bool callback_train_object(vision_msgs::TrainObject::Request& req, vision_msgs::TrainObject::Response& resp)
{
    std::cout << "ObjReco.->Training object " << req.name << " Jebusly..." << std::endl;
    if( req.name == "" )
    {
     	std::cout << "ObjReco.->ERROR!: objects must have a name to be trained" << std::endl;
        return false;
    }
    cv::Mat img, cloud;
    //transform_cloud_wrt_base(req.point_cloud, img, cloud);
    return true;
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING OBJECT RECOGNIZER BY MR. YISUS (CORRECTED AND IMPROVED BY MARCOSOFT)" << std::endl;
    ros::init(argc, argv, "obj_reco_node");
    ros::NodeHandle n;
    ros::ServiceServer srvFindLines = n.advertiseService("/vision/line_finder/find_lines_ransac", callback_find_lines);
    ros::ServiceServer srvRecogObjs = n.advertiseService("/vision/obj_reco/recognize_objects", callback_recog_objs);
    ros::ServiceServer srvRecogObj  = n.advertiseService("/vision/obj_reco/recognize_object" , callback_recog_obj );
    ros::ServiceServer srvTrainObj  = n.advertiseService("/vision/obj_reco/train_object", callback_train_object);
    ros::Rate loop(30);
    tf_listener = new tf::TransformListener();


    std::string training_dir = ros::package::getPath("obj_reco") + std::string("/training_dir");
    if(ros::param::has("~debug"))
        ros::param::get("~debug", Utils::debug);
    if(ros::param::has("~training_dir"))
        ros::param::get("~training_dir", training_dir);
    if(ros::param::has("~min_x"))
        ros::param::get("~min_x", Utils::min_x);
    if(ros::param::has("~max_x"))
        ros::param::get("~max_x", Utils::max_x);
    if(ros::param::has("~min_y"))
        ros::param::get("~min_y", Utils::min_y);
    if(ros::param::has("~max_y"))
        ros::param::get("~max_y", Utils::max_y);
    if(ros::param::has("~min_z"))
        ros::param::get("~min_z", Utils::min_z);
    if(ros::param::has("~max_z"))
        ros::param::get("~max_z", Utils::max_z);
    if(ros::param::has("~normals_tol"))
        ros::param::get("~normals_tol", PlaneExtractor::normals_tol);

    while(ros::ok() && cv::waitKey(10) != 27)
    {
        cv::waitKey(10);
        ros::spinOnce();
        loop.sleep();
    }
}
