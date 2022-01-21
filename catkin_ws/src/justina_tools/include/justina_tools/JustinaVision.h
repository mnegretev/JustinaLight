#pragma once
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "vision_msgs/FindLines.h"
#include "vision_msgs/RecognizeObjects.h"
#include "vision_msgs/TrainObject.h"
#include "hardware_msgs/GetRgbd.h"
#include "boost/date_time/posix_time/posix_time.hpp"
#include <boost/thread/thread.hpp>

class JustinaVision
{
private:
    static bool is_node_set;
    //Services for getting point cloud
    static ros::ServiceClient cltGetRgbdWrtKinect;
    static ros::ServiceClient cltGetRgbdWrtRobot;
    //Services for line finding
    static ros::ServiceClient cltFindLines;
    //Services for object recognition
    static ros::ServiceClient cltRecogObjects;
    //Services for training objects
    static ros::ServiceClient cltTrainObject;
public:
    static bool setNodeHandle(ros::NodeHandle* nh);
    //Methods for line finding
    static bool findLine(float& x1, float& y1, float& z1, float& x2, float& y2, float& z2);
    //Methods for object recognition
    static bool recognizeObjects(std::vector<vision_msgs::VisionObject>& recog_objects);
    static bool recognizeObject(std::string name, geometry_msgs::Pose& centroid);
    //Methods for object training
    static bool trainObject(std::string name);
};
