#pragma once
#include <iostream>
#include <vector>
#include <cmath>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include "ros/ros.h"
#include <ros/package.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/PointCloud2.h"
#include "hardware_msgs/GetRgbd.h"
#include "pcl_conversions/pcl_conversions.h"
#include "tf/transform_listener.h"
#include "tf/transform_datatypes.h"
#include "tf_conversions/tf_eigen.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Image.h"
#include <sstream>
#include <string>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

class JustinaTools
{
private:
    static bool is_node_set;
    static tf::TransformListener* tf_listener;
    static std::string pathDeviceScript;
    
public:
    static void initialize();
    static void laserScanToPclWrtRobot(const sensor_msgs::LaserScan::ConstPtr& scans, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    static void PointCloud2Msg_ToCvMat(sensor_msgs::PointCloud2& pc_msg, cv::Mat& bgr_dest, cv::Mat& pc_dest);
    static void PointCloud2Msg_ToCvMat(const sensor_msgs::PointCloud2::ConstPtr& pc_msg, cv::Mat& bgr_dest, cv::Mat& pc_dest);
    static bool transformPoint(std::string src_frame, float inX, float inY, float inZ, std::string dest_frame, float& outX, float& outY, float& outZ);
    static bool transformPose(std::string src_frame, float inX, float inY, float inZ, float inRoll, float inPitch, float inYaw,
                              std::string dest_frame, float& outX, float& outY, float& outZ, float& outRoll, float& outPitch, float& outYaw);
    static bool transformPose(std::string src_frame, std::vector<float>& xyz_rpy_in, std::string dest_frame, std::vector<float>& xyz_rpy_out);
    static std::string startRecordSpeach(std::string competition, std::string test);
    static void stopRecordSpeach();
    static void removeAudioRecord(std::string path);
};
