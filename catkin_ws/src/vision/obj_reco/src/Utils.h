#pragma once
#include "opencv2/opencv.hpp"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_ros/transforms.h"
#include "visualization_msgs/Marker.h"
#include "justina_tools/JustinaTools.h"

class Utils
{
public:
    Utils();
    ~Utils();

    static bool debug;
    static float min_x;
    static float min_y;
    static float min_z;
    static float max_x;
    static float max_y;
    static float max_z;
    static void filter_by_distance(cv::Mat& cloud, cv::Mat& img, cv::Mat& filtered_cloud, cv::Mat& filtered_img);
    static void transform_cloud_wrt_base(sensor_msgs::PointCloud2& , cv::Mat& , cv::Mat& , tf::TransformListener* );
    static float dist_point_to_segment(float px, float py, float pz, float x1, float y1, float z1, float x2, float y2, float z2);
    static float dist_point_to_segment(float px, float py, float x1, float y1, float x2, float y2);
    static visualization_msgs::Marker get_lines_marker(std::vector<geometry_msgs::Point> lines);
};
