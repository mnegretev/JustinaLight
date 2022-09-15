#include "geometry_msgs/Point.h"
#include "shape_msgs/Plane.h"
#include "sensor_msgs/PointCloud2.h"
#include "opencv2/opencv.hpp"
#include "Utils.h"

class PlaneExtractor
{
public:
    PlaneExtractor();
    ~PlaneExtractor();

    static float normals_tol;
    static std::vector<cv::Point> extract_horizontal_planes(cv::Mat& img_src, cv::Mat& cloud_src);
    static cv::Mat get_horizontal_normals(cv::Mat& cloud);
    static std::vector<cv::Vec4i> find_lines(cv::Mat& cloud);
    static std::vector<geometry_msgs::Point> find_table_border(sensor_msgs::PointCloud2& point_cloud_msg, tf::TransformListener* tf_listener);
};
