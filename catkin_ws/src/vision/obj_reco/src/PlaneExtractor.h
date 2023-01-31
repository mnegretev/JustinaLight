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
    static int   canny_threshold1;
    static int   canny_threshold2;
    static int   canny_window_size;
    static int   hough_rho;
    static float hough_theta;
    static int   hough_threshold;
    static int   hough_min_lines_length;
    static int   hough_max_lines_gap;
    
    static std::vector<cv::Point> extract_horizontal_planes(sensor_msgs::PointCloud2& point_cloud_msg, tf::TransformListener* tf_listener);
    static cv::Mat get_horizontal_normals(cv::Mat& cloud);
    static std::vector<cv::Vec4i> find_horizontal_lines(cv::Mat& cloud);
    static cv::Vec4i find_nearest_horizontal_line(std::vector<cv::Vec4i>& lines, cv::Mat& cloud);
    static std::vector<geometry_msgs::Point> find_table_border(sensor_msgs::PointCloud2& point_cloud_msg, tf::TransformListener* tf_listener);
};
