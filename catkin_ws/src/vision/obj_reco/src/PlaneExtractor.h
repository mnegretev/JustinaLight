#include "shape_msgs/Plane.h"
#include "opencv2/opencv.hpp"

class PlaneExtractor
{
public:
    PlaneExtractor();
    ~PlaneExtractor();

    static bool debug;
    static float min_x;
    static float min_y;
    static float min_z;
    static float max_x;
    static float max_y;
    static float max_z;
    static float normals_tol;
    static std::vector<cv::Point> extractPlane(cv::Mat& img_src, cv::Mat& cloud_src);
    static void filter_by_distance(cv::Mat& cloud, cv::Mat& img, cv::Mat& filtered_cloud, cv::Mat& filtered_img);
    static cv::Mat get_horizontal_normals(cv::Mat& cloud);
};
