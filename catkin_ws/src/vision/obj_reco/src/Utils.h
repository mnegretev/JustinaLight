#include "opencv2/opencv.hpp"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_ros/transforms.h"
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
};
