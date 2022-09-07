#include "Utils.h"

bool  Utils::debug = false;
float Utils::min_x =  0.3;
float Utils::min_y = -2.0;
float Utils::min_z =  0.3;
float Utils::max_x =  2.0;
float Utils::max_y =  2.0;
float Utils::max_z =  2.0;

void Utils::transform_cloud_wrt_base(sensor_msgs::PointCloud2& cloud, cv::Mat& bgr_dest, cv::Mat& cloud_dest,
                                     tf::TransformListener* tf_listener)
{
    std::cout <<"ObjReco.->Point cloud frame: " << cloud.header.frame_id << std::endl;
    if(cloud.header.frame_id != "base_link")
    {
        std::cout << "ObjReco.->Transforming point cloud to robot reference" << std::endl;
        pcl_ros::transformPointCloud("base_link", cloud, cloud, *tf_listener);
    }
    JustinaTools::PointCloud2Msg_ToCvMat(cloud, bgr_dest, cloud_dest);
    Utils::filter_by_distance(cloud_dest, bgr_dest, cloud_dest, bgr_dest);
}


void Utils::filter_by_distance(cv::Mat& cloud, cv::Mat& img, cv::Mat& filtered_cloud, cv::Mat& filtered_img)
{
    // This function is intended to keep point only in a given bounding box, e.g., to remove floot and distant walls
    // The function DOES NOT return a smaller point cloud. It returns a cloud with all non valid points set to zero. 
    cv::Mat valid_points;
    cv::inRange(cloud, cv::Scalar(Utils::min_x, Utils::min_y, Utils::min_z),
                cv::Scalar(Utils::max_x, Utils::max_y, Utils::max_z), valid_points);
    filtered_cloud = cloud.clone();
    filtered_img   = img.clone();
    for(size_t i=0; i<img.rows; i++)
        for(size_t j=0; j<img.cols; j++)
            if(!valid_points.data[i*img.cols + j])
            {
                filtered_cloud.at<cv::Vec3f>(i,j) = cv::Vec3f(0,0,0);
                filtered_img.at<cv::Vec3b>(i,j)   = cv::Vec3b(0,0,0);
            }
    cv::imshow("Filtered Img", img);
    cv::imshow("Filtered Cloud", cloud);
}
