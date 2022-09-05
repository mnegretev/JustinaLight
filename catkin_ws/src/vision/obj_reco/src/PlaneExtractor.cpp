#include "PlaneExtractor.h"

bool  PlaneExtractor::debug = false;
float PlaneExtractor::min_x =  0.3;
float PlaneExtractor::min_y = -2.0;
float PlaneExtractor::min_z =  0.3;
float PlaneExtractor::max_x =  2.0;
float PlaneExtractor::max_y =  2.0;
float PlaneExtractor::max_z =  2.0;
float PlaneExtractor::normals_tol = 0.8;

void PlaneExtractor::filter_by_distance(cv::Mat& cloud, cv::Mat& img, cv::Mat& filtered_cloud, cv::Mat& filtered_img)
{
    // This function is intended to keep point only in a given bounding box, e.g., to remove floot and distant walls
    // The function DOES NOT return a smaller point cloud. It returns a cloud with all non valid points set to zero. 
    cv::Mat valid_points;
    cv::inRange(cloud, cv::Scalar(PlaneExtractor::min_x, PlaneExtractor::min_y, PlaneExtractor::min_z),
                cv::Scalar(PlaneExtractor::max_x, PlaneExtractor::max_y, PlaneExtractor::max_z), valid_points);
    filtered_cloud = cloud.clone();
    filtered_img   = img.clone();
    for(size_t i=0; i<img.rows; i++)
        for(size_t j=0; j<img.cols; j++)
            if(!valid_points.data[i*img.cols + j])
            {
                filtered_cloud.at<cv::Vec3f>(i,j) = cv::Vec3f(0,0,0);
                filtered_img.at<cv::Vec3b>(i,j)   = cv::Vec3b(0,0,0);
            }
}

cv::Mat PlaneExtractor::get_horizontal_normals(cv::Mat& cloud)
{
    float mag;
    cv::Vec3f p, p_ne, p_se, p_nw, p_sw, v1, v2, normal;
    cv::Mat normals = cv::Mat::zeros(cv::Size(cloud.cols, cloud.rows), CV_32FC3);
    for(size_t i=1; i<cloud.rows-1; i++)
        for(size_t j=1; j<cloud.cols-1; j++)
        {
            p    = cloud.at<cv::Vec3f>(i,j);
            p_ne = cloud.at<cv::Vec3f>(i+1,j+1);
            p_nw = cloud.at<cv::Vec3f>(i+1,j-1);
            p_sw = cloud.at<cv::Vec3f>(i-1,j-1);
            p_se = cloud.at<cv::Vec3f>(i-1,j+1);
            if(p == cv::Vec3f(0,0,0) || p_ne == cv::Vec3f(0,0,0) || p_nw == cv::Vec3f(0,0,0) ||
               p_sw == cv::Vec3f(0,0,0) || p_se == cv::Vec3f(0,0,0))
                continue;
            normal = (p_ne - p_sw).cross(p_nw - p_se);
            mag = sqrt(normal[0]*normal[0] + normal[1]*normal[1] + normal[2]*normal[2]);
            if(mag != 0) normal /= mag;
            if(normal[2] < 0) normal = -normal;
            if(normal[2] < PlaneExtractor::normals_tol)
                continue;
            normals.at<cv::Vec3f>(i,j) = normal;
        }
    return normals;
}
