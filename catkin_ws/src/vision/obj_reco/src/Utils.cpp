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
    cv::imshow("Filtered Img", filtered_img);
    cv::imshow("Filtered Cloud", filtered_cloud);
}

float Utils::dist_point_to_segment(float px, float py, float pz, float x1, float y1, float z1, float x2, float y2, float z2)
{
    float ax = px - x1;
    float ay = py - y1;
    float az = pz - z1;
    float bx = x2 - x1;
    float by = y2 - y1;
    float bz = z2 - z1;
    float bm = sqrt(bx*bx + by*by + bz*bz);
    if(bm == 0) return sqrt(ax*ax + ay*ay + az*az);
    bx /= bm;
    by /= bm;
    bz /= bm;
    float projection = ax*bx + ay*by + az*bz;
    if(projection < 0) return sqrt(ax*ax + ay*ay + az*az);
    if(projection > 1) return sqrt((px-x2)*(px-x2) + (py-y2)*(py-y2) + (pz-z2)*(pz-z2));
    return sqrt(ax*ax + ay*ay + az*az - projection*projection);
}

float Utils::dist_point_to_segment(float px, float py, float x1, float y1, float x2, float y2)
{
    float ax = px - x1;
    float ay = py - y1;
    float bx = x2 - x1;
    float by = y2 - y1;
    float bm = sqrt(bx*bx + by*by);
    if(bm == 0) return sqrt(ax*ax + ay*ay);
    bx /= bm;
    by /= bm;
    float projection = ax*bx + ay*by;
    if(projection < 0) return sqrt(ax*ax + ay*ay);
    if(projection > 1) return sqrt((px-x2)*(px-x2) + (py-y2)*(py-y2));
    return sqrt(ax*ax + ay*ay - projection);
}

visualization_msgs::Marker Utils::get_lines_marker(std::vector<geometry_msgs::Point> lines)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "obj_reco_markers";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 0.6;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration(10.0);
    geometry_msgs::Point p1, p2;
    for(int i=0; i<lines.size(); i+=2)
    {
        marker.points.push_back(lines[i]);
        marker.points.push_back(lines[i+1]);
    }
    return marker;
}
