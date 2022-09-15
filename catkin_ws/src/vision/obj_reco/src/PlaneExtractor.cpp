#include "PlaneExtractor.h"
#include "Utils.h"

float PlaneExtractor::normals_tol = 0.8;

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
    cv::imshow("Normals", normals);
    return normals;
}

std::vector<cv::Vec4i> PlaneExtractor::find_lines(cv::Mat& cloud)
{
    std::vector<cv::Vec4i> lines;
    cv::Mat grayscale_cloud, borders;
    cv::cvtColor(cloud, grayscale_cloud, cv::COLOR_BGR2GRAY);
    grayscale_cloud.convertTo(grayscale_cloud, CV_8UC1, 255);
    cv::imshow("Gray normals", grayscale_cloud);
    cv::Canny(grayscale_cloud, borders, 30, 100, 3);
    cv::imshow("Canny", borders);
    cv::HoughLinesP(borders, lines, 1, CV_PI/180, 30, 30, 10);
    return lines;
}

std::vector<geometry_msgs::Point> PlaneExtractor::find_table_border(sensor_msgs::PointCloud2& point_cloud_msg, tf::TransformListener* tf_listener)
{
    std::vector<geometry_msgs::Point> table_border;
    cv::Mat img, cloud;
    Utils::transform_cloud_wrt_base(point_cloud_msg, img, cloud, tf_listener);
    Utils::filter_by_distance(cloud, img, cloud, img);
    cv::Mat normals = PlaneExtractor::get_horizontal_normals(cloud);
    std::vector<cv::Vec4i> lines = PlaneExtractor::find_lines(normals);
    
    float min_dist = 1000;
    cv::Vec4i nearest_line;
    cv::Vec3f p1;
    cv::Vec3f p2;
    for(int i=0; i<lines.size(); i++)
    {
        p1 = cloud.at<cv::Vec3f>(lines[i][1], lines[i][0]);
        p2 = cloud.at<cv::Vec3f>(lines[i][3], lines[i][2]);
        if(fabs(p1[2] - p2[2]) > 0.08 || p1[0] == 0 || p1[1] == 0 || p1[2] == 0 ||
            p2[0] == 0 || p2[1] == 0 || p2[2] == 0)
            continue;
        float d = Utils::dist_point_to_segment(0, 0, 0, p1[0], p1[1], p1[2], p2[0], p2[1], p2[2]);
        std::cout << "Line " << i << ": " << p1 << "    " << p2 << "   distance: " << d << std::endl;
        if(d < min_dist)
        {
            min_dist = d;
            nearest_line = lines[i];
        }
        cv::line(cloud, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0, 10*i/255.0,(255 - 10*i)/255.0), 3, 8);
        cv::putText(cloud, std::to_string(i), cv::Point(lines[i][0], lines[i][1]), cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255,255,255), 2);
    }
    cv::imshow("Lines", cloud);
    p1 = cloud.at<cv::Vec3f>(nearest_line[1], nearest_line[0]);
    p2 = cloud.at<cv::Vec3f>(nearest_line[3], nearest_line[2]);
    geometry_msgs::Point msg_p1, msg_p2;
    msg_p1.x = p1[0];
    msg_p1.y = p1[1];
    msg_p1.z = p1[2];
    msg_p2.x = p2[0];
    msg_p2.y = p2[1];
    msg_p2.z = p2[2];
    table_border.push_back(msg_p1);
    table_border.push_back(msg_p2);
    cv::line(img, cv::Point(nearest_line[0], nearest_line[1]), cv::Point(nearest_line[2], nearest_line[3]), cv::Scalar(255,0,0), 3, 8);
    cv::imshow("Found border", img);
    return table_border;
}
