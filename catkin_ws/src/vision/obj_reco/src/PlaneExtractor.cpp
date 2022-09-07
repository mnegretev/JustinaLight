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
    for(size_t i=0; i<lines.size(); i++)
        cv::line(cloud, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), cv::Scalar(255,0,0), 3, 8);
    cv::imshow("Lines", cloud);
    return lines;
}
