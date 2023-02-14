#include "ros/ros.h"
#include "pcl_ros/transforms.h"
#include "vision_msgs/FindLines.h"
#include "justina_tools/JustinaTools.h"


tf::TransformListener* tf_listener;
bool debug_mode = true;

cv::Mat calculate_normals(cv::Mat pointCloud, cv::Mat mask)
{
    cv::Mat normals = cv::Mat::zeros(pointCloud.rows, pointCloud.cols, CV_32FC3);
    if( !mask.data )
        mask = cv::Mat::ones(pointCloud.rows, pointCloud.cols, CV_8UC1);
    
    cv::Point3f pointi;
    cv::Point3f topLeft;
    cv::Point3f topRight;
    cv::Point3f downLeft;
    cv::Point3f downRight;
    
    cv::Point3f normal_1;
    cv::Point3f normal_2;
    cv::Point3f normal;

    cv::Point3f viewPoint(0.0,0.0,0.0);
    
    for( int idxRow = 1 ; idxRow < pointCloud.rows - 1 ; idxRow++ )
    {
        for( int idxCol = 1 ; idxCol < pointCloud.cols - 1 ; idxCol++ )
        {
            //if( mask.at<uchar>( idxRow,idxCol ) == 0.0 )                                                              
            //continue;                                                                                         
            
            // Getting Vectors                                                                                          
            pointi = pointCloud.at<cv::Vec3f>(idxRow, idxCol);
            
            topLeft = pointCloud.at<cv::Vec3f>(idxRow-1, idxCol-1);
            topRight = pointCloud.at<cv::Vec3f>(idxRow-1, idxCol+1);
            downLeft = pointCloud.at<cv::Vec3f>(idxRow+1, idxCol-1);
            downRight = pointCloud.at<cv::Vec3f>(idxRow+1, idxCol+1);
            
            if( topLeft.x == 0.0 && topLeft.y == 0.0 && topLeft.z == 0.0 )
                continue;
            if( topRight.x == 0.0 && topRight.y == 0.0 && topRight.z == 0.0 )
                continue;
            if( downLeft.x == 0.0 && downLeft.y == 0.0 && downLeft.z == 0.0 )
                continue;
            if( downRight.x == 0.0 && downRight.y == 0.0 && downRight.z == 0.0 )
                continue;
            
            // Normals                                                                                                  
            normal_1 = topRight - downLeft;
            normal_2 = topLeft - downRight;

            // Normal by cross product (v x h)                                                                          
            normal  = normal_2.cross(normal_1);
            
            //consider the another normal due to the first one it's negative                                            
            if(normal.z <  0)
                normal = normal_1.cross(normal_2);
            
            
            // Make normal unitary and assignin to mat                                                                  
            float norm = sqrt(normal.x*normal.x + normal.y*normal.y + normal.z*normal.z);
            if(norm != 0.0f)
            {
                // Proyecting all normals over XY plane                                                             
                //if(normal.dot(viewPoint-pointi)<0)                                                                
                //      normal *= -1;                                                                               
                if( normal.z < 0 )
                    normal *= -1;
                
                normals.at<cv::Vec3f>(idxRow, idxCol) = ( 1.0f / norm ) * normal;
            }
        }
    }
    return normals;
}

cv::Vec4i find_line(cv::Mat xyz_cloud)
{
    // PARAMS: Valid Points                                                                                                     
    double floorDistRemoval = 0.15;
    // PARAMS: Normals Extraction                                                                                               
    int blurSize = 5;
    double normalZThreshold = 0.8;
    // PARAMS: Planes RANSAC                                                                                                    
    double maxDistToPlane = 0.02;
    int maxIterations = 1000;
    int minPointsForPlane = xyz_cloud.rows*xyz_cloud.cols*0.05;
    // PARAMS: Object Extracction                                                                                               
    double minObjDistToPlane = maxDistToPlane;
    double maxObjDistToPlane = 0.25;
    double minDistToContour = 0.02;
    double maxDistBetweenObjs = 0.05;
    
    // For removing floor and far far away points                                                                               
    cv::Mat validPointCloud;
    cv::inRange(xyz_cloud, cv::Scalar(-1.0, -1.0, floorDistRemoval), cv::Scalar(1.5, 1.0, 2.0), validPointCloud);
    
    // Getting Normals                                                                                                          
    cv::Mat pointCloudBlur;
    cv::blur(xyz_cloud, pointCloudBlur, cv::Size(blurSize, blurSize));
    cv::Mat normals = calculate_normals(pointCloudBlur, cv::Mat());
    
    // Getting Mask of Normals pointing horizonaliy                                                                             
    cv::Mat  horizontalNormals;
    cv::inRange( normals, cv::Scalar(-1.0, -1.0, normalZThreshold), cv::Scalar(1.0,1.0, 1.0), horizontalNormals);
    
    // Mask of horizontal normals and valid.                                                                                    
    cv::Mat horizontalsValidPoints = horizontalNormals & validPointCloud;
}

bool callback_find_lines(vision_msgs::FindLines::Request& req, vision_msgs::FindLines::Response& resp)
{
    std::cout << "EXECUTING srvFindLines (Yisus Version)" << std::endl;
    if(req.point_cloud.header.frame_id != "base_link")
        pcl_ros::transformPointCloud("base_link", req.point_cloud, req.point_cloud, *tf_listener);
    cv::Mat bgrImg;
    cv::Mat xyzCloud;
    JustinaTools::PointCloud2Msg_ToCvMat(req.point_cloud, bgrImg, xyzCloud);
    cv::Vec4i find_line(xyzCloud);
    
    return true;
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING LINE FINDER BY MR. YISUS (improved by Marcosoft)" << std::endl;
    ros::init(argc, argv, "obj_reco_node");
    ros::NodeHandle n;
    ros::ServiceServer srvFindLines = n.advertiseService("/vision/line_finder/find_lines_ransac", callback_find_lines);
    ros::Rate loop(30);
    tf_listener = new tf::TransformListener();

    if(ros::param::has("~debug"))
        ros::param::get("~debug", debug_mode);

    while(ros::ok())
    {
        ros::spinOnce();
        loop.sleep();
    }
}
