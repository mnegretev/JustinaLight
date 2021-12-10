#include <opencv2/highgui/highgui.hpp>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "hardware_msgs/GetRgbd.h"
#include "tf/transform_listener.h"
#include "tf_conversions/tf_eigen.h"

cv::Mat img_depth;
cv::Mat img_bgr  ;
tf::TransformListener* tf_listener;

void initialize_rosmsg(sensor_msgs::PointCloud2& msg, int width, int height, std::string frame_id)
{
    msg.header.frame_id = frame_id;
    msg.width  = width;
    msg.height = height;
    msg.is_bigendian = false;
    msg.point_step   = 16;
    msg.row_step     = 16 * msg.width;
    msg.is_dense     = false;
    sensor_msgs::PointField f;
    f.name     = "x";
    f.offset   = 0;
    f.datatype = 7;
    f.count    = 1;
    msg.fields.push_back(f);
    f.name     = "y";
    f.offset   = 4;
    msg.fields.push_back(f);
    f.name     = "z";
    f.offset   = 8;
    msg.fields.push_back(f);
    f.name     = "rgba";
    f.offset   = 12;
    f.datatype = 6;
    msg.fields.push_back(f);
    msg.data.resize(msg.row_step * msg.height);
}

void cvmat_2_rosmsg_buffer(cv::Mat& depth, cv::Mat& bgr, sensor_msgs::PointCloud2& msg)
{
    //This function ONLY COPIES POINT DATA. For all headers, use initialize_msg();
    for(size_t i=0; i < bgr.rows*bgr.cols; i++)
    {
        memcpy(&msg.data[i*16], &depth.data[12*i], 12);
        memcpy(&msg.data[i*16 + 12], &bgr.data[3*i], 3);
        msg.data[16*i + 15] = 255;
        unsigned char* p = (unsigned char*)(&msg.data[16*i + 7]);
        *p ^= 0b10000000; //Multiply by -1 the Y coordinate
    }
}

bool get_transform_to_baselink(Eigen::Affine3d& e)
{
    tf::StampedTransform tf;
    try{
        tf_listener->lookupTransform("base_link", "kinect_link", ros::Time(0), tf);
    }
    catch(...){
        e = Eigen::Affine3d::Identity();
        return false;
    }
    tf::transformTFToEigen(tf, e);
    return true;
}

void downsamle_cloud(sensor_msgs::PointCloud2& src, sensor_msgs::PointCloud2& dst, int downsampling)
{
    //This function ONLY COPIES POINT DATA. For all headers, use initialize_msg();
    for(int i=0; i < dst.width; i++)
        for(int j=0; j < dst.height; j++)
            memcpy(&dst.data[16*(j*dst.width + i)], &src.data[16 * downsampling *(j*src.width + i)], 16);
}

void transform_cloud_to_base_link(sensor_msgs::PointCloud2& src, sensor_msgs::PointCloud2& dst)
{
    Eigen::Affine3d cam_to_robot;
    get_transform_to_baselink(cam_to_robot);
    unsigned char* p = (unsigned char*)(&src.data[0]);
    unsigned char* q = (unsigned char*)(&dst.data[0]);
    for(size_t i=0; i < src.width*src.height; i++)
    {
        Eigen::Vector3d v(*((float*)(p)), *((float*)(p+4)), *((float*)(p+8)));
        v = cam_to_robot * v;
        *(q    ) = v.x();
        *(q + 4) = v.y();
        *(q + 8) = v.z();
        p += src.point_step;
        q += dst.point_step;
    }
}

bool callback_rgbd_wrt_kinect(hardware_msgs::GetRgbd::Request& req, hardware_msgs::GetRgbd::Response& resp)
{
}

bool callback_rgbd_wrt_robot (hardware_msgs::GetRgbd::Request& req, hardware_msgs::GetRgbd::Response& resp)
{
}

bool callback_rgbd_kinect_downsampled(hardware_msgs::GetRgbd::Request& req, hardware_msgs::GetRgbd::Response& resp)
{
}

bool callback_rgbd_robot_downsampled (hardware_msgs::GetRgbd::Request& req, hardware_msgs::GetRgbd::Response& resp)
{
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING KINECT MANAGER BY MARCOSOF ..." << std::endl;
    ros::init(argc, argv, "kinect_man");
    ros::NodeHandle n;
    ros::Rate loop(30);
    tf_listener = new tf::TransformListener();

    int  downsampling = 1;
    if(ros::param::has("~downsampling"))
        ros::param::get("~downsampling", downsampling);
    
    std::cout << "KinectMan.->Waiting for transform from kinect link to base link for 15 seconds..." << std::endl;
    tf_listener->waitForTransform("base_link", "kinect_link", ros::Time(0), ros::Duration(15.0));
    Eigen::Affine3d tf_kinect_to_base;
    if(!get_transform_to_baselink(tf_kinect_to_base))
        std::cout << "KinectMan.->CANNOT FIND TRANSFORM FROM KINECT LINK TO BASE LINK. TRANSFORMING CLOUD WITH IDENTITY MATRIX!!!!" << std::endl;
    
    ros::Publisher pubKinectFrame = n.advertise<sensor_msgs::PointCloud2>("/hardware/kinect/rgbd_wrt_kinect",1);
    ros::Publisher pubRobotFrame  = n.advertise<sensor_msgs::PointCloud2>("/hardware/kinect/rgbd_wrt_robot", 1);
    ros::Publisher pubDownsampledKinect = n.advertise<sensor_msgs::PointCloud2>("/hardware/kinect/rgbd_wrt_kinect_downsampled",1);
    ros::Publisher pubDownsampledRobot  = n.advertise<sensor_msgs::PointCloud2>("/hardware/kinect/rgbd_wrt_robot_downsampled", 1);
    ros::ServiceServer srvRgbdKinect = n.advertiseService("/hardware/kinect/get_rgbd_wrt_kinect", callback_rgbd_wrt_kinect);
    ros::ServiceServer srvRgbdKinectDownsampled = n.advertiseService("/hardware/kinect/get_rgbd_wrt_kinect_downsampled", callback_rgbd_kinect_downsampled);
    ros::ServiceServer srvRgbdRobot = n.advertiseService("/hardware/kinect/get_rgbd_wrt_robot" , callback_rgbd_wrt_robot );                              
    ros::ServiceServer srvRgbdRobotDownsampled = n.advertiseService("/hardware/kinect/get_rgbd_wrt_robot_downsampled", callback_rgbd_robot_downsampled );
    sensor_msgs::PointCloud2 msgCloudKinect;
    sensor_msgs::PointCloud2 msgCloudKinectDownsampled;
    sensor_msgs::PointCloud2 msgCloudRobot;           
    sensor_msgs::PointCloud2 msgCloudRobotDownsampled;

    initialize_rosmsg(msgCloudKinect, 640, 480, "kinect_link");
    initialize_rosmsg(msgCloudKinectDownsampled, 640/downsampling, 480/downsampling, "kinect_link");
    initialize_rosmsg(msgCloudRobot,  640, 480, "base_link");
    initialize_rosmsg(msgCloudRobotDownsampled,  640/downsampling, 480/downsampling, "base_link"  );
    
    cv::VideoCapture capture;
    std::cout << "KinectMan.->Triying to initialize kinect sensor... " << std::endl;
    capture.open(CV_CAP_OPENNI);
    if(!capture.isOpened())
    {
        std::cout << "KinectMan.->Cannot open kinect :'(" << std::endl;
        return -1;
    }
    capture.set(CV_CAP_OPENNI_DEPTH_GENERATOR_REGISTRATION, CV_CAP_OPENNI_DEPTH_GENERATOR_REGISTRATION_ON);
    std::cout << "KinectMan.->Kinect sensor started :D" << std::endl;

    while(ros::ok())
    {
        if(!capture.grab())
        {
            loop.sleep();
            continue;
        }
        
        capture.retrieve(img_depth, CV_CAP_OPENNI_POINT_CLOUD_MAP);
        capture.retrieve(img_bgr  , CV_CAP_OPENNI_BGR_IMAGE);
        if(pubKinectFrame.getNumSubscribers() > 0 || pubDownsampledKinect.getNumSubscribers() > 0 ||
           pubRobotFrame.getNumSubscribers() > 0  || pubDownsampledRobot.getNumSubscribers() > 0)
            cvmat_2_rosmsg_buffer(img_depth, img_bgr, msgCloudKinect);
        if(pubDownsampledKinect.getNumSubscribers() > 0 || pubDownsampledRobot.getNumSubscribers() > 0)
            downsamle_cloud(msgCloudKinect, msgCloudKinectDownsampled, downsampling);
        
        if(pubKinectFrame.getNumSubscribers() > 0)
            pubKinectFrame.publish(msgCloudKinect);
        if(pubDownsampledKinect.getNumSubscribers() > 0)
            pubDownsampledKinect.publish(msgCloudKinectDownsampled);
        
        if(pubRobotFrame.getNumSubscribers() > 0)
        {
            transform_cloud_to_base_link(msgCloudKinect, msgCloudRobot);
            pubRobotFrame.publish(msgCloudRobot);
        }
        if(pubDownsampledRobot.getNumSubscribers() > 0)
        {
            transform_cloud_to_base_link(msgCloudKinectDownsampled, msgCloudRobotDownsampled);
            pubDownsampledRobot.publish(msgCloudRobotDownsampled);
        }
        
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}
