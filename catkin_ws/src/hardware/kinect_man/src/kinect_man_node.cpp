#include <opencv2/highgui/highgui.hpp>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "hardware_msgs/GetRgbd.h"
#include "tf/transform_listener.h"
#include "pcl_ros/transforms.h"

cv::Mat img_depth;
cv::Mat img_bgr  ;
tf::TransformListener* tf_listener;
int downsampling = 1;

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
        char* p = (char*)(&msg.data[16*i + 7]);
        *p ^= 0b10000000; //Multiply by -1 the Y coordinate
    }
}

void downsamle_cloud(sensor_msgs::PointCloud2& src, sensor_msgs::PointCloud2& dst, int downsampling)
{
    //This function ONLY COPIES POINT DATA. For all headers, use initialize_msg();
    void* p = (void*)&src.data[0];
    void* q = (void*)&dst.data[0];
    unsigned int k1 = 0;
    unsigned int k2 = 0;
    for(unsigned int i=0; i < dst.height; i++, k2+=src.row_step*(downsampling-1))
        for(unsigned int j=0; j < dst.width; j++, k1+=16, k2+=16*downsampling)
            memcpy((q + k1), (p + k2), 16);
}

bool callback_rgbd_wrt_kinect(hardware_msgs::GetRgbd::Request& req, hardware_msgs::GetRgbd::Response& resp)
{
    initialize_rosmsg(resp.point_cloud, 640, 480, "kinect_link");
    cvmat_2_rosmsg_buffer(img_depth, img_bgr, resp.point_cloud);
    resp.point_cloud.header.stamp = ros::Time::now();
    return true;
}

bool callback_rgbd_wrt_robot (hardware_msgs::GetRgbd::Request& req, hardware_msgs::GetRgbd::Response& resp)
{
    initialize_rosmsg(resp.point_cloud, 640, 480, "kinect_link");
    cvmat_2_rosmsg_buffer(img_depth, img_bgr, resp.point_cloud);
    pcl_ros::transformPointCloud("base_link", resp.point_cloud, resp.point_cloud, *tf_listener);
    resp.point_cloud.header.frame_id = "base_link";
    resp.point_cloud.header.stamp = ros::Time::now();
    return true;
}

bool callback_rgbd_kinect_downsampled(hardware_msgs::GetRgbd::Request& req, hardware_msgs::GetRgbd::Response& resp)
{
    sensor_msgs::PointCloud2 temp;
    initialize_rosmsg(temp, 640, 480, "kinect_link");
    cvmat_2_rosmsg_buffer(img_depth, img_bgr, temp);
    initialize_rosmsg(resp.point_cloud, 640/downsampling, 480/downsampling, "kinect_link");
    downsamle_cloud(temp, resp.point_cloud, downsampling);
    resp.point_cloud.header.stamp = ros::Time::now();
    return true;
}

bool callback_rgbd_robot_downsampled (hardware_msgs::GetRgbd::Request& req, hardware_msgs::GetRgbd::Response& resp)
{
    sensor_msgs::PointCloud2 temp;
    initialize_rosmsg(temp, 640, 480, "kinect_link");
    cvmat_2_rosmsg_buffer(img_depth, img_bgr, temp);
    initialize_rosmsg(resp.point_cloud, 640/downsampling, 480/downsampling, "kinect_link");
    downsamle_cloud(temp, resp.point_cloud, downsampling);
    pcl_ros::transformPointCloud("base_link", resp.point_cloud, resp.point_cloud, *tf_listener);
    resp.point_cloud.header.frame_id = "base_link";
    resp.point_cloud.header.stamp = ros::Time::now();
    return true;
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING KINECT MANAGER BY MARCOSOF ..." << std::endl;
    ros::init(argc, argv, "kinect_man");
    ros::NodeHandle n;
    ros::Rate loop(30);
    tf_listener = new tf::TransformListener();

    std::string video_file_name = "";
    if(ros::param::has("~downsampling"))
        ros::param::get("~downsampling", downsampling);
    if(ros::param::has("~video"))
        ros::param::get("~video", video_file_name);
    
    std::cout << "KinectMan.->Waiting for transform from kinect link to base link for 15 seconds..." << std::endl;
    tf_listener->waitForTransform("base_link", "kinect_link", ros::Time(0), ros::Duration(1.0));
    
    ros::Publisher pub_cloud_kinect = n.advertise<sensor_msgs::PointCloud2>("/hardware/kinect/rgbd_wrt_kinect",1);
    ros::Publisher pub_cloud_robot  = n.advertise<sensor_msgs::PointCloud2>("/hardware/kinect/rgbd_wrt_robot", 1);
    ros::Publisher pub_downsampled_kinect = n.advertise<sensor_msgs::PointCloud2>("/hardware/kinect/rgbd_wrt_kinect_downsampled",1);
    ros::Publisher pub_downsampled_robot  = n.advertise<sensor_msgs::PointCloud2>("/hardware/kinect/rgbd_wrt_robot_downsampled", 1);
    n.advertiseService("/hardware/kinect/get_rgbd_wrt_kinect", callback_rgbd_wrt_kinect);
    n.advertiseService("/hardware/kinect/get_rgbd_wrt_kinect_downsampled", callback_rgbd_kinect_downsampled);
    n.advertiseService("/hardware/kinect/get_rgbd_wrt_robot" , callback_rgbd_wrt_robot );                              
    n.advertiseService("/hardware/kinect/get_rgbd_wrt_robot_downsampled", callback_rgbd_robot_downsampled );
    sensor_msgs::PointCloud2 msg_cloud_kinect;
    sensor_msgs::PointCloud2 msg_cloud_kinect_downsampled;
    sensor_msgs::PointCloud2 msg_cloud_robot;           
    sensor_msgs::PointCloud2 msg_cloud_robot_downsampled;

    initialize_rosmsg(msg_cloud_kinect, 640, 480, "kinect_link");
    initialize_rosmsg(msg_cloud_kinect_downsampled, 640/downsampling, 480/downsampling, "kinect_link");
    initialize_rosmsg(msg_cloud_robot,  640, 480, "base_link");
    initialize_rosmsg(msg_cloud_robot_downsampled,  640/downsampling, 480/downsampling, "base_link"  );
    
    cv::VideoCapture capture;
    std::cout << "KinectMan.->Triying to initialize kinect sensor... " << std::endl;
    if(video_file_name == "")
        capture.open(CV_CAP_OPENNI);
    else capture.open(video_file_name);
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

        if(pub_cloud_kinect.getNumSubscribers() > 0 || pub_downsampled_kinect.getNumSubscribers() > 0 ||
           pub_cloud_robot.getNumSubscribers() > 0  || pub_downsampled_robot.getNumSubscribers() > 0)
            cvmat_2_rosmsg_buffer(img_depth, img_bgr, msg_cloud_kinect);
        
        if(pub_downsampled_kinect.getNumSubscribers() > 0 || pub_downsampled_robot.getNumSubscribers() > 0)
            downsamle_cloud(msg_cloud_kinect, msg_cloud_kinect_downsampled, downsampling);
        
        if(pub_cloud_kinect.getNumSubscribers() > 0)
            pub_cloud_kinect.publish(msg_cloud_kinect);
        if(pub_downsampled_kinect.getNumSubscribers() > 0)
            pub_downsampled_kinect.publish(msg_cloud_kinect_downsampled);
        
        if(pub_cloud_robot.getNumSubscribers() > 0)
        {
            pcl_ros::transformPointCloud("base_link", msg_cloud_kinect, msg_cloud_robot, *tf_listener);
            pub_cloud_robot.publish(msg_cloud_robot);
        }
        if(pub_downsampled_robot.getNumSubscribers() > 0)
        {
            pcl_ros::transformPointCloud("base_link", msg_cloud_kinect_downsampled, msg_cloud_robot_downsampled, *tf_listener);
            pub_downsampled_robot.publish(msg_cloud_robot_downsampled);
        }
        
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}
