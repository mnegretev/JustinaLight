#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <boost/foreach.hpp>
#include "hardware_msgs/GetRgb.h"
#define foreach BOOST_FOREACH

cv::Mat frame;
//ros::Publisher pubImageIROS;
bool gazebo;
bool first_image;
sensor_msgs::Image image_gazebo;


bool callbackGetRGB(hardware_msgs::GetRgb::Request &req, hardware_msgs::GetRgb::Response &res)
{
    if(!gazebo){
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        res.imageBGR = *msg;
    }
    else{
        res.imageBGR = image_gazebo;
    }
    //cv::imshow("Web cam", frame);
    //pubImageIROS.publish(*msg);
    return true;
}

void callbackGazeboImage(const sensor_msgs::Image::ConstPtr& msg){
    image_gazebo = *msg;
    first_image=true;
}

int main(int argc, char** argv)
{
    std::string file_name = "";
    std::cout << "INITIALIZING WEBCAM MANAGER BY REY ..." << std::endl;
    ros::init(argc, argv, "webcam_man");
    ros::NodeHandle n;
    ros::Rate rate(30);

    std::string device = "/dev/justinaWebCam";
    int width = 640;
    int height = 480;
    gazebo = false;
    first_image = false;

    if(ros::param::has("~device"))
        ros::param::get("~device", device);
    if(ros::param::has("~width"))
        ros::param::get("~width", width);
    if(ros::param::has("~height"))
        ros::param::get("~height", height);
    if(ros::param::has("~gazebo"))
        ros::param::get("~gazebo", gazebo);


    image_transport::ImageTransport it(n);
    image_transport::Publisher pubImage = it.advertise("/hardware/webcam_man/image_raw", 1);
    //image_transport::Publisher pubUsb = it.advertise("/usb_cam/image_raw", 1);
    ros::ServiceServer servImage = n.advertiseService("/hardware/webcam_man/image_raw", callbackGetRGB);
    ros::Subscriber subKinectFrame = n.subscribe("/camera/color/image_raw", 1, callbackGazeboImage);
    //pubImageIROS = n.advertise<sensor_msgs::Image>("/erlc/rgb_2/image", 1);
    
    if(!gazebo){
        cv::VideoCapture capture(device);
        capture.set(cv::CAP_PROP_FRAME_WIDTH, width);
        capture.set(cv::CAP_PROP_FRAME_HEIGHT, height);
        if(!capture.isOpened())
        {
            std::cout << "WebCam.->Cannot open webcam :'( on device " << device << std::endl;
            return 1;
        }
        std::cout << "WebCam.->Webcam sensor started :D" << std::endl;

        
        while(ros::ok() && cv::waitKey(1) != 'q')
        {
            capture.read(frame);
            
            if(pubImage.getNumSubscribers() > 0)
            {
                sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
                pubImage.publish(msg);
            }
            
            //cv::imshow("Web cam", frame);
            
            ros::spinOnce();
            rate.sleep();
        }
    }
    else{
        std::cout << "KinectMan.->Gazebo Webcam sensor started :D" << std::endl;
        while(ros::ok())
        {
            if(first_image)
                pubImage.publish(image_gazebo);
            ros::spinOnce();
            rate.sleep();
        }
    }
    return 1;
}
