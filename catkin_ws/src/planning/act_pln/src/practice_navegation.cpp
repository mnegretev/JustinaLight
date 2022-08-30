#include <stdlib.h>
#include "std_msgs/Float32.h"
#include "ros/ros.h" //ncludes all the headers to use the most common public pieces of the ROS system
//#include "tf/transfrom_listener.h"//inherits from Transformer and automatically subscribes to ROS transform messages
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseStamped.h"


geometry_msgs::PoseStamped position;
int main (int argc, char** argv)
{
    std::cout << "Moving arms and base practice"<< std::endl;
    ros::init(argc, argv, "moving_practice");
    ros::NodeHandle n("~");
    ros::Publisher pub_aleatory_p     = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);

    ros::Rate loop(0.2);
    ros::Time ros_time;
    loop.sleep();
    while(ros::ok())
    {
        position.pose.position.x = (rand()%1000)/100.0-5.0;
        position.pose.position.y = (rand()%1000)/100.0-5.0;
        float angle = (rand()%628)/100.0-3.14;
        position.pose.orientation.z = sin(angle/2);
        position.pose.orientation.w = cos(angle/2);
        std::cout <<"Moving to:"<< position.pose.position.x <<" "<<position.pose.position.y << std::endl;

        pub_aleatory_p.publish(position);
        loop.sleep();
        ros:: spinOnce();
    }
    
}
