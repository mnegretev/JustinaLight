#include <stdlib.h>
#include "std_msgs/Float32.h"
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseStamped.h"
#include "justina_tools/JustinaNavigation.h"


geometry_msgs::PoseStamped position;
int main (int argc, char** argv)
{
    std::cout << "Moving arms and base practice"<< std::endl;
    ros::init(argc, argv, "moving_practice");
    ros::NodeHandle n("~");
    JustinaNavigation::setNodeHandle(&n);
    
    ros::Rate loop(10);
    loop.sleep();
    while(ros::ok())
    {
        float goal_x = (rand()%1000)/100.0;
        float goal_y = (rand()%1000)/100.0-1.0;
        float goal_a = (rand()%628)/100.0-3.14;
        std::cout <<"Moving to:"<< goal_x <<" "<< goal_y << " " << goal_a << std::endl;
        if(JustinaNavigation::getClose(goal_x, goal_y, goal_a, 30000))
            std::cout << "Global goal reached" << std::endl;
        else
            std::cout << "Cannnot navigate to goal pose" << std::endl;
        std::cout << "Moving backwards" << std::endl;
        JustinaNavigation::moveDist(-0.3, 10000);
        std::cout << "Moving forward" << std::endl;
        JustinaNavigation::moveDist(0.3, 10000);
        std::cout << "Moving left" << std::endl;
        JustinaNavigation::moveLateral(0.2, 10000);
        std::cout << "Moving right" << std::endl;
        JustinaNavigation::moveLateral(-0.2, 10000);
        loop.sleep();
        ros:: spinOnce();
    }
    
}
