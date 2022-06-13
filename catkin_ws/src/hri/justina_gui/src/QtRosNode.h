#pragma once
#include <iostream>
#include <cmath>
#include <QThread>
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/GetPlan.h"
#include "sensor_msgs/PointCloud2.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "tf/transform_listener.h"
#include "manip_msgs/InverseKinematicsForPose.h"
#include "manip_msgs/InverseKinematics.h"
#include "manip_msgs/ForwardKinematics.h"
#include "vision_msgs/FindLines.h"
#include "vision_msgs/TrainObject.h"
#include "vision_msgs/RecognizeObjects.h"
#include "vision_msgs/RecognizeObject.h"

class QtRosNode : public QThread
{
Q_OBJECT
public:
    QtRosNode();
    ~QtRosNode();

    ros::NodeHandle* n;
    ros::Publisher pubCmdVel;
    ros::Publisher pubTorso;
    ros::Publisher pubLaGoalQ;
    ros::Publisher pubRaGoalQ;
    ros::Publisher pubHdGoalQ;
    ros::Publisher pubLaGoalGrip;
    ros::Publisher pubRaGoalGrip;
    ros::Subscriber subLaCurrentQ;
    ros::Subscriber subRaCurrentQ;
    ros::ServiceClient cltLaInverseKinematics;
    ros::ServiceClient cltRaInverseKinematics;
    ros::ServiceClient cltLaForwardKinematics;
    ros::ServiceClient cltRaForwardKinematics;
    ros::ServiceClient cltFindLines;
    ros::ServiceClient cltTrainObject;
    ros::ServiceClient cltRecogObjects;
    ros::ServiceClient cltRecogObject;
    tf::TransformListener tf_listener;
    
    geometry_msgs::Twist cmd_vel;
    bool publishing_cmd_vel;
    bool gui_closed;
    std::vector<float> la_current_q;
    std::vector<float> ra_current_q;
    std::vector<float> la_current_cartesian;
    std::vector<float> ra_current_cartesian;
    
    void run();
    void setNodeHandle(ros::NodeHandle* nh);

    void publish_cmd_vel(float linear_frontal, float linear_lateral, float angular);
    void start_publishing_cmd_vel(float linear_frontal, float linear_lateral, float angular);
    void stop_publishing_cmd_vel();
    void get_robot_pose(float& robot_x, float& robot_y, float& robot_a);

    void publish_torso_position(float tr);
    void publish_la_goal_angles(float a1, float a2, float a3, float a4, float a5, float a6, float a7);
    void publish_ra_goal_angles(float a1, float a2, float a3, float a4, float a5, float a6, float a7);
    void publish_la_grip_angles(float a);
    void publish_ra_grip_angles(float a);
    void publish_head_angles(float pan, float tilt);
    void callback_la_current_q(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void callback_ra_current_q(const std_msgs::Float32MultiArray::ConstPtr& msg);
    bool call_la_inverse_kinematics(std::vector<float>& cartesian, std::vector<float>& articular);
    bool call_ra_inverse_kinematics(std::vector<float>& cartesian, std::vector<float>& articular);
    bool call_la_forward_kinematics(std::vector<float>& articular, std::vector<float>& cartesian);
    bool call_ra_forward_kinematics(std::vector<float>& articular, std::vector<float>& cartesian);

    bool call_find_lines();
    bool call_train_object(std::string name);
    bool call_recognize_objects();
    bool call_recognize_object(std::string name);
signals:
    void updateGraphics();
    void onRosNodeFinished();
    
};
