#include "QtRosNode.h"

QtRosNode::QtRosNode()
{
    this->gui_closed = false;
    publishing_cmd_vel = false;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.x = 0;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = 0;
    la_current_q.resize(7);
    ra_current_q.resize(7);
    la_current_cartesian.resize(6);
    ra_current_cartesian.resize(6);
}

QtRosNode::~QtRosNode()
{
}

void QtRosNode::run()
{    
    ros::Rate loop(30);
    pubCmdVel     = n->advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    pubTorso      = n->advertise<std_msgs::Float64>("/torso_controller/command", 1);
    pubLaGoalQ    = n->advertise<std_msgs::Float32MultiArray>("/hardware/left_arm/goal_pose", 1);
    pubRaGoalQ    = n->advertise<std_msgs::Float32MultiArray>("/hardware/right_arm/goal_pose", 1);
    pubHdGoalQ    = n->advertise<std_msgs::Float32MultiArray>("/hardware/head/goal_pose", 1);
    pubLaGoalGrip = n->advertise<std_msgs::Float32>("/hardware/left_arm/goal_gripper", 1);
    pubRaGoalGrip = n->advertise<std_msgs::Float32>("/hardware/right_arm/goal_gripper", 1);
    subLaCurrentQ = n->subscribe("/hardware/left_arm/current_pose" , 1, &QtRosNode::callback_la_current_q, this);
    subRaCurrentQ = n->subscribe("/hardware/right_arm/current_pose", 1, &QtRosNode::callback_ra_current_q, this);
    cltLaInverseKinematics=n->serviceClient<manip_msgs::InverseKinematicsForPose>("/manipulation/la_inverse_kinematics");
    cltRaInverseKinematics=n->serviceClient<manip_msgs::InverseKinematicsForPose>("/manipulation/ra_inverse_kinematics");
    cltLaForwardKinematics=n->serviceClient<manip_msgs::ForwardKinematics>("/manipulation/la_forward_kinematics");
    cltRaForwardKinematics=n->serviceClient<manip_msgs::ForwardKinematics>("/manipulation/ra_forward_kinematics");
    cltFindLines          =n->serviceClient<vision_msgs::FindLines>       ("/vision/line_finder/find_lines_ransac");
    cltTrainObject        =n->serviceClient<vision_msgs::TrainObject>     ("/vision/obj_reco/train_object");
    cltRecogObjects       =n->serviceClient<vision_msgs::RecognizeObjects>("/vision/obj_reco/recognize_objects");
    cltRecogObject        =n->serviceClient<vision_msgs::RecognizeObject >("/vision/obj_reco/recognize_object");
    int pub_zero_counter = 5;
    while(ros::ok() && !this->gui_closed)
    {
        if(publishing_cmd_vel)
        {
            pubCmdVel.publish(cmd_vel);
            pub_zero_counter = 5;
        }
        else if(--pub_zero_counter > 0)
        {
            if(pub_zero_counter <= 0)
                pub_zero_counter = 0;
            pubCmdVel.publish(cmd_vel);
        }
        ros::spinOnce();
        emit updateGraphics();
        loop.sleep();
    }
    emit onRosNodeFinished();
}

void QtRosNode::setNodeHandle(ros::NodeHandle* nh)
{
    this->n = nh;
}

void QtRosNode::publish_cmd_vel(float linear_frontal, float linear_lateral, float angular)
{
    cmd_vel.linear.x = linear_frontal;
    cmd_vel.linear.y = linear_lateral;
    cmd_vel.angular.z = angular;
    pubCmdVel.publish(cmd_vel);
}

void QtRosNode::start_publishing_cmd_vel(float linear_frontal, float linear_lateral, float angular)
{
    cmd_vel.linear.x = linear_frontal;
    cmd_vel.linear.y = linear_lateral;
    cmd_vel.angular.z = angular;
    publishing_cmd_vel = true;
}

void QtRosNode::stop_publishing_cmd_vel()
{
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.angular.z = 0;
    publishing_cmd_vel = false;
}

void QtRosNode::get_robot_pose(float& robot_x, float& robot_y, float& robot_a)
{
    tf::StampedTransform t;
    tf::Quaternion q;
    tf_listener.waitForTransform("map", "base_link", ros::Time::now(), ros::Duration(0.5));
    tf_listener.lookupTransform("map", "base_link", ros::Time(0), t);
    robot_x = t.getOrigin().x();
    robot_y = t.getOrigin().y();
    q = t.getRotation();
    robot_a = atan2(q.z(), q.w())*2;
}

void QtRosNode::publish_torso_position(float tr)
{
    std_msgs::Float64 msg;
    msg.data = tr;
    pubTorso.publish(msg);
}

void QtRosNode::publish_la_goal_angles(float a1, float a2, float a3, float a4, float a5, float a6, float a7)
{
    std_msgs::Float32MultiArray msg;
    msg.data.resize(7);
    msg.data[0] = a1;
    msg.data[1] = a2;
    msg.data[2] = a3;
    msg.data[3] = a4;
    msg.data[4] = a5;
    msg.data[5] = a6;
    msg.data[6] = a7;
    pubLaGoalQ.publish(msg);
}

void QtRosNode::publish_ra_goal_angles(float a1, float a2, float a3, float a4, float a5, float a6, float a7)
{
    std_msgs::Float32MultiArray msg;
    msg.data.resize(7);
    msg.data[0] = a1;
    msg.data[1] = a2;
    msg.data[2] = a3;
    msg.data[3] = a4;
    msg.data[4] = a5;
    msg.data[5] = a6;
    msg.data[6] = a7;
    pubRaGoalQ.publish(msg);
}

void QtRosNode::publish_la_grip_angles(float a)
{
    std_msgs::Float32 msg;
    msg.data = a;
    pubLaGoalGrip.publish(msg);
}

void QtRosNode::publish_ra_grip_angles(float a)
{
    std_msgs::Float32 msg;
    msg.data = a;
    pubRaGoalGrip.publish(msg);
}

void QtRosNode::publish_head_angles(float pan, float tilt)
{
    std_msgs::Float32MultiArray msg;
    msg.data.resize(2);
    msg.data[0] = pan;
    msg.data[1] = tilt;
    pubHdGoalQ.publish(msg);
}

void QtRosNode::callback_la_current_q(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    la_current_q = msg->data;
    call_la_forward_kinematics(la_current_q, la_current_cartesian);
}

void QtRosNode::callback_ra_current_q(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    ra_current_q = msg->data;
    call_ra_forward_kinematics(ra_current_q, ra_current_cartesian);
}

bool QtRosNode::call_la_inverse_kinematics(std::vector<float>& cartesian, std::vector<float>& articular)
{
    manip_msgs::InverseKinematicsForPose srv;
    srv.request.x = cartesian[0];
    srv.request.y = cartesian[1];
    srv.request.z = cartesian[2];
    srv.request.roll  = cartesian[3];
    srv.request.pitch = cartesian[4];
    srv.request.yaw   = cartesian[5];
    if(!cltLaInverseKinematics.call(srv))
        return false;
    articular.clear();
    articular.push_back(srv.response.q1);
    articular.push_back(srv.response.q2);
    articular.push_back(srv.response.q3);
    articular.push_back(srv.response.q4);
    articular.push_back(srv.response.q5);
    articular.push_back(srv.response.q6);
    articular.push_back(srv.response.q7);
    return true;
}

bool QtRosNode::call_ra_inverse_kinematics(std::vector<float>& cartesian, std::vector<float>& articular)
{
    manip_msgs::InverseKinematicsForPose srv;
    srv.request.x = cartesian[0];
    srv.request.y = cartesian[1];
    srv.request.z = cartesian[2];
    srv.request.roll  = cartesian[3];
    srv.request.pitch = cartesian[4];
    srv.request.yaw   = cartesian[5];
    if(!cltRaInverseKinematics.call(srv))
        return false;
    articular.clear();
    articular.push_back(srv.response.q1);
    articular.push_back(srv.response.q2);
    articular.push_back(srv.response.q3);
    articular.push_back(srv.response.q4);
    articular.push_back(srv.response.q5);
    articular.push_back(srv.response.q6);
    articular.push_back(srv.response.q7);
    return true;
}

bool QtRosNode::call_la_forward_kinematics(std::vector<float>& articular, std::vector<float>& cartesian)
{
    cartesian.resize(6);
    for(int i=0; i<cartesian.size(); i++) cartesian[i] = 0;
    manip_msgs::ForwardKinematics srv;
    srv.request.q1 = articular[0];
    srv.request.q2 = articular[1];
    srv.request.q3 = articular[2];
    srv.request.q4 = articular[3];
    srv.request.q5 = articular[4];
    srv.request.q6 = articular[5];
    srv.request.q7 = articular[6];
    if(!cltLaForwardKinematics.call(srv))
        return false;
    cartesian[0] = srv.response.x;
    cartesian[1] = srv.response.y;
    cartesian[2] = srv.response.z;
    cartesian[3] = srv.response.roll;
    cartesian[4] = srv.response.pitch;
    cartesian[5] = srv.response.yaw;
    return true;

}

bool QtRosNode::call_ra_forward_kinematics(std::vector<float>& articular, std::vector<float>& cartesian)
{
    cartesian.resize(6);
    for(int i=0; i<cartesian.size(); i++) cartesian[i] = 0;
    manip_msgs::ForwardKinematics srv;
    srv.request.q1 = articular[0];
    srv.request.q2 = articular[1];
    srv.request.q3 = articular[2];
    srv.request.q4 = articular[3];
    srv.request.q5 = articular[4];
    srv.request.q6 = articular[5];
    srv.request.q7 = articular[6];
    if(!cltLaForwardKinematics.call(srv))
        return false;
    cartesian[0] = srv.response.x;
    cartesian[1] = srv.response.y;
    cartesian[2] = srv.response.z;
    cartesian[3] = srv.response.roll;
    cartesian[4] = srv.response.pitch;
    cartesian[5] = srv.response.yaw;
    return true;
}

bool QtRosNode::call_find_lines()
{
    vision_msgs::FindLines srv;
    boost::shared_ptr<sensor_msgs::PointCloud2 const> ptr;
    ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/hardware/kinect/rgbd_wrt_kinect", ros::Duration(1.0));
    if(ptr==NULL)
    {
        std::cout << "JustinaGUI.->Cannot get point cloud before calling train object service..." << std::endl;
        return false;
    }
    srv.request.point_cloud = *ptr;
    cltFindLines.call(srv);
}

bool QtRosNode::call_train_object(std::string name)
{
    vision_msgs::TrainObject srv;
    boost::shared_ptr<sensor_msgs::PointCloud2 const> ptr;
    ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/hardware/kinect/rgbd_wrt_kinect", ros::Duration(1.0));
    if(ptr==NULL)
    {
        std::cout << "JustinaGUI.->Cannot get point cloud before calling train object service..." << std::endl;
        return false;
    }
    srv.request.point_cloud = *ptr;
    srv.request.name = name;
    cltTrainObject.call(srv);
}

bool QtRosNode::call_recognize_objects()
{
    vision_msgs::RecognizeObjects srv;
    boost::shared_ptr<sensor_msgs::PointCloud2 const> ptr;
    ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/hardware/kinect/rgbd_wrt_kinect", ros::Duration(1.0));
    if(ptr==NULL)
    {
        std::cout << "JustinaGUI.->Cannot get point cloud before calling train object service..." << std::endl;
        return false;
    }
    srv.request.point_cloud = *ptr;
    cltRecogObjects.call(srv);
}

bool QtRosNode::call_recognize_object(std::string name)
{
    vision_msgs::RecognizeObject srv;
    boost::shared_ptr<sensor_msgs::PointCloud2 const> ptr;
    ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/hardware/kinect/rgbd_wrt_kinect", ros::Duration(1.0));
    if(ptr==NULL)
    {
        std::cout << "JustinaGUI.->Cannot get point cloud before calling train object service..." << std::endl;
        return false;
    }
    srv.request.point_cloud = *ptr;
    srv.request.name = name;
    cltRecogObject.call(srv);
}
