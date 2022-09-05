#include "justina_tools/JustinaNavigation.h"

bool JustinaNavigation::is_node_set;
actionlib_msgs::GoalStatus JustinaNavigation::_navigation_status;
actionlib_msgs::GoalStatus JustinaNavigation::_simple_move_status;
bool JustinaNavigation::_stop;

//Subscribers for stop signals
ros::Subscriber JustinaNavigation::subStop;
ros::Subscriber JustinaNavigation::subNavigationStop;
//Subscribers for checking goal-pose-reached signal
ros::Subscriber JustinaNavigation::subNavigationStatus;
ros::Subscriber JustinaNavigation::subSimpleMoveStatus;
//Publishers and subscribers for operating the simple_move node
ros::Publisher JustinaNavigation::pubSimpleMoveDist;
ros::Publisher JustinaNavigation::pubSimpleMoveDistAngle;
ros::Publisher JustinaNavigation::pubSimpleMoveLateral;
//Publishers and subscribers for mvn_pln
ros::Publisher JustinaNavigation::pubMvnPlnGetCloseXYA;
ros::Publisher JustinaNavigation::pubNavigationStop;
//Publishers and subscribers for localization
tf::TransformListener* JustinaNavigation::tf_listener;

bool JustinaNavigation::setNodeHandle(ros::NodeHandle* nh)
{
    if(JustinaNavigation::is_node_set)
        return true;
    if(nh == 0)
        return false;

    std::cout << "JustinaNavigation.->Setting ros node..." << std::endl;
    subStop                = nh->subscribe("/stop"                    , 10, &JustinaNavigation::callbackStop);
    subNavigationStop      = nh->subscribe("/navigation/stop"         , 10, &JustinaNavigation::callbackNavigationStop);
    subNavigationStatus    = nh->subscribe("/navigation/status"       , 10, &JustinaNavigation::callbackNavigationStatus);
    subSimpleMoveStatus    = nh->subscribe("/simple_move/goal_reached", 10, &JustinaNavigation::callbackSimpleMoveStatus);
    pubSimpleMoveDist      = nh->advertise<std_msgs::Float32          >("/simple_move/goal_dist", 10);
    pubSimpleMoveDistAngle = nh->advertise<std_msgs::Float32MultiArray>("/simple_move/goal_dist_angle", 10);
    pubSimpleMoveLateral   = nh->advertise<std_msgs::Float32          >("/simple_move/goal_dist_lateral", 10);
    pubMvnPlnGetCloseXYA   = nh->advertise<geometry_msgs::PoseStamped >("/move_base_simple/goal", 10);
    pubNavigationStop      = nh->advertise<std_msgs::Empty>            ("/navigation/stop", 10);
    tf_listener = new tf::TransformListener();

    is_node_set = true;
    _stop = false;
    _navigation_status.status  = actionlib_msgs::GoalStatus::PENDING;
    _simple_move_status.status = actionlib_msgs::GoalStatus::PENDING;
    return true;
}

//Methods for checking if goal position is reached.
bool JustinaNavigation::isLocalGoalReached()
{
    return _simple_move_status.status == actionlib_msgs::GoalStatus::SUCCEEDED;
}

bool JustinaNavigation::isGlobalGoalReached()
{
    return _navigation_status.status == actionlib_msgs::GoalStatus::SUCCEEDED;
}

bool JustinaNavigation::waitForLocalGoalReached(int timeOut_ms)
{
    JustinaNavigation::_stop = false;
    JustinaNavigation::_simple_move_status.status = actionlib_msgs::GoalStatus::PENDING;
    int attempts = timeOut_ms/100;
    ros::Rate loop(10);
    while(ros::ok() && JustinaNavigation::_simple_move_status.status == actionlib_msgs::GoalStatus::PENDING &&
          !JustinaNavigation::_stop && attempts-- >= 0)
    {
        loop.sleep();
        ros::spinOnce();
    }
    
    while(ros::ok() && JustinaNavigation::_simple_move_status.status == actionlib_msgs::GoalStatus::ACTIVE &&
          !JustinaNavigation::_stop && attempts-- >= 0)
    {
        loop.sleep();
        ros::spinOnce();
    }
    JustinaNavigation::_stop = false; //This flag is set True in the subscriber callback
    return JustinaNavigation::_simple_move_status.status == actionlib_msgs::GoalStatus::SUCCEEDED;
}

bool JustinaNavigation::waitForGlobalGoalReached(int timeOut_ms)
{
    JustinaNavigation::_stop = false;
    JustinaNavigation::_navigation_status.status = actionlib_msgs::GoalStatus::PENDING;
    int attempts = timeOut_ms/100;
    ros::Rate loop(10);
    while(ros::ok() && JustinaNavigation::_navigation_status.status == actionlib_msgs::GoalStatus::PENDING &&
          !JustinaNavigation::_stop && attempts-- >= 0)
    {
        loop.sleep();
        ros::spinOnce();
    }
    while(ros::ok() && JustinaNavigation::_navigation_status.status == actionlib_msgs::GoalStatus::ACTIVE &&
          !JustinaNavigation::_stop && attempts-- >= 0)
    {
        loop.sleep();
        ros::spinOnce();
    }
    JustinaNavigation::_stop = false;
    return JustinaNavigation::_navigation_status.status == actionlib_msgs::GoalStatus::SUCCEEDED;
}

//Methods for robot localization
void JustinaNavigation::getRobotPoseWrtMap(float& currentX, float& currentY, float& currentTheta)
{
    tf::StampedTransform transform;
    tf::Quaternion q;
    JustinaNavigation::tf_listener->lookupTransform("map", "base_link", ros::Time(0), transform);
    currentX = transform.getOrigin().x();
    currentY = transform.getOrigin().y();
    q = transform.getRotation();
    currentTheta = atan2((float)q.z(), (float)q.w()) * 2;
}

void JustinaNavigation::getRobotPoseWrtOdom(float& currentX, float& currentY, float& currentTheta)
{
    tf::StampedTransform transform;
    tf::Quaternion q;
    JustinaNavigation::tf_listener->lookupTransform("odom", "base_link", ros::Time(0), transform);
    currentX = transform.getOrigin().x();
    currentY = transform.getOrigin().y();
    q = transform.getRotation();
    currentTheta = atan2((float)q.z(), (float)q.w()) * 2;
}

//These methods use the simple_move node
void JustinaNavigation::startMoveDist(float distance)
{
    std_msgs::Float32 msg;
    msg.data = distance;
    JustinaNavigation::_simple_move_status.status = actionlib_msgs::GoalStatus::PENDING;
    pubSimpleMoveDist.publish(msg);
    ros::spinOnce();
    ros::Duration(0.0333333).sleep();
}

void JustinaNavigation::startMoveDistAngle(float distance, float angle)
{
    std_msgs::Float32MultiArray msg;
    msg.data.push_back(distance);
    msg.data.push_back(angle);
    JustinaNavigation::_simple_move_status.status = actionlib_msgs::GoalStatus::PENDING;
    pubSimpleMoveDistAngle.publish(msg);
    ros::spinOnce();
    ros::Duration(0.0333333).sleep();
}

void JustinaNavigation::startMoveLateral(float distance)
{
    std_msgs::Float32 msg;
    msg.data = distance;
    JustinaNavigation::_simple_move_status.status = actionlib_msgs::GoalStatus::PENDING;
    pubSimpleMoveLateral.publish(msg);
    ros::spinOnce();
    ros::Duration(0.0333333).sleep();
}

bool JustinaNavigation::moveDist(float distance, int timeOut_ms)
{
    JustinaNavigation::startMoveDist(distance);
    return JustinaNavigation::waitForLocalGoalReached(timeOut_ms);
}

bool JustinaNavigation::moveDistAngle(float distance, float angle, int timeOut_ms)
{
    JustinaNavigation::startMoveDistAngle(distance, angle);
    return JustinaNavigation::waitForLocalGoalReached(timeOut_ms);
}

bool JustinaNavigation::moveLateral(float distance, int timeOut_ms)
{
    JustinaNavigation::startMoveLateral(distance);
    return JustinaNavigation::waitForLocalGoalReached(timeOut_ms);
}

//These methods use the mvn_pln node.
void JustinaNavigation::startGetClose(float x, float y, float angle)
{
    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = "map";
    msg.pose.position.x = x;
    msg.pose.position.y = y;
    msg.pose.position.z = 0;
    msg.pose.orientation.x = 0;
    msg.pose.orientation.y = 0;
    msg.pose.orientation.z = sin(angle/2);
    msg.pose.orientation.w = cos(angle/2);
    JustinaNavigation::_navigation_status.status = actionlib_msgs::GoalStatus::PENDING;
    pubMvnPlnGetCloseXYA.publish(msg);
    ros::spinOnce();
    ros::Duration(0.0333333).sleep();
}

void JustinaNavigation::startGetClose(std::string location)
{
    
}

bool JustinaNavigation::getClose(float x, float y, float angle, int timeOut_ms)
{
    JustinaNavigation::startGetClose(x,y,angle);
    return JustinaNavigation::waitForGlobalGoalReached(timeOut_ms);
}

bool JustinaNavigation::getClose(std::string location, int timeOut_ms)
{
    
}

void JustinaNavigation::stopNavigation()
{
    std_msgs::Empty msg;
    pubNavigationStop.publish(msg);
}


//Callbacks for subscribers
void JustinaNavigation::callbackStop(const std_msgs::Empty::ConstPtr& msg)
{
    JustinaNavigation::_stop = true;
}

void JustinaNavigation::callbackNavigationStop(const std_msgs::Empty::ConstPtr& msg)
{
    JustinaNavigation::_stop = true;
}

void JustinaNavigation::callbackSimpleMoveStatus(const actionlib_msgs::GoalStatus::ConstPtr& msg)
{
    JustinaNavigation::_simple_move_status = *msg;
}

void JustinaNavigation::callbackNavigationStatus(const actionlib_msgs::GoalStatus::ConstPtr& msg)
{
    JustinaNavigation::_navigation_status = *msg;
}

