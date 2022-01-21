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
    tf_listener = new tf::TransformListener();

    is_node_set = true;
    _stop = false;
    _navigation_status.status  = actionlib_msgs::GoalStatus::ABORTED;
    _simple_move_status.status = actionlib_msgs::GoalStatus::ABORTED;
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
    
}

bool JustinaNavigation::waitForGlobalGoalReached(int timeOut_ms)
{
}

//Methods for robot localization
void JustinaNavigation::getRobotPoseWrtMap(float& currentX, float& currentY, float& currentTheta)
{
}

void JustinaNavigation::getRobotPoseWrtOdom(float& currentX, float& currentY, float& currentTheta)
{
}

//These methods use the simple_move node
void JustinaNavigation::startMoveDist(float distance)
{
}

void JustinaNavigation::startMoveDistAngle(float distance, float angle)
{
}

void JustinaNavigation::startMoveLateral(float distance)
{
}

bool JustinaNavigation::moveDist(float distance, int timeOut_ms)
{
}

bool JustinaNavigation::moveDistAngle(float distance, float angle, int timeOut_ms)
{
}

bool JustinaNavigation::moveLateral(float distance, int timeOut_ms)
{
}

//These methods use the mvn_pln node.
void JustinaNavigation::startGetClose(float x, float y, float angle)
{
}

void JustinaNavigation::startGetClose(std::string location)
{
}

bool JustinaNavigation::getClose(float x, float y, float angle, int timeOut_ms)
{
}

bool JustinaNavigation::getClose(std::string location, int timeOut_ms)
{
}


//Callbacks for subscribers
void JustinaNavigation::callbackStop(const std_msgs::Empty::ConstPtr& msg)
{
}

void JustinaNavigation::callbackNavigationStop(const std_msgs::Empty::ConstPtr& msg)
{
}

void JustinaNavigation::callbackSimpleMoveStatus(const actionlib_msgs::GoalStatus::ConstPtr& msg)
{
}

void JustinaNavigation::callbackNavigationStatus(const actionlib_msgs::GoalStatus::ConstPtr& msg)
{
}

