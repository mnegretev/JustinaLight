#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Float64MultiArray, Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import *

RATE = 100
WHEEL_RADIUS = 0.08
BASE_RADIUS  = 0.23
ALPHA_LEFT   =  math.pi/3
ALPHA_RIGHT  = -math.pi/3
ALPHA_BACK   =  math.pi

def callback_twist(msg):
    global goal_x, goal_y, goal_a
    goal_x = msg.linear.x
    goal_y = msg.linear.y
    goal_a = msg.angular.z

def main():
    global goal_x, goal_y, goal_a
    print("INITIALIZING OMNI BASE GAZEBO CONTROL BY MARCOSOFT...")
    rospy.init_node("omni_base_gazebo_control")
    rospy.Subscriber("/cmd_vel", Twist, callback_twist)
    clt_apply_effort   = rospy.ServiceProxy("/gazebo/apply_joint_effort", ApplyJointEffort)
    clt_get_properties = rospy.ServiceProxy("/gazebo/get_joint_properties", GetJointProperties)
    req_effort_left  = ApplyJointEffortRequest()
    req_effort_right = ApplyJointEffortRequest()
    req_effort_back  = ApplyJointEffortRequest()
    req_effort_left.joint_name  = 'justina::wheel_left_joint'
    req_effort_right.joint_name = 'justina::wheel_right_joint'
    req_effort_back.joint_name  = 'justina::wheel_right_joint'
    req_effort_left.duration  = rospy.Duration(1.0/RATE)
    req_effort_right.duration = rospy.Duration(1.0/RATE)
    req_effort_back.duration  = rospy.Duration(1.0/RATE)
    req_prop_left  = GetJointPropertiesRequest()
    req_prop_right = GetJointPropertiesRequest()
    req_prop_back  = GetJointPropertiesRequest()
    req_prop_left.joint_name  = 'justina::wheel_left_joint'
    req_prop_right.joint_name = 'justina::wheel_right_joint'
    req_prop_back.joint_name  = 'justina::wheel_back_joint'
    loop = rospy.Rate(RATE)
    goal_x, goal_y, goal_a = 0,0,0
    last_error_left  = 0
    last_error_right = 0
    last_error_back  = 0
    s_alpha_left  = math.sin(ALPHA_LEFT)
    s_alpha_right = math.sin(ALPHA_RIGHT)
    s_alpha_back  = math.sin(ALPHA_BACK)
    c_alpha_left  = math.cos(ALPHA_LEFT)
    c_alpha_right = math.cos(ALPHA_RIGHT)
    c_alpha_back  = math.cos(ALPHA_BACK)
    
    while not rospy.is_shutdown():
        resp_left  = clt_get_properties(req_prop_left)
        resp_right = clt_get_properties(req_prop_right)
        resp_back  = clt_get_properties(req_prop_back)
        current_left, current_right, current_back = resp_left.rate[0], resp_right.rate[0], resp_back.rate[0]
        goal_left  = -(-s_alpha_left *goal_x + c_alpha_left *goal_y + BASE_RADIUS*goal_a)/WHEEL_RADIUS
        goal_right = -(-s_alpha_right*goal_x + c_alpha_right*goal_y + BASE_RADIUS*goal_a)/WHEEL_RADIUS
        goal_back  = -(-s_alpha_back *goal_x + c_alpha_back *goal_y + BASE_RADIUS*goal_a)/WHEEL_RADIUS
        error_left  = goal_left  - current_left
        error_right = goal_right - current_right
        error_back  = goal_back  - current_back
        d_error_left  = error_left  - last_error_left
        d_error_right = error_right - last_error_right
        d_error_back  = error_back  - last_error_back
        last_error_left  = error_left  
        last_error_right = error_right 
        last_error_back  = error_back  
        #print([error_left, error_right, error_back])
        req_effort_left.effort  = 0.5*(goal_left  - current_left ) + 0.1*d_error_left
        req_effort_right.effort = 0.5*(goal_right - current_right) + 0.1*d_error_right
        req_effort_back.effort  = 0.5*(goal_back  - current_back ) + 0.1*d_error_back
        # req_effort_left.effort  = 0.0
        # req_effort_right.effort = 0.0
        # req_effort_back.effort  = 0.0
        clt_apply_effort(req_effort_left)
        clt_apply_effort(req_effort_right)
        clt_apply_effort(req_effort_back)
        loop.sleep()


if __name__ == '__main__':
    main()
    
