#!/usr/bin/env python
#
# ESCUELA DE INVIERNO DE ROBOTICA 2022 - FEDERACION MEXICANA DE ROBOTICA
# EJERCICIO FINAL - PLANEACION DE ACCIONES CON MAQUINAS DE ESTADOS
#

import rospy
import tf
import math
import time
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan, GetPlanRequest
from sensor_msgs.msg import PointCloud2
from sound_play.msg import SoundRequest
from hri_msgs.msg import RecognizedSpeech
from vision_msgs.srv import RecognizeObject, RecognizeObjectRequest
from manip_msgs.srv import InverseKinematicsForPose, InverseKinematicsForPoseRequest
from actionlib_msgs.msg import GoalStatus

def callback_recognized_speech(msg):
    global recognized_speech, new_task, executing_task
    if executing_task:
        return
    new_task = True
    recognized_speech = msg.hypothesis[0]

def callback_navigation_status(msg):
    global goal_reached
    goal_reached = msg.status == GoalStatus.SUCCEEDED

def parse_command(cmd):
    obj_options = ['JUICE', 'CUP', 'DEODORANT', 'SOAP']
    loc_options = ['LIVINGROOM', 'KITCHEN']
    locations   = {'LIVINGROOM':[5.04, 2.42, 1.5708], 'KITCHEN':[9.58, 2.73, 0]}
    obj = ""
    loc = ""
    for o in obj_options:
        if o in cmd:
            obj = o
            break
    for l in loc_options:
        if l in cmd:
            loc = locations[l]
            break
    obj = obj.lower()
    print("ActPln.->Parsed command: " + cmd + "   OBJ=" + obj + "   LOC=" + str(loc))
    return obj, loc

def move_left_arm(q1,q2,q3,q4,q5,q6,q7):
    global pubLaGoalPose
    msg = Float32MultiArray()
    msg.data = [q1,q2,q3,q4,q5,q6,q7]
    pubLaGoalPose.publish(msg)
    time.sleep(5.0)

def move_left_gripper(q):
    global pubLaGoalGripper
    pubLaGoalGripper.publish(q)
    time.sleep(1.0)

def move_right_arm(q1,q2,q3,q4,q5,q6,q7):
    global pubRaGoalPose
    msg = Float32MultiArray()
    msg.data = [q1,q2,q3,q4,q5,q6,q7]
    pubRaGoalPose.publish(msg)
    time.sleep(5.0)

def move_right_gripper(q):
    global pubRaGoalGripper
    pubRaGoalGripper.publish(q)
    time.sleep(1.0)

def move_head(pan, tilt):
    global pubHdGoalPose
    msg = Float32MultiArray()
    msg.data = [pan, tilt]
    pubHdGoalPose.publish(msg)
    time.sleep(1.0)

def move_base(distance, angle):
    global pubGoalDistAngle
    msg = Float32MultiArray()
    msg.data = [distance,angle]
    pubGoalDistAngle.publish(msg)
    time.sleep(5.0)

def go_to_goal_pose(goal_x, goal_y, goal_a):
    global pubGoalPose
    goal_pose = PoseStamped()
    goal_pose.pose.position.x = goal_x
    goal_pose.pose.position.y = goal_y
    goal_pose.pose.orientation.w = math.cos(goal_a/2.0)
    goal_pose.pose.orientation.z = math.sin(goal_a/2.0)
    pubGoalPose.publish(goal_pose)

def say(text):
    global pubSay
    msg = SoundRequest()
    msg.sound   = -3
    msg.command = 1
    msg.volume  = 1.0
    msg.arg2    = "voice_kal_diphone"
    msg.arg = text
    pubSay.publish(msg)

def main():
    global new_task, recognized_speech, executing_task, goal_reached
    global pubLaGoalPose, pubLaGoalGripper, pubRaGoalPose, pubRaGoalGripper, pubHdGoalPose
    global pubGoalDistAngle, pubGoalPose, pubSay
    print("EXECUTING ACTION PLANNER BY MARCOSOFT...")
    rospy.init_node("act_pln")
    rospy.Subscriber('/hri/sp_rec/recognized', RecognizedSpeech, callback_recognized_speech)
    rospy.Subscriber('/navigation/status', GoalStatus, callback_navigation_status)
    pubGoalPose      = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    pubSay           = rospy.Publisher('/robotsound', SoundRequest, queue_size=10)
    pubGoalDistAngle = rospy.Publisher('/simple_move/goal_dist_angle', Float32MultiArray, queue_size=10)
    pubLaGoalPose = rospy.Publisher('/hardware/left_arm/goal_pose' , Float32MultiArray, queue_size=10)
    pubRaGoalPose = rospy.Publisher('/hardware/right_arm/goal_pose', Float32MultiArray, queue_size=10)
    pubHdGoalPose = rospy.Publisher('/hardware/head/goal_pose'     , Float32MultiArray, queue_size=10)
    pubLaGoalGripper = rospy.Publisher('/hardware/left_arm/goal_gripper' , Float32, queue_size=10)
    pubRaGoalGripper = rospy.Publisher('/hardware/right_arm/goal_gripper', Float32, queue_size=10)
    
    listener = tf.TransformListener()
    loop = rospy.Rate(10)
    print("Waiting for services...")
    rospy.wait_for_service('/manipulation/la_inverse_kinematics')
    rospy.wait_for_service('/vision/obj_reco/recognize_object')
    print("Services are now available.")
    clt_la_inverse_kin = rospy.ServiceProxy("/manipulation/la_inverse_kinematics", InverseKinematicsForPose)
    clt_ra_inverse_kin = rospy.ServiceProxy("/manipulation/ra_inverse_kinematics", InverseKinematicsForPose)
    clt_find_object = rospy.ServiceProxy("/vision/obj_reco/recognize_object", RecognizeObject)

    new_task = False
    executing_task = False
    recognized_speech = ""
    goal_reached = False

    current_state = "SM_INIT"
    requested_object   = ""
    requested_location = [0,0]

    #
    # EJERCICIO FINAL
    # Agregue funciones 'say' en los estados de modo que el robot
    # indique por voz la parte de la tarea que se esta ejecutando
    #
    
    while not rospy.is_shutdown():
        if current_state == "SM_INIT":
            print("Waiting for new task")
            current_state = "SM_WAITING_NEW_TASK"
        elif current_state == "SM_WAITING_NEW_TASK":
            if new_task:
                requested_object, requested_location = parse_command(recognized_speech)
                print("New task received: " + requested_object + " to  " + str(requested_location))
                say("Executing the command, " + recognized_speech)
                current_state = "SM_MOVE_HEAD"
                new_task = False
                executing_task = True
                
        elif current_state == "SM_MOVE_HEAD":
            print("Moving head to look at table...")
            move_head(0, -0.9)
            current_state = "SM_FIND_OBJECT"
            
        elif current_state == "SM_FIND_OBJECT":
            print("Trying to find object: " + requested_object)
            req_find_object = RecognizeObjectRequest()
            req_find_object.point_cloud = rospy.wait_for_message("/hardware/kinect/rgbd_wrt_kinect", PointCloud2)
            req_find_object.name  = requested_object
            recog_object_position = clt_find_object(req_find_object).recog_object.pose.position
            print("Object found at: " + str([recog_object_position.x, recog_object_position.y, recog_object_position.z]))
            current_state = "SM_INVERSE_KINEMATICS"
            
        elif current_state == "SM_INVERSE_KINEMATICS":
            obj_p = PointStamped()
            obj_p.header.frame_id = "base_link"
            obj_p.header.stamp = rospy.Time(0)
            obj_p.point.x, obj_p.point.y, obj_p.point.z = recog_object_position.x, recog_object_position.y, recog_object_position.z
            target_frame = "shoulders_left_link" if recog_object_position.y > 0 else "shoulders_right_link"
            print("Transforming " + requested_object + " position to " + target_frame)
            obj_p = listener.transformPoint(target_frame, obj_p)
            print("Trying to get inverse kinematics for pose " + str([obj_p.point.x,obj_p.point.y,obj_p.point.z]))
            req_ik = InverseKinematicsForPoseRequest()
            req_ik.x = obj_p.point.x
            req_ik.y = obj_p.point.y
            req_ik.z = obj_p.point.z + 0.1
            req_ik.roll  = 3.0
            req_ik.pitch = -1.57
            req_ik.yaw   = -3.0
            if target_frame == "shoulders_left_link":
                resp_ik = clt_la_inverse_kin(req_ik)
            else:
                resp_ik = clt_ra_inverse_kin(req_ik)
            print("IK Result: " + str([resp_ik.q1,resp_ik.q2,resp_ik.q3,resp_ik.q4,resp_ik.q5,resp_ik.q6,resp_ik.q7]))
            current_state = "SM_MOVE_LEFT_ARM" if target_frame == "shoulders_left_link" else "SM_MOVE_RIGHT_ARM"
            
        elif current_state == "SM_MOVE_LEFT_ARM":
            say("I am going to take the " + requested_object)
            print("Moving left manipulator to stand by position")
            move_left_arm(-0.6, 0, 0, 2.0, 0, 0.6, 0)
            move_left_gripper(0.7)
            print("Moving left manipulator to object position")
            move_left_arm(resp_ik.q1, resp_ik.q2, resp_ik.q3, resp_ik.q4, resp_ik.q5, resp_ik.q6, resp_ik.q7)
            move_left_gripper(-0.1)
            move_left_arm(resp_ik.q1, resp_ik.q2, resp_ik.q3, resp_ik.q4+0.1, resp_ik.q5, resp_ik.q6+0.1, resp_ik.q7)
            print("Moving backwards")
            move_base(-0.4, 0)
            move_left_arm(-0.6, 0, 0, 1.8, 0, 0.6, 0)
            current_state = "SM_INIT"
            current_state = "SM_START_NAVIGATION"
            
        elif current_state == "SM_MOVE_RIGHT_ARM":
            print("Moving right manipulator to stand by position")
            move_right_arm(-0.6, 0, 0, 2.0, 0, 0.6, 0)
            move_right_gripper(0.7)
            print("Moving right manipulator to object position")
            move_right_arm(resp_ik.q1, resp_ik.q2, resp_ik.q3, resp_ik.q4, resp_ik.q5, resp_ik.q6, resp_ik.q7)
            move_right_gripper(-0.1)
            move_right_arm(resp_ik.q1, resp_ik.q2, resp_ik.q3, resp_ik.q4+0.1, resp_ik.q5, resp_ik.q6+0.1, resp_ik.q7)
            print("Moving backwards")
            move_base(-0.4, 0)
            move_right_arm(-0.6, 0, 0, 1.8, 0, 0.6, 0)
            current_state = "SM_START_NAVIGATION"
            
        elif current_state == "SM_START_NAVIGATION":
            print("Sending goal position to " + str(requested_location))
            say("I am going to move")
            go_to_goal_pose(requested_location[0], requested_location[1],requested_location[2])
            goal_reached = False
            current_state = "SM_WAIT_FOR_MOVEMENT_FINISHED"
            
        elif current_state == "SM_WAIT_FOR_MOVEMENT_FINISHED":
            if goal_reached:
                print("Goal point reached")
                goal_reached = False
                move_base(0.3,0)
                current_state = "SM_LEAVE_OBJECT"

        elif current_state == "SM_LEAVE_OBJECT":
            say("I am going to leave the " + requested_object)
            if target_frame == "shoulders_left_link":
                move_left_arm(resp_ik.q1, resp_ik.q2, resp_ik.q3, resp_ik.q4, resp_ik.q5, resp_ik.q6, resp_ik.q7)
                move_left_gripper(0.5)
                move_base(-0.3, 0)
                move_left_arm(-0.6, 0, 0, 2.0, 0, 0.6, 0)
                move_left_gripper(0.0)
                move_left_arm(0,0,0,0,0,0,0)
            else:
                move_right_arm(resp_ik.q1, resp_ik.q2, resp_ik.q3, resp_ik.q4, resp_ik.q5, resp_ik.q6, resp_ik.q7)
                move_right_gripper(0.5)
                move_base(-0.3, 0)
                move_right_arm(-0.6, 0, 0, 2.0, 0, 0.6, 0)
                move_right_gripper(0.0)
                move_right_arm(0,0,0,0,0,0,0)            
            go_to_goal_pose(3.04, 0.88,-1.5708)
            goal_reached = False
            current_state = "SM_WAIT_FOR_RETURN"

        elif current_state == "SM_WAIT_FOR_RETURN":
            if goal_reached:
                print("Return point reached")
                goal_reached = False
                current_state = "SM_APPROACH_TO_TABLE"

        elif current_state == "SM_APPROACH_TO_TABLE":
            go_to_goal_pose(3.34, 0.5,-1.5708)
            goal_reached = False
            current_state = "SM_WAIT_FOR_APPROACHING"

        elif current_state == "SM_WAIT_FOR_APPROACHING":
            if goal_reached:
                print("Start point reached")
                goal_reached = False
                move_base(0.15, 0)
                say("I am ready for a new command")
                current_state = "SM_INIT"
                executing_task = False
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
