#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JoinState
from gazebo_msgs.srv import ApplyJointEffort
from gazebo_msgs.srv import GetJointProperties
import tf

def callback_torque(msg):
    global torque
    torque = msg.data

def main():
    print("INTIALIZING CONTROL RESENDER NODE...")
    rospy.init_node("control_resender")
    br = tf.TransformBroadcaster()
    rospy.Subscriber("/left_arm/torque", Float32MultiArray, callback_torque)
    pub_joint_states = rospy.Publisher("/joint_states", JointState, queue_size=1)
    joint_states = JointState()
    joint_states.name = ["la_1_joint", "la_2_joint", "la_3_joint", "la_4_joint", "la_5_joint", "la_6_joint", "la_7_joint"]
    joint_states.position = [0, 0, 0, 0, 0, 0, 0]
    gazebo_joints = ["justina::la_1_joint", "justina::la_2_joint", "justina::la_3_joint", "justina::la_4_joint", "justina::la_5_joint", "justina::la_6_joint", "justina::la_7_joint"]

    while not rospy.is_shutdown():
        joint_states.header.stamp = rospy.Time.now()
        joint_states.position = [0,0,0,0,0,0,0]
        pub_joint_states.publish(joint_states)
    

if __name__ == '__main__':
    main()
