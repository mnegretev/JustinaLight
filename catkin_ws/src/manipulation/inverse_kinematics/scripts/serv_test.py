#!/usr/bin/env python
import math
import rospy
import tf
import tf.transformations as tft
import numpy
import urdf_parser_py.urdf
from geometry_msgs.msg import PointStamped, Pose
from manip_msgs.srv import *

#import matplotlib.pyplot as plt
#from pylab import *
from tf.transformations import euler_from_quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

   
def main():
    print ("Nodo cliente de genera Trayectoria")
    # Inicialice un nodo ROS con el nombre service_client
    rospy.init_node('srv_test_client')
    # Espere a que se ejecute el servicio qu esta solicitando
    rospy.wait_for_service('/manipulation/cartesian_traj')
    # Crea la conexion al servicio y un objeto 
    trajectory_srv_client  = rospy.ServiceProxy('/manipulation/cartesian_traj', GetCartesianTrajectory)
    
    msg = GetCartesianTrajectoryRequest() 
    # Punto de inicio   GUI: Th1 = -0.2000, Th4 = 0.2000
    # - Translation: [0.169, -0.000, -0.717]
    # - Rotation: in Quaternion [-0.000, -0.149, -0.000, 0.989]
    # - Posicion actual q: [0.14, -0.0, 0.0, 0.16, 0.0, -0.01, 0.0]
    # 
    # Punto meta  GUI: Th1 = -0.6000, Th4 = 1.8000
    # - Translation: [0.156, -0.000, -0.468]
    # - Rotation: in Quaternion [-0.000, -0.506, -0.000, 0.863]
    # - Posicion actual q: [-0.59, 0.0, -0.0, 1.70, -0.0, -0.05, -0.0]

    msg.p1.position.x, msg.p1.position.y, msg.p1.position.z = 0.169, -0.000, -0.717
    msg.p1.orientation.x , msg.p1.orientation.y ,msg.p1.orientation.z, msg.p1.orientation.w = -0.000, -0.149, -0.000, 0.989
    
    msg.p2.position.x, msg.p2.position.y, msg.p2.position.z = 0.156, -0.000, -0.468
    msg.p2.orientation.x , msg.p2.orientation.y ,msg.p2.orientation.z, msg.p2.orientation.w = -0.000, -0.506, -0.000, 0.863

    result = trajectory_srv_client(msg) 
    
    print("respuesta: ", result)

    """

    #while not rospy.is_shutdown():

    # Espere a que se ejecute el servicio traj_q
    rospy.wait_for_service('/manipulation/q_traj')
    # Crea la conexion al servicio y un objeto 
    ik_trj_srv_client  = rospy.ServiceProxy('/manipulation/q_traj', InverseKinematicsForTrajectory)
    
    msg = InverseKinematicsForTrajectoryRequest() 
    
    msg.cartesian_trajectory = result
    result2 = ik_trj_srv_client(msg) # Retorna articular_trajectory
    print("respuesta 2: ", result2)
    """
        

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    