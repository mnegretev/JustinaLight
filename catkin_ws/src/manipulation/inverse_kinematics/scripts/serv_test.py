#!/usr/bin/env python
import math
import rospy
import tf
import tf.transformations as tft
import numpy
import urdf_parser_py.urdf
from geometry_msgs.msg import PointStamped
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
    
    msg = GetCartesianTrajectorRequest()  
    result = trajectory_srv_client(msg) 
    print(result)
    #while not rospy.is_shutdown():
        

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    