#!/usr/bin/env python
import math
import rospy
import matplotlib.pyplot as plt
from pylab import *
import tf
import tf.transformations as tft
from tf.transformations import euler_from_quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import urdf_parser_py.urdf
from geometry_msgs.msg import PointStamped

from manip_msgs.srv import *

from std_srvs.srv import Empty, EmptyResponse

tm = 0.05

# Calcula polinomio 5o orden para una variable
def calcula_tray(tt, pi, pf, vi, vf, ai, af):
    print("tiempo de muestreo tm: ",tm)
    for p in range(0,len(tt)-1):
        # Rango de tiempo en el que se evalua el polinomio
        t=np.arange(tt[p],tt[p+1],tm)
        c = np.ones(len(t))

        M = np.array([[1, tt[p], tt[p]**2, tt[p]**3, tt[p]**4, tt[p]**5],
            [0, 1, 2*tt[p], 3*tt[p]**2, 4*tt[p]**3, 5*tt[p]**4],
            [0, 0, 2, 6*tt[p], 12*tt[p]**2, 20*tt[p]**3],
            [1, tt[p+1], tt[p+1]**2, tt[p+1]**3, tt[p+1]**4, tt[p+1]**5],
            [0, 1, 2*tt[p+1], 3*tt[p+1]**2, 4*tt[p+1]**3, 5*tt[p+1]**4],
            [0, 0, 2, 6*tt[p+1], 12*tt[p+1]**2, 20*tt[p+1]**3]])
        # Vector de condiciones iniciales
        b=[pi, vi, ai, pf, vf, af]
        b = np.vstack(b)
        # Solucion a la ecuacion matricial
        Minv = np.linalg.inv(M)
        a = np.dot(Minv,b) 
        #print("a dim", np.shape(a)) 
        # Formacion del polinomio de 5o grado
        xpos = a[0][0]*c + a[1][0]*t +a[2][0]*t**2 + a[3][0]*t**3 + a[4][0]*t**4 + a[5][0]*t**5
        xvel = a[1][0]*c +2*a[2][0]*t +3*a[3][0]*t**2 +4*a[4][0]*t**3 +5*a[5][0]*t**4
        xacel = 2*a[2][0]*c + 6*a[3][0]*t +12*a[4][0]*t**2 +20*a[5][0]*t**3
    
    print("Numero de puntos", len(xpos))

    return [t, xpos, xvel, xacel]


tm = 0.05 # tiempo de muestreo

def grafica_tray(t, x, y, z, lbl, a,b,c,y_label):
    plot(t,x,t,y,t,z)
    grid()
    xlabel('t[s]')        # Etiqueta del eje OX
    ylabel(y_label)        # Etiqueta del eje OY
    title(lbl)    # Titulo del grafico
    legend((a,b,c),
    prop = {'size': 10}, loc='upper right')
    show()


    
def cartesian_traj(tt, pi, pf, vi, vf, ai, af):
    """
    Genera trayectorias para x,y,z,R,P a partir de condiciones iniciales y finales, 
    empaqueta cada punto en un mensaje JointTrajectoryPoint dentro de una lista en
    un objeto JointTrajectory y retorna este objeto
    """
    
    p6, v6, a6, t = np.empty(0), np.empty(0), np.empty(0), np.empty(0)
    # Posiciones ocurren en el orden x,y,z,R,P, Orientacion: Row, Pitch, Yaw 
    for i in range(6):  # Genera las trayectorias en el espacio cartesiano
        t, p, v, a = calcula_tray(tt, pi[i], pf[i], vi[i], vf[i], ai[i], af[i])
        p6 ,v6,a6 = np.append(p6, [p]), np.append(v6, [v]), np.append(a6, [a])

    p6, v6 , a6 = np.reshape(p6, (6, len(p))), np.reshape(v6, (6, len(v))), np.reshape(a6, (6, len(a)))
    # Preparando datos para enviarlos
    traj = JointTrajectory()    #trayectoria con los puntos en 3d
    traj.joint_names = ["x","y","z","row","pitch","yaw"]
    i = 0
    tfs = 0
    for i in range(len(p)):    # Para cada punto de la trayectoria
        point = JointTrajectoryPoint()  # Creamos un objeto que almacena los datos de 1 punto
        point.positions = p6[0,i], p6[1,i],p6[2,i], (p6[3,i]), (p6[4,i]), (p6[5,i])
        #point.velocities = 0,0,0,0,0,0#v6[0,i],v6[1,i],v6[2,i],v6[3,i],v6[4,i],v6[5,i]
        #point.accelerations = 0,0,0,0,0,0#a6[0,i],a6[1,i],a6[2,i],a6[3,i],a6[4,i],a6[5,i]
        traj.points.append(point)
        point.time_from_start.secs = tfs
        #print("time from start", point.time_from_start.secs)
        tfs += tm

    grafica_tray(t, p6[0,:], p6[1,:], p6[2,:],'Posicion en X,Y,Z','X', 'Y','Z','Distancia[m]')
    grafica_tray(t, p6[3,:], p6[4,:], p6[5,:],'Orientacion Row, Pitch, Yaw','R','P','Y','Distancia[grados]')

    return traj


def callback_trajectory(req):  #request es de tipo Pose
    # req = Pose p1, p2 (orientacion en cuaterniones) , float64 t
    angles1 = tft.euler_from_quaternion([req.p1.orientation.x , req.p1.orientation.y ,req.p1.orientation.z, req.p1.orientation.w])
    angles2 = tft.euler_from_quaternion([req.p2.orientation.x , req.p2.orientation.y ,req.p2.orientation.z, req.p2.orientation.w])
    
    ang1, ang2 = list(angles1), list(angles2)
    t = req.t 
    tt = np.array([0,t])
    # pi = [0.15, 0.7, 0.4, 0.0, 0.0, 0.0]
    pi = [req.p1.positions.x, req.p1.positions.y, req.p1.positions.z, ang1[0], ang1[1], ang1[2]]
    #pf = [0.35, 0.4, 0.5, 90, 0.0, 0.0]
    pf = [req.p2.positions.x, req.p2.positions.y, req.p2.positions.z, ang2[0], ang2[1], ang2[2]]

    vi = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    vf = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    ai = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    af = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    trajectory = cartesian_traj(tt, pi, pf, vi, vf, ai, af)
    
    print(cartesian_traj(tt, pi, pf, vi, vf, ai, af))
    print("Se llamo al servicio genera trayectoria espacio cartesiano")
    return trajectory    # Retorna un objeto JointTrajectory


def main():
    rospy.init_node("generate_trajectory_3d")
    rospy.Service('/traj_srv', GetCartesianTrajectory, callback_trajectory)
    rospy.spin() # mantain the service open.

    loop = rospy.Rate(10)
    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == '__main__':
    main()