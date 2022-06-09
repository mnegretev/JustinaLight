#!/usr/bin/env python
import math
import rospy
import tf
import tf.transformations as tft
import numpy
import urdf_parser_py.urdf
from geometry_msgs.msg import PointStamped
from manip_msgs.srv import *
from tf.transformations import euler_from_quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint



def get_model_info():
    global joints, transforms
    robot_model = urdf_parser_py.urdf.URDF.from_parameter_server()
    joints = {'left': [None for i in range(8)], 'right': [None for i in range(8)]}
    transforms = {'left':[], 'right':[]}
    for joint in robot_model.joints:
        for i in range(1,8):
            joints['left' ][i-1] = joint if joint.name == ('la_'+str(i)+'_joint') else joints['left' ][i-1]
            joints['right'][i-1] = joint if joint.name == ('ra_'+str(i)+'_joint') else joints['right'][i-1]
        joints['left' ][7] = joint if joint.name == 'la_grip_center_joint' else joints['left' ][7]
        joints['right'][7] = joint if joint.name == 'ra_grip_center_joint' else joints['right'][7]
    for joint in joints['left']:
        T = tft.translation_matrix(joint.origin.xyz)
        R = tft.euler_matrix(joint.origin.rpy[0], joint.origin.rpy[1], joint.origin.rpy[2])
        transforms['left'].append(tft.concatenate_matrices(T,R))
    for joint in joints['right']:
        T = tft.translation_matrix(joint.origin.xyz)
        R = tft.euler_matrix(joint.origin.rpy[0], joint.origin.rpy[1], joint.origin.rpy[2])
        transforms['right'].append(tft.concatenate_matrices(T,R))

def angles_in_joint_limits(q, arm):
    for i in range(len(q)):
        if q[i] < joints[arm][i].limit.lower or q[i] > joints[arm][i].limit.upper:
            print("InverseKinematics.->Articular position out of joint bounds")
            return False
    return True
    
def direct_kinematics(q, arm):
    global transforms, joints
    
    H = tft.identity_matrix()
    for i in range(len(q)):
        H  = tft.concatenate_matrices(H, transforms[arm][i], tft.rotation_matrix(q[i], joints[arm][i].axis))
    H  = tft.concatenate_matrices(H, transforms[arm][7])
    return numpy.asarray([H[0,3], H[1,3], H[2,3]] + list(tft.euler_from_matrix(H)))

def jacobian(q, arm):
    delta_q = 0.000001   
    J = numpy.asarray([[0.0 for a in q] for i in range(6)])##numpy, ndarray
    qn = numpy.asarray([q,]*len(q)) + delta_q*numpy.identity(len(q))
    qp = numpy.asarray([q,]*len(q)) - delta_q*numpy.identity(len(q)) 
    for i in range(len(q)):
        J[:,i] = (direct_kinematics(qn[i], arm) - direct_kinematics(qp[i], arm))/delta_q/2.0
     
    return J

def jacobian_3x7(q, arm):   
    delta_q = 0.000001   
    J = numpy.asarray([[0.0 for a in q] for i in range(3)])   # 3x7
    qn = numpy.asarray([q,]*len(q)) + delta_q*numpy.identity(len(q))  # 7x7
    qp = numpy.asarray([q,]*len(q)) - delta_q*numpy.identity(len(q))  # 7x7
    for i in range(len(q)):
        x, y,z, R, P, Y = (direct_kinematics(qn[i], arm) - direct_kinematics(qp[i], arm))/delta_q/2.0
        J[:,i] = x, y, z
    return J

def inverse_kinematics_xyzrpy(x, y, z, roll, pitch, yaw, arm,q = numpy.asarray([-0.5, 0.6, 0.3, 2.0, 0.3, 0.2, 0.3])):
    pd = numpy.asarray([x,y,z,roll,pitch,yaw])
    p  = direct_kinematics(q, arm)
    iterations = 0
    while numpy.linalg.norm(p - pd) > 0.00001 and iterations < 20:
        J = jacobian(q, arm)
        J_3x7 = jacobian_3x7(q, arm)
        err = p - pd
        err[3:6] = (err[3:6] + math.pi)%(2*math.pi) - math.pi
        err_xyz = p[0:3]-pd[0:3]
        
        q = (q - numpy.dot(numpy.linalg.pinv(J), err) + math.pi)%(2*math.pi) - math.pi
        q_J3x7 = q - numpy.dot(numpy.linalg.pinv(J_3x7), err_xyz)
        p = direct_kinematics(q, arm)
        iterations +=1
        
    if iterations < 20 and angles_in_joint_limits(q, arm):
        print("InverseKinematics.->IK for " + arm + " arm solved after " + str(iterations) + " iterations: " + str(q))
        return q
    else:
        print("InverseKinematics.->Cannot solve IK for " + arm + " arm. Max attempts exceeded. ")
        return False

t = 5       # tiempo de la trayectoria
tm = 0.05   # tiempo de muestreo

def calcula_tray(tt, pi, pf, vi, vf, ai, af):  # Calcula polinomio 5o orden para una variable
    tm = 0.1
    print("tiempo de muestreo tm: ",tm)
    for p in range(0,len(tt)-1):
        # Rango de tiempo en el que se evalua el polinomio
        t=numpy.arange(tt[p],tt[p+1],tm)
        c = numpy.ones(len(t))

        M = numpy.array([[1, tt[p], tt[p]**2, tt[p]**3, tt[p]**4, tt[p]**5],
            [0, 1, 2*tt[p], 3*tt[p]**2, 4*tt[p]**3, 5*tt[p]**4],
            [0, 0, 2, 6*tt[p], 12*tt[p]**2, 20*tt[p]**3],
            [1, tt[p+1], tt[p+1]**2, tt[p+1]**3, tt[p+1]**4, tt[p+1]**5],
            [0, 1, 2*tt[p+1], 3*tt[p+1]**2, 4*tt[p+1]**3, 5*tt[p+1]**4],
            [0, 0, 2, 6*tt[p+1], 12*tt[p+1]**2, 20*tt[p+1]**3]])
        # Vector de condiciones iniciales
        b=[pi, vi, ai, pf, vf, af]
        b = numpy.vstack(b)
        # Solucion a la ecuacion matricial
        Minv = numpy.linalg.inv(M)
        a = numpy.dot(Minv,b) 
        # polinomio de 5o grado
        xpos = a[0][0]*c + a[1][0]*t +a[2][0]*t**2 + a[3][0]*t**3 + a[4][0]*t**4 + a[5][0]*t**5
        xvel = a[1][0]*c +2*a[2][0]*t +3*a[3][0]*t**2 +4*a[4][0]*t**3 +5*a[5][0]*t**4
        xacel = 2*a[2][0]*c + 6*a[3][0]*t +12*a[4][0]*t**2 +20*a[5][0]*t**3
    
    print("Numero de puntos", len(xpos))

    return [t, xpos, xvel, xacel]

    
def cartesian_traj(tt, pi, pf, vi, vf, ai, af):
    p6, v6, a6, t = numpy.empty(0), numpy.empty(0), numpy.empty(0), numpy.empty(0)
    # Posiciones ocurren en el orden x,y,z,R,P, Orientacion: Row, Pitch, Yaw 
    for i in range(6):  # Genera las trayectorias en el espacio cartesiano
        t, p, v, a = calcula_tray(tt, pi[i], pf[i], vi[i], vf[i], ai[i], af[i])
        p6 ,v6,a6 = numpy.append(p6, [p]), numpy.append(v6, [v]), numpy.append(a6, [a])

    p6, v6 , a6 = numpy.reshape(p6, (6, len(p))), numpy.reshape(v6, (6, len(v))), numpy.reshape(a6, (6, len(a)))
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
        point.time_from_start.secs = tfs
        traj.points.append(point)
        tfs += tm

    #grafica_tray(t, p6[0,:], p6[1,:], p6[2,:],'Posicion en X,Y,Z','X', 'Y','Z','Distancia[m]')
    #grafica_tray(t, p6[3,:], p6[4,:], p6[5,:],'Orientacion Row, Pitch, Yaw','R','P','Y','Distancia[grados]')

    return traj

def callback_trajectory_3d(req):  #request es de tipo Pose
    angles1 = tft.euler_from_quaternion([req.p1.orientation.x , req.p1.orientation.y ,req.p1.orientation.z, req.p1.orientation.w])
    angles2 = tft.euler_from_quaternion([req.p2.orientation.x , req.p2.orientation.y ,req.p2.orientation.z, req.p2.orientation.w])
    ang1, ang2 = list(angles1), list(angles2)
    tt = numpy.array([0,t])
    pii = [req.p1.position.x, req.p1.position.y, req.p1.position.z, ang1[0], ang1[1], ang1[2]]
    pf = [req.p2.position.x, req.p2.position.y, req.p2.position.z, ang2[0], ang2[1], ang2[2]]
    vi, vf = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    ai, af = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    c_tr = cartesian_traj(tt, pii, pf, vi, vf, ai, af)
    resp = GetCartesianTrajectoryResponse()
    resp.trajectory = c_tr
	
    callback_trajectory_q( c_tr, 'left')
    
    return resp    # Retorna un objeto JointTrajectory

def callback_trajectory_q(req, arm, q_estim):  # Trayectoria en espacio articular: recibe un JointTrajectory
    qs = numpy.empty(0)
    # Formar la primera estimacion con posicion actual
    n_p = len(req.points)   # Numero de puntos en la trayectoria
    print("numero de puntos traj*****", n_p)
    # Formar las sucesivas suposicion con el punto anterior al objetivo
    i = 0
    for i in range(n_p):
        q_obt = inverse_kinematics_xyzrpy(req.points[i].positions[0], req.points[i].positions[1], req.points[i].positions[2], req.points[i].positions[3], req.points[i].positions[4], req.points[i].positions[5], arm,q_estim)
        qs = numpy.append(qs, [q_obt])  # Guarda cada punto obtenido en q en un arreglo
        q_estim = q_obt  # Actualiza la estimacion
        print("punto q#",i)
    
    qs = numpy.reshape(qs, (n_p,7)) # Redimensiona el arreglo
    traj_q = JointTrajectory()    #trayectoria con puntos en espacio articular
    traj_q.joint_names = ["q1","q2","q3","q4","q5","q6","q7"]
    i=0
    tfs = 0
  
    for element in qs:
        point = JointTrajectoryPoint()  # Creamos un objeto que almacena los datos de 1 punto
        point.positions = element[0], element[1], element[2], element[3], element[4], element[5], element[6]
        traj_q.points.append(point)
        point.time_from_start.secs = tfs
        tfs += tm
        
    return traj_q



def trajectory_q_j3x7(req, arm, q_estim):  # Trayectoria en espacio articular: recibe un JointTrajectory
    qs = numpy.empty(0)
    # Formar la primera estimacion con posicion actual
    n_p = len(req.points)   # Numero de puntos en la trayectoria
    print("numero de puntos traj*****", n_p)
    # Formar las sucesivas suposicion con el punto anterior al objetivo
    i = 0
    for i in range(n_p):
        q_obt = inverse_kinematics_xyzrpy(req.points[i].positions[0], req.points[i].positions[1], req.points[i].positions[2], req.points[i].positions[3], req.points[i].positions[4], req.points[i].positions[5], arm,q_estim)
        qs = numpy.append(qs, [q_obt])  # Guarda cada punto obtenido en q en un arreglo
        q_estim = q_obt  # Actualiza la estimacion
        print("punto q#",i)
    
    qs = numpy.reshape(qs, (n_p,7)) # Redimensiona el arreglo
    traj_q = JointTrajectory()    #trayectoria con puntos en espacio articular
    traj_q.joint_names = ["q1","q2","q3","q4","q5","q6","q7"]
    i=0
    tfs = 0
  
    for element in qs:
        point = JointTrajectoryPoint()  # Creamos un objeto que almacena los datos de 1 punto
        point.positions = element[0], element[1], element[2], element[3], element[4], element[5], element[6]
        traj_q.points.append(point)
        point.time_from_start.secs = tfs
        tfs += tm
        
    return traj_q



def callback_LA_ik_for_trajectory(req):
    
    # Punto de inicio   GUI: Th1 = 0.2000, Th4 = 0.2000
    # - Translation: [0.169, -0.000, -0.717]
    # - Rotation: in RPY (radian) [-0.0, -0.299, 0.0]
    # - Position q: [0.14, -0.0, 0.0, 0.16, 0.0, -0.01, 0.0]
    
    init_estim = [0.14, 0, 0, 0.16, 0,-0.01, 0]
    tt = numpy.array([0,t])
    pi = [0.169, -0.0, -0.717, -0.0, -0.299, 0.01]  #***********
    pf = [req.x, req.y, req.z, req.roll, req.pitch, req.yaw]
    print("Punto final xyz", pf)
    vi, vf = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    ai, af = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    c_tr = cartesian_traj(tt, pi, pf, vi, vf, ai, af)
    
    traj_q = callback_trajectory_q(c_tr, 'left',init_estim)
    traj_q_J3x7 = trajectory_q_j3x7(c_tr, 'left',init_estim)
    resp = InverseKinematicsResponse()
    resp2 = InverseKinematicsResponse()
    resp = traj_q
    resp2 = traj_q_J3x7
    
    return resp

def callback_RA_ik_for_trajectory(req):

    # Punto de inicio   GUI: Th1 = 0.2000, Th4 = 0.2000
    # - Translation: [0.171, -0.0, -0.716]
    # - Rotation: in RPY (radian) [-0.0, -0.493,- 0.0]
    # - Position q: [0.2, 0.0, 0.0, 0.2, 0.0, 0.2, 0.0]
    init_estim = [0.2, 0.0, 0.0, 0.2, 0.0, 0.2, 0.0]
    tt = numpy.array([0,t])
    pi = [0.19, -0.0, -0.703,-0.0, -0.493,- 0.0] 
    pf = [req.x, req.y, req.z, req.roll, req.pitch, req.yaw]
    vi, vf = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    ai, af = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    c_tr = cartesian_traj(tt, pi, pf, vi, vf, ai, af)
    print("trayectoria cartesiana: ",c_tr)
    traj_q = callback_trajectory_q( c_tr, 'right',init_estim)
    resp = InverseKinematicsResponse()
    resp = traj_q
    print("Servicio realizado")
    return resp

def callback_la_ik_for_pose(req):
    q = inverse_kinematics_xyzrpy(req.x, req.y, req.z, req.roll, req.pitch, req.yaw, 'left')
    if q is None:
        return None
    resp = InverseKinematicsForPoseResponse()
    [resp.q1, resp.q2, resp.q3, resp.q4, resp.q5, resp.q6, resp.q7] = [q[0], q[1], q[2], q[3], q[4], q[5], q[6]]
    return resp

def callback_ra_ik_for_pose(req):
    q = inverse_kinematics_xyzrpy(req.x, req.y, req.z, req.roll, req.pitch, req.yaw, 'right')
    if q is None:
        return False
    resp = InverseKinematicsForPoseResponse()
    [resp.q1, resp.q2, resp.q3, resp.q4, resp.q5, resp.q6, resp.q7] = [q[0], q[1], q[2], q[3], q[4], q[5], q[6]]
    return resp

def callback_la_dk(req):
    x = direct_kinematics([req.q1, req.q2, req.q3, req.q4, req.q5, req.q6, req.q7], 'left')
    resp = ForwardKinematicsResponse()
    [resp.x, resp.y, resp.z, resp.roll, resp.pitch, resp.yaw] = x
    return resp

def callback_ra_dk(req):
    x = direct_kinematics([req.q1, req.q2, req.q3, req.q4, req.q5, req.q6, req.q7], 'left')
    resp = ForwardKinematicsResponse()
    [resp.x, resp.y, resp.z, resp.roll, resp.pitch, resp.yaw] = x
    return resp

def main():
    print("INITIALIZING INVERSE KINEMATIC NODE BY MARCOSOFT...")
    rospy.init_node("ik_geometric")
    get_model_info()
    rospy.Service("/manipulation/la_inverse_kinematics", InverseKinematicsForPose, callback_la_ik_for_pose)
    rospy.Service("/manipulation/ra_inverse_kinematics", InverseKinematicsForPose, callback_ra_ik_for_pose)
    rospy.Service("/manipulation/la_direct_kinematics", ForwardKinematics, callback_la_dk)
    rospy.Service("/manipulation/ra_direct_kinematics", ForwardKinematics, callback_ra_dk)
    # Servicio que genera trayectoria en el espacio cartesiano
    rospy.Service("/manipulation/cartesian_traj", GetCartesianTrajectory, callback_trajectory_3d)
    # Servicio que resuleve la IK para un punto distante 
    rospy.Service("/manipulation/LA_inverse_kinematics", InverseKinematics, callback_LA_ik_for_trajectory)
    rospy.Service("/manipulation/RA_inverse_kinematics", InverseKinematics, callback_RA_ik_for_trajectory)
    loop = rospy.Rate(10)
    while not rospy.is_shutdown():
        jacobian_3x7

        loop.sleep()

if __name__ == '__main__':
    main()
