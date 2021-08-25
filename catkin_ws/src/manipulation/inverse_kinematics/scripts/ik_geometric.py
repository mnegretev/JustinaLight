#!/usr/bin/env python
import math
import rospy
import tf
import tf.transformations as tft
import numpy
import urdf_parser_py.urdf
from geometry_msgs.msg import PointStamped
from custom_msgs.msg import ArmConfiguration
from custom_msgs.srv import InverseKinematicsForPose
from custom_msgs.srv import InverseKinematicsForPoseRequest
from custom_msgs.srv import InverseKinematicsForPoseResponse

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

def are_valid_angles(q, arm):
    for i in range(len(q)):
        if q[i] < joints[arm][i].limit.lower or q[i] > joints[arm][i].limit.upper:
            print("InverseKinematics.->Candidate solution out of angle bounds")
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

def _inverse_kinematics_xyzrpy(x, y, z, roll, pitch, yaw, arm):
    pd = numpy.asarray([x,y,z,roll,pitch,yaw])
    q  = numpy.zeros(7)
    #q  = numpy.asarray([numpy.random.uniform(joints[arm][i].limit.lower, joints[arm][i].limit.upper) for i in range(7)])
    p  = direct_kinematics(q, arm)
    iterations = 0
    while numpy.linalg.norm(p - pd) > 0.01 and iterations < 100:
        J = jacobian(q, arm)
        q = (q - numpy.dot(numpy.linalg.pinv(J), (p - pd)))%(2*math.pi)
        p = direct_kinematics(q, arm)
        iterations +=1
    for i in range(len(q)):
        q[i] = q[i] - 2*math.pi if q[i] > math.pi else q[i]
    if iterations < 100:
        print("InverseKinematics.->IK for " + arm + " arm solved after " + str(iterations) + " iterations: " + str(q))
        return q
    else:
        print("InverseKinematics.->Cannot solve IK for " + arm + " arm. Max attempts exceeded. ")
        return None

def inverse_kinematics_xyzrpy(x, y, z, roll, pitch, yaw, arm):
    pd = numpy.asarray([x,y,z,roll,pitch,yaw])
    attempts = 0
    solved = False
    while not solved and attempts < 50:
        q  = numpy.asarray([numpy.random.uniform(joints[arm][i].limit.lower, joints[arm][i].limit.upper) for i in range(7)])*(attempts/20.0)
        p  = direct_kinematics(q, arm)
        iterations = 0
        print("Current attempt: " + str(attempts))
        while numpy.linalg.norm(p - pd) > 0.01 and iterations < 20:
            J = jacobian(q, arm)
            q = (q - numpy.dot(numpy.linalg.pinv(J), (p - pd)))%(2*math.pi)
            p = direct_kinematics(q, arm)
            iterations +=1
        if iterations < 20:
            for i in range(len(q)):
                q[i] = q[i] - 2*math.pi if q[i] > math.pi else q[i]
            solved = are_valid_angles(q, arm)
        attempts+=1
    if solved:
        print("InverseKinematics.->IK for " + arm + " arm solved after " + str((attempts-1)*50 + iterations) + " iterations: " + str(q))
        return q
    else:
        print("InverseKinematics.->Cannot solve IK for " + arm + " arm. Max attempts exceeded. ")
        print("Last q: " + str(q))
        return None

def inverse_kinematics_xyz(x, y, z, arm):
    pd = numpy.asarray([x,y,z])
    attempts = 0
    solved = False
    while not solved and attempts < 50:
        #q  = numpy.asarray([numpy.random.uniform(joints[arm][i].limit.lower, joints[arm][i].limit.upper) for i in range(7)])*(attempts/20.0)
        q  = numpy.zeros(7)
        p  = direct_kinematics(q, arm)[0:3]
        iterations = 0
        print("Current attempt: " + str(attempts))
        while numpy.linalg.norm(p - pd) > 0.01 and iterations < 20:
            J = jacobian(q, arm)[0:3,:]
            q = (q - numpy.dot(numpy.linalg.pinv(J), (p - pd)))%(2*math.pi)
            p = direct_kinematics(q, arm)[0:3]
            iterations +=1
        if iterations < 20:
            for i in range(len(q)):
                q[i] = q[i] - 2*math.pi if q[i] > math.pi else q[i]
            solved = are_valid_angles(q, arm)
        attempts+=1
    if solved:
        print("InverseKinematics.->IK for " + arm + " arm solved after " + str((attempts-1)*50 + iterations) + " iterations: " + str(q))
        return q
    else:
        print("InverseKinematics.->Cannot solve IK for " + arm + " arm. Max attempts exceeded. ")
        print("Last q: " + str(q))
        return None

def callback_la_ik_for_pose(req):
    q = inverse_kinematics_xyzrpy(req.x, req.y, req.z, req.roll, req.pitch, req.yaw, 'left')
    #q = inverse_kinematics_xyz(req.x, req.y, req.z, 'left')
    if q is None:
        return None
    resp = InverseKinematicsForPoseResponse()
    [resp.q1, resp.q2, resp.q3, resp.q4, resp.q5, resp.q6, resp.q7] = [q[0], q[1], q[2], q[3], q[4], q[5], q[6]]
    return resp

def callback_ra_ik_for_pose(req):
    q = inverse_kinematics_xyzrpy(req.x, req.y, req.z, req.roll, req.pitch, req.yaw, [0,0,0,0,0,0,0], 'right')
    if q is None:
        return False
    resp = InverseKinematicsForPoseResponse()
    [resp.q1, resp.q2, resp.q3, resp.q4, resp.q5, resp.q6, resp.q7] = [q[0], q[1], q[2], q[3], q[4], q[5], q[6]]
    return resp

def main():
    print("INITIALIZING INVERSE KINEMATIC NODE BY MARCOSOFT...")
    rospy.init_node("ik_geometric")
    get_model_info()
    rospy.Service("/manipulation/la_inverse_kinematics", InverseKinematicsForPose, callback_la_ik_for_pose)
    rospy.Service("/manipulation/ra_inverse_kinematics", InverseKinematicsForPose, callback_ra_ik_for_pose)
    loop = rospy.Rate(10)
    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == '__main__':
    main()
