# -*- coding: utf-8 -*-

from __future__ import print_function
import numpy as np
import os
import math
import pinocchio as pin
from pinocchio.utils import *
from utils.math_tools import Math
import time as tm 

def setRobotParameters():

    # link lengths
    l1 = 0.05/2 + 0.05 #base_link->yaw_link (from the center of the box (box_heigth/2) + radius of the sphere)
    l2 = 0.0 # yaw_link->pitch_link (intersecting axis)
    l3 = 0.05 + 5.5/2 #spheric joint->prismatic (from the center of the sphere (radius) to half of the prismatic arm)
    l4 = 5.5/2 - 0.05 #prismatic->wrist1 (from half of the primastic link to the center of wrist1 joint (positioned on radius distance from the end of the prism arm))
    l5 = 1.0 - 0.015#wrist1->wrist2 (wrist1 length. The center of wrist2 joint is attached to the end of wrist1 arm)
    l6 = 0.015 #wrist2 joint radius
    l7 = 0.015 + 0.1 + 2*0.03 #wrist2-> end of microphone: (from the center of wrist2 joint to the ee (radius of wrist2) + end effector's length (microphone length: cylinder+sphere)
    lengths = np.array([l1, l2, l3, l4, l5, l6, l7])


    # masses
    #set m0, m1 ...

    #link_masses = np.array([m0, m1, m2, m3, m4, m5])
    
    # com of the links expressed in the respective link frame
    # com_link =  model.inertias[idx].lever (Pinocchio)
    com0 = np.array([0., 0., 0.]) # base link
    com1 = np.array([0., 0., 0.]) # shoulder link
    com2 = np.array([0.  , 0.  , 0.28]) #upper_arm_link
    com3 = np.array([0.  , 0.  , 0.25]) #forearm_link
    com4 = np.array([0., 0., 0.]) # wrist_1_link
    #w_com_link = data.oMi[idx].rotation.dot(com_link) + data.oMi[idx].translation

    # inertia tensors of the links  w.r.t. to own CoM of each link expressed in the respective link frames
    I_0 = np.array([[0.00443333156,           0.0,    0.0],
                    [          0.0, 0.00443333156,    0.0],
                    [          0.0,           0.0, 0.0072]])

    I_1 = np.array([[0.010267495893,            0.0,     0.0],
                    [           0.0, 0.010267495893,     0.0],
                    [           0.0,            0.0, 0.00666]])

    I_2 = np.array([[0.22689067591,            0.0,       0.0],
                    [           0.0, 0.22689067591,       0.0],
                    [           0.0,           0.0, 0.0151074]])
    
    I_3 = np.array([[0.049443313556,            0.0,     0.0],
                    [           0.0, 0.049443313556,     0.0],
                    [           0.0,           0.0, 0.004095]])

    I_4 = np.array([[0.111172755531,            0.0,     0.0],
                    [           0.0, 0.111172755531,     0.0],
                    [           0.0,            0.0, 0.21942]])

    inertia_tensors = np.array([I_0, I_1, I_2, I_3, I_4])

    coms = np.array([com0, com1, com2, com3, com4])

    return lengths
    #, inertia_tensors, link_masses, coms


def directKinematics(q):
    # define link lengths from urdf
    link_length = setRobotParameters()
    #,_,_,_
    l1 = link_length[0]
    l2 = link_length[1]
    l3 = link_length[2]
    l4 = link_length[3]
    l5 = link_length[4]
    l6 = link_length[5]
    l7 = link_length[6]

    q1 = q[0]
    q2 = q[1]
    d3 = q[2]   #prismatic
    q4 = q[3]
    q5 = q[4]


    # constants from URDF
    ceiling_x, ceiling_y, ceiling_z = 2.5, 6.0, 4.0     #base_link ref


    # LOCAL homogeneous transformation matrices

    # base_link
    # translation from world to ceiling
    T_w_base = np.array([[1, 0, 0, ceiling_x],
                       [0, 1, 0, ceiling_y],
                       [0, 0, 1, ceiling_z],
                       [0, 0, 0, 1]])

    # base_link->yaw_link
    # joint rotation about z axis (q1), translation l1 along -z axis
    T_base_yaw_t = np.array([[ 1, 0, 0, 0],
                       [ 0,  1, 0, 0],
                       [0, 0, 1,  -l1],
                       [ 0, 0, 0,  1]])
    T_base_yaw_r = np.array([[ math.cos(q1), -math.sin(q1), 0, 0],
                       [ math.sin(q1),  math.cos(q1), 0, 0],
                       [0, 0, 1,  0],
                       [ 0, 0, 0,  1]])
    T_base_yaw = T_base_yaw_t.dot(T_base_yaw_r)

    # yaw_link->pitch_link
    # joint rotation about y axis (q2)
    T_yaw_pitch = np.array([[math.cos(q2), 0, math.sin(q2), 0],
                       [            0, 1,            0, 0],
                       [-math.sin(q2), 0, math.cos(q2), 0],
                       [0, 0, 0, 1]])

    #pitch_link->prismatic_link
    # translation l3 along -z
    T_pitch_prism_t = np.array([[1, 0, 0, 0],
                                [0, 1, 0, 0],
                                [0, 0, 1, -l3],
                                [0, 0, 0, 1]])
    # prismatic extension d3 along -z
    T_pitch_prism_ext = np.array([[1, 0, 0, 0],
                                [0, 1, 0, 0],
                                [0, 0, 1, d3],
                                [0, 0, 0, 1]])
    T_pitch_prism = T_pitch_prism_t.dot(T_pitch_prism_ext)

    # prismatic_link->wrist_1
    # joint rotation about y axis (q4), translation of l4 along -z
    T_prism_wrist1_t = np.array([[1, 0, 0, 0],
                        [ 0, 1, 0, 0],
                        [0, 0, 1, -l4],
                        [ 0, 0, 0,  1]])
    T_prism_wrist1_r = np.array([[math.cos(q4), 0, math.sin(q4), 0],
                        [ 0, 1, 0, 0],
                        [-math.sin(q4), 0, math.cos(q4), 0],
                        [ 0, 0, 0,  1]])
    T_prism_wrist1 = T_prism_wrist1_t.dot(T_prism_wrist1_r)

    # wrist1->wrist2
    # joint rotation about y axis (q5), translation of l6 along -z
    T_wrist1_wrist2_t = np.array([[1, 0, 0, 0],
                       [0,  1, 0, 0],
                       [0,  0,  1, -l5],
                        [ 0, 0, 0, 1]])
    T_wrist1_wrist2_r = np.array([[math.cos(q5), 0, math.sin(q5), 0],
                       [0,  1, 0, 0],
                       [-math.sin(q5),  0,  math.cos(q5), 0],
                        [ 0, 0, 0, 1]])
    T_wrist1_wrist2 = T_wrist1_wrist2_t.dot(T_wrist1_wrist2_r)

    # wrist2->end-effector
    # only rigid translation -z of l6
    T_wrist2_end_wrist2 = np.array([[1,  0, 0,  0],
                     [0,   1, 0, 0],
                     [0,   0, 1, -l6],
                     [0,   0, 0,  1]])
    #offset of microphone
    T_wrist2_end_mic = np.array([[1,  0, 0,  0],
                     [0,   1, 0, 0],
                     [0,   0, 1, -l7],
                     [0,   0, 0,  1]])

    T_wrist2_mic = T_wrist2_end_wrist2.dot(T_wrist2_end_mic)

    # GLOBAL homogeneous transformation matrices
    T_w_jyaw = T_w_base.dot(T_base_yaw)
    T_w_jpitch = T_w_jyaw.dot(T_yaw_pitch)
    T_w_prism = T_w_jpitch.dot(T_pitch_prism)
    T_w_wr1 = T_w_prism.dot(T_prism_wrist1)
    T_w_wr2 = T_w_wr1.dot(T_wrist1_wrist2)
    T_w_mic = T_w_wr2.dot(T_wrist2_mic)

    return T_w_base, T_w_jyaw, T_w_jpitch, T_w_prism, T_w_wr1, T_w_wr2, T_w_mic

'''
    This function computes the Geometric Jacobian of the end-effector expressed in the base link frame 
'''
def computeEndEffectorJacobian(q):

    # compute direct kinematics 
    T_w_base, T_w_jyaw, T_w_jpitch, T_w_prism, T_w_wr1, T_w_wr2, T_w_mic = directKinematics(q)


    # link position vectors
    p_w_jyaw = T_w_jyaw[:3,3]
    p_w_jpitch = T_w_jpitch[:3,3]
    p_w_wr1 = T_w_wr1[:3,3]
    p_w_wr2 = T_w_wr2[:3,3]
    p_w_mic = T_w_mic[:3,3]


    # z vectors for rotations
    z1 = T_w_jyaw[:3,2] # Z axis
    z2 = T_w_jpitch[:3,1] # Y axis
    d3 = T_w_prism[:3,2] # translation Z axis
    z4 = T_w_wr1[:3,1] # Y axis
    z5 = T_w_wr2[:3,1] # Y axis

    # vectors from link i to end-effector
    p_w__jyaw_mic = p_w_mic - p_w_jyaw
    p_w__jpitch_mic = p_w_mic - p_w_jpitch
    p_w__wr1_mic = p_w_mic - p_w_wr1
    p_w__wr2_mic = p_w_mic - p_w_wr2

    # linear and angular part of Jacobian matrix. z3 (called d3 is the prismatic joint so J_p and J_o are computed accordingly)
    J_p = np.hstack((np.cross(z1,p_w__jyaw_mic).reshape(3,1) , np.cross(z2,p_w__jpitch_mic).reshape(3,1) , d3.reshape(3, 1), np.cross(z4,p_w__wr1_mic).reshape(3,1) , np.cross(z5,p_w__wr2_mic).reshape(3,1) ))
    J_o = np.hstack((z1.reshape(3,1), z2.reshape(3,1), np.zeros((3,1)), z4.reshape(3,1), z5.reshape(3,1)))

    # Jacobian matrix and joint axes both expressed in the world frame) 
    J = np.vstack(( J_p, J_o))

    return J,z1,z2,d3,z4,z5


def geometric2analyticJacobian(J,T_w_mic):
    R_w_mic = T_w_mic[:3,:3]
    math_utils = Math()
    rpy_ee = math_utils.rot2eul(R_w_mic)
    roll = rpy_ee[0]
    pitch = rpy_ee[1]
    yaw = rpy_ee[2]

    # compute the mapping between euler rates and angular velocity
    T_w = np.array([[math.cos(yaw)*math.cos(pitch),  -math.sin(yaw), 0],
                    [math.sin(yaw)*math.cos(pitch),   math.cos(yaw), 0],
                    [             -math.sin(pitch),               0, 1]])

    T_a = np.array([np.vstack((np.hstack((np.identity(3), np.zeros((3,3)))),
                                          np.hstack((np.zeros((3,3)),np.linalg.inv(T_w)))))])


    J_a = np.dot(T_a, J)

    return J_a[0]


def numericalInverseKinematics(p_d, q0, line_search = False, wrap = False):
    math_utils = Math()

    # hyper-parameters
    epsilon = 1e-06 # Tolerance for stopping criterion
    lambda_ = 1e-08  # Regularization or damping factor (1e-08->0.01)
    max_iter = 100  # Maximum number of iterations
    # For line search only
    #gamma = 0.5
    beta = 0.5 # Step size reduction

    # initialization of variables
    iter = 0
    alpha = 1  # Step size
    log_grad = []
    log_err = []

    # Inverse kinematics with line search
    while True:
        # evaluate  the kinematics for q0
        J,_,_,_,_,_ = computeEndEffectorJacobian(q0)
        _, _, _, _, _, _, T_w_mic = directKinematics(q0)

        p_mic = T_w_mic[:3,3]
        R = T_w_mic[:3,:3]
        rpy = math_utils.rot2eul(R)
        roll = rpy[0]
        p_mic = np.append(p_mic,roll)

        # error
        e_bar = p_mic - p_d
        J_bar = geometric2analyticJacobian(J,T_w_mic)
        # take first 4 rows correspondent to our task
        J_bar = J_bar[:4,:]
        # evaluate the gradient
        grad = J_bar.T.dot(e_bar)

        log_grad.append(np.linalg.norm(grad))
        log_err.append(np.linalg.norm(e_bar))

        if np.linalg.norm(grad) < epsilon:
            print("IK Convergence achieved!, norm(grad) :", np.linalg.norm(grad) )
            print("Inverse kinematics solved in {} iterations".format(iter))     
            break
        if iter >= max_iter:                
            print("Warning: Max number of iterations reached, the iterative algorithm has not reached convergence to the desired precision. Error is:  ", np.linalg.norm(e_bar))
            break
        # Compute the error
        JtJ= np.dot(J_bar.T,J_bar) + np.identity(J_bar.shape[1])*lambda_
        JtJ_inv = np.linalg.inv(JtJ)
        P = JtJ_inv.dot(J_bar.T)
        dq = - P.dot(e_bar)

        if not line_search:
            q1 = q0 + dq * alpha
            q0 = q1
        else:
            print("Iter # :", iter)
            # line search loop
            while True:
                #update
                q1 = q0 + dq*alpha
                # evaluate  the kinematics for q1
                _, _, _, _, _, _, T_w_mic1 = directKinematics(q1)
                p_mic1 = T_w_mic1[:3,3]
                R1 = T_w_mic1[:3,:3]
                rpy1 = math_utils.rot2eul(R1)
                roll1 = rpy1[0]
                p_mic1 = np.append(p_mic1,roll1)
                e_bar_new = p_mic1 - p_d
                #print "e_bar1", np.linalg.norm(e_bar_new), "e_bar", np.linalg.norm(e_bar)

                error_reduction = np.linalg.norm(e_bar) - np.linalg.norm(e_bar_new)
                threshold = 0.0 # more restrictive gamma*alpha*np.linalg.norm(e_bar)

                if error_reduction < threshold:
                    alpha = beta*alpha
                    print (" line search: alpha: ", alpha)
                else:
                    q0 = q1
                    alpha = 1
                    break

        iter += 1
           

 
    # wrapping prevents from outputs outside the range -2pi, 2pi
    if wrap:
        for i in range(len(q0)):
            while q0[i] >= 2 * math.pi:
                q0[i] -= 2 * math.pi
            while q0[i] < -2 * math.pi:
                q0[i] += 2 * math.pi

    return q0, log_err, log_grad




