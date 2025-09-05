#!/usr/bin/env python3

import pinocchio as pin
from pinocchio.utils import *
import numpy as np
from numpy import nan
import math
import time as tm 
from utils.common_functions import *
from utils.ros_publish import RosPub
from utils.kin_utils import directKinematics, computeEndEffectorJacobian, geometric2analyticJacobian, numericalInverseKinematics
from utils.math_tools import Math
import matplotlib.pyplot as plt
import conf as conf

math_utils = Math()

def direct_kin_test(robot, frame_id, q, qd):
    # direct kinematics function
    T_w_base, T_w_jyaw, T_w_jpitch, T_w_prism, T_w_wr1, T_w_wr2, T_w_mic = directKinematics(q)
    # compare with Pinocchio built-in functions 
    robot.computeAllTerms(q, qd)
    x = robot.framePlacement(q, frame_id).translation
    o = robot.framePlacement(q, frame_id).rotation
    position_diff = x - T_w_mic[:3,3]
    rotation_diff = o - T_w_mic[:3,:3]
    print("Direct Kinematics - ee position, difference with Pinocchio library:", position_diff)
    print("Direct Kinematics - ee orientation, difference with Pinocchio library:\n", rotation_diff)
    print("------------------------------------------")

def jacobian_test(frame_id, robot, q):
    _, _, _, _, _, _, T_w_mic = directKinematics(q)
    J,z1,z2,d3,z4,z5 = computeEndEffectorJacobian(q)
    # compare with Pinocchio
    Jee = robot.frameJacobian(q, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    jacobian_diff = J - Jee
    print("Direct Kinematics - ee Geometric Jacobian (6X4 matrix), difference with Pinocchio library:\n", jacobian_diff)

    J_a = geometric2analyticJacobian(J, T_w_mic)
    print("Geometric Jacobian:\n", J)
    print("Analytic Jacobian:\n", J_a)
    print("------------------------------------------")

def inv_kin_test(frame_id):
   
    # randomized initial guess. This np function draws from the interval of values set by the first 2 parameters (360 degrees) 6 numbers (as the joints) that are our q0s.
    q_i  = conf.q0

    p =  np.random.uniform(-np.pi, np.pi, 4)

    # solution of the numerical ik
    q_f, log_err, log_grad = numericalInverseKinematics(p, q_i, line_search = False, wrap = False)

    # compare ik solution with dk results
    T_w_base, T_w_jyaw, T_w_jpitch, T_w_prism, T_w_wr1, T_w_wr2, T_w_mic = directKinematics(q_f)
    rpy = math_utils.rot2eul(T_w_mic[:3,:3])
    task_diff = p - np.hstack((T_w_mic[:3,3],rpy[0]))

    print("Desired End effector \n", p)
    print("Point obtained with IK solution \n", np.hstack((T_w_mic[:3, 3], rpy[0])))
    print("Norm of error at the end-effector position: \n", np.linalg.norm(task_diff))
    print("Initial joint positions\n", q_i)
    print("Final joint positions\n", q_f)

