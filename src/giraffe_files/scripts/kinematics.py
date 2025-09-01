#!/usr/bin/env python3

import pinocchio as pin
from pinocchio.utils import *
import numpy as np
from numpy import nan
import math
import time as tm 
from utils.common_functions import *
from utils.ros_publish import RosPub
from utils.kin_dyn_utils import directKinematics
from utils.kin_dyn_utils import computeEndEffectorJacobian
from utils.kin_dyn_utils import geometric2analyticJacobian
from utils.math_tools import Math
import matplotlib.pyplot as plt


def direct_kin_test(robot, frame_id, q, qd):
    # direct kinematics function
    T_w_base, T_w_jyaw, T_w_jpitch, T_w_prism, T_w_wr, T_w_mic, T_w_t = directKinematics(q)
    # compare with Pinocchio built-in functions 
    robot.computeAllTerms(q, qd)
    x = robot.framePlacement(q, frame_id).translation
    o = robot.framePlacement(q, frame_id).rotation
    position_diff = x - T_w_t[:3,3]
    rotation_diff = o - T_w_t[:3,:3]
    print("Direct Kinematics - ee position, difference with Pinocchio library:", position_diff)
    print("Direct Kinematics - ee orientation, difference with Pinocchio library:\n", rotation_diff)
    print("------------------------------------------")

def jacobian_test(frame_id, robot, q):
    _, _, _, _, _, _, T_w_t = directKinematics(q)
    J,z1,z2,z3,z4,z5 = computeEndEffectorJacobian(q)
    # compare with Pinocchio
    Jee = robot.frameJacobian(q, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    jacobian_diff = J - Jee
    print("Direct Kinematics - ee Geometric Jacobian (6X4 matrix), difference with Pinocchio library:\n", jacobian_diff)

    J_a = geometric2analyticJacobian(J, T_w_t)
    print("Geometric Jacobian:\n", J)
    print("Analytic Jacobian:\n", J_a)