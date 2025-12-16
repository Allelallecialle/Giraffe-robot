#!/usr/bin/env python3

import pinocchio as pin
from pinocchio.utils import *
import numpy as np
from numpy import nan
import math
import time as tm 
from utils.common_functions import *
from utils.trajectory_utils import fifthOrderPolynomialTrajectory, compute_trajectory
from utils.ros_publish import RosPub
import conf as conf


time_log = []
q_final_log = []
qd_final_log = []
qdd_final_log = []

T = 2.0     # trajectory duration

q0 = conf.q0
qd0 = conf.qd0
qdd0 = conf.qdd0


def gen_trajectory(robot, frame_id, q_des, qd_des, qdd_des):
    t = 0.0
    # compute coefficients once
    coeffs_all = compute_trajectory(q0, q_des, qd0, qd_des, qdd0, qdd_des, T)

    while t <= T:
        q_final = []
        qd_final = []
        qdd_final = []
        for i in range(len(q0)):
            q, qd, qdd = evaluate_pol(coeffs_all[i], t)
            q_final.append(q)
            qd_final.append(qd)
            qdd_final.append(qdd)

        #ros_pub.publish(robot, q_des, qd_des)

        q_final = np.array(q_final)
        qd_final = np.array(qd_final)
        qdd_final = np.array(qdd_final)

        t += dt

    return q_final, qd_final

def test_trajectory(robot, frame_id):
    # final random desired joint state
    q_des = np.random.uniform(
        low=[-np.pi, -np.pi/2, 0.1, -np.pi/2, -np.pi/2],
        high=[ np.pi,  np.pi/2, 5.0,  np.pi/2,  np.pi/2]
    )
    qd_des  = np.zeros(5)
    qdd_des = np.zeros(5)
    print("Random target q_des (rad):", q_des)


    q_final, qd_final = gen_trajectory(robot, frame_id, q_des, qd_des, qdd_des)
    # Update fk
    pin.forwardKinematics(robot.model, robot.data, q_final, qd_final)
    pin.updateFramePlacement(robot.model, robot.data, frame_id)

    final_pos = data.oMf[frame_id].translation
    final_orient_rpy = pin.rpy.matrixToRpy(data.oMf[frame_id].rotation)
    print("Final Position of the ee:", final_pos)
    print("Final Orientation of the ee (RPY - deg):", np.degrees(final_orient_rpy))
    print("Pitch final (deg):", np.degrees(final_orient_rpy[1]))
    print("Pitch desired (deg):", np.degrees(pitch_des_final))