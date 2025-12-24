#!/usr/bin/env python3

import pinocchio as pin
from pinocchio.utils import *
import numpy as np
from numpy import nan
import math
import time as tm 
from utils.common_functions import *
from utils.kin_utils import *
from utils.trajectory_utils import fifthOrderPolynomialTrajectory, compute_trajectory
from polynomial_trajectory import *
from utils.ros_publish import RosPub
import conf as conf

def refined_task_simulation(robot, frame_id, ros_pub, p_des, rpy_des):
    q0 = conf.q0
    qd0 = conf.qd0
    qdd0 = conf.qdd0
 
    q = conf.q0
    qd = conf.qd0
    qdd = conf.qdd0

    time = 0.0
    T = conf.T     # trajectory duration
    zero_cart = np.array([ 0.0, 0.0,0.0])

    # initialize actual variables
    p = p0
    pd = pd0
    pdd = pdd0
    rpy = zero_cart
    # initialize reference variables
    p_des = p_des
    pd_des = zero_cart
    pdd_des = zero_cart
    rpy_des = rpy_desù

    # Compute initial end effector position and velocity from q0
    p0 = robot.data.oMf[robot.model.getFrameId(conf.frame_name)].translation.copy()
    rpy0 = pin.rpy.matrixToRpy(robot.data.oMf[robot.model.getFrameId(conf.frame_name)].rotation)
    pd0 = zero_cart
    pdd0 = zero_cart

    q_f, _, _ = numericalInverseKinematics(np.append(p_des, rpy_des), p0, line_search = False, wrap = False)
    while time < T:
        # as done in polynomial_trajestory
        p, pd, pdd = compute_trajectory(p0, q_f, T, time)
        

        #pol_trj_simulation(robot, frame_id, ros_pub, np.append(p_des, rpy_des))
        # define the error e on position and pitch degrees (4 dimesions)
        e_p = p_des - p
        e_rpy = rpy_des - rpy
        total_error = np.hstack([e_p, e_rpy])
        # same on velocity
        e_pd = pd_des - pd
        e_rpy_d = e_rpy_d_des - rpy_d
        task_vel_error = np.hstack([v_error, pitch_vel_error])
        # computed torque control: edd + Kd*ed + Kp*e = 0
        pdd_des = pdd_des + conf.Kd_pos @ v_error + conf.Kp_pos @ p_error
        rpy_dd_des = pitch_acc_des + conf.Kd_pitch * pitch_vel_error + conf.Kp_pitch * pitch_error
        total_error_dd = np.hstack([pdd_des, rpy_dd_des])






        #publish joint variables
        ros_pub.publish(robot, q, qd)
        tm.sleep(conf.dt*conf.SLOW_FACTOR)

        # stops the while loop if  you prematurely hit CTRL+C
        if ros_pub.isShuttingDown():
            print ("Shutting Down")
            break
        
    ros_pub.deregister_node()