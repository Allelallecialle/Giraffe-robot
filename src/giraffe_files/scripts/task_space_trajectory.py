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

def task_simulation(robot, frame_id, ros_pub, p_des, rpy_des):
    q0 = conf.q0
    qd0 = conf.qd0
    qdd0 = conf.qdd0
 
    q = conf.q0
    qd = conf.qd0
    qdd = conf.qdd0

    time = 0.0
    T = conf.T     # trajectory duration
    zero_cart = np.array([ 0.0, 0.0,0.0])

    # Compute initial end effector position and velocity from q0
    #p0 = robot.framePlacement(conf.q0, conf.frame_name, True).translation
    p0 = robot.data.oMf[robot.model.getFrameId(conf.frame_name)].translation.copy()
    rpy0 = pin.rpy.matrixToRpy(robot.data.oMf[robot.model.getFrameId(conf.frame_name)].rotation)
    pd0 = zero_cart
    pdd0 = zero_cart

    # initialize actual variables
    p = p0
    pd = pd0
    pdd = pdd0
    rpy = zero_cart
    # initialize reference variables
    p_des = p_des
    pd_des = zero_cart
    pdd_des = zero_cart
    rpy_des = rpy_des

    # CONTROL LOOP
    while True:
        pol_trj_simulation(robot, frame_id, ros_pub, np.append(p_des, rpy_des))
        robot.computeAllTerms(q, qd) 
        # joint space inertia matrix                
        M = robot.mass(q, False)
        # bias terms                
        h = robot.nle(q, qd, False)
        #gravity terms                
        g = robot.gravity(q)
        
        # compute the Jacobian of the end-effector in the world frame
        J6 = robot.frameJacobian(q, conf.frame_name, False, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)                    
        # take first 3 rows of J6 cause we have a point contact            
        J = J6[:3,:] 
        # compute  the end-effector acceleration due to joint velocity Jdot*qd         
        Jdqd = robot.frameClassicAcceleration(q, qd, None, conf.frame_name).linear    
        # compute frame end effector position and velocity in the WF   
        p = robot.framePlacement(q, conf.frame_name).translation  

        pd = J.dot(qd)  

        M_inv = np.linalg.inv(M)
        # Moore-penrose pseudoinverse  A^# = (A^T*A)^-1 * A^T with A = J^T
        JTpinv = np.linalg.inv(J.dot(J.T)).dot(J)
        
        # joint space inertia matrix reflected at the end effector (J*M^-1*Jt)^-1
        lambda_= np.linalg.inv(J.dot(M_inv).dot(J.T))  # J should be full row rank  otherwise add a damping term
        

        # PD control (cartesian task) + postural task
        # null space torques (postural task)
        tau0 = conf.Kp_postural*(conf.q0-q) - conf.Kd_postural*qd
        tau_null = N.dot(tau0)
        tau = J.T.dot(F_des)  + tau_null

        #plot position
        plotEndeff('ee position', 1,time_log, p_log, p_des_log)
        #plotEndeff('velocity', 2, time_log, p_log, p_des_log, pd_log, pd_des_log, rpy_log, rpy_des_log)
        try:
            ORIENTATION_CONTROL
            plotEndeff('euler angles', 3,time_log, rpy_log, rpy_des_log)
            plotEndeff('orientation error', 4,time_log, error_o_log)
        except: 
            pass   
        plt.show(block=True)


