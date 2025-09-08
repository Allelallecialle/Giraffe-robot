#!/usr/bin/env python3

import pinocchio as pin
from pinocchio.utils import *
import numpy as np
from numpy import nan
import math
import time as tm 
from utils.common_functions import *
from utils.dyn_utils import getg, getM, getC, forward_dynamics
from utils.ros_publish import RosPub
import conf as conf

def dynamics_test(robot, frame_id):
    # Init loggers
    q_log = np.empty((5))*nan
    q_des_log = np.empty((5))*nan
    qd_log = np.empty((5))*nan
    qd_des_log = np.empty((5))*nan
    qdd_log = np.empty((5))*nan
    qdd_des_log = np.empty((5))*nan
    tau_log = np.empty((5))*nan
    f_log = np.empty((3,0))*nan
    x_log = np.empty((3,0))*nan
    time_log =  np.empty((0,0))*nan

    time = 0.0

    # randomized initial guess. This np function draws from the interval of values set by the first 2 parameters (360 degrees) 5 numbers (as the joints) that are our q0s.
    q = np.random.uniform(-np.pi, np.pi, 5)
    qd = np.random.uniform(-np.pi, np.pi, 5)
    qdd = np.random.uniform(-np.pi, np.pi, 5)
    print(f"Initial random joint positions q: {q}")
    print(f"Initial random joint velocities qd: {qd}")
    print(f"Initial random joint accelerations qdd: {qdd}")



    # initialize Pinocchio variables
    robot.computeAllTerms(q, qd)
    joint_types = np.array(['revolute', 'revolute', 'prismatic', 'revolute', 'revolute'])
    # compute RNEA
    tau = pin.rnea(robot.model, robot.data, q, qd, qdd)
    print(f"RNEA: {tau}")

    print("------------------------------------------")
    # Compute g,M,C
    # gravity terms
    g = getg(q, robot, joint_types=joint_types)
    # compute joint space inertia matrix with Pinocchio
    M = getM(q,robot,joint_types=joint_types)
    # compute Coriolis term with Pinocchio
    C = getC(q,qd,robot,joint_types=joint_types)

    print(f"Gravity: {g}")
    print(f"Inertia M: {M}")
    print(f"Coriolis C: {C}")
    
    print("------------------------------------------")
    #Compute forward dynamics
    qdd_fd = forward_dynamics(robot,joint_types,tau,q,qd)
    print(f"Joint acceleration qdd computed by forward dynamics: {qdd}")
    #Compare the input acceleration and the one computed with forward dynamics
    print(f"Difference of accelerations, input and computed with forward dynamics: {qdd_fd-qdd}")

    print("------------------------------------------")
    # Simulation of forward dynamics
    # Forward Euler Integration
    qd = qd + qdd_fd * conf.dt
    q = q + conf.dt * qd  + 0.5 * pow(conf.dt,2) * qdd_fd

    # Log Data into a vector
    time_log = np.append(time_log, time)
    q_log = np.vstack((q_log, q ))
    q_des_log= np.vstack((q_des_log, q_des))
    qd_log= np.vstack((qd_log, qd))
    qd_des_log= np.vstack((qd_des_log, qd_des))
    qdd_log= np.vstack((qdd_log, qdd_fd))
    qdd_des_log= np.vstack((qdd_des_log, qdd_des))

    # update time
    time = time + conf.dt
                
    #publish joint variables
    tm.sleep(conf.dt*conf.SLOW_FACTOR)
    
    # plot joint variables
    plotJoint('position', time_log, q_log, q_des_log, qd_log, qd_des_log, qdd_log, qdd_des_log, tau_log)
    plotJoint('velocity', time_log, q_log, q_des_log, qd_log, qd_des_log, qdd_log, qdd_des_log, tau_log)
    plotJoint('acceleration', time_log, q_log, q_des_log, qd_log, qd_des_log, qdd_log, qdd_des_log, tau_log)
    plotJoint('torque', time_log, q_log, q_des_log, qd_log, qd_des_log, qdd_log, qdd_des_log, tau_log)