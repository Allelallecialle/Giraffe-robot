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
    q_log = np.empty((0, 5))
    q_des_log = np.empty((0, 5))
    qd_log = np.empty((0, 5))
    qd_des_log = np.empty((0, 5))
    qdd_log = np.empty((0, 5))
    qdd_des_log = np.empty((0, 5))
    tau_log = np.empty((0, 5))
    time_log = np.empty((0,))
    
    # desired positions set to the default initial positions in conf.py
    q_des = conf.q0
    qd_des = conf.qd0
    qdd_des = conf.qdd0

    time = 0.0
    dyn_sim_duration = 1.0

    # randomized initial guess of robot position. This np function draws from the interval of values set by the first 2 parameters (360 degrees) 5 numbers (as the joints) that are our q0s.
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
    print(f"Joint acceleration qdd_fd computed by forward dynamics: {qdd_fd}")
    #Compare the input acceleration and the one computed with forward dynamics
    print(f"Difference of accelerations. Input qdd - qdd forward dynamics (qdd_fd-qdd): {qdd_fd-qdd}")

    print("------------------------------------------")
    # Simulation of forward dynamics
    # Add damping to stop smoothly
    damping =  -0.5*qd
    end_stop_tau = np.zeros(5)
    jl_K = 10000
    jl_D = 100
    q_max = np.array([np.pi,   np.pi/2, 5.5, np.pi/2, np.pi/2])
    q_min = np.array([-np.pi, -np.pi/2, 0.0, -np.pi/2, -np.pi/2])
    end_stop_tau =  (q > q_max) * (jl_K * (q_max - q) + jl_D * (-qd)) +  (q  < q_min) * (jl_K * (q_min - q) + jl_D * (-qd))

    final_tau = end_stop_tau + damping
    # Main loop to simulate dynamics
    while time < dyn_sim_duration:
        # Forward Euler Integration
        qd = qd + qdd_fd * conf.dt
        q = q + conf.dt * qd  + 0.5 * pow(conf.dt,2) * qdd_fd

        # Log Data into a vector
        time_log = np.append(time_log, time)
        q_log = np.vstack((q_log, q))
        q_des_log= np.vstack((q_des_log, q_des))
        qd_log= np.vstack((qd_log, qd))
        qd_des_log= np.vstack((qd_des_log, qd_des))
        qdd_log= np.vstack((qdd_log, qdd_fd))
        qdd_des_log= np.vstack((qdd_des_log, qdd_des))
        tau_log = np.vstack((tau_log, final_tau))
        
        # update time
        time = time + conf.dt
                    
        #publish joint variables
        tm.sleep(conf.dt*conf.SLOW_FACTOR)
    
    # Transpose the stacked arrays to plot correctly
    q_log = np.array(q_log).T
    qd_log = np.array(qd_log).T
    qdd_log = np.array(qdd_log).T
    q_des_log = np.array(q_des_log).T
    qd_des_log = np.array(qd_des_log).T
    qdd_des_log = np.array(qdd_des_log).T
    tau_log = np.array(tau_log).T

    # plot joint variables in 4 graphs
    plotJoint('position', time_log, q_log, q_des_log, qd_log, qd_des_log, qdd_log, qdd_des_log, tau_log)
    plt.figure(1)
    plotJoint('velocity', time_log, q_log, q_des_log, qd_log, qd_des_log, qdd_log, qdd_des_log, tau_log)
    plt.figure(2)
    plotJoint('acceleration', time_log, q_log, q_des_log, qd_log, qd_des_log, qdd_log, qdd_des_log, tau_log)
    plt.figure(3)
    plotJoint('torque', time_log, q_log, q_des_log, qd_log, qd_des_log, qdd_log, qdd_des_log, tau_log)
    plt.figure(4)
    plt.show()
    input('Press enter to continue')