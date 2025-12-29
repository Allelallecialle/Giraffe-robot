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

def dynamics_test(robot, frame_id, ros_pub, q_des, qd_des, qdd_des):
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
    q = conf.q0
    qd = conf.qd0
    qdd = conf.qdd0

    time = 0.0
    dyn_sim_duration = 2.0

    print(f"Initial joint positions q: {q}")
    print(f"Initial joint velocities qd: {qd}")
    print(f"Initial joint accelerations qdd: {qdd}")
    print("------------------------------------------")
    print(f"Final random joint positions q: {q_des}")
    print(f"Final random joint velocities qd: {qd_des}")
    print(f"Final random joint accelerations qdd: {qdd_des}")

    # initialize Pinocchio variables
    robot.computeAllTerms(q, qd)
    joint_types = np.array(['revolute', 'revolute', 'prismatic', 'revolute', 'revolute'])
    
    # Main loop to simulate dynamics
    while time < dyn_sim_duration:
        # compute RNEA
        #tau = pin.rnea(robot.model, robot.data, q_des, qd_des, qdd_des)
        tau = pin.rnea(robot.model, robot.data, q, qd, qdd)
        print(f"RNEA: {tau}")

        print("------------------------------------------")
        # Compute g,M,C with pinocchio
        # gravity term
        g = robot.gravity(q)

        # compute joint space inertia matrix with Pinocchio
        M  = np.zeros((5,5))
        M = robot.mass(q, False)
        #for i in range(5):
        #   ei = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        #   ei[i] = 1
        #   tau = pin.rnea(robot.model, robot.data, q, np.array([0.0,0.0,0.0,0.0,0.0]) ,ei)
        #   M[:5,i] = tau - g

        # compute bias term with Pinocchio (C+g)
        h = robot.nle(q, qd, False)
        
        #shorten the output to make it more readable
        np.set_printoptions(suppress=True, precision=3)
        print(f"Gravity: {g}")
        print(f"Inertia M: {M}")
        print(f"Coriolis C: {h - g}")
        print(f"Bias term h: {h}")
        
        print("------------------------------------------")
        # Simulation of forward dynamics
        # Add damping to stop smoothly
        damping =  -20 * qd
        jl_K = 10000
        jl_D = 10
        q_min = np.array([-np.pi, -np.pi/2, 0.0, -np.pi/2, -np.pi/2])
        q_max = np.array([np.pi,   np.pi/2, 5.5, np.pi/2, np.pi/2])
        end_stop_tau = np.zeros(5)
        end_stop_tau =  (q > q_max) * (jl_K * (q_max - q) + jl_D * (-qd)) +  (q  < q_min) * (jl_K * (q_min - q) + jl_D * (-qd))

        #compute accelerations from torques
        final_tau = end_stop_tau + damping + g

        qdd = np.linalg.inv(M).dot(final_tau - h)

        print("------------------------------------------")
        #Compute forward dynamics
        qdd_fd = forward_dynamics(robot,joint_types,final_tau,q,qd)
        print(f"Joint acceleration qdd_fd computed by forward dynamics: {qdd_fd}")
        #Compare the input acceleration and the one computed with forward dynamics
        print(f"Difference of accelerations. Input qdd - qdd forward dynamics (qdd_fd-qdd): {qdd_fd - qdd}")


        # Forward Euler Integration
        qd = qd + qdd * conf.dt
        q = q + conf.dt * qd  + 0.5 * pow(conf.dt,2) * qdd

        # Log Data into a vector
        time_log = np.append(time_log, time)
        q_log = np.vstack((q_log, q))
        q_des_log= np.vstack((q_des_log, q_des))
        qd_log= np.vstack((qd_log, qd))
        qd_des_log= np.vstack((qd_des_log, qd_des))
        qdd_log= np.vstack((qdd_log, qdd))
        qdd_des_log= np.vstack((qdd_des_log, qdd_des))
        tau_log = np.vstack((tau_log, final_tau))
        
        # update time
        time = time + conf.dt
     
        #publish joint variables
        ros_pub.publish(robot, q, qd)
        tm.sleep(conf.dt*conf.SLOW_FACTOR)

        # stops the while loop if  you prematurely hit CTRL+C
        if ros_pub.isShuttingDown():
            print ("Shutting Down")
            break
        
    ros_pub.deregister_node()
    
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