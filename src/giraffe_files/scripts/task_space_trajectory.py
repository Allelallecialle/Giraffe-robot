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


zero_cart = np.array([0.0, 0.0, 0.0])

#Function to compute the trajectory as in polynomial_trajecotry.py
def task_space_trajectory(time, robot, frame_id, p0, pitch0):
    p_des = np.zeros(3)
    pd_des = np.zeros(3)
    pdd_des = np.zeros(3)
    
    p_final = conf.p_cart_des

    # if-else because we want to minimize the trj duration but maximum is 7s
    if time < conf.T:
        for i in range(3):
            a = fifthOrderPolynomialTrajectory(conf.T, p0[i], p_final[i])
            p_des[i] = a[0] + a[1]*time + a[2]*time**2 + a[3]*time**3 + a[4]*time**4 + a[5]*time**5
            pd_des[i] = a[1] + 2 * a[2] * time + 3 * a[3] * time ** 2 + 4 * a[4] * time ** 3 + 5 * a[5] * time ** 4
            pdd_des[i] = 2 * a[2] + 6 * a[3] * time + 12 * a[4] * time ** 2 + 20 * a[5] * time ** 3

        a = fifthOrderPolynomialTrajectory(conf.T, pitch0, conf.pitch_des_deg)
        rpy_des= a[0] + a[1]*time + a[2]*time**2 + a[3]*time**3 + a[4]*time**4 + a[5]*time**5
        pd_rpy_des = a[1] + 2 * a[2] * time + 3 * a[3] * time ** 2 + 4 * a[4] * time ** 3 + 5 * a[5] * time ** 4
        pdd_rpy_des = 2 * a[2] + 6 * a[3] * time + 12 * a[4] * time ** 2 + 20 * a[5] * time ** 3

    else:
        p_des = conf.p_cart_des
        rpy_des = conf.pitch_des_deg

        pd_des = zero_cart
        pd_rpy_des = zero_cart

        pdd_des = zero_cart
        pdd_rpy_des = zero_cart

    return p_des, pd_des, pdd_des, rpy_des, pd_rpy_des, pdd_rpy_des


#Function for the computed torque
def task_space_computed_torque(robot, model, data, q, qd, task_ref, frame_id):
    pin.forwardKinematics(model, data, q, qd)
    pin.updateFramePlacement(model, data, frame_id)

    p_des, pd_des, pdd_des, rpy_des, pd_rpy_des, pdd_rpy_des = task_ref

    J = pin.getFrameJacobian(model, data, frame_id, pin.LOCAL_WORLD_ALIGNED)
    J_task = np.vstack([J[0:3, :], J[4, :]])    # position + pitch

    p = data.oMf[frame_id].translation
    rpy = pin.rpy.matrixToRpy(data.oMf[frame_id].rotation)[1]

    twist = J.dot(qd)
    pd = twist[:3]
    pd_rpy = twist[4]

    # define the error e on position and pitch degrees (4 dimesions)
    e_p = p_des - p
    e_rpy = rpy_des - rpy
    total_error = np.hstack([e_p, e_rpy])
    # same on velocity
    e_pd = pd_des - pd
    e_rpy_d = pd_rpy_des - pd_rpy
    task_vel_error = np.hstack([e_pd, e_rpy_d])

    # computed torque control: pdd + Kd*ed + Kp*e
    pdd_task = pdd_des + conf.Kd_pos @ e_pd + conf.Kp_pos @ e_p
    pdd_rpy_task = pdd_rpy_des + conf.Kd_pitch * e_rpy_d + conf.Kp_pitch * e_rpy
    total_error_pdd = np.hstack([pdd_task, pdd_rpy_task])


    # Dynamics
    M = robot.mass(q, False)
    h = robot.nle(q, qd, False)
    g = robot.gravity(q)
    Minv = np.linalg.inv(M)

    Jdot = pin.getFrameJacobianTimeVariation(model, data, frame_id, pin.LOCAL_WORLD_ALIGNED)
    Jdot_task = np.vstack([Jdot[0:3, :], Jdot[4, :]])

    Lambda = np.linalg.inv(J_task @ Minv @ J_task.T + 1e-6*np.eye(4))
    Jbar = Minv @ J_task.T @ Lambda

    qdd_task = Jbar.dot((total_error_pdd - Jdot_task @ qd))

    return qdd_task, M, h

# Function to compute the desired reference posture (q0_computed) using inverse kinematics
def postural_task(model, data, pitch_des_final, frame_id):
    # initialize q
    q_ik = conf.q0.copy()

    for i in range(5000):
        pin.forwardKinematics(model, data, q_ik)
        pin.updateFramePlacement(model, data, frame_id)

        # Initialize with fk of the current pose q0
        p_ik = data.oMf[frame_id].translation
        rpy_ik = pin.rpy.matrixToRpy(data.oMf[frame_id].rotation)[1]
        
        # define the error e on position and pitch degrees (4 dimesions)
        e_p =  p_ik - conf.p_cart_des
        e_rpy = rpy_ik - conf.pitch_des_deg
        total_error = np.hstack([e_p, e_rpy])

        # Jacobian as in computed_torque function
        J_ik = pin.computeFrameJacobian(model, data, q_ik, frame_id, pin.LOCAL_WORLD_ALIGNED)
        J_ik_posture = np.vstack([J_ik[:3, :], J_ik[4, :]])

        Lambda = np.linalg.inv(J_ik_posture @ J_ik_posture.T + 1e-6*np.eye(4))
        J_inv_posture = J_ik_posture.T @ Lambda
        #Integrate
        qd_ik = -J_inv_posture @ total_error
        q_ik = pin.integrate(model, q_ik, qd_ik * conf.dt)

    q0_computed = q_ik.copy()

    return q0_computed

#Function for the null space postural task (point 7 of the assignmnet)
def null_space_posture(q, qd, J_task, q0_computed):
    # from labs code:
    #Null space projector
    N = np.eye(len(q)) - J_task.T @ np.linalg.pinv(J_task.T)
    # PD control (cartesian task) + postural task
    # null space torques (postural task)
    tau0 = conf.Kp_postural * (q0_computed - q) - conf.Kd_postural * qd
    tau_null = N.dot(tau0)
    return tau_null

# Function to run the actual simulation
def run_task_simulation(robot, frame_id, ros_pub, p_des, rpy_des):
    q0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0]) 
    # Uncomment to start from the homing config and not q0=[0,0,0,0,0]:
    #q0 = conf.q0   
    qd0 = conf.qd0
    qdd0 = conf.qdd0

    q = conf.q0
    qd = conf.qd0
    qdd = conf.qdd0

    # initialize actual variables
    p = conf.q0
    pd = conf.qd0
    pdd = conf.qdd0
    rpy = np.zeros(3)
    # initialize reference variables
    p_des = p_des
    rpy_des = rpy_des
    pd_des = np.zeros(3)
    pdd_des = np.zeros(3)

    time_log = []
    p_log = []
    p_des_log = []
    pitch_log = []
    pitch_des_log = []
    # Init loggers
    q_log = np.empty((0, 5))
    q_des_log = np.empty((0, 5))
    qd_log = np.empty((0, 5))
    qd_des_log = np.empty((0, 5))
    qdd_log = np.empty((0, 5))
    qdd_des_log = np.empty((0, 5))
    tau_log = np.empty((0, 5))
    

    time = 0.0
    T = conf.T     # trajectory duration

    # Compute initial end effector position and velocity from q0
    pin.forwardKinematics(robot.model, robot.data, conf.q0)
    pin.updateFramePlacement(robot.model, robot.data, frame_id)
    p0 = robot.data.oMf[frame_id].translation.copy()
    pitch0 = pin.rpy.matrixToRpy(robot.data.oMf[frame_id].rotation)[1]
    q0_computed = postural_task(robot.model, robot.data, rpy_des, frame_id)

    # to set the wanted trajectory duration
    while time < T:
        task_ref = task_space_trajectory(time, robot, frame_id, p0, pitch0)
        qdd_task, M, h = task_space_computed_torque(robot, robot.model, robot.data, q, qd, task_ref, frame_id)

        J = pin.getFrameJacobian(robot.model, robot.data, frame_id, pin.LOCAL_WORLD_ALIGNED)
        J_task = np.vstack([J[0:3, :], J[4, :]])

        qdd_null = null_space_posture(q, qd, J_task, q0_computed)

        qdd = qdd_task + qdd_null
        tau = M @ qdd + h

        #integrate
        qd += qdd * conf.dt
        q = q + qd*conf.dt + 0.5*conf.dt*conf.dt*qdd

        #publish joint variables
        ros_pub.publish(robot, q, qd, tau)
        # marker to indicate the desired position
        ros_pub.add_marker(p_des)
        time = time + conf.dt
        tm.sleep(conf.dt*conf.SLOW_FACTOR)

        #for plots
        pin.forwardKinematics(robot.model, robot.data, q, qd)
        pin.updateFramePlacement(robot.model, robot.data, frame_id)
        p_curr = robot.data.oMf[frame_id].translation.copy()
        pitch_curr = pin.rpy.matrixToRpy(robot.data.oMf[frame_id].rotation)[1]
        # Log Data into a vector
        time_log = np.append(time_log, time)
        q_log = np.vstack((q_log, q))
        q_des_log= np.vstack((q_des_log, q))
        qd_log= np.vstack((qd_log, qd))
        qd_des_log= np.vstack((qd_des_log, qd))
        qdd_log= np.vstack((qdd_log, qdd))
        qdd_des_log= np.vstack((qdd_des_log, qdd))
        tau_log = np.vstack((tau_log, tau))
        

        #time_log.append(time)
        p_log.append(p_curr)
        p_des_log.append(task_ref[0])
        pitch_log.append(pitch_curr)
        pitch_des_log.append(task_ref[3])

        # stops the while loop if  you prematurely hit CTRL+C
        if ros_pub.isShuttingDown():
            print ("Shutting Down")
            break
    return (q, qd, time,
        time_log,
        np.array(p_log),
        np.array(p_des_log),
        np.array(pitch_log),
        np.array(pitch_des_log),
        q_log,
        q_des_log,
        qd_log,
        qd_des_log,
        qdd_log,
        qdd_des_log,
        tau_log)


def test_simulation(robot, frame_id, p_des, rpy_des, q_final, qd_final, time):
    # Update forward kinematiccs 
    pin.forwardKinematics(robot.model, robot.data, q_final, qd_final)
    pin.updateFramePlacement(robot.model, robot.data, frame_id)

    # Print final pose
    final_p = robot.data.oMf[frame_id].translation
    final_rpy = pin.rpy.matrixToRpy(robot.data.oMf[frame_id].rotation)
    print("Final Position of the End Effector:", final_p)
    print("Position desired:", conf.p_cart_des)
    print("Pitch final:", np.degrees(final_rpy[1]))
    print("Pitch desired:", np.degrees(conf.pitch_des_deg))
    print("Time: ", time)

def plot_simulation(time_log, p_log, p_des_log, pitch_log, pitch_des_log, q_log, q_des_log, qd_log, qd_des_log, qdd_log, qdd_des_log, tau_log):
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
    input("Press enter to continue")
    pos_error = np.linalg.norm(p_log - p_des_log, axis=1)
    pitch_error = pitch_log - pitch_des_log

    plt.figure(5)
    plt.plot(time_log, pos_error)
    plt.axhline(0.02, linestyle='--')  # tolerance band
    plt.xlabel("Time [s]")
    plt.ylabel("||p - p_des|| [m]")
    plt.title("Cartesian position error")
    plt.grid()

    plt.figure(6)
    plt.plot(time_log, pitch_error)
    plt.axhline(0.01, linestyle='--')
    plt.axhline(-0.01, linestyle='--')
    plt.xlabel("Time [s]")
    plt.ylabel("Pitch error [rad]")
    plt.title("Pitch error")
    plt.grid()

    plt.show()
    #to not close plots immediately
    input("Press enter to continue")
