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
    pd0 = zero_cart
    pdd0 = zero_cart

    p_des = zero_cart
    pd_des = zero_cart
    pdd_des = zero_cart
    
    p_final = conf.p_cart_des

    # if-else because we want to minimize the trj duration but maximum is 7s
    if time < conf.T:
        for i in range(3):
            a = fifthOrderPolynomialTrajectory(conf.T, p0[i], p_final[i])
            p_des[i] = a[0] + a[1]*time + a[2]*time**2 + a[3]*time**3 + a[4]*time**4 + a[5]*time**5
            pd_des[i] = a[1] + 2 * a[2] * time + 3 * a[3] * time ** 2 + 4 * a[4] * time ** 3 + 5 * a[5] * time ** 4
            pdd_des[i] = 2 * a[2] + 6 * a[3] * time + 12 * a[4] * time ** 2 + 20 * a[5] * time ** 3
            #p_des[i], pd_des[i], pdd_des[i] = fifth_order_poly(t, conf.T, p0[i], p_final[i])

        a = fifthOrderPolynomialTrajectory(conf.T, pitch0, conf.pitch_des_deg)
        rpy_des= a[0] + a[1]*time + a[2]*time**2 + a[3]*time**3 + a[4]*time**4 + a[5]*time**5
        pd_rpy_des = a[1] + 2 * a[2] * time + 3 * a[3] * time ** 2 + 4 * a[4] * time ** 3 + 5 * a[5] * time ** 4
        pdd_rpy_des = 2 * a[2] + 6 * a[3] * time + 12 * a[4] * time ** 2 + 20 * a[5] * time ** 3
        #rpy_des, pd_rpy_des, pdd_rpy_des = fifth_order_poly(t, conf.T, rpy0, conf.pitch_des_deg)

    else:
        p_des = p_des
        rpy_des = rpy_des

        pd_des = zero_cart
        pd_rpy_des = zero_cart

        pdd_des = zero_cart
        pdd_rpy_des = zero_cart

    return p_des, pd_des, pdd_des, rpy_des, pd_rpy_des, pdd_rpy_des


#Function for the computed torque
def task_space_computed_torque(model, data, q, qd, task_ref, frame_id):
    pin.forwardKinematics(model, data, q, qd)
    pin.updateFramePlacement(model, data, frame_id)

    p_des, pd_des, pdd_des, rpy_des, pd_rpy_des, pdd_rpy_des = task_ref

    J = pin.getFrameJacobian(model, data, frame_id, pin.LOCAL_WORLD_ALIGNED)
    J_task = np.vstack([J[0:3, :], J[4, :]])

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

    # computed torque control: edd + Kd*ed + Kp*e = 0
    pdd_task = pdd_des + conf.Kd_pos @ e_pd + conf.Kp_pos @ e_p
    pdd_rpy_task = pdd_rpy_des + conf.Kd_pitch * e_rpy_d + conf.Kp_pitch * e_rpy
    total_error_pdd = np.hstack([pdd_task, pdd_rpy_task])


    # Dynamics
    M = pin.crba(model, data, q)
    h = pin.nonLinearEffects(model, data, q, qd)
    Minv = np.linalg.inv(M)

    Jdot = pin.getFrameJacobianTimeVariation(model, data, frame_id, pin.LOCAL_WORLD_ALIGNED)
    Jdot_task = np.vstack([Jdot[0:3, :], Jdot[4, :]])

    Lambda = np.linalg.inv(J_task @ Minv @ J_task.T + 1e-6*np.eye(4))
    Jbar = Minv @ J_task.T @ Lambda

    qdd_task = Jbar.dot((total_error_pdd - Jdot_task @ qd))

    return qdd_task, M, h



#Function for the null space postural task (point 7 of the assignmnet)
def null_space_posture(q, qd, J_task):
    # from labs code:
    #Null space projector
    N = np.eye(len(q)) - J_task.T @ np.linalg.pinv(J_task.T)
    # PD control (cartesian task) + postural task
    # null space torques (postural task)
    tau0 = conf.Kp_postural * (conf.q0 - q) - conf.Kd_postural * qd
    tau_null = N.dot(tau0)
    return tau_null

# Function to run the actual simulation
def run_task_simulation(robot, frame_id, ros_pub, p_des, rpy_des):
    q0 = conf.q0
    qd0 = conf.qd0
    qdd0 = conf.qdd0

    q = conf.q0
    qd = conf.qd0
    qdd = conf.qdd0

    # initialize actual variables
    p = conf.q0
    pd = conf.qd0
    pdd = conf.qdd0
    rpy = zero_cart
    # initialize reference variables
    p_des = p_des
    rpy_des = rpy_des
    pd_des = zero_cart
    pdd_des = zero_cart

    time = 0.0
    T = conf.T     # trajectory duration

    # Compute initial end effector position and velocity from q0
    pin.forwardKinematics(robot.model, robot.data, conf.q0)
    pin.updateFramePlacement(robot.model, robot.data, frame_id)
    p0 = robot.data.oMf[frame_id].translation.copy()
    pitch0 = pin.rpy.matrixToRpy(robot.data.oMf[frame_id].rotation)[1]

    while time < T:
        task_ref = task_space_trajectory(time, robot, frame_id, p0, pitch0)

        qdd_task, M, h = task_space_computed_torque(robot.model, robot.data, q, qd, task_ref, frame_id)

        J = pin.getFrameJacobian(robot.model, robot.data, frame_id, pin.LOCAL_WORLD_ALIGNED)
        J_task = np.vstack([J[0:3, :], J[4, :]])

        qdd_null = null_space_posture(q, qd, J_task)

        qdd = qdd_task + qdd_null
        tau = M @ qdd + h

        #integrate
        qd += qdd * conf.dt
        q = pin.integrate(robot.model, q, qd * conf.dt)

        #publish joint variables
        ros_pub.publish(robot, q, qd, tau)
        # marker to indicate the desired position
        ros_pub.add_marker(p_des)
        time = time + conf.dt
        tm.sleep(conf.dt*conf.SLOW_FACTOR)

        # stops the while loop if  you prematurely hit CTRL+C
        if ros_pub.isShuttingDown():
            print ("Shutting Down")
            break
    return q, qd

def test_simulation(robot, frame_id, p_des, rpy_des, q_final, qd_final):
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