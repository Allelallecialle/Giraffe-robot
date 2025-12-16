#!/usr/bin/env python3

import pinocchio as pin
from pinocchio.utils import *
import numpy as np
from numpy import nan
import math
import time as tm 
from utils.common_functions import *
from utils.kin_utils import numericalInverseKinematics
from utils.trajectory_utils import fifthOrderPolynomialTrajectory, compute_trajectory
from utils.ros_publish import RosPub
import conf as conf

def execute_pol_trj(robot, frame_id):
    
    T = 2    # trajectory duration

    q0 = conf.q0
    qd0 = conf.qd0
    qdd0 = conf.qdd0

    # desired task space position
    q_i  = conf.q0
    p =  np.random.uniform(-np.pi, np.pi, 4)
    # solution of the numerical ik
    q_f, log_err, log_grad = numericalInverseKinematics(p, q_i, line_search = False, wrap = False)


    tm.sleep(1.)
    ros_pub.publish(robot, conf.q0)
    tm.sleep(2.)
    while np.count_nonzero(q - q_f) :
        # Polynomial trajectory
        for i in range(4):
            a = coeffTraj(3.0,conf.q0[i],q_f[i])
            q[i] = a[0] + a[1]*time + a[2]*time**2 + a[3]*time**3 + a[4]*time**4 + a[5]*time**5
            qd[i] = a[1] + 2 * a[2] * time + 3 * a[3] * time ** 2 + 4 * a[4] * time ** 3 + 5 * a[5] * time ** 4
            qdd[i] = 2 * a[2] + 6 * a[3] * time + 12 * a[4] * time ** 2 + 20 * a[5] * time ** 3

        # update time
        time = time + conf.dt

        # Log Data into a vector
        time_log = np.append(time_log, time)
        q_log = np.vstack((q_log, q ))
        qd_log= np.vstack((qd_log, qd))
        qdd_log= np.vstack((qdd_log, qdd))


        #publish joint variables
        ros_pub.publish(robot, q, qd)
        ros_pub.add_marker(p)
        ros.sleep(conf.dt*conf.SLOW_FACTOR)

        # stops the while loop if  you prematurely hit CTRL+C
        if ros_pub.isShuttingDown():
            print ("Shutting Down")
            break

    plotJoint('position', time_log, q_log.T)