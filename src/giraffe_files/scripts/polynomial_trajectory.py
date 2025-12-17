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
from utils.ros_publish import RosPub
import conf as conf

def pol_trj_simulation(robot, frame_id, ros_pub, p):
    q0 = conf.q0
    qd0 = conf.qd0
    qdd0 = conf.qdd0
 
    q = conf.q0
    qd = conf.qd0
    qdd = conf.qdd0

    time = 0.0
    T = conf.T     # trajectory duration

    # desired task space position
    q_i  = conf.q0
    print("Initial task space position: ", q_i)
    print("Desired position: ", p)
    # solution of the numerical ik
    q_f, log_err, log_grad = numericalInverseKinematics(p, q_i, line_search = False, wrap = False)

    tm.sleep(1.)
    ros_pub.publish(robot, conf.q0)
    tm.sleep(2.)
    while np.count_nonzero(q - q_f) :
        q, qd, qdd = compute_trajectory(q0, q_f, T, time)

        time = time + conf.dt
        #publish joint variables
        ros_pub.publish(robot, q, qd)
        ros_pub.add_marker(p)
        ros.sleep(conf.dt*conf.SLOW_FACTOR)

        # stops the while loop if  you prematurely hit CTRL+C
        if ros_pub.isShuttingDown():
            print ("Shutting Down")
            break