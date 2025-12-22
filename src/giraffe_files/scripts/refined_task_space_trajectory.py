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

    while time < T:
        pol_trj_simulation(robot, frame_id, ros_pub, np.append(p_des, rpy_des))






        #publish joint variables
        ros_pub.publish(robot, q, qd)
        tm.sleep(conf.dt*conf.SLOW_FACTOR)

        # stops the while loop if  you prematurely hit CTRL+C
        if ros_pub.isShuttingDown():
            print ("Shutting Down")
            break
        
    ros_pub.deregister_node()