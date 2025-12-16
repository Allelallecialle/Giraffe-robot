#!/usr/bin/env python3

import pinocchio as pin
from pinocchio.utils import *
import numpy as np
from numpy import nan
import math
import time as tm
from utils.common_functions import *
from utils.ros_publish import RosPub
from utils.kin_utils import directKinematics, computeEndEffectorJacobian, numericalInverseKinematics
from utils.trajectory_utils import fifthOrderPolynomialTrajectory, compute_trajectory
from utils.math_tools import Math
import matplotlib.pyplot as plt
from utils.common_functions import plotJoint
import subprocess

import conf as conf
from kinematics import *
from task_space_control import * 
from dynamics import * 
from polynomial_trj import execute_pol_trj


rospack = rospkg.RosPack()
urdf_path = os.path.join(rospack.get_path("giraffe_files"), "urdf", "giraffe_robot.urdf")
robot = RobotWrapper.BuildFromURDF(urdf_path)
ros_pub = RosPub(urdf_path)


# Init variables
zero = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
time = 0.0

# Init loggers
q_log = np.empty((5))*nan
qd_log = np.empty((5))*nan
qdd_log = np.empty((5))*nan
time_log =  0

q0 = conf.q0
qd0 = conf.qd0
qdd0 = conf.qdd0

q = conf.q0
qd = conf.qd0
qdd = conf.qdd0

q_des = conf.q0
qd_des = conf.qd0
qdd_des = conf.qdd0


# get the ID corresponding to the frame we want to control
assert(robot.model.existFrame(conf.frame_name))
frame_id = robot.model.getFrameId(conf.frame_name)

print("Running Giraffe Robot project...")

print("[vis] Robot visualization")
print("[kin] Test kinematics")
print("[pol] Simulation of random polynomial trajectory")
print("[dyn] Simulation with RNEA")
print("[tsp] Simulation in task space")

answer = input("What do you want to do?[vis/kin/pol/dyn/tsp]\n")

if answer.lower() == 'vis':
    print("Visualizing robot in RViz")

    while (not ros.is_shutdown()):
        # execute bash command roslaunch. Ctrl+C to stop
        subprocess.run("roslaunch giraffe_files visualize.launch", shell=True, executable="/bin/bash")
        if ros_pub.isShuttingDown():
            print ("Shutting Down")
            break

    ros_pub.deregister_node()


elif answer.lower() == 'kin':
    print("Testing direct and differential kinematics...")
    direct_kin_test(robot, frame_id, q, qd)
    jacobian_test(frame_id, robot, q)
    inv_kin_test(frame_id)

elif answer.lower() == 'dyn':
    print("Testing dynamics...")
    dynamics_test(robot, frame_id)
    ros_pub.publish(robot, q, qd)

elif answer.lower() == 'pol':
    print("Simulating polynomial trajectory...")
    T = conf.T     # trajectory duration

    # desired task space position
    q_i  = conf.q0
    #randomize the final position setting boundaries of the room
    p = np.array([
    np.random.uniform(0.0, 5.0),     # x
    np.random.uniform(0.0, 12.0),    # y
    np.random.uniform(0.0, 4.0),     # z
    np.random.uniform(-np.pi, np.pi) # pitch
    ])
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

elif answer.lower() == 'tsp':
    print("Simulating task space trajectory...")
    #ros_pub = RosPub("giraffe_robot")
    #test_trajectory(robot, frame_id)
    execute_pol_trj(robot, frame_id)


    #pitch_des_final = conf.pitch_des_deg
    #q_des = np.array([math.radians(45), -math.radians(10), 5.0, 0.0, math.radians(15)])
    

    
    

else:
    print("Please input a valid option.")


#if ros_pub and ros.is_shutdown():  
#    ros_pub.deregister_node()

print("Exiting main.py...")
exit(0)