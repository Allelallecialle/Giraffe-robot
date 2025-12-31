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
from polynomial_trajectory import pol_trj_simulation
from utils.math_tools import Math
import matplotlib.pyplot as plt
from utils.common_functions import plotJoint
import subprocess

import conf as conf
from kinematics import *
from task_space_trajectory import * 
from dynamics import * 


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
    # Ctrl+C to stop
    ros_pub.publish(robot, conf.q0, np.zeros(robot.nv))

    while (not ros.is_shutdown()):
        if ros_pub.isShuttingDown():
            print ("Shutting Down")
            break


elif answer.lower() == 'kin':
    print("Testing direct and differential kinematics...")

    q = conf.q0
    qd = conf.qd0

    direct_kin_test(robot, frame_id, q, qd)
    jacobian_test(frame_id, robot, q)
    inv_kin_test(frame_id)

elif answer.lower() == 'dyn':
    print("Testing dynamics...")

    # randomized initial guess of robot position
    q_des = np.array([
    np.random.uniform(-np.pi, np.pi),     # yaw
    np.random.uniform(-np.pi/2, np.pi/2),         # pitch
    np.random.uniform(0.0, 5.5),          # prismatic
    np.random.uniform(-np.pi/2, np.pi/2),     # wrist 1
    np.random.uniform(-np.pi/2, np.pi/2)      # wrist 2
    ])
    qd_des = np.zeros(5)
    qdd_des = np.zeros(5)

    dynamics_test(robot, frame_id, ros_pub, q_des, qd_des, qdd_des)


elif answer.lower() == 'pol':
    print("Simulating polynomial trajectory...")
        #randomize the final position setting boundaries of the room
    p = np.array([
    np.random.uniform(0.0, 5.0),     # x
    np.random.uniform(0.0, 12.0),    # y
    np.random.uniform(0.0, 4.0),     # z
    np.random.uniform(-np.pi, np.pi) # pitch
    ])
    pol_trj_simulation(robot, frame_id, ros_pub, p)
    
    
elif answer.lower() == 'tsp':
    print("Simulating task space trajectory...")
    p_des = conf.p_cart_des
    rpy_des = conf.pitch_des_deg
    q_final, qd_final, time, time_log, p_log, p_des_log, pitch_log, pitch_des_log = run_task_simulation(robot, frame_id, ros_pub, p_des, rpy_des)
    test_simulation(robot, frame_id, p_des, rpy_des, q_final, qd_final, time)
    plot_simulation(time_log, p_log, p_des_log, pitch_log, pitch_des_log)


else:
    print("Please input a valid option.")


if ros_pub is not None:
    ros_pub.deregister_node()
print("Exiting main.py...")
exit(0)