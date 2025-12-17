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
from task_space_control import * 
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
    
    # randomized initial guess of robot position
    q_des = np.array([
    np.random.uniform(-np.pi, np.pi),     # yaw
    np.random.uniform(-np.pi/2, np.pi/2),         # pitch
    np.random.uniform(-5.5/2, 5.5),          # prismatic
    np.random.uniform(-np.pi/2, np.pi/2),     # wrist 1
    np.random.uniform(-np.pi/2, np.pi/2)      # wrist 2
    ])
    qd_des = np.zeros(5)
    qdd_des = np.zeros(5)

    dynamics_test(robot, frame_id, ros_pub, q_des, qd_des, qdd_des)


elif answer.lower() == 'pol':
    print("Simulating polynomial trajectory...")
    pol_trj_simulation(robot, frame_id, ros_pub)
    

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