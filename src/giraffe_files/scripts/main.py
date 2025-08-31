#!/usr/bin/env python3

import numpy as np 
import matplotlib.pyplot as plt 
import pinocchio as pin

robot = getRobotModel("giraffe", generate_urdf=True)
data = robot.data
model = robot.model

print("Running Giraffe Robot project...")

print("[vis] Robot visualization")
print("[kin] Test kinematics")
print("[dyn] Simulation with RNEA")
print("[tsp] Simulation in task space")

answer = input("What do you want to do?[vis/kin/dyn/tsp]")

if answer.lower() == 'vis':
     print("Robot visualization in RViz")    
    """ ros_pub = RosPub("giraffe_robot")
    ros_pub.publish(robot, conf.q0, np.zeros(robot.nv)) 
    print("RViz start. Press Ctrl+C to stop.")
    
    while ros_pub is not None and not ros_pub.isShuttingDown():
        try:
            ros.sleep(1.0) 
        except ros.ROSInterruptException:
            break """
    print("RViZ visualization ended.")

elif answer.lower() == 'kin':

elif answer.lower() == 'dyn':

elif answer.lower() == 'tsp':


else:
    print("Please input a valid option.")


print("Exiting main.py...")
exit(0)