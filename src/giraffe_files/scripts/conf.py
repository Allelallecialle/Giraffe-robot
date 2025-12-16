import numpy as np
import os
import math

dt = 0.001                   # controller time step (seconds)
SLOW_FACTOR = 1              # to slow down simulation
frame_name = 'microphone'       # name of the frame to control (end-effector) in the URDF
T = 7

# Initial Conditions
q0 =   np.array([0.0, -math.pi/3,  2.75, -math.pi/3, 0.0]) # position
qd0 =  np.array([0.0, 0.0, 0.0, 0.0, 0.0])                    # velocity
qdd0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0])                    # accelerations

# Pitch orientation desired
pitch_des_deg = np.radians(-30.0)

# Initial joint configuration of urdf
q_urdf = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

# End-Effector desired position [x, y, z]
p_des = np.array([1.0, 2.0, 1.0, pitch_des_deg])
