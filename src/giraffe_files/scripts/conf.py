import numpy as np
import os
import math

dt = 0.001                   # controller time step (seconds)
SLOW_FACTOR = 1              # to slow down simulation
frame_name = 'microphone'       # name of the frame to control (end-effector) in the URDF

# Initial Conditions
q0 =   np.array([0.0, -math.pi/3,  math.pi/6, -math.pi/3, 0.0]) # position
qd0 =  np.array([0.0, -5.0, 0.5, 0.0, 0.0])                    # velocity
qdd0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0])                    # accelerations


# Initial joint configuratio
qhome = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

# End-Effector desired position [x, y, z]
pdes = np.array([1.0, 2.0, 1.0])

# Pitch orientation desired
pitch_des_deg = -30.0