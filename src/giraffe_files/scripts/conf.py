import numpy as np
import os
import math

dt = 0.001                   # controller time step (seconds)
SLOW_FACTOR = 1              # to slow down simulation
frame_name = 'microphone'       # name of the frame to control (end-effector) in the URDF
T = 7   #trajectory duration

# Initial Conditions
q0 =   np.array([np.pi/2, -np.pi/4,  2.75, -np.pi/4, 0.0]) # position
qd0 =  np.array([0.0, 0.0, 0.0, 0.0, 0.0])                    # velocity
qdd0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0])                    # accelerations

# End-Effector desired position [x, y, z] and pitch
# Cartesian coordinates desired
p_cart_des = np.array([1.0, 2.0, 1.0])
# Pitch orientation desired
pitch_des_deg = np.radians(-30.0)

# Control gains
Kp_pos = np.diag([1000., 1000., 1000.])
Kd_pos = 2 * np.sqrt(Kp_pos)

Kp_pitch = 100.0
Kd_pitch = 2 * np.sqrt(Kp_pitch)

Kp_postural = 10.0
Kd_postural = 2 * np.sqrt(Kp_postural)

# Initial joint configuration of urdf
q_urdf = np.array([0.0, -math.pi/3,  2.75, -math.pi/3, 0.0])
