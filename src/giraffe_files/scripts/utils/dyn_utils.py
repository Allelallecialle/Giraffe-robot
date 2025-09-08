# -*- coding: utf-8 -*-

import numpy as np
import pinocchio as pin


# computation of gravity terms
def getg(q,robot,joint_types):
    qd = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    qdd = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    g = pin.rnea(robot.model, robot.data, q, qd ,qdd)
    return g


# computation of generalized mass matrix
def getM(q,robot,joint_types):
    n = len(q)
    M = np.zeros((n,n))
    for i in range(n):
        ei = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        ei[i] = 1
        g = getg(q,robot,joint_types)
        tau = pin.rnea(robot.model, robot.data, q, np.array([0,0,0,0,0]),ei) - g
        # fill in the column of the inertia matrix
        M[:5,i] = tau        
        
    return M

# computation of Coriolis and centrifugal term
def getC(q,qd,robot,joint_types):
    qdd = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    g = getg(q,robot,joint_types)
    C = pin.rnea(robot.model, robot.data,q,qd,qdd) - g
    return C      

def forward_dynamics(robot,joint_types,tau,q,qd):
    # Find qdd with tau,q,qd as inputs
    g = getg(q, robot, joint_types)
    M = getM(q, robot, joint_types)
    C = getC(q, qd, robot, joint_types)

    qdd = np.linalg.inv(M).dot(tau - (C + g))
    return qdd