#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Aug 25 15:25:05 2024

"""

import numpy as np
from modern_robotics import se3ToVec, MatrixLog6, TransInv, FKinBody, JacobianBody

# Fucntion that prints the report requested in the assignment
def printing_report(i, thetalist, Tsb, Vb, omega_b, v_b):
    # Get as input the:
    # 1. iteration number, i
    # 2. thetalist
    # 3. Tsb
    # 4. Vb
    # 5. omega_b
    # 6. v_b
    print("Iteration",  i, ":")
    print('-------------')
    
    print("joint vector:")
    print(thetalist)
    print()
    
    print("SE(3) end - effector config Tsb:")
    print(np.round(Tsb, decimals=2))
    print() 
    
    print("error twist V_b:")
    print(Vb)
    print()
    
    print("angular error magnitude ||omega_b||:")
    print(omega_b)
    print()
    
    print("linear error magnitude ||v_b||:")
    print(v_b)
    print()
    

def IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev):
    """Computes inverse kinematics in the body frame for an open chain robot

    :param Blist: The joint screw axes in the end-effector frame when the
                  manipulator is at the home position, in the format of a
                  matrix with axes as the columns
    :param M: The home configuration of the end-effector
    :param T: The desired end-effector configuration Tsd
    :param thetalist0: An initial guess of joint angles that are close to
                       satisfying Tsd
    :param eomg: A small positive tolerance on the end-effector orientation
                 error. The returned joint angles must give an end-effector
                 orientation error less than eomg
    :param ev: A small positive tolerance on the end-effector linear position
               error. The returned joint angles must give an end-effector
               position error less than ev
    :return thetalist: Joint angles that achieve T within the specified
                       tolerances,
    :return success: A logical value where TRUE means that the function found
                     a solution and FALSE means that it ran through the set
                     number of maximum iterations without finding a solution
                     within the tolerances eomg and ev.
    Uses an iterative Newton-Raphson root-finding method.
    The maximum number of iterations before the algorithm is terminated has
    been hardcoded in as a variable called maxiterations. It is set to 20 at
    the start of the function, but can be changed if needed.

    Example Input:
        Blist = np.array([[0, 0, -1, 2, 0,   0],
                          [0, 0,  0, 0, 1,   0],
                          [0, 0,  1, 0, 0, 0.1]]).T
        M = np.array([[-1, 0,  0, 0],
                      [ 0, 1,  0, 6],
                      [ 0, 0, -1, 2],
                      [ 0, 0,  0, 1]])
        T = np.array([[0, 1,  0,     -5],
                      [1, 0,  0,      4],
                      [0, 0, -1, 1.6858],
                      [0, 0,  0,      1]])
        thetalist0 = np.array([1.5, 2.5, 3])
        eomg = 0.01
        ev = 0.001
    Output:
        (np.array([1.57073819, 2.999667, 3.14153913]) - Final, True, joint_vectors_iterates - for all iterations)
    """
    thetalist = np.array(thetalist0).copy()
    i = 0
    maxiterations = 20
    Vb = se3ToVec(MatrixLog6(np.dot(TransInv(FKinBody(M, Blist, \
                                                      thetalist)), T)))
    err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg \
          or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev
          
    

    # Calculating Tsb 
    Tsb = FKinBody(M, Blist, thetalist)
    # Calculating omega_b
    omega_b = np.linalg.norm([Vb[0], Vb[1], Vb[2]])
    # Calculating v_b
    v_b = np.linalg.norm([Vb[3], Vb[4], Vb[5]])

    # Calling the function to print the report 
    printing_report(i, thetalist, Tsb, Vb, omega_b, v_b)
    
    # The joint vector for a given iteration
    joint_vectors_iterates = []
    # Appending for the first iteration
    joint_vectors_iterates.append(thetalist)
    
    while err and i < maxiterations:
        thetalist = thetalist \
                    + np.dot(np.linalg.pinv(JacobianBody(Blist, \
                                                         thetalist)), Vb)
        i = i + 1
        Vb \
        = se3ToVec(MatrixLog6(np.dot(TransInv(FKinBody(M, Blist, \
                                                       thetalist)), T)))
        err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg \
              or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev
             
        # Calculating Tsb 
        Tsb = FKinBody(M, Blist, thetalist)
        # Calculating omega_b
        omega_b = np.linalg.norm([Vb[0], Vb[1], Vb[2]])
        # Calculating v_b
        v_b = np.linalg.norm([Vb[3], Vb[4], Vb[5]])
        
        # Calling the function to print the report 
        printing_report(i, thetalist, Tsb, Vb, omega_b, v_b)
        # Appending for the given iteration
        joint_vectors_iterates.append(thetalist)
    
    # Tranforming it from nested lists to array
    joint_vectors_iterates = np.array(joint_vectors_iterates)
    # Rounding
    joint_vectors_iterates = np.round(joint_vectors_iterates, decimals=6)
    # Saveing it as a csv
    np.savetxt("iterates.csv", joint_vectors_iterates, delimiter = ",")     
    return (thetalist, not err, joint_vectors_iterates)


#%%

# Convrestion Factor form mm to m
mm_to_m = 10 ** -3
# Parmeters taken from the book as requested
W1 = 109 * mm_to_m; W2 = 82 * mm_to_m
L1 = 425 * mm_to_m; L2 = 392 * mm_to_m
H1 = 89 * mm_to_m; H2 = 95 * mm_to_m

# The M Matrix
M = np.array([[-1, 0, 0, L1 + L2],
              [0, 0, 1, W1 + W2],
              [0, 1, 0, H1 - H2],
              [0, 0, 0, 1]])

# The B lists
B1 = [0, 1, 0, W1 + W2, 0, L1 + L2]
B2 = [0, 0, 1, H2, -L1 -L2, 0]
B3 = [0, 0, 1, H2, -L2, 0]
B4 = [0, 0, 1, H2, 0, 0]
B5 = [0, -1, 0, -W2, 0, 0]
B6 = [0, 0, 1, 0, 0, 0]

# Converting to array and Transopose
B = np.array([B1, B2, B3, B4, B5, B6]).T

# Thresholds for convergence
EOMG = 0.001
EV = 0.0001

# INITIAL GUESS!!!
theta_list_0 = np.array([-3.86,  1.21, -1.7, -2.7,  0.7, 1.6 ])

# Tsd as specified in the assignment
Tsd = np.array([[0, 1, 0, -0.5],
                [0, 0, -1, 0.1],
                [-1, 0, 0, 0.1],
                [0, 0, 0, 1]])

# Calling the function IKinBodyIterates
theta, is_conv, joint_vectors_iterates = IKinBodyIterates(Blist=B, M=M, T=Tsd, thetalist0=theta_list_0, eomg=EOMG, ev=EV)

# Printing convergence flag and the final solution
print("is converged?")
print(is_conv)
print('joint vector final: ', theta)

