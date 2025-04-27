#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm, logm
from lab6_header import *

def VecToso3(omg):
    return np.array([[0,      -omg[2],  omg[1]],
                     [omg[2],       0, -omg[0]],
                     [-omg[1], omg[0],       0]])

def VecTose3(V):
    return np.r_[np.c_[VecToso3([V[0], V[1], V[2]]), [V[3], V[4], V[5]]],
                 np.zeros((1, 4))]

def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for S1~6, as well as the M matrix
	M = np.array([[0, -1, 0, 390],
			      [0, 0, -1, 401],
			      [1, 0, 0, 215.5],
			      [0, 0, 0, 1]])
	
	w1 = np.array([0, 0, 1])
	q1 = np.array([-150, 150, 0])
	v1 = -np.cross(w1, q1)
	S1 = np.concatenate((w1, v1))

	w2 = np.array([0, 1, 0])
	q2 = np.array([-150, 0, 162])
	v2 = -np.cross(w2, q2)
	S2 = np.concatenate((w2, v2))

	w3 = np.array([0, 1, 0])
	q3 = np.array([94, 0, 162])
	v3 = -np.cross(w3, q3)
	S3 = np.concatenate((w3, v3))

	w4 = np.array([0, 1, 0])
	q4 = np.array([94+213, 0, 162])
	v4 = -np.cross(w4, q4)
	S4 = np.concatenate((w4, v4))

	w5 = np.array([1, 0, 0])
	q5 = np.array([0, 150+120-93+83, 162])
	v5 = -np.cross(w5, q5)
	S5 = np.concatenate((w5, v5))

	w6 = np.array([0, 1, 0])
	q6 = np.array([94+213+83, 0, 162])
	v6 = -np.cross(w6, q6)
	S6 = np.concatenate((w6, v6))

	S = np.hstack((S1.reshape(-1, 1), S2.reshape(-1, 1), S3.reshape(-1, 1), S4.reshape(-1, 1), S5.reshape(-1, 1), S6.reshape(-1, 1)))

	# ==============================================================#
	return M, S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value
	return_value = [None, None, None, None, None, None]

	# print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
	M, S = Get_MS()

	T = M
	for i in range(5, -1, -1):
		T = expm(VecTose3(S[:, i]) * theta[i]) @ T

	# ==============================================================#

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value

"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
	# =================== Your code starts here ====================#
	# Convert the positon of gripper from world frame to base frame 
	gripper_0 = np.array([xWgrip + 150, yWgrip - 150, zWgrip - 10])

	# Calulate the position of wrist's center
	L9 = 53.5
	theta_yaw = np.radians(yaw_WgripDegree)
	cen_0 = np.array([gripper_0[0] - L9*np.cos(theta_yaw), gripper_0[1] - L9*np.sin(theta_yaw), gripper_0[2]])
	# print("Center:", cen_0)

	# Calulate joint angle 1
	theta_cen = np.arctan2(cen_0[1], cen_0[0])
	angle_cen = np.degrees(theta_cen)
	d_cen = np.sqrt(cen_0[0]**2 + cen_0[1]**2)
	theta_a = np.arcsin((120 - 93 + 83) / d_cen)
	angle_a = np.degrees(theta_a)
	angle_1 = angle_cen - angle_a
	theta_1 = np.radians(angle_1)

	# Calulate joint angle 6
	angle_6 = 90 - yaw_WgripDegree + angle_1
	theta_6 = np.radians(angle_6)

	# Calulate the position of 3end
	L7 = 83
	end_cen_proj = np.array([-L7, -(120 - 93 + 83)])
	R_0_cen = np.array([[np.cos(theta_1), -np.sin(theta_1)],
						[np.sin(theta_1), np.cos(theta_1)]])
	end_0_proj = R_0_cen @ end_cen_proj + cen_0[:2]
	L8 = 82
	L10 = 59
	end_0 = np.append(end_0_proj, cen_0[2] + L8 + L10)
	# print("3end:", end_0)

	# Calulate joint angle 2, 3, 4
	L1 = 152
	L3 = 244
	L5 = 213
	j1_0 = np.array([0, 0, L1])
	d_j1_3end = np.linalg.norm(j1_0 - end_0, ord=2)
	theta_3 = np.pi - np.arccos((L3**2 + L5**2 - d_j1_3end**2) / (2 * L3 * L5))
	theta_c = np.arccos((L3**2 + d_j1_3end**2 - L5**2) / (2 * L3 * d_j1_3end))
	theta_d = np.arctan2(end_0[2] - j1_0[2], np.sqrt(end_0[0]**2 + end_0[1]**2))
	theta_2 = -(theta_c + theta_d)
	theta_4 = -(theta_2 + theta_3)

	# print(f"Angles: {angle_1}, {np.degrees(theta_2)}, {np.degrees(theta_3)}, {np.degrees(theta_4)}, -90.0, {angle_6}")

	theta1 = theta_1
	theta2 = theta_2
	theta3 = theta_3
	theta4 = theta_4
	theta5 = -np.pi/2
	theta6 = theta_6
	# ==============================================================#
	return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)
