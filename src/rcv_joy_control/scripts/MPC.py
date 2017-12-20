#!/usr/bin/python
#
# Code made by Feiyang Liu(liuf@kth.se), Tianxiao Zhao(tzh@kth.se)

"""A simple implementation of MPC using cvxopt as solver
"""

import time
import numpy as np
from math import hypot
import math
from cvxopt import matrix, solvers
from math import pi


class MPC:

	"""MPC Controller
	"""

	def __init__(self, Hc=5, Hp=15, Ts_MPC=0.09):
		self.Hc = Hc
		self.Hp = Hp
		self.Ts_MPC = Ts_MPC


	def update(self, z, true_path, p_curv, p_beta):
		# Dimentions of input variables:
		# z = (x,y,yaw)  #(1x3) 
		# true_path: (x0, y0, yaw0, x1, y1, yaw1,...)  #(3N x 1)

		# parameters for rcv
		# w_x = 10
		# w_y = 50
		# w_yaw = 0
		# w_beta = 100
		# w_curv = 1000

		w_x = 10
		w_y = 50
		w_yaw = 1000
		w_beta = 10	
		w_curv = 10

		a_flPlane = 0.9728
		b_flPlane = 0.9866
		c_flPlane = 0.0079

		a_frPlane = 1.0331
		b_frPlane = 1.0090
		c_frPlane = -0.0079

		a_rlPlane = -0.9915
		b_rlPlane = 0.9929
		c_rlPlane = -0.0078

		a_rrPlane = -1.0178
		b_rrPlane = 1.0050
		c_rrPlane = -0.0066

		maxdeltafl = 0.2255
		mindeltafl = -0.2480

		maxdeltafr = 0.2640
		mindeltafr = -0.2910

		maxdeltarl = 0.3150
		mindeltarl = -0.2235

		maxdeltarr = 0.2330
		mindeltarr = -0.2937

		maxdeltarate = 0.3700
		mindeltarate = -0.3700

		#global init_flag, prev_curv, prev_beta
		#if init_flag == []:
		prev_curv = p_curv
		prev_beta = p_beta
		
		#need to modify how to load the path
		true_path = np.array(true_path)
		length = len(true_path)
		zref = np.reshape(true_path,(1,length))
		true_path = np.reshape(true_path,(int(length/3),3))

		#z0 = [z[0], z[1], z[2]]
		#z0.extend(list(zref[0, :]))
		#zref = np.array([z0])
		#print(true_path.shape)

		# Translate path from global (X,Y,YAW) to local (x,y,yaw) coordinate
		#zref = np.zeros((1, 3 * (self.Hp + self.Hc)))
		curv_beta_ref = np.zeros((1, 2 * (self.Hp + self.Hc)))
		traj_crab = np.zeros((self.Hp + self.Hc, 2))
		vref = np.zeros((1, self.Hp + self.Hc))

		#rot = np.array([[np.cos(z[2]), np.sin(z[2])],[-np.sin(z[2]), np.cos(z[2])]])
		
		#for i in range(self.Hp + self.Hc - 1):
		#	z_temp = true_path[i, :]
		#	z_next = true_path[i+1, :]
		#	rot = np.array([[np.cos(z_temp[2]), np.sin(z_temp[2])],[-np.sin(z_temp[2]), np.cos(z_temp[2])]])
		#	xy_temp = np.dot(rot, np.array([[z_temp[0] - z[0]],[z_temp[1] - z[1]]]))
		#	yaw_temp = z_temp[2] - z[2]
		#	zref[0,3 * i] = xy_temp[0,:]
		#	zref[0,3 * i + 1] = xy_temp[1,:]
		#	zref[0,3 * i + 2] = yaw_temp
			#print(zref)

		#zref[0,3 * (self.Hp + self.Hc - 1)] = zref[0,3 * (self.Hp + self.Hc - 2)]
		#zref[0,3 * (self.Hp + self.Hc - 1) + 1] = zref[0, 3 * (self.Hp + self.Hc - 2) + 1]
		#zref[0,3 * (self.Hp + self.Hc - 1) + 2] = zref[0, 3 * (self.Hp + self.Hc - 2) + 2]	

		#z0 = [0.0, 0.0, 0.0]
		#z0.extend(list(zref[0, :]))
		#zref = np.array([z0])


		# Calculating curvature (kappa) and crabbing (beta) reference
		for i in range(self.Hp + self.Hc - 1):
			rot = np.array([[np.cos(zref[0, 3*i+2]), np.sin(zref[0, 3*i+2])], [-np.sin(zref[0, 3*i+2]), np.cos(zref[0, 3*i+2])]])
			xy_crab = np.dot(rot, np.array([[zref[0, 3*i+3] - zref[0, 3*i]], [zref[0, 3*i+4] - zref[0, 3*i+1]]]))
			traj_crab[i, 0] = xy_crab[0,0]
			traj_crab[i, 1] = xy_crab[1,0]
			#print("i: {0}, rot: {1}".format(i, rot))
			#print("i: {0}, xy_crab: {1}".format(i, xy_crab))

		traj_crab[self.Hp + self.Hc - 1, 0] = traj_crab[self.Hp + self.Hc - 2, 0]
		traj_crab[self.Hp + self.Hc - 1, 1] = traj_crab[self.Hp + self.Hc - 2, 1]

		#print(traj_crab)


		for j in range(self.Hp + self.Hc - 2):
			if hypot((zref[0,3*j]-zref[0,3*j+3]), (zref[0,3*j+1] - zref[0,3*j+4])) < 0.001:
				curv_beta_ref[0,2*j] = np.sin(0.5*(zref[0,3*j+5] - zref[0,3*j+2])) / 0.0005
			else:	
				curv_beta_ref[0,2*j] = 0.5 * hypot((zref[0,3*j]-zref[0,3*j+3]), (zref[0,3*j+1] - zref[0,3*j+4])) / np.sin(0.5*(zref[0,3*j+5] - zref[0,3*j+2]))
				curv_beta_ref[0,2*j] = 1.0/curv_beta_ref[0,2*j] if curv_beta_ref[0,2*j] != 0.0 else 10.0
			if abs(traj_crab[j+1,1] - traj_crab[j,1]) < 0.0001:
				curv_beta_ref[0,2*j+1] = 0.001
			elif traj_crab[j+1,0] - traj_crab[j,0] < 0.0001 and traj_crab[j+1,0] - traj_crab[j,0] >= 0:
				curv_beta_ref[0,2*j+1] = pi / 2.0
			elif traj_crab[j+1,0] - traj_crab[j,0] > -0.0001 and traj_crab[j+1,0] - traj_crab[j,0] < 0:
				curv_beta_ref[0,2*j+1] = - pi / 2.0
			else:
				curv_beta_ref[0,2*j+1] = math.atan2(traj_crab[j+1,1] - traj_crab[j,1], traj_crab[j+1, 0] - traj_crab[j,0])

			if curv_beta_ref[0,2*j] > 1:
				curv_beta_ref[0,2*j] = 1
			elif curv_beta_ref[0,2*j] < -1:
				curv_beta_ref[0,2*j] = -1
			else:
				curv_beta_ref[0,2*j] = curv_beta_ref[0,2*j]

			#print("curv: {0}, beta: {1}".format(curv_beta_ref[0,2*j], curv_beta_ref[0,2*j+1]))

		
		curv_beta_ref[0, 2*(self.Hp + self.Hc) - 4] =  curv_beta_ref[0, 2*(self.Hp + self.Hc) - 6]
		curv_beta_ref[0, 2*(self.Hp + self.Hc) - 3] =  curv_beta_ref[0, 2*(self.Hp + self.Hc) - 5]
		curv_beta_ref[0, 2*(self.Hp + self.Hc) - 2] =  curv_beta_ref[0, 2*(self.Hp + self.Hc) - 4]
		curv_beta_ref[0, 2*(self.Hp + self.Hc) - 1] =  curv_beta_ref[0, 2*(self.Hp + self.Hc) - 3]

		# Calculating velocity reference vref
		for i in range(self.Hp + self.Hc - 1):
			vref[0,i] = hypot((zref[0,3*i+3] - zref[0,3*i]), (zref[0,3*i+4] - zref[0,3*i+1])) / self.Ts_MPC

		#print('xref0','xref1','yref0','yref1',zref[0,0], zref[0,3], zref[0,1], zref[0,4])
		vref[0,self.Hp + self.Hc - 1] = vref[0,self.Hp + self.Hc - 2]

		# calculate curv_beta_ref (from here)
		pathx = list(true_path[:, 0])
		pathy = list(true_path[:, 1])	
		pathyaw = list(true_path[:, 2])

		pathx.insert(0, pathx[0])
		pathy.insert(0, pathy[0])
		pathyaw.insert(0, pathyaw[0])

		[TruePathRef, CalcPathRef, curv_beta, vref_out] = MPCLatRefCurv(np.reshape(vref, (self.Hp + self.Hc, 1)), self.Hc, self.Hp, pathx, pathy, pathyaw, self.Ts_MPC)
		curv_beta_new = np.reshape(curv_beta, (1, 2 * (self.Hp + self.Hc)))
		# (end here)

		z_dif = np.array([[-zref[0,0]], [-zref[0,1]], [-zref[0,2]]])

		yaw_aux = zref[0,np.arange(2,3*(self.Hp+self.Hc),3)] - zref[0,2]

		#Constraints Initialization
		SteeringPlanes = np.array([[a_flPlane],[b_flPlane], [a_frPlane], [b_frPlane], [a_rlPlane], [b_rlPlane], [a_rrPlane], [b_rrPlane]])

		ubfl = maxdeltafl - (curv_beta_ref[0,np.arange(0,2*self.Hc,2)] * a_flPlane + curv_beta_ref[0,np.arange(1,2*self.Hc,2)] * b_flPlane + c_flPlane)
		lbfl = mindeltafl - (curv_beta_ref[0,np.arange(0,2*self.Hc,2)] * a_flPlane + curv_beta_ref[0,np.arange(1,2*self.Hc,2)] * b_flPlane + c_flPlane)

		ubfr = maxdeltafr - (curv_beta_ref[0,np.arange(0,2*self.Hc,2)] * a_frPlane + curv_beta_ref[0,np.arange(1,2*self.Hc,2)] * b_frPlane + c_frPlane)
		lbfr = mindeltafr - (curv_beta_ref[0,np.arange(0,2*self.Hc,2)] * a_frPlane + curv_beta_ref[0,np.arange(1,2*self.Hc,2)] * b_frPlane + c_frPlane)

		ubrl = maxdeltarl - (curv_beta_ref[0,np.arange(0,2*self.Hc,2)] * a_rlPlane + curv_beta_ref[0,np.arange(1,2*self.Hc,2)] * b_rlPlane + c_rlPlane)
		lbrl = mindeltarl - (curv_beta_ref[0,np.arange(0,2*self.Hc,2)] * a_rlPlane + curv_beta_ref[0,np.arange(1,2*self.Hc,2)] * b_rlPlane + c_rlPlane)

		ubrr = maxdeltarr - (curv_beta_ref[0,np.arange(0,2*self.Hc,2)] * a_rrPlane + curv_beta_ref[0,np.arange(1,2*self.Hc,2)] * b_rrPlane + c_rrPlane)
		lbrr = mindeltarr - (curv_beta_ref[0,np.arange(0,2*self.Hc,2)] * a_rrPlane + curv_beta_ref[0,np.arange(1,2*self.Hc,2)] * b_rrPlane + c_rrPlane)

		ubflRate = np.zeros((self.Hc, 1))
		lbflRate = np.zeros((self.Hc, 1))
		ubflRate[0,0] = maxdeltarate * self.Ts_MPC + (prev_curv * a_flPlane + prev_beta * b_flPlane) - (curv_beta_ref[0,0] * a_flPlane + curv_beta_ref[0,1] * b_flPlane)
		lbflRate[0,0] = mindeltarate * self.Ts_MPC + (prev_curv * a_flPlane + prev_beta * b_flPlane) - (curv_beta_ref[0,0] * a_flPlane + curv_beta_ref[0,1] * b_flPlane)

		ubfrRate = np.zeros((self.Hc, 1))
		lbfrRate = np.zeros((self.Hc, 1))
		ubfrRate[0,0] = maxdeltarate * self.Ts_MPC + (prev_curv * a_frPlane + prev_beta * b_frPlane) - (curv_beta_ref[0,0] * a_frPlane + curv_beta_ref[0,1] * b_frPlane)
		lbfrRate[0,0] = mindeltarate * self.Ts_MPC + (prev_curv * a_frPlane + prev_beta * b_frPlane) - (curv_beta_ref[0,0] * a_frPlane + curv_beta_ref[0,1] * b_frPlane)

		ubrlRate = np.zeros((self.Hc, 1))
		lbrlRate = np.zeros((self.Hc, 1))
		ubrlRate[0,0] = maxdeltarate * self.Ts_MPC + (prev_curv * a_rlPlane + prev_beta * b_rlPlane) - (curv_beta_ref[0,0] * a_rlPlane + curv_beta_ref[0,1] * b_rlPlane)
		lbrlRate[0,0] = mindeltarate * self.Ts_MPC + (prev_curv * a_rlPlane + prev_beta * b_rlPlane) - (curv_beta_ref[0,0] * a_rlPlane + curv_beta_ref[0,1] * b_rlPlane)

		ubrrRate = np.zeros((self.Hc, 1))
		lbrrRate = np.zeros((self.Hc, 1))
		ubrrRate[0,0] = maxdeltarate * self.Ts_MPC + (prev_curv * a_rrPlane + prev_beta * b_rrPlane) - (curv_beta_ref[0,0] * a_rrPlane + curv_beta_ref[0,1] * b_rrPlane)
		lbrrRate[0,0] = mindeltarate * self.Ts_MPC + (prev_curv * a_rrPlane + prev_beta * b_rrPlane) - (curv_beta_ref[0,0] * a_rrPlane + curv_beta_ref[0,1] * b_rrPlane)

		#Matrices Init
		R = np.zeros((2 * self.Hc, 2 * self.Hc))
		Q = np.zeros((3 * (self.Hc + self.Hp),3 * (self.Hc + self.Hp)))
		A = np.zeros((3 * (self.Hc + self.Hp), 3))
		Ai = np.zeros(((self.Hc + self.Hp), 3, 3))
		B = np.zeros((3 * (self.Hc + self.Hp), 2 * self.Hc))
		Bi = np.zeros(((self.Hc + self.Hp), 3, 2))

		#Matrices fill-up for control horizon
		Ai[0, :, :] = np.array([[1, 0, self.Ts_MPC * (-np.sin(yaw_aux[0] + curv_beta_ref[0,1]) * vref[0,0])], [0, 1, self.Ts_MPC * (np.cos(yaw_aux[0] + curv_beta_ref[0,1]) * vref[0,0])], [0, 0, 1]])
		Bi[0, :, :] = np.array([[0, self.Ts_MPC * (-np.sin(yaw_aux[0] + curv_beta_ref[0,1]) * vref[0,0])], [0, self.Ts_MPC * (np.cos(yaw_aux[0] + curv_beta_ref[0,1]) * vref[0,0])], [self.Ts_MPC * vref[0,0], 0]])

		A[np.arange(0,3,1), :] = Ai[0,:,:]
		B[np.arange(0,3,1), 0] = Bi[0,:,0]
		B[np.arange(0,3,1), 1] = Bi[0,:,1]	
	
		for i in range(self.Hc-1):
			#Constraints Initialization
			ubflRate[i+1, 0] = maxdeltarate * self.Ts_MPC + (curv_beta_ref[0,2*i] * a_flPlane + curv_beta_ref[0,2*i+1] * b_flPlane) - (curv_beta_ref[0,2 * (i + 1)] * a_flPlane + curv_beta_ref[0,2 * (i + 1)+1] * b_flPlane)
			lbflRate[i+1, 0] = mindeltarate * self.Ts_MPC + (curv_beta_ref[0,2*i] * a_flPlane + curv_beta_ref[0,2*i+1] * b_flPlane) - (curv_beta_ref[0,2 * (i + 1)] * a_flPlane + curv_beta_ref[0,2 * (i + 1)+1] * b_flPlane)

			ubfrRate[i+1, 0] = maxdeltarate * self.Ts_MPC + (curv_beta_ref[0,2*i] * a_frPlane + curv_beta_ref[0,2*i+1] * b_frPlane) - (curv_beta_ref[0,2 * (i + 1)] * a_frPlane + curv_beta_ref[0,2 * (i + 1)+1] * b_frPlane)
			lbfrRate[i+1, 0] = mindeltarate * self.Ts_MPC + (curv_beta_ref[0,2*i] * a_frPlane + curv_beta_ref[0,2*i+1] * b_frPlane) - (curv_beta_ref[0,2 * (i + 1)] * a_frPlane + curv_beta_ref[0,2 * (i + 1)+1] * b_frPlane)

			ubrlRate[i+1, 0] = maxdeltarate * self.Ts_MPC + (curv_beta_ref[0,2*i] * a_rlPlane + curv_beta_ref[0,2*i+1] * b_rlPlane) - (curv_beta_ref[0,2 * (i + 1)] * a_rlPlane + curv_beta_ref[0,2 * (i + 1)+1] * b_rlPlane)
			lbrlRate[i+1, 0] = mindeltarate * self.Ts_MPC + (curv_beta_ref[0,2*i] * a_rlPlane + curv_beta_ref[0,2*i+1] * b_rlPlane) - (curv_beta_ref[0,2 * (i + 1)] * a_rlPlane + curv_beta_ref[0,2 * (i + 1)+1] * b_rlPlane)

			ubrrRate[i+1, 0] = maxdeltarate * self.Ts_MPC + (curv_beta_ref[0,2*i] * a_rrPlane + curv_beta_ref[0,2*i+1] * b_rrPlane) - (curv_beta_ref[0,2 * (i + 1)] * a_rrPlane + curv_beta_ref[0,2 * (i + 1)+1] * b_rrPlane)
			lbrrRate[i+1, 0] = mindeltarate * self.Ts_MPC + (curv_beta_ref[0,2*i] * a_rrPlane + curv_beta_ref[0,2*i+1] * b_rrPlane) - (curv_beta_ref[0,2 * (i + 1)] * a_rrPlane + curv_beta_ref[0,2 * (i + 1)+1] * b_rrPlane)

			#Matrices fill-up for control horizon

			Ai[i+1, :, :] = np.array([[1, 0, self.Ts_MPC * (-np.sin(yaw_aux[i+1] + curv_beta_ref[0, 2*i+3]) * vref[0,i+1])],[0, 1, self.Ts_MPC * (np.cos(yaw_aux[i+1] + curv_beta_ref[0,2*i+3]) * vref[0,i+1])], [0, 0, 1]])
			Bi[i+1, :, :] = np.array([[0, self.Ts_MPC * (-np.sin(yaw_aux[i+1] + curv_beta_ref[0, 2*i+3]) * vref[0,i+1])],[0, self.Ts_MPC * (np.cos(yaw_aux[i+1] + curv_beta_ref[0,2*i+3]) * vref[0,i+1])],[self.Ts_MPC * vref[0,i+1], 0]])

			A[np.arange(3*i+3,3*i+6,1), :] = np.dot(Ai[i + 1,:,:], A[np.arange(3*i,3*i+3,1),:])

			for j in range(i):
				midB = np.zeros((3,2))
				midB[:, 0] = B[np.arange(3*i,3*i+3,1), 2*j]
				midB[:, 1] = B[np.arange(3*i,3*i+3,1), 2*j + 1]
				finalB = np.dot(Ai[i+1,:,:], midB)
				B[np.arange(3*i+3,3*i+6,1), 2*j] = finalB[:, 0]
				B[np.arange(3*i+3,3*i+6,1), 2*j+1] = finalB[:, 1]

			B[np.arange(3*i+3, 3*i+6, 1), 2*i+2] = Bi[i+1, :, 0]
			B[np.arange(3*i+3, 3*i+6, 1), 2*i+3] = Bi[i+1, :, 1]

			Q[3*i, 3*i] = w_x
			Q[3*i+1, 3*i+1] = w_y
			Q[3*i+2, 3*i+2] = w_yaw
			R[2*i+1, 2*i+1] = w_beta
			R[2*i, 2*i] = w_curv

		index = self.Hc - 1
		Q[3*index, 3*index] = w_x
		Q[3*index+1, 3*index+1] = w_y
		Q[3*index+2, 3*index+2] = w_yaw
		R[2*index+1, 2*index+1] = w_beta
		R[2*index, 2*index] = w_curv

		#Matrices fill-up for prediction horizon
		for i in range(self.Hp):
			Ai[i+self.Hc, :, :] = np.array([[1, 0, self.Ts_MPC * (-np.sin(yaw_aux[i+self.Hc] + curv_beta_ref[0, 2*self.Hc+2*i-1]) * vref[0,i+self.Hc])], [0, 1, self.Ts_MPC * (np.cos(yaw_aux[i + self.Hc] + curv_beta_ref[0,2*self.Hc + 2*i+1]) * vref[0,i + self.Hc])],[0, 0, 1]])
			Bi[i+self.Hc, :, :] = np.array([[0, self.Ts_MPC * (-np.sin(yaw_aux[i + self.Hc] + curv_beta_ref[0, 2*self.Hc +2*i+1]) * vref[0,i+self.Hc])], [0, self.Ts_MPC * (np.cos(yaw_aux[i + self.Hc] + curv_beta_ref[0, 2*self.Hc + 2*i+1]) * vref[0,i + self.Hc])], [self.Ts_MPC * vref[0,i + self.Hc], 0]])

			A[np.arange(i*3+3*self.Hc,3*i+3+3*self.Hc,1), :] = np.dot(Ai[i+self.Hc,:,:], A[np.arange(3*i-3+3*self.Hc,3*i+3*self.Hc,1),:])
			for j in range(self.Hc-1):
				midB = np.zeros((3,2))
				midB[:, 0] = B[np.arange(3*i-3+3*self.Hc,3*i+3*self.Hc,1), 2*j]
				midB[:, 1] = B[np.arange(3*i-3+3*self.Hc,3*i+3*self.Hc,1), 2*j+1]
				finalB = np.dot(Ai[i+self.Hc,:,:], midB)
				B[np.arange(i*3 + (3*self.Hc), i*3+3 + (3*self.Hc), 1), 2*j] = finalB[:, 0]
				B[np.arange(i*3 + (3*self.Hc), i*3+3 + (3*self.Hc), 1), 2*j+1] = finalB[:, 1]
				#B[np.arange(i*3+3*self.Hc,3*i+3+3*self.Hc,1), np.arange(2*j,2*j+2,1)] = np.dot(Ai[i+self.Hc,:,:],B[np.arange(3*i-3+3*self.Hc,3*i+3*self.Hc,1),np.arange(2*j,2*j+2,1)])
			
			midB = np.zeros((3,2))
			midB[:, 0] = B[np.arange(3*i-3+3*self.Hc,3*i+3*self.Hc,1), 2*self.Hc - 2]
			midB[:, 1] = B[np.arange(3*i-3+3*self.Hc,3*i+3*self.Hc,1), 2*self.Hc - 1]
			finalB = np.dot(Ai[i+self.Hc, :, :], midB)
			B[np.arange(i*3 + (3*self.Hc), i*3+3 + (3*self.Hc), 1), 2*self.Hc - 2] = finalB[:, 0] + Bi[i+self.Hc, :, 0]
			B[np.arange(i*3 + (3*self.Hc), i*3+3 + (3*self.Hc), 1), 2*self.Hc - 1] = finalB[:, 1] + Bi[i+self.Hc, :, 1]			
			#B[np.arange(i*3+(3*self.Hc),i*3+3+(3*self.Hc),1),np.arange(2*self.Hc,2*self.Hc+2,1)] = np.dot(Ai[i+self.Hc,:,:],B[np.arange(3*i-3+3*self.Hc,3*i+3*self.Hc,1),np.arange(2*self.Hc-2,2*self.Hc,1)])
			Q[3 * i + (3 * self.Hc), 3 * i + (3 * self.Hc)] = w_x
			Q[3 * i + 1 + (3 * self.Hc), 3 * i + 1 + (3 * self.Hc)] = w_y
			Q[3 * i + 2 + (3 * self.Hc), 3 * i + 2 + (3 * self.Hc)] = w_yaw

		Hess = np.dot(np.dot(np.transpose(B), Q), B) + R
		Hess = (Hess + np.transpose(Hess)) / 2
		lin = 2 * np.dot(np.dot(np.dot(np.transpose(B), Q), A), z_dif) 
		#lin = 2 * (np.dot(np.dot(np.dot(np.transpose(B), Q), A),z_dif) + np.dot(R,np.transpose(curv_beta_ref[0,np.arange(0,2*self.Hc,1)])))

		sol = self.cvxsolve(Hess, lin, ubfl, lbfl, ubfr, lbfr, ubrl, lbrl, ubrr, lbrr, ubflRate, lbflRate, ubfrRate, lbfrRate, ubrlRate, lbrlRate, ubrrRate, lbrrRate, self.Hc)
		res_dif = sol['x']
		
		#curv_beta_Hc = curv_beta_ref[0, np.arange(0,2*self.Hc,1)] + res_dif
		curv_beta_Hc = curv_beta_new[0, np.arange(0,2*self.Hc,1)] + res_dif

		curv_out = curv_beta_Hc[0, 0]
		beta_out = curv_beta_Hc[0, 1]
		v_out = vref[0,0]

		if (np.isnan(curv_out) or np.isnan(beta_out)):
			beta_out = 0.0
			curv_out = 0.0
			v_out = vref[0,0]

		prev_curv = curv_out
		prev_beta = beta_out

		return curv_out, beta_out, vref


	def cvxsolve(self, Hess, lin, ubfl, lbfl, ubfr, lbfr, ubrl, lbrl, ubrr, lbrr, ubflRate, lbflRate, ubfrRate, lbfrRate, ubrlRate, lbrlRate, ubrrRate, lbrrRate, Hc):

		a_flPlane = 0.9728
		b_flPlane = 0.9866
		c_flPlane = 0.0079

		a_frPlane = 1.0331
		b_frPlane = 1.0090
		c_frPlane = -0.0079

		a_rlPlane = -0.9915
		b_rlPlane = 0.9929
		c_rlPlane = -0.0078

		a_rrPlane = -1.0178
		b_rrPlane = 1.0050
		c_rrPlane = -0.0066

		steerplane = matrix([a_flPlane, b_flPlane, a_frPlane, b_frPlane, a_rlPlane, b_rlPlane, a_rrPlane, b_rrPlane])
		plane = matrix(steerplane,(2,4))

		A = matrix(0.0, (8*Hc, 2*Hc))
		for i in range(4):
			for j in range(Hc):
				A[i*2*Hc + j, 2*j] = plane[0, i]
				A[i*2*Hc + j, 2*j + 1] = plane[1, i]
				A[i*2*Hc + j + Hc, 2*j] = -plane[0, i]
				A[i*2*Hc + j + Hc, 2*j + 1] = -plane[1, i]


		bound = np.array([ubfl, -lbfl, ubfr, -lbfr, ubrl, -lbrl, ubrr, -lbrr])
		B = matrix(0.0,(8*Hc, 1))
		for i in range(8):
			B[(i*Hc):(i*Hc+Hc),0] = matrix(bound[i,:],(Hc,1))

		C = matrix(0.0,(8*Hc, 2*Hc))
		for j in range(4):
			C[j*2*Hc, 0] = plane[0, j]
			C[j*2*Hc, 1] = plane[1, j]
			C[j*2*Hc + Hc, 0] = -plane[0, j]
			C[j*2*Hc + Hc, 1] = -plane[1, j]

		for i in range(4):
			for j in range(Hc - 1):
				C[i*2*Hc + j + 1, 2*j] = -plane[0, i]
				C[i*2*Hc + j + 1, 2*j + 1] = -plane[1, i]
				C[i*2*Hc + j + 1, 2*j + 2] = plane[0, i]
				C[i*2*Hc + j + 1, 2*j + 3] = plane[1, i]

				C[i*2*Hc + j + 1 + Hc, 2*j] = plane[0, i]
				C[i*2*Hc + j + 1 + Hc, 2*j + 1] = plane[1, i]
				C[i*2*Hc + j + 1 + Hc, 2*j + 2] = -plane[0, i]
				C[i*2*Hc + j + 1 + Hc, 2*j + 3] = -plane[1, i]

		D = matrix(0.0,(8*Hc, 1))
		boundplane = np.array([ubflRate, -lbflRate, ubfrRate, -lbfrRate, ubrlRate, -lbrlRate, ubrrRate, -lbrrRate])
		#print(boundplane)
		for i in range(8):
			D[(i*Hc):(i*Hc+Hc),0] = matrix(boundplane[i,:,:])
		#print(D)


		constrain1 = matrix(0.0,(16*Hc, 2*Hc))
		constrain1[0:8*Hc, :] = A
		constrain1[8*Hc:16*Hc, :] = C
		constrain2 = matrix([[B],[D]],(16*Hc, 1))

		Hess = matrix(Hess)
		lin = matrix(lin)

		solvers.options['show_progress'] = False
		output = solvers.qp(Hess, lin, constrain1, constrain2)

		return output


# ============================================================================
# some help function for generating curv_ref
def MPCLatRefCurv(vref, Hc, Hp, pathx, pathy, pathyaw, Ts_MPC):
#function [TruePathRef,CalcPathRef,curv_beta,vref_out] = MPCLatRefCurv(vref,pathx,pathy,pathyaw,Hc,Hp,Ts_MPC,...
#                                                             prev_curv,maxcurvrate_rcv,mincurvrate_rcv,maxcurv_rcv,mincurv_rcv )
	
	#Code made by Goncalo Collares Pereira (gpcp@kth.se)    
	
	#This function computes the trajectory points for the lateral
	#controller assuming the path only contains curvature and using
	#circumference arcs to connect two points.
														 
	# TruePathRef=zeros(3*(Hc+Hp+1),1);  % (181*1)
	# CalcPathRef=zeros(3*(Hc+Hp+1),1);  % (181*1)
	# vref_out=vref;

	TruePathRef = np.zeros((3*(Hc+Hp+1),1))
	CalcPathRef = np.zeros((3*(Hc+Hp+1),1))
	vref_out=vref
	
	# xref=zeros(Hc+Hp+1,1);    % (61*1)
	# yref=zeros(Hc+Hp+1,1);    % (61*1)
	# yawref=zeros(Hc+Hp+1,1);

	xref = np.zeros((Hc+Hp+1,1))
	yref = np.zeros((Hc+Hp+1,1))
	yawref = np.zeros((Hc+Hp+1,1))

	# xref(1)=pathx(1);  % first reference for cost calculation
	# yref(1)=pathy(1);  
	# yawref(1)=pathyaw(1);

	xref[0, 0] = pathx[0]
	yref[0, 0] = pathy[0]
	yawref[0, 0] = pathyaw[0]

	# TruePathRef(1:3)=[xref(1);yref(1);yawref(1)]; % same for True and Calc
	# CalcPathRef(1:3)=[xref(1);yref(1);yawref(1)];

	TruePathRef[0, 0] = xref[0, 0]
	TruePathRef[1, 0] = yref[0, 0]
	TruePathRef[2, 0] = yawref[0, 0]
	CalcPathRef[0, 0] = xref[0, 0]
	CalcPathRef[1, 0] = yref[0, 0]
	CalcPathRef[2, 0] = yawref[0, 0]

	# dist=zeros(Hc+Hp,1);   % (60*1)
	# curv_beta=zeros(2*(Hc+Hp),1); % (120*1)
	# dist(:)=vref*Ts_MPC; 
	# m=1;
	# d_trav=0;
	# flag_endpath=0;
	# flag_error=0;

	dist = np.zeros((Hc+Hp,1))
	curv_beta = np.zeros((2*(Hc+Hp),1))
	dist += vref*Ts_MPC
	m = 1
	d_trav = 0
	flag_endpath = 0
	flag_error = 0

	# prev_m=0;
	# prev_ratio=0;

	prev_m = 0
	prev_ratio = 0

	# arc_len=-1;
	# R1=-1;
	# R2=-1;
	# Curv=0;
	# point=[0;0];

	arc_len = -1
	R1 = -1
	R2 = -1
	Curv = 0
	point = np.zeros((2,1))


	# for i=1:Hc+Hp

	for i in range(Hc+Hp):
		
	   #dist(i)=abs(dist(i));

		dist[i, 0] = abs(dist[i, 0])

		#while(d_trav-dist(i)<-10^-5)

		while d_trav - dist[i, 0] < -1e-5:

			# if(m>(length(pathx)-1))
			#    flag_endpath=1;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
			#    break;
			# end

			if m > len(pathx) - 1:
				flag_endpath = 1
				break

			#b1=[pathx(m)+cos(pathyaw(m)+pi/2);pathy(m)+sin(pathyaw(m)+pi/2)];
			#b2=[pathx(m+1)+cos(pathyaw(m+1)+pi/2);pathy(m+1)+sin(pathyaw(m+1)+pi/2)];
			# point=lineintersect([pathx(m),pathy(m),b1(1),b1(2)],[pathx(m+1),pathy(m+1),b2(1),b2(2)]);
			# R1=hypot(pathx(m)-point(1),pathy(m)-point(2));
			# R2=hypot(pathx(m+1)-point(1),pathy(m+1)-point(2));

			b1 = np.array([[pathx[m-1] + np.cos(pathyaw[m-1] + pi/2)],[pathy[m-1]+np.sin(pathyaw[m-1] + pi/2)]])
			b2 = np.array([[pathx[m] + np.cos(pathyaw[m] + pi/2)],[pathy[m]+np.sin(pathyaw[m] + pi/2)]])
			point = lineintersect(np.array([pathx[m-1], pathy[m-1], b1[0,0], b1[1,0]]), np.array([pathx[m], pathy[m], b2[0,0], b2[1,0]]))
			R1 = hypot((pathx[m-1] - point[0,0]), (pathy[m-1] - point[1,0]))
			R2 = hypot((pathx[m] - point[0,0]), (pathy[m] - point[1,0]))


			# if(isinf(R1)||isinf(R2))
			# 	Curv=0;
			# else
			# 	Curv=sign(vref(i))*sign(pathyaw(m+1)-pathyaw(m))/((R1+R2)/2);
			# end
			
			if np.isinf(R1) or np.isinf(R2):
				Curv = 0
			else:
				Curv = np.sign(vref[i, 0])*np.sign(pathyaw[m] - pathyaw[m-1])/((R1+R2)/2)

			# if(abs(Curv)<10^-6)
			# 	arc_len=hypot(pathx(m+1)-pathx(m),pathy(m+1)-pathy(m));
			# else
			# 	arc_len=abs(((R1+R2)/2)*(pathyaw(m+1)-pathyaw(m)));
			# end

			if abs(Curv) < 1e-6:
				arc_len = hypot((pathx[m] - pathx[m-1]), (pathy[m] - pathy[m-1]))
			else:
				arc_len = abs(((R1+R2)/2)*(pathyaw[m]-pathyaw[m-1]))

			# dist(i)=dist(i)-arc_len;
			# m=m+1;

			dist[i, 0] -= arc_len
			m += 1
		#end
		# if(flag_endpath==1)
			#  break;
		# end

		if flag_endpath == 1:
			break

		if m<1:
			flag_error=1
			break
		elif m==1:
			yawref[i+1,0] = yawref[i,0]
			xref[i+1,0] = xref[i,0]
			yref[i+1,0] = yref[i,0]
			ratio=0
			Curv=prev_curv
		else:
			if prev_m==m:
				ratio=prev_ratio+(dist[i,0]/arc_len)
			else:
				ratio=(dist[i,0]-d_trav+arc_len)/arc_len

			yawref[i+1,0]=ratio*(pathyaw[m-1]-pathyaw[m-2])+ pathyaw[m-2]
			#[xref(i+1),yref(i+1),Curv]=CalcPathPoint(Curv,ratio,pathx(m),pathy(m),pathx(m-1),pathy(m-1),R1,R2,yawref(i+1),point,xref(i),yref(i),yawref(i))
			[out1,out2,out3] = CalcPathPoint(Curv,ratio,pathx[m-1],pathy[m-1],pathx[m-2],pathy[m-2],R1,R2,yawref[i+1,0],point,xref[i,0],yref[i,0],yawref[i,0])
			xref[i+1,0] = out1
			yref[i+1,0] = out2
			Curv = out3

	if m < 1:
		flag_error = 1
		#break

		if i==0:
			curv_beta[2*i,0] = LimitsCurv(Curv,prev_curv,0.37,-0.37,Ts_MPC,0.22,-0.225)
		else:
			curv_beta[2*i,0] = LimitsCurv(Curv,curv_beta[2*(i-1), 0],0.37,-0.37,Ts_MPC,0.22,-0.225)

		#TruePathRef[3*(i+1):3*(i+2), 0]=[xref(i+1);yref(i+1);yawref(i+1)];
		TruePathRef[3*(i+1), 0] = xref[i+1,0]
		TruePathRef[3*(i+1)+1, 0] = yref[i+1,0]
		TruePathRef[3*(i+1)+2, 0] = yawref[i+1,0]

		[CalcPathRef[3*(i+1),0],CalcPathRef[3*(i+1)+1,0],CalcPathRef[3*(i+1)+2,0]] = EgoPredSimple(CalcPathRef[3*i,0],CalcPathRef[3*i+1,0],CalcPathRef[3*i+2,0],vref[i,0],curv_beta[2*i,0],curv_beta[2*i+1,0],Ts_MPC );

		d_trav -= dist[i,0]
		prev_m=m
		prev_ratio=ratio

	if flag_error==0:
		if flag_endpath==1:
			while i<=Hc+Hp-1:
				if i > 0:
					curv_beta[2*i,0]=curv_beta[2*i-2,0]
				else:
					curv_beta[2*i,0]=0

				[TruePathRef[3*(i+1),0],TruePathRef[3*(i+1)+1,0],TruePathRef[3*(i+1)+2,0]] = EgoPredSimple(TruePathRef[3*i,0],TruePathRef[3*i+1,0],TruePathRef[3*i+2,0],vref[i,0],curv_beta[2*i,0],curv_beta[2*i+1,0],Ts_MPC );
				[CalcPathRef[3*(i+1),0],CalcPathRef[3*(i+1)+1,0],CalcPathRef[3*(i+1)+2,0]] = EgoPredSimple(CalcPathRef[3*i,0],CalcPathRef[3*i+1,0],CalcPathRef[3*i+2,0],vref[i,0],curv_beta[2*i,0],curv_beta[2*i+1,0],Ts_MPC );

				i += 1
	else:
		# curv_beta=zeros(2*(Hc+Hp),1)
		# TruePathRef=zeros(3*(Hc+Hp+1),1)
		# CalcPathRef=zeros(3*(Hc+Hp+1),1)
		# vref_out=zeros(Hc+Hp,1)
		curv_beta = np.zeros((2*(Hc+Hp),1))
		TruePathRef = np.zeros((3*(Hc+Hp+1),1))
		CalcPathRef = np.zeros((3*(Hc+Hp+1),1))
		vref_out = np.zeros((Hc+Hp,1))

	return [TruePathRef, CalcPathRef, curv_beta, vref_out]


def lineintersect(l1,l2):
	m1 = (l1[3] - l1[1])/(l1[2] - l1[0])
	b1 = l1[1] - m1*l1[0]
	m2 = (l2[3] - l2[1])/(l2[2] - l2[0])
	b2 = l2[1] - m2*l2[0]

	if abs(m2) != float('Inf') and abs(m1) != float('Inf') and abs(m2) != float('NaN') and abs(m1) != float('NaN'):
		x = (b1 - b2)/(m2 - m1)

		if x == float('NaN'):
			x = float('Inf')
			y = float('Inf')
		else:
			if abs(x) == float('Inf'):
				y = x
			else:
				y = m1*x + b1
	else:
		if abs(m1) == float('Inf') and abs(m2) != float('Inf'):
			x = l1[0]
			y = m2*x + b2
		elif abs(m2) == float('Inf') and abs(m1) != float('Inf'):
			x = l2[0]
			y = m1*x + b1
		else:
			x = float('Inf')
			y = float('Inf')

	point = np.array([[x], [y]])

	return point


def CalcPathPoint(Curv,ratio,pathx_point,pathy_point,pathx_prev_point,pathy_prev_point,R1,R2,yawref,R_center,xref_prev,yref_prev,yawref_prev):

	Curv_out = Curv
	R=R1+(R2-R1)*ratio

	if abs(Curv_out) < 1e-6:
		xref = ratio*(pathx_point-pathx_prev_point)+pathx_prev_point;
		yref=ratio*(pathy_point-pathy_prev_point)+pathy_prev_point
	elif Curv_out < 0:
		xref = np.cos(yawref+pi/2.0)*R+R_center[0,0]
		yref = np.sin(yawref+pi/2.0)*R+R_center[1,0]
	else:
		xref = np.cos(yawref-pi/2.0)*R+R_center[0,0]
		yref = np.sin(yawref-pi/2.0)*R+R_center[1,0]

	dist_path_points = hypot(xref-xref_prev,yref-yref_prev)
	expected_dist = abs((yawref_prev-yawref)*R)
	error_margin = 1

	if (dist_path_points > (expected_dist+error_margin)):
		Curv_out = -Curv
		if abs(Curv_out)<10^-6:
			xref = ratio*(pathx_point-pathx_prev_point)+pathx_prev_point
			yref = ratio*(pathy_point-pathy_prev_point)+pathy_prev_point
		elif Curv_out < 0:
			xref = np.cos(yawref+pi/2.0)*R+R_center[0,0]
			yref = np.sin(yawref+pi/2.0)*R+R_center[1,0]
		else:
			xref = np.cos(yawref-pi/2.0)*R+R_center[0,0]
			yref = np.sin(yawref-pi/2.0)*R+R_center[1,0]
		
	return [ xref,yref,Curv_out ]
		
		
		
def LimitsCurv(Curv,prev_curv,maxcurvrate_rcv,mincurvrate_rcv,Ts_MPC,maxcurv_rcv,mincurv_rcv):

	if Curv>maxcurv_rcv:
		Curv = maxcurv_rcv
	elif  Curv<mincurv_rcv:
		Curv = mincurv_rcv

	return Curv


def EgoPredSimple(x,y,yaw,vel,curv,beta,dt):

	while dt > 0.01:
		x = x+0.01*vel*np.cos(beta+yaw)
		y = y+0.01*vel*np.sin(beta+yaw)
		yaw = yaw+curv*vel*0.01
		dt = dt-0.01
		
	xnew = x+dt*vel*np.cos(beta+yaw)
	ynew = y+dt*vel*np.sin(beta+yaw)
	yawnew = yaw+curv*vel*dt

	return [xnew,ynew,yawnew]


