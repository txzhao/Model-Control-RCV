#!/usr/bin/python

"""A simple implementation of pure pursuit control
"""

import numpy as np
import math


class PurePursuit:

	"""MPC Controller
	"""

	def __init__(self, lookahead=20, planPath=None):
		self.lookahead = lookahead
		self.planPath = planPath
	

	def update(self, rela_state):
		desire = 0
		Path = np.transpose(np.array(self.planPath))

		delta_x = Path[:, 0] - rela_state[0]
		delta_y = Path[:, 1] - rela_state[1]
		dist_list = (delta_x*delta_x + delta_y*delta_y)**0.5

		index = np.argmin(dist_list)
		if index + self.lookahead < Path.shape[0] - 1:
			desire = index + self.lookahead
		else:
			desire = Path.shape[0] - 1

		newPoint = Path[desire]
		error_x = newPoint[0] - rela_state[0] 
		error_y = newPoint[1] - rela_state[1]
		dist = math.hypot(error_x, error_y)
		y_translate = -np.sin(rela_state[2])*error_x + np.cos(rela_state[2])*error_y
		kappa = 2*(y_translate/dist**2)

		return kappa