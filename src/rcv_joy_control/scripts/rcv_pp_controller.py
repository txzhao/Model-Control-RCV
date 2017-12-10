#!/usr/bin/env python

# Copyright 2017 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import os
import rospy
from rcv_msgs.msg import Control
from rcv_msgs.msg import control_command
from rcv_common_msgs.msg import control_command as rcv_ctrl_cmd
from rosgraph_msgs.msg import Log
from nav_msgs.msg import Odometry
from PID import PID 
import math
import numpy as np
from matplotlib import pyplot as plt
import tf


STEERING_AXIS = 0
THROTTLE_AXIS = 4
thre = 0.2


class Translator:
	def __init__(self, operations=None, planPath=None):
		self.operations = operations
		self.planPath = planPath
		self.linear_v = 0
		self.index = 0
		self.x0 = 3.0
		self.y0 = -12.0
		self.x = self.x0
		self.y = self.y0
		self.yaw = 0
		self.dist = 0
		self.pre_dist = 0
		self.ref_v = 3.0
		self.header = None
		self.park_state = False
		#self.pub = rospy.Publisher('rcv_control_cmd', Control, queue_size=1)
		self.pub = rospy.Publisher('rcv_control_cmd', control_command, queue_size=1)
		self.rcv_pub = rospy.Publisher("/control_command", rcv_ctrl_cmd, queue_size=1)
		#self.vel_sub = rospy.Subscriber('rosout_agg', Log, self.velcallback)
		self.state_sub = rospy.Subscriber('/base_pose_ground_truth', Odometry, self.statecallback)
		self.last_published_time = rospy.get_rostime()
		self.timer = rospy.Timer(rospy.Duration(1.0/20.0), self.timer_callback)
	
	# this callback is not used
	def velcallback(self, data):
		self.linear_v = data.msg
	
	def statecallback(self, data):
		self.header = data.header
		self.x = data.pose.pose.position.x
		self.y = data.pose.pose.position.y
		quaternion = (
			data.pose.pose.orientation.x,
			data.pose.pose.orientation.y,
			data.pose.pose.orientation.z,
			data.pose.pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		self.yaw = euler[2]
		self.linear_v = math.sqrt(data.twist.twist.linear.x**2 + data.twist.twist.linear.y**2)
				
	def timer_callback(self, event):
		if self.last_published_time < rospy.get_rostime() + rospy.Duration(1.0/20):
			if self.operations is None:
				self.pathControl()
			else:
				self.move_new()

	# interface takes throttle, brake, steer as input
	def move(self):
		command = Control()
		#self.sub = rospy.Subscriber('rosout_agg', Log, self.velcallback)
		#rospy.Subscriber('/base_pose_ground_truth', Odometry, self.statecallback)
		
		rcv_v = float(self.linear_v)
		ref_v = float(self.operations[1])	
		
		if self.operations[0]:
			if abs(rcv_v - ref_v)/ref_v < thre:
				command.steer = self.operations[2]
			else:
				ref_v = self.operations[1]
				pid = PID(P=3, I=0.5, D=0)
				pid.SetPoint = float(ref_v)
				pid.setSampleTime = rospy.Duration(1.0/20)
				pid.update(float(rcv_v))

				if pid.output < 0:
					command.throttle = 0
					command.brake = abs(pid.output)
				else:
					command.throttle = abs(pid.output)
					command.brake = 0
		else:
			command.throttle = self.operations[1]
			command.brake = self.operations[2]
			command.shift_gears = self.operations[3]
			command.steer = self.operations[4]

		print("linear velocity: {}, throttle: {}, brake: {}".format(rcv_v, command.throttle, command.brake))
		self.pub.publish(command)

	# interface takes torques, kappa, beta as input
	def move_new(self):
		command = control_command()
		#self.sub = rospy.Subscriber('rosout_agg', Log, self.velcallback)
		#rospy.Subscriber('/base_pose_ground_truth', Odometry, self.statecallback)

		rcv_v = float(self.linear_v)
		ref_v = float(self.operations[2])
		
		if self.operations[0]:
			if abs(rcv_v - ref_v)/ref_v < thre:
				command.kappa = self.operations[3]
			else:
				ref_v = self.operations[1]
				pid = PID(P=1.2, I=10, D=0.1)
				pid.SetPoint = float(ref_v)
				pid.setSampleTime = rospy.Duration(1.0/20)
				pid.update(float(rcv_v))

				#print("pid output: "+ str(pid.output))
				command.fl_torque = pid.output
				command.fr_torque = pid.output
				command.rl_torque = pid.output
				command.rr_torque = pid.output
		else:
			command.fl_torque = self.operations[1]
			command.fr_torque = self.operations[2]
			command.rl_torque = self.operations[3]
			command.rr_torque = self.operations[4]
			command.kappa = self.operations[5]
			command.beta = self.operations[6]

		self.pub.publish(command)

	def pathControl(self):
		command = control_command()
		command.header = self.header
		rcv_v = float(self.linear_v)

		# pid control for velocity
		pid = PID(P=2, I=1.2, D=0)
		pid.SetPoint = float(self.ref_v)
		pid.setSampleTime = rospy.Duration(1.0/20)
		pid.update(float(rcv_v))

		torque_thresh = -0.01
		output = pid.output
		if pid.output < torque_thresh:
			output = torque_thresh

		command.fl_torque = output
		command.fr_torque = output
		command.rl_torque = output
		command.rr_torque = output

		# pure pursuit for path follow
		lookahead = 20
		desire = 0
		Path = np.transpose(np.array(self.planPath))

		delta_x = Path[:, 0] - self.x + self.x0
		delta_y = Path[:, 1] - self.y + self.y0
		dist_list = (delta_x*delta_x + delta_y*delta_y)**0.5

		index = np.argmin(dist_list)
		if index + lookahead < Path.shape[0] - 1:
			desire = index + lookahead
		else:
			desire = Path.shape[0] - 1

		newPoint = Path[desire]
		newPoint_x = newPoint[0] + self.x0
		newPoint_y = newPoint[1] + self.y0
		#new_yaw = newPoint[2] - self.yaw
		error_x = newPoint_x - self.x 
		error_y = newPoint_y - self.y
		dist = math.hypot(error_x, error_y)
		y_translate = -np.sin(self.yaw)*error_x + np.cos(self.yaw)*error_y
		command.kappa = 2*(y_translate/dist**2)
		command.beta = 0		#TODO: how to set beta in pure pursuit
		command.park_engage = self.park_state

		# stop the car if it approaches the end point
		# if desire == len(Path) - 1 and self.pre_dist < dist:
		# 	self.ref_v = 0
		# else:
		# 	self.ref_v = 1.2
		# self.pre_dist = dist

		self.pub.publish(command)

		command.fl_torque *= 1000
		command.fr_torque *= 1000
		command.rl_torque *= 1000
		command.rr_torque *= 1000

		if command.fl_torque > 30:
			command.fl_torque = 30
			command.fr_torque = 30
			command.rl_torque = 30
			command.rr_torque = 30
		elif command.fl_torque < -30:
			command.fl_torque = -30
			command.fr_torque = -30
			command.rl_torque = -30
			command.rr_torque = -30

		self.rcv_pub.publish(command)
		print("x: {5}, y: {6}, dist1: {7} dist: {0}, desire_point: {1}, current_point {2}, kappa: {3}, velocity: {4}"
			.format(dist, desire, index, command.kappa, rcv_v, self.x, self.y, dist_list[index]))


def interpolate(shape):
	route_x = []
	route_y = []
	points_per_meter = 5

	for index in range(1, len(shape)):
		dist_x = shape[index][0] - shape[index - 1][0]
		dist_y = shape[index][1] - shape[index - 1][1]
		print(dist_y)
		len_temp = (dist_x**2 + dist_y**2)**0.5

		num_points = int(len_temp * float(points_per_meter))
		for num in range(0, num_points):
			temp_x = shape[index - 1][0] + num * dist_x / num_points
			temp_y = shape[index - 1][1] + num * dist_y / num_points

			route_x.append(temp_x)
			route_y.append(temp_y)

	if route_x == []:
		route_x.append(shape[0][0])
		route_y.append(shape[0][1])

	direction_list = []
	for index in range(1, len(route_x)):
		x = route_x[index] - route_x[index-1]
		y = route_y[index] - route_y[index-1]

		direction = np.arctan2(y, x)
		direction_list.append(direction)

	direction_list.append(0)

	return [route_x, route_y, direction_list]


if __name__ == '__main__':
	path_name = "path3.dat"
	path_dir = "/home/el2425/catkin_ws/src/car_demo/car_demo/src/paths/"
	path_path = os.path.join(path_dir, path_name)
	rospy.init_node('rcv_controller', anonymous=True)

	print("Let's control your RCV")
	m_a = input("Choose automatic (1) or torque (0) control: ")
		
	if m_a:
		p_v = input("track path (1) or velocity (0): ")
		if p_v:
			planPath = []
			scale = 0.2
			with open(path_path) as f:
				for line in f:
					cur_data = [float(x) for x in line.split(',')]
					cur_data[0] *= scale
					planPath.append(cur_data)
			t = Translator(planPath=interpolate(planPath))  
		else:
			vel = input("Input reference velocity: ")
			kappa = input("Input reference kappa: ")
			operations = [m_a, vel, kappa]
			t = Translator(operations=operations)
	else:
		fl_torque = input("Input the front-left wheel torque: ")
		fr_torque = input("Input the front-right wheel torque: ")
		rl_torque = input("Input the rear-left wheel torque: ")
		rr_torque = input("Input the rear-right wheel torque: ")
		kappa = input("Input the kappa: ")
		beta = input("Input the beta: ")
		operations = [m_a, fl_torque, fr_torque, rl_torque, rr_torque, kappa, beta]
		t = Translator(operations=operations)
	
	rospy.loginfo("rcv_controller starting...")
	rospy.spin()

