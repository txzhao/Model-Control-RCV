#!/usr/bin/env python

"""Main file for controlling RCV
"""

import os
import rospy
import math
import numpy as np
from matplotlib import pyplot as plt
import csv
import glob
import ConfigParser

import tf
from rcv_common_msgs.msg import control_command
from nav_msgs.msg import Odometry
from PID import PID 
from MPC import MPC
from PurePursuit import PurePursuit


class RCVControl:

	"""entry point to different control method
	"""

	def __init__(self, config=None, operations=None, ctrl_spec=None, planPath=None, ref_v=0.0):
		self.config = config
		self.operations = operations
		self.ctrl_spec = ctrl_spec
		self.planPath = planPath
		self.linear_v = 0
		self.x0 = 0.0
		self.y0 = 0.0
		self.x = self.x0
		self.y = self.y0
		self.yaw = 0
		self.ref_v = ref_v
		self.counter = 0
		self.pre_curv = 0.0
		self.pre_beta = 0.0
		self.log_path = ConfigSectionMap(config, "Log")['directory']
		self.pub = rospy.Publisher('rcv_control_cmd', control_command, queue_size=1)
		self.state_sub = rospy.Subscriber('/communication/dspace/rcv_odom', Odometry, self.state_callback)
		self.last_published_time = rospy.get_rostime()
		self.timer = rospy.Timer(rospy.Duration(config.getfloat("RospyTimer", "duration")), self.timer_callback)
	

	# callback function for getting feedbacks from gazebo
	def state_callback(self, data):
		if self.counter == 0:
			self.x0 = data.pose.pose.position.x
			self.y0 = data.pose.pose.position.y
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
	

	# callback function for calling different control methods
	def timer_callback(self, event):
		if self.counter == 0:
			if self.ctrl_spec:
				self.log_path += ConfigSectionMap(self.config, "Log")['filempc']
			else:
				self.log_path += ConfigSectionMap(self.config, "Log")['filepp']
			file_list = [int((os.path.basename(x)).replace('.csv', '')) for x in glob.glob(self.log_path)]
			self.log_path = self.log_path.replace('*', str(max(file_list) + 1)) if len(file_list) > 0 else self.log_path.replace('*', '1')
		self.counter += 1

		if self.last_published_time < rospy.get_rostime() + rospy.Duration(self.config.getfloat("RospyTimer", "duration")):
			if self.operations is None:
				if self.ctrl_spec:
					self.pathControlMPC()
				else:
					self.pathControlPP()
			else:
				self.manControl()


	# manual control - directly apply torques, kappa and beta to RCV
	def manControl(self):
		command = control_command()
		
		command.fl_torque = self.operations[1]
		command.fr_torque = self.operations[2]
		command.rl_torque = self.operations[3]
		command.rr_torque = self.operations[4]
		command.kappa = self.operations[5]
		command.beta = self.operations[6]

		self.pub.publish(command)


	# pure pursuit control
	def pathControlPP(self):
		command = control_command()
		rcv_v = float(self.linear_v)

		# pid control for velocity
		pid = PID(P=self.config.getfloat("PID", "p"), 
			I=self.config.getfloat("PID", "i"), 
			D=self.config.getfloat("PID", "d"))
		pid.SetPoint = float(self.ref_v)
		pid.setSampleTime = rospy.Duration(self.config.getfloat("PID", "samplingtime"))
		pid.update(float(rcv_v))

		torque_thresh = self.config.getfloat("PID", "torquethresh")
		output = pid.output
		if pid.output < torque_thresh:
			output = torque_thresh

		torque_scale = self.config.getfloat("InnerDynamics", "torquescale")
		torque_limit = self.config.getfloat("InnerDynamics", "torquelimit")

		output *= torque_scale
		if output > torque_limit:
			output = torque_limit
		elif output < -torque_limit:
			output = -torque_limit

		command.fl_torque = output
		command.fr_torque = output
		command.rl_torque = output
		command.rr_torque = output

		# create pure pursuit and update
		inte_path = self.interpolate(shape=self.planPath, 
			points_per_meter=self.config.getint("Interpolate", "pointspermeter"))
		purePursuit = PurePursuit(lookahead=self.config.getint("PurePursuit", "lookahead"), planPath=inte_path)
		command.kappa = purePursuit.update([self.x - self.x0, self.y - self.y0, self.yaw])
		command.beta = 0		#TODO: how to set beta in pure pursuit

		self.pub.publish(command)

		# save data as a log file
		with open(self.log_path, 'a') as newFile:
			newFileWriter = csv.writer(newFile)
			log_file = open(self.log_path)
			numline = len(log_file.readlines())
			if numline == 0:
				newFileWriter.writerow(['torque', 'kappa', 'velocity', 'x', 'y'])
			newFileWriter.writerow([command.fl_torque, command.kappa, rcv_v, self.x, self.y])

		print("torque: {0}, kappa: {1}, velocity: {2}, x: {3}, y: {4}"
			.format(command.fl_torque, command.kappa, rcv_v, self.x, self.y))


	# MPC control
	def pathControlMPC(self):
		command = control_command()
		rcv_v = float(self.linear_v)

		# pid control for velocity
		pid = PID(P=self.config.getfloat("PID", "p"), 
			I=self.config.getfloat("PID", "i"), 
			D=self.config.getfloat("PID", "d"))
		pid.SetPoint = float(self.ref_v)
		pid.setSampleTime = rospy.Duration(self.config.getfloat("PID", "samplingtime"))
		pid.update(float(rcv_v))

		torque_thresh = self.config.getfloat("PID", "torquethresh")
		output = pid.output
		if pid.output < torque_thresh:
			output = torque_thresh

		torque_scale = self.config.getfloat("InnerDynamics", "torquescale")
		torque_limit = self.config.getfloat("InnerDynamics", "torquelimit")
		
		output *= torque_scale
		if output > torque_limit:
			output = torque_limit
		elif output < -torque_limit:
			output = -torque_limit

		command.fl_torque = output
		command.fr_torque = output
		command.rl_torque = output
		command.rr_torque = output

		# create MPC and update
		inte_path = self.interpolate(shape=self.planPath, 
			points_per_meter=self.config.getint("Interpolate", "pointspermeter"))
		Path = np.transpose(np.array(inte_path))
		length = Path.shape[0]
		delta_x = Path[:, 0] - self.x + self.x0
		delta_y = Path[:, 1] - self.y + self.y0
		dist_list = (delta_x*delta_x + delta_y*delta_y)**0.5
		index = np.argmin(dist_list)

		Hc = self.config.getint("MPC", "hc")
		Hp = self.config.getint("MPC", "hp")
		Ts_MPC = self.config.getfloat("MPC", "samplingtime")
		mpc = MPC(Hc=Hc, Hp=Hp, Ts_MPC=Ts_MPC)
		cur_state = np.array([self.x - self.x0, self.y - self.y0, self.yaw])

		if index+Hc+Hp < length:
			end = index+Hc+Hp
			true_path = Path[index:end, :]
		else:
			self.ref_v = 0
			true_path = Path[index:, :]

		true_path = np.reshape(true_path, (true_path.shape[0]*true_path.shape[1], 1))
		true_path = [float(i) for i in list(true_path)]

		curv, beta, vref = mpc.update(cur_state, true_path, self.pre_curv, self.pre_beta)
		command.kappa = curv
		command.beta = beta
		self.ref_v = vref[0, 0]
		self.pre_curv = curv
		self.pre_beta = beta

		self.pub.publish(command)

		# save data as a log file
		with open(self.log_path, 'a') as newFile:
			newFileWriter = csv.writer(newFile)
			log_file = open(self.log_path)
			numline = len(log_file.readlines())
			if numline == 0:
				newFileWriter.writerow(['torque', 'kappa', 'beta', 'velocity', 'x', 'y'])
			newFileWriter.writerow([command.fl_torque, command.kappa, command.beta, rcv_v, self.x, self.y])

		print("torque: {0}, kappa: {1}, beta: {2}, velocity: {3}, x: {4}, y: {5}"
			.format(command.fl_torque, command.kappa, command.beta, rcv_v, self.x, self.y))


	# interpolation of path points
	def interpolate(self, shape, points_per_meter):
		route_x = []
		route_y = []

		for index in range(1, len(shape)):
			dist_x = shape[index][0] - shape[index - 1][0]
			dist_y = shape[index][1] - shape[index - 1][1]
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


# help function to read strings from config file
def ConfigSectionMap(config, section):
	dict1 = {}
	options = config.options(section)
	for option in options:
		try:
			dict1[option] = config.get(section, option)
			if dict1[option] == -1:
				DebugPrint("skip: %s" % option)
		except:
			print("exception on %s!" % option)
			dict1[option] = None
	return dict1


if __name__ == '__main__':
	Config = ConfigParser.ConfigParser()
	Config.read("/home/el2425/catkin_ws/src/rcv_joy_control/scripts/configs/config.ini")

	path_name = ConfigSectionMap(config=Config, section="Path")['name']
	path_dir = ConfigSectionMap(config=Config, section="Path")['directory']
	scale_x = Config.getfloat("Path", "scalex")
	scale_y = Config.getfloat("Path", "scaley")
	path_path = os.path.join(path_dir, path_name)

	rospy.init_node('rcv_controller')

	print("Let's control your RCV")
	m_a = input("Choose automatic (1) or torque (0) control: ")
		
	if m_a:
		ctrl_spec = input("MPC (1) or Pure Pursuit (0): ")
		planPath = []
		with open(path_path) as f:
			for line in f:
				cur_data = [float(x) for x in line.split(',')]
				cur_data[0] *= scale_x
				cur_data[1] *= scale_y
				planPath.append(cur_data)
		if ctrl_spec:
			t = RCVControl(config=Config, ctrl_spec=ctrl_spec, planPath=planPath)
		else:
			ref_v = input("Input reference velocity: ")
			t = RCVControl(config=Config, ctrl_spec=ctrl_spec, planPath=planPath, ref_v=ref_v)  
	else:
		fl_torque = input("Input the front-left wheel torque: ")
		fr_torque = input("Input the front-right wheel torque: ")
		rl_torque = input("Input the rear-left wheel torque: ")
		rr_torque = input("Input the rear-right wheel torque: ")
		kappa = input("Input the kappa: ")
		beta = input("Input the beta: ")
		operations = [m_a, fl_torque, fr_torque, rl_torque, rr_torque, kappa, beta]
		t = RCVControl(config=Config, operations=operations)
	
	rospy.spin()

