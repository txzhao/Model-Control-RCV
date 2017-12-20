#!/usr/bin/env python

import os
import numpy as np
from matplotlib import pyplot as plt
from nav_msgs.msg import Odometry
import tf
import rospy
import math
import ConfigParser


def plot_xy(msg):
	global counter
	Config = ConfigParser.ConfigParser()
	Config.read("/home/el2425/catkin_ws/src/car_demo/car_demo/src/configs/config.ini")

	path_name = ConfigSectionMap(config=Config, section="Path")['name']
	path_dir = ConfigSectionMap(config=Config, section="Path")['directory']
	scale_x = Config.getfloat("Path", "scalex")
	scale_y = Config.getfloat("Path", "scaley")
	path_path = os.path.join(path_dir, path_name)

	if counter == 0:
		x0 = msg.pose.pose.position.x
		y0 = msg.pose.pose.position.y
		planPath = []
		with open(path_path) as f:
			for line in f:
				cur_data = [float(x) for x in line.split(',')]
				cur_data[0] *= scale_x
				cur_data[1] *= scale_y
				planPath.append(cur_data)
		Path = np.array(interpolate(shape=planPath, 
			points_per_meter=Config.getint("Interpolate", "pointspermeter")))

		if Config.getint("LivePlot", "lineorscatter"):
			plt.plot(x0 + Path[0, :], y0 + Path[1, :], linewidth=2.0)
		else:
			plt.plot(x0 + Path[0, :], y0 + Path[1, :], 'ko')
		plt.draw()
		plt.pause(Config.getfloat("LivePlot", "plotpause"))

	elif counter%Config.getint("LivePlot", "samplingrate") == 0:
		cur_x = msg.pose.pose.position.x
		cur_y = msg.pose.pose.position.y
		quaternion = (
			msg.pose.pose.orientation.x,
			msg.pose.pose.orientation.y,
			msg.pose.pose.orientation.z,
			msg.pose.pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		cur_yaw = euler[2]
		dx = np.sign(np.cos(cur_yaw))*math.sqrt(1.0/(1.0 + (np.tan(cur_yaw))**2))
		dy = dx*np.tan(cur_yaw)

		plt.plot(cur_x, cur_y, 'ro')
		ax = plt.axes()
		ax.arrow(x=cur_x, y=cur_y, dx=dx, dy=dy, color='k')
		plt.draw()
		plt.pause(Config.getfloat("LivePlot", "plotpause"))
	
	plt.title("Path name: " + path_name)
	plt.xlabel('x')
	plt.ylabel('y')
	counter += 1


def interpolate(shape, points_per_meter):
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
	counter = 0
	rospy.init_node("plotter")
	rospy.Subscriber('/base_pose_ground_truth', Odometry, plot_xy)
	plt.ion()
	plt.show()
	rospy.spin()