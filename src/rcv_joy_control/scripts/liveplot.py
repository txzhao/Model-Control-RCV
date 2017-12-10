#!/usr/bin/env python

import os
import numpy as np
from matplotlib import pyplot as plt
from nav_msgs.msg import Odometry
import rospy
import math

def plot_xy(msg):
	global counter
	path_name = "path3.dat"
	path_dir = "/home/el2425/catkin_ws/src/car_demo/car_demo/src/paths/"
	path_path = os.path.join(path_dir, path_name)
	x0 = 3.0
	y0 = -12.0
	scale = 0.2

	if counter == 0:
		planPath = []
		with open(path_path) as f:
			for line in f:
				cur_data = [float(x) for x in line.split(',')]
				cur_data[0] *= scale
				planPath.append(cur_data)
		Path = np.array(interpolate(planPath))

		plt.plot(x0 + Path[0, :], y0 + Path[1, :], linewidth=2.0)
		#plt.plot(x0 + Path[0, :], y0 + Path[1, :], 'ko')
		plt.draw()
		plt.pause(1e-12)

	elif counter%100 == 0:
		cur_x = msg.pose.pose.position.x
		cur_y = msg.pose.pose.position.y
		cur_yaw = msg.pose.pose.orientation.z
		dx = math.sqrt(1.0/(1.0 + (np.tan(cur_yaw))**2))
		dy = dx*np.tan(cur_yaw)

		plt.plot(cur_x, cur_y, 'ro')
		ax = plt.axes()
		ax.arrow(x=cur_x, y=cur_y, dx=dx, dy=dy, width=0.04, head_length=1.5, color='k')
		plt.draw()
		plt.pause(1e-12)
	
	plt.title("Path name: " + path_name)
	plt.xlabel('x')
	plt.ylabel('y')
	counter += 1

def interpolate(shape):
	route_x = []
	route_y = []
	points_per_meter = 5

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

if __name__ == '__main__':
	counter = 0
	rospy.init_node("plotter")
	rospy.Subscriber('/base_pose_ground_truth', Odometry, plot_xy)
	plt.ion()
	plt.show()
	rospy.spin()