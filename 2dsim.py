#!/usr/bin/env python

from __future__ import division
import numpy as np
import rospy
import matplotlib.pyplot as plt

from geometry_msgs.msg import WrenchStamped, PoseStamped, PointStamped
from roboteq_msgs.msg import Command
from nav_msgs.msg import Odometry, OccupancyGrid

rospy.set_param('inertia', ([1, 1, 2]))
rospy.set_param('drag', ([.5, 1, 1]))
rospy.set_param('mass', 544)
rospy.set_param('wind', ([1, 1, 1]))

class Navsim():
	def __init__(self, t=0, pose0=np.array([0, 0, 0]), twist0=np.array([0, 0, 0]), wind=True):
		self.des_force = np.array([0, 0, 0])

		rospy.Subscriber("/wrench/cmd", WrenchStamped, self.wrench_cb)
		self.publisher = rospy.Publisher("/ref/2dsim", Odometry, )
		# self.force = np.float64([0, 0])
		# X, Y, and angle in world frame
		self.pose = np.float64(pose0)
		# X, Y, and rate of angle change in boat frame (Surge, Sway, yaw rate)
		self.twist = np.float64(twist0)
		# kg, kg, kg*m^2
		self.inertia = np.array(rospy.get_param('inertia'), dtype=np.float64)
		# Boat is longer than shorter, hence .5, 1, 1. 
		self.drag = np.array(rospy.get_param('drag'), dtype=np.float64)
		# Time in seconds
		self.t = t
		# Wind is a function of time that returns a wrench
		if wind:
			self.wind = np.array(rospy.get_param('wind'), dtype=np.float64)
		else:
			self.wind = lambda t: np.zeros(3, dtype=np.float64)

	def step(self, dt, wrench):
		s = np.sin(self.pose[2])
		c = np.cos(self.pose[2])
		# Rotation Matrix converts body to world by default
		R = np.array([[c, -s, 0],[s, c, 0], [0, 0, 1]])
		wrench = np.array(wrench)
		posedot, twistdot = self.state_deriv(np.float64(wrench), R)
		self.pose = self.pose + posedot*dt + 0.5*R.dot(twistdot)*dt**2
		self.twist = self.twist + twistdot*dt
		self.t = self.t + dt

	def state_deriv(self, wrench, R):
		posedot = R.dot(self.twist)
		twistdot = (1/self.inertia) * (wrench - self.drag*self.twist + R.T.dot(self.wind(self.t)))
		return posedot, twistdot

	def wrench_cb(self, msg):
		''' Grab new wrench
			This is the 'b' in Ax = b
		'''
		self.force = msg.wrench.force
		self.torque = msg.wrench.torque
		# Set desired force and torque as numpy array
		self.des_force = np.array((force.x, force.y, torque.z))
if __name__ == '__main__':
	

	# rospy.Subscriber('/odom', Odometry, odom_callback)
	# rospy.Subscriber('ros_params', )
	# coordinate conversion server
	# Prep

	poses = []
	twists = []
	wrenches = []
	times = []
	wind = True
	navsim = Navsim(pose0=np.array([0, 0, np.pi/2]), wind=wind)
	duration = 10
	dt = 0.05
	# rospy.Publisher('/odom_arb')
	# Simulate
	for i in xrange(int(duration/dt)):
		# Record current
		poses.append(navsim.pose)
		twists.append(navsim.twist)
		times.append(navsim.t)
		# Increment sim
		# wrench = controller.compute_wrench(navsim.pose, navsim.twist, goal_pose, goal_twist)
		wrenches.append(np.array([navsim.des_force[0], navsim.des_force[1], navsim.des_force[2]]))
		navsim.step(dt, wrenches[-1])


	poses = np.array(poses)
	twists = np.array(twists)
	wrenches = np.array(wrenches)
	times = np.array(times)


	# Figure for individual results
	fig1 = plt.figure()
	fig1.suptitle('State Evolution', fontsize=20)
	fig1rows = 2
	fig1cols = 4

	# Plot x position
	ax1 = fig1.add_subplot(fig1rows, fig1cols, 1)
	ax1.set_title('East Position (m)', fontsize=16)
	ax1.plot(times, poses[:, 0], 'k')
	ax1.grid(True)

	# Plot y position
	ax1 = fig1.add_subplot(fig1rows, fig1cols, 2)
	ax1.set_title('North Position (m)', fontsize=16)
	ax1.plot(times, poses[:, 1], 'k')
	ax1.grid(True)

	# Plot yaw position
	ax1 = fig1.add_subplot(fig1rows, fig1cols, 3)
	ax1.set_title('Heading (deg)', fontsize=16)
	ax1.plot(times, np.rad2deg(poses[:, 2]), 'k')
	ax1.grid(True)

	# Plot control efforts
	ax1 = fig1.add_subplot(fig1rows, fig1cols, 4)
	ax1.set_title('Wrench (N, N, N*m)', fontsize=16)
	ax1.plot(times, wrenches[:, 0], 'b',
	         times, wrenches[:, 1], 'g',
	         times, wrenches[:, 2], 'r')
	ax1.grid(True)

	# Plot x velocity
	ax1 = fig1.add_subplot(fig1rows, fig1cols, 5)
	ax1.set_title('Surge (m/s)', fontsize=16)
	ax1.plot(times, twists[:, 0], 'k')
	ax1.set_xlabel('Time (s)')
	ax1.grid(True)

	# Plot y velocity
	ax1 = fig1.add_subplot(fig1rows, fig1cols, 6)
	ax1.set_title('Sway (m/s)', fontsize=16)
	ax1.plot(times, twists[:, 1], 'k')
	ax1.set_xlabel('Time (s)')
	ax1.grid(True)

	# Plot yaw velocity
	ax1 = fig1.add_subplot(fig1rows, fig1cols, 7)
	ax1.set_title('Yaw (deg/s)', fontsize=16)
	ax1.plot(times, np.rad2deg(twists[:, 2]), 'k')
	ax1.set_xlabel('Time (s)')
	ax1.grid(True)

	# Plot parametric
	ax1 = fig1.add_subplot(fig1rows, fig1cols, 8)
	ax1.set_title('Position (deg/s)', fontsize=16)
	ax1.scatter(poses[0, 0], poses[0, 1])
	ax1.plot(poses[:, 0], poses[:, 1], 'k')
	ax1.set_xlabel('Eastness (m)')
	ax1.set_ylabel('Northness (m)')
	ax1.grid(True)

	plt.show()