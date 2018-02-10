#!/usr/bin/env python

from __future__ import division
import numpy as np
import rospy
import matplotlib.pyplot as plt
import sys
import tf.transformations as trns
import os

from geometry_msgs.msg import WrenchStamped, PoseStamped, PointStamped
from roboteq_msgs.msg import Command
from nav_msgs.msg import Odometry, OccupancyGrid

import rosbag
from std_msgs.msg import Int32, String

rospy.set_param('inertia', ([1, 1, 2]))
rospy.set_param('drag', ([.5, 1, 1]))
rospy.set_param('mass', 544)

class Navsim():
	def __init__(self, t=0, pose0=np.array([0, 0, 0]), twist0=np.array([0, 0, 0]), wind=None):

		rospy.init_node('2Dsim')
		self.des_force = np.array([0, 0, 0])

		self.world_frame_id = None
		self.body_frame_id = None 
		self.state = None
		self.get_ref = None


		self.subscriber = rospy.Subscriber("/wrench/cmd", WrenchStamped, self.wrench_cb)
		rospy.Subscriber('/odom', Odometry, self.odom_cb)
		self.publisher = rospy.Publisher("/ref/2dsim", Odometry, queue_size=1)


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
			self.wind = wind
		else:
			self.wind = lambda t: np.zeros(3, dtype=np.float64)
		rospy.spin()

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
		self.des_force = np.array((self.force.x, self.force.y, self.torque.z))
	# return self.des_force

	def pack_odom(self):
		"""
		Converts a state vector into an Odometry message
		with a given header timestamp.
		"""
		msg = Odometry()
		msg.header.stamp = self.t
		msg.header.frame_id = self.world_frame_id
		msg.child_frame_id = self.body_frame_id
		msg.pose.pose.position.x, msg.pose.pose.position.y = self.pose[0:2]
		# quat = trns.quaternion_from_euler(0, 0, self.twist)
		msg.pose.pose.orientation = self.pose[2]
		msg.twist.twist.linear.x, msg.twist.twist.linear.y = self.twist[0:2]
		msg.twist.twist.angular.z = self.twist[2]
		return msg

	def unpack_odom(self, msg):		
		"""
		Converts an Odometry message into a state vector.

		"""
		q = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
		msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
		return np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, trns.euler_from_quaternion(q)[2],
							msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z])
	def odom_cb(self, msg):
		"""
		Expects an Odometry message.
		Stores the current state of the vehicle tracking the plan.
		Reference frame information is also recorded.
		Determines if the vehicle is tracking well.

		"""
		self.world_frame_id = msg.header.frame_id
		self.body_frame_id = msg.child_frame_id
		self.state = self.unpack_odom(msg)
		last_update_time = self.t
		if self.get_ref is not None and last_update_time is not None:
			error = np.abs(self.erf(self.get_ref(self.rostime() - last_update_time), self.state))
			if np.all(error < 2 * np.array(params.real_tol)):
				self.tracking = True
			else:
				self.tracking = False
if __name__ == '__main__':
	

	# Check to see if additional command line argument was passed.
	# Allows 2D sim to play bags on its own, bag must be in the same directory. Fix this later. 

	# coordinate conversion server
	# Prep
	poses = []
	twists = []
	wrenches = []
	times = []
	wind = True
	try:
		# If the user does pass a bag in, we pull some attributes and run the code. 
		bag_name = sys.argv[1]
		bag = rosbag.Bag(bag_name, 'r')
		start_time = bag.get_start_time()
		end_time = bag.get_end_time()
		duration = end_time - start_time
		# We have to create a subprocess to run the bag and the sim simultaneously. 
		pid = os.fork()
		# Parent fork i.e. the simulator. 
		if pid:

			navsim = Navsim(pose0=np.array([0, 0, np.pi/2]), wind=lambda t: np.array([5, 0, 0]))
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
				navsim.publisher.publish(navsim.pack_odom())
				wrenches.append(navsim.des_force)
				navsim.step(dt, wrenches[-1])
		# Child process i.e. the bag file being played
		else:
			rosbag.rosbag_main.play_cmd([bag_name])
	# If nothing gets passed we continue as normal. 
	except IndexError:
		duration = 20
	
	navsim = Navsim(pose0=np.array([0, 0, np.pi/2]), wind=lambda t: np.array([5, 0, 0]))
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
		navsim.publisher.publish(navsim.pack_odom())
		wrenches.append(navsim.des_force)
		navsim.step(dt, wrenches[-1])


	poses = np.array(poses)
	twists = np.array(twists)
	wrenches = np.array(wrenches)
	times = np.array(times)
	# print(wrenches)


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