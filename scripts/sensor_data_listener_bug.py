#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import sensor_msgs.msg
from itertools import *
from operator import itemgetter


import tf
import random
import numpy as np
import math


def Clip(x, l, r):
	return max(min(x, r), l)
def get_dist(P1, P2):
	P1 = np.array(P1)
	P2 = np.array(P2)
	return pow(pow(P1 - P2, 2).sum(), 0.5)
	
def Close(P1, P2):
	return (get_dist(P1, P2) < 0.1)

class robot:
	def __init__(self):
		self.LINX = 0.0  # Always forward linear velocity.
		self.angz = 0
		self.PI = 3.1415926
		self.pos = [0, 0]
		self.tgt = [10,10]
		self.MODE = 'follow the line'
		self.Ranges = [0] * 720
		self.Obstacle_TH = 1.8
		self.angle_increment = 0
		self.wall_distance = 0
		self.d_left = 0
		self.d_right = 0
		self.d_front = 0
		self.dist = 0
		self.theta = 0
		self.yaw = 0
		self.stage = 1
		
	def Follow_the_line(self):
		delta = self.theta - self.yaw
		
		if abs(delta) < self.PI / 180 * 15 and self.d_front < self.Obstacle_TH:
			print('close!!')
			self.MODE = 'follow the wall step one'
			return
		if delta > self.PI:
			delta = delta - 2 * self.PI
		elif delta < -self.PI:
			delta = 2 * self.PI + delta
		
		if abs(delta) > self.PI / 3:
			self.LINX = 0
		else:
			self.LINX = Clip((self.d_front - self.Obstacle_TH) * 1.2, 0.5, 1.6)
		self.angz = delta * 1.5
		return 
		
	def Follow_the_wall_step_one(self):
		if self.MODE == 'follow the wall step one':
			right_side = min(self.Ranges[660:720])
			left_side = min(self.Ranges[0:60])
			if left_side < right_side:
				self.MODE = 'follow the wall step one by left'
			else:
				self.MODE = 'follow the wall step one by right'
					
		if self.MODE == 'follow the wall step one by left':
			if self.d_left > self.Obstacle_TH or self.d_front < self.Obstacle_TH: #no move and just rotate 
				self.angz = -0.8
				self.LINX = 0
				return
			else:
				self.wall_distance = Clip(self.d_left, 0.9, 1.2)
				self.MODE = 'follow the wall by left'
				
		elif self.MODE == 'follow the wall step one by right':
			if self.d_right > self.Obstacle_TH or self.d_front < self.Obstacle_TH:
				self.angz = 0.8
				self.LINX = 0
				return
			else:
				self.wall_distance = Clip(self.d_right, 0.9, 1.2)
				self.MODE = 'follow the wall by right'
		self.Follow_the_wall()
		
	def Follow_the_wall(self):
		delta = self.theta - self.yaw
		if delta < 0:
			delta = 2 * self.PI + delta
		index = int (delta / self.angle_increment)
	
		if index < 60:
			d_tgt = min(self.Ranges[index - 60 + 720: 720] + self.Ranges[0: index + 60])
		elif index + 60 > 720:
			d_tgt = min(self.Ranges[index - 60 : 720] + self.Ranges[0 : index + 60 - 720])
		else:
			d_tgt = min(self.Ranges[index - 60: index + 60])
		print("the distance in the target direction: %.2f" % d_tgt)	
	
	
	
		if d_tgt > self.Obstacle_TH * 2:
			self.MODE = 'follow the line'
			return
		else:
			if self.MODE == 'follow the wall by right':
				if self.d_front < self.Obstacle_TH:
					self.MODE = 'follow the wall step one by right'
					return
				self.angz = Clip( (self.wall_distance - self.d_right) * 0.4, -0.3 , 0.3)
			elif self.MODE == 'follow the wall by left':
				if self.d_front < self.Obstacle_TH:
					self.MODE = 'follow the wall step one by left'
					return
				self.angz = Clip( -(self.wall_distance - self.d_left) * 0.4, -0.3 , 0.3)
			self.LINX = 0.4
			return
		

	def Odometry_process(self, data):
		P = data.pose.pose.position
		self.dist += get_dist(self.pos, [P.x, P.y])
		self.pos = [P.x, P.y]
		self.theta = math.atan2(self.tgt[1] - self.pos[1] , self.tgt[0] - self.pos[0])
		O = data.pose.pose.orientation
		_, _, self.yaw = tf.transformations.euler_from_quaternion([O.x, O.y, O.z, O.w])
	
	
	def LaserScanProcess(self, data):
		self.Ranges = data.ranges
		self.d_front = min(self.Ranges[660:720] + self.Ranges[0:60])
		self.d_left = min(self.Ranges[0:210])
		self.d_right = min(self.Ranges[510:720])
		self.angle_increment = data.angle_increment
		
		'''
		range_angels = np.arange(len(data.ranges))
		ranges = np.array(data.ranges)
		range_mask = (ranges > THRESHOLD)
		ranges = list(range_angels[range_mask])

		# print(ranges)
		gap_list = []
		for k, g in groupby(enumerate(ranges), lambda (i, x): i - x):
			gap_list.append(map(itemgetter(1), g))
		gap_list.sort(key=len)
		largest_gap = gap_list[-1]
		min_angle, max_angle = largest_gap[0] * ((data.angle_increment) * 180 / PI), largest_gap[-1] * (
					(data.angle_increment) * 180 / PI)
		average_gap = (max_angle - min_angle) / 2

		turn_angle = min_angle + average_gap

		# print(min_angle, max_angle)
		# print(max_gap,average_gap,turn_angle)

		global LINX
		global angz
		if average_gap < max_gap:
			LINX = -2
			angz = 100 * Kp * (random.random() - 0.5)
		elif average_gap < free_gap:
			LINX = 1
			angz = Kp * (-1) * (90 - turn_angle)  # + 50 * Kp * (random.random() - 0.5)
		else:
			LINX = 1.5
			angz = Kp * (-1) * (90 - turn_angle) + 50 * Kp * (random.random() - 0.5)
		'''
	def Print(self):
		print('the status:' + self.MODE)
		print("the distance in front of you: %.2f" % self.d_front)
		print("the distance in right of you: %.2f" % self.d_right)
		print("the distance in left of you: %.2f"  % self.d_left)
		print('the wall distance: %.2lf' % self.wall_distance)
		print("the linx: %.2f" % self.LINX)
		print("the angz: %.2f" % self.angz)
	def Check(self):
		if Close(self.pos, self.tgt):
			if self.stage == 8:
				print(self.dist)
				return 'finish'
			else:
				self.stage += 1
				if self.stage % 2 == 0:
					self.tgt = [0,0]
				elif self.stage == 3:
					self.tgt = [10, -10]
				elif self.stage == 5:
					self.tgt = [-10, -10]
				elif self.stage == 7:
					self.tgt = [-10, 10]
		return 'next'
def main():
	Rot = robot()
	
	
	rospy.init_node('listener', anonymous=True)
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	rospy.Subscriber("scan", sensor_msgs.msg.LaserScan, Rot.LaserScanProcess)
	rospy.Subscriber("/odom", Odometry, Rot.Odometry_process)
	rate = rospy.Rate(10)  # 10hz
	
	while not rospy.is_shutdown():
		rate.sleep()
		if Rot.MODE == 'follow the line':
			Rot.Follow_the_line()
		else:
			Rot.Follow_the_wall_step_one()
		
		command = Twist()
		command.linear.x = Rot.LINX
		command.angular.z = Rot.angz
		pub.publish(command)
		
		Rot.Print()
		
		if Rot.Check() == 'finish':
			break	

			


if __name__ == '__main__':
	main()

