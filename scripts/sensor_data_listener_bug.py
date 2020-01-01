#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import sensor_msgs.msg
import tf

import random
import numpy as np
from itertools import *
from operator import itemgetter
import math

LINX = 0.0  # Always forward linear velocity.
angz = 0

PI = 3.1415926

pos_x = 0
pos_y = 0


tgt_x = 10
tgt_y = 10

yaw = 0

MODE = 'follow the line'
Ranges = [0] * 720
Obstacle_TH = 1.8
angle_increment = 0


wall_distance = 0
d_left = 0
d_right = 0
d_front = 0


real_v = 0
def Clip(x, l, r):
	return max(min(x, r), l)


def LaserScanProcess(data):
	global angle_increment
	global Ranges
	global d_left
	global d_right
	global d_front
	
	d_front = min(Ranges[660:720] + Ranges[0:60])
	d_left = min(Ranges[0:210])
	d_right = min(Ranges[510:720])
	
	
	Ranges = data.ranges
	angle_increment = data.angle_increment
	
	
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



def get_dist(x, y, X, Y):
	temp_dist = math.pow(x - X, 2) + math.pow(y - Y, 2)
	temp_dist = math.sqrt(temp_dist)
	return temp_dist

def Close(x1, y1, x2, y2):
	temp_dist = get_dist(x1, y1, x2, y2)
	return (temp_dist < 0.1)

def Odometry_process(data):
	global pos_x
	global pos_y
	
	X = data.pose.pose.position.x
	Y = data.pose.pose.position.y


	pos_x = X
	pos_y = Y  # update the record

	OX = data.pose.pose.orientation.x
	OY = data.pose.pose.orientation.y
	OZ = data.pose.pose.orientation.z
	OW = data.pose.pose.orientation.w
	global yaw
	roll, pitch, yaw = tf.transformations.euler_from_quaternion([OX, OY, OZ, OW])


def Follow_the_line():
	global yaw
	global LINX
	global angz
	global Ranges
	global MODE
	global d_front
	global pos_x
	global pos_y
	global tgt_x
	global tgt_y
	
	
	theta = math.atan2(tgt_y - pos_y , tgt_x - pos_x)
	if abs(yaw - theta) < 1e-1:
		if d_front < Obstacle_TH:
			print('close!!')
			MODE = 'follow the wall step one'
			return
		else:
			LINX = Clip((d_front - Obstacle_TH) * 1.2, 0.5, 1.6)
			angz = 0
			return
	
	

	delta = theta - yaw
	if delta > PI:
		delta = delta - 2 * PI
	elif delta < -PI:
		delta = 2 * PI + delta
	
	if abs(delta) > PI / 3:
		LINX = 0
	else:
		LINX = Clip((d_front - Obstacle_TH) * 1.2, 0.5, 1.6)
	angz = delta * 1.5
	return
		
def Follow_the_wall():
	global Ranges
	global pos_x
	global pos_y
	global MODE
	global yaw
	global LINX
	global angz
	global d_front
	global d_left
	global d_right
	global tgt_x
	global tgt_y
	global wall_distance
	if MODE == 'follow the wall step one':
		right_side = min(Ranges[660:720])
		left_side = min(Ranges[0:60])
		print("left side:" + str(left_side))
		print("right_side:" + str(right_side))
		
		if left_side < right_side:
			MODE = 'follow the wall step one by left'
			return
		else:
			MODE = 'follow the wall step one by right'
			return
			
	if MODE == 'follow the wall step one by left':
		if d_left > Obstacle_TH or d_front < Obstacle_TH:
			angz = -0.8
			LINX = 0
			return
		else:
			wall_distance = Clip(d_left, 0.9, 1.2)
			MODE = 'follow the wall by left'
			return
	elif MODE == 'follow the wall step one by right' :
		if d_right > Obstacle_TH or d_front < Obstacle_TH:
			angz = 0.8
			LINX = 0
			return
		else:
			wall_distance = Clip(d_right, 0.9, 1.2)
			MODE = 'follow the wall by right'
			return

	theta = math.atan2(tgt_y - pos_y , tgt_x - pos_x)
	delta = theta - yaw
	
	if delta < 0:
		delta = 2 * PI + delta
	index = int (delta / angle_increment)
	print("index:" + str (index))
	
	if index < 60:
		d_tgt = min(Ranges[index - 60 + 720: 720] + Ranges[0: index + 60])
	elif index + 60 > 720:
		d_tgt = min(Ranges[index - 60 : 720] + Ranges[0 : index + 60 - 720])
	else:
		d_tgt = min(Ranges[index - 60: index + 60])
	
	print("the distance in the target direction: " + str(d_tgt) )	
	
	
	if d_tgt > Obstacle_TH * 3:
		MODE = 'follow the line'
		return
	
	else:
		if MODE == 'follow the wall by right':
			if d_front < Obstacle_TH:
				MODE = 'follow the wall step one by right'
				return
			angz = Clip( (wall_distance - d_right) * 0.4, -0.3 , 0.3)
		elif MODE == 'follow the wall by left':
			if d_front < Obstacle_TH:
				MODE = 'follow the wall step one by left'
				return
			angz = Clip( -(wall_distance - d_left) * 0.4, -0.3 , 0.3)
		LINX = 0.4
		return

def main():
	global MODE
	global LINX
	global angz
	global d_front
	global d_right
	global d_left
	global tgt_x
	global tgt_y
	global pos_x
	global pos_y
	global yaw
	global wall_distance
	
	rospy.init_node('listener', anonymous=True)
	
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	rospy.Subscriber("scan", sensor_msgs.msg.LaserScan, LaserScanProcess)
	rospy.Subscriber("/odom", Odometry, Odometry_process)
	
	
	rate = rospy.Rate(10)  # 10hz
	
	
	stage = 1
	last_x = 0
	last_y = 0
	last_yaw = 0
	dist = 0
	back_MODE = MODE
	count = 0
	while not rospy.is_shutdown():
		print('the status:' + MODE)
		real_v = get_dist(last_x, last_y, pos_x, pos_y)
		real_w = abs(last_yaw - yaw)
		dist += real_v 
		
		last_x = pos_x
		last_y = pos_y
		last_yaw = yaw

			
		if MODE == 'stuck':
			if count == 0:
				MODE = back_MODE
			else:
				count -= 1
				angz = random.random() - 0.5
				LINX = -0.5
		elif MODE == 'follow the line':
			Follow_the_line()
		else:
			Follow_the_wall()
		
			
		
		command = Twist()
		command.linear.x = LINX
		command.angular.z = angz
		
		
		pub.publish(command)
		
		'''
		if real_v < 0.003 and real_w < 0.003 and MODE != 'stuck':
			back_MODE = MODE
			count = 10
			MODE = 'stuck'
		'''
		print("the distance in front of you: " + str(d_front))
		print("the distance in right of you: " + str(d_right))
		print("the distance in left of you: "  + str(d_left))
		print('the wall distance:' + str(wall_distance))
		print("the linx: " + str(LINX))
		print("the angz: " + str(angz))
		print("the real_v: " + str(real_v))
		print("the real_w: " + str(real_w))
		
		
		if Close(pos_x, pos_y, tgt_x, tgt_y):
			if stage == 8:
				print(dist)
				break
			else:
				stage += 1
				if stage % 2 == 0:
					tgt_x = 0
					tgt_y = 0
				elif stage == 3:
					tgt_x = 10
					tgt_y = -10
				elif stage == 5:
					tgt_x = -10
					tgt_y = -10
				elif stage == 7:
					tgt_x = -10
					tgt_y = 10
					
		rate.sleep()

			


if __name__ == '__main__':
	main()

