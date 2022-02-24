#!/usr/bin/env python
import math
from math import atan2, degrees
import rospy
import time
from geometry_msgs.msg import Twist
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import sys
import signal
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import String
from robot import Robot, Spot
object_ahead = bool(False)
robot_ahead = bool(False)
roll = pitch = yaw = 0.0
orders = " "
Ready = bool(False)
assigned = bool(False)
right = bool(False)
TargetX = float(0)
TargetY = float(0)

R1 = Robot(0, 0, 0, 0)
R2 = Robot(0, 0, 0, 0)
R3 = Robot(0, 0, 0, 0)
S1 = Spot(0, 0, 1)
S2 = Spot(0, 0, 2)
S3 = Spot(0, 0, 3)
S1.number = 1
S2.number = 2
S3.number = 3


def update_orders(msg):
	global orders
	orders = msg.data
	print(orders)


def get_rotation_pos(msg):
	global roll, pitch, yaw
	orientation_q = msg.pose.pose.orientation
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
	R1.heading = yaw
	R1.x = float(msg.pose.pose.position.x)
	R1.y = float(msg.pose.pose.position.y)


def check_laser(msg):
	global object_ahead
	object_ahead = False
	object_ahead_distance = min(msg.ranges[0], msg.ranges[10], msg.ranges[20], msg.ranges[350], msg.ranges[340])
	if object_ahead_distance < 0.8:
		object_ahead = True
		print("object ahead")


def check_robot2(msg):
	global robot_ahead
	robot_ahead = False
	R2.x = float(msg.pose.pose.position.x)
	R2.y = float(msg.pose.pose.position.y)
	distance = math.sqrt((R2.x - R1.x)**2 + (R2.y - R1.y)**2)
	if distance < 1:  # only do any of this if robot2 is close
		Xdistance = R2.x - R1.x
		Ydistance = R2.y - R1.y
		if (-0.25*np.pi) < R1.heading <= (0.75*np.pi):  # check northwest
			if Xdistance >= 0 and Ydistance >= 0:
				robot_ahead = True
		if (0.25*np.pi) < R1.heading <= (1.25*np.pi):  # check southwest
			if Xdistance <= 0 and Ydistance >= 0:
				robot_ahead = True
		if (-1.25*np.pi) < R1.heading <= (-0.25*np.pi):  # check southeast
			if Xdistance <= 0 and Ydistance <= 0:
				robot_ahead = True
		if (-0.75*np.pi) < R1.heading <= (0.25*np.pi):  # check northeast
			if Xdistance >= 0 and Ydistance <= 0:
				robot_ahead = True


def check_robot3(msg):
	global robot_ahead
	R3.x = float(msg.pose.pose.position.x)
	R3.y = float(msg.pose.pose.position.y)
	distance = math.sqrt((R3.x - R1.x)**2 + (R3.y - R1.y)**2)
	if distance < 1:  # only do any of this if robot3 is close
		Xdistance = R3.x - R1.x
		Ydistance = R3.y - R1.y
		if (-0.25*np.pi) < R1.heading <= (0.75*np.pi):  # check northwest
			if Xdistance >= 0 and Ydistance >= 0:
				robot_ahead = True
		if (0.25*np.pi) < R1.heading <= (1.25*np.pi):  # check southwest
			if Xdistance <= 0 and Ydistance >= 0:
				robot_ahead = True
		if (-1.25*np.pi) < R1.heading <= (-0.25*np.pi):  # check southeast
			if Xdistance <= 0 and Ydistance <= 0:
				robot_ahead = True
		if (-0.75*np.pi) < R1.heading <= (0.25*np.pi):  # check northeast
			if Xdistance >= 0 and Ydistance <= 0:
				robot_ahead = True


def go(msg):
	global object_ahead
	global orders
	global assigned
	global TargetX
	global TargetY
	global Ready
	global right
	all_robots = [R1, R2, R3]
	all_spots = [S1, S2, S3]
	if orders == "1":
		assigned = False
		right = False
		for item in all_robots:
			item.spot = 0
		for item in all_spots:
			item.occupied = False
		if object_ahead:
			move = Twist()
			move.linear.x = 0.0
			move.angular.z = 1.0
			move.linear.y = 0.0
			pub = rospy.Publisher("/robot1/cmd_vel", Twist, queue_size=1)
			pub.publish(move)
		if robot_ahead:
			move = Twist()
			move.linear.x = 0.0
			move.angular.z = 0.5
			move.linear.y = 0.0
			pub = rospy.Publisher("/robot1/cmd_vel", Twist, queue_size=1)
			pub.publish(move)
		if not object_ahead and not robot_ahead:
			move = Twist()
			move.linear.x = 0.5
			move.angular.z = 0.0
			move.linear.y = 0.0
			pub = rospy.Publisher("/robot1/cmd_vel", Twist, queue_size=1)
			pub.publish(move)
	if orders == "0":
		assigned = False
		for item in all_robots:
			item.spot = 0
		for item in all_spots:
			item.occupied = False
		right = False
		move = Twist()
		move.linear.x = 0.0
		move.angular.z = 0.0
		move.linear.y = 0.0
		pub = rospy.Publisher("/robot1/cmd_vel", Twist, queue_size=2)
		pub.publish(move)
	if orders == "2":
		S1.x = 0
		S1.y = 0
		S2.x = 1
		S2.y = 0
		S3.x = 2
		S3.y = 0
	if orders == "3":
		S1.x = 2
		S1.y = 2
		S2.x = 1
		S2.y = 1
		S3.x = 2
		S3.y = 0
	if orders == "5":
		S1.x = 2
		S1.y = 1.5
		S2.x = 1.25
		S2.y = 1.2
		S3.x = 1.25
		S3.y = 1.7
	if orders == "2" or "3" or "4" or "5":
		Ready = False
		center_x = float((S1.x + S2.x + S3.x)/3)
		center_y = float((S1.y + S2.y + S3.y)/3)
		if not assigned:
			print("robots are not assigned yet")
			move = Twist()
			move.linear.x = 0.0
			move.angular.z = 0.0
			move.linear.y = 0.0
			pub = rospy.Publisher("/robot1/cmd_vel", Twist, queue_size=2)
			pub.publish(move)
			R1Dist = math.sqrt(((center_x - R1.x)**2)+((center_y - R1.y)**2))  # distance r1's position to "formation center"
			R2Dist = math.sqrt(((center_x - R2.x)**2)+((center_y - R2.y)**2))
			R3Dist = math.sqrt(((center_x - R3.x)**2)+((center_y - R3.y)**2))
			robot_distances = [0.0, R1Dist, R2Dist, R3Dist]
			available_spots = [S1, S2, S3]
			decision_message = " "
			while not assigned:
				if max(robot_distances) == R1Dist:
					robot_distances.remove(R1Dist)
					for item in available_spots:
						if item.occupied:
							available_spots.remove(item)
					check_distance = 100
					for item in available_spots:
						potential_distance = math.sqrt((item.x - R1.x)**2 + (item.y - R1.y)**2)
						if potential_distance < check_distance:
							check_distance = potential_distance
							R1.spot = item.number 
					for item in available_spots:
						if R1.spot == item.number:
							item.occupied = True
					decision_message += " R1spot" + str(R1.spot) 
				if len(robot_distances) == 1:
					assigned = True
					break
				if max(robot_distances) == R2Dist:
					robot_distances.remove(R2Dist)
					for item in available_spots:
						if item.occupied:
							available_spots.remove(item)
					check_distance = 100
					for item in available_spots:
						potential_distance = math.sqrt((item.x - R2.x)**2 + (item.y - R2.y)**2)
						if potential_distance < check_distance:
							check_distance = potential_distance
							R2.spot = item.number 
					for item in available_spots:
						if R2.spot == item.number:
							item.occupied = True
					decision_message += " R2spot" + str(R2.spot) 
				if len(robot_distances) == 1:
					assigned = True
					break
				if max(robot_distances) == R3Dist:
					robot_distances.remove(R3Dist)
					for item in available_spots:
						if item.occupied:
							available_spots.remove(item)
					check_distance = 100
					for item in available_spots:
						potential_distance = math.sqrt((item.x - R3.x)**2 + (item.y - R3.y)**2)
						if potential_distance < check_distance:
							check_distance = potential_distance
							R3.spot = item.number 
					for item in available_spots:
						if R3.spot == item.number:
							item.occupied = True
				decision_message += " R3spot" + str(R3.spot) 
				if len(robot_distances) == 1:
					assigned = True
					break
			print(decision_message)
			pub2 = rospy.Publisher('/robot1/chatter', String, queue_size=1)
			rate = rospy.Rate(10)
			pub2.publish(decision_message)
		for item in all_spots:
			if R1.spot == item.number:  # the robot needs to point to its assigned position.
				angle = atan2(R1.y - item.y, R1.x - item.x) - np.pi
				TargetX = item.x
				TargetY = item.y
		if angle < -3.1415:
			angle += (2*np.pi)  # important line of code to make sure the angle value stays in the correct range of values
		if (angle-0.3) < R1.heading < (angle + 0.3):
			Ready = True
			move = Twist()
			move.linear.x = 0.0
			move.angular.z = 0.0
			move.linear.y = 0.0
			pub = rospy.Publisher("/robot1/cmd_vel", Twist, queue_size=2)
			pub.publish(move)
		if not Ready:
			target_distance = math.sqrt(((R1.y-TargetY)**2)+((R1.x - TargetX)**2))
			if target_distance > 0.05:
				if 0 <= R1.heading < (np.pi/2):  # get the rover to turn right in some cases, depending on its quadrant
					if (np.pi/-2) <= angle <= 0:
						right = True
				if (np.pi/-2) <= R1.heading < 0:
					if (np.pi*-1) <= angle <= (np.pi/-2):
						right = True
				if (np.pi/-1) <= R1.heading < (np.pi/-2):
					if (np.pi/2) <= angle <= np.pi:
						right = True
				if (np.pi/2) <= R1.heading < np.pi:
					if 0 <= angle <= (np.pi/2):
						right = True
				move = Twist()
				move.linear.x = 0.0
				move.angular.z = 1.0
				move.linear.y = 0.0
				if right:
					move.angular.z = -1.0
				pub = rospy.Publisher("/robot1/cmd_vel", Twist, queue_size=2)
				pub.publish(move)
				print("must face target")
			if target_distance <= 0.05:
				move = Twist()
				move.linear.x = 0.0
				move.angular.z = 0.0
				move.linear.y = 0.0
				pub = rospy.Publisher("/robot1/cmd_vel", Twist, queue_size=2)
				pub.publish(move)
				print("reached target")
		if Ready:
			print("drive ahead")
			right = False
			target_distance = math.sqrt(((R1.y-TargetY)**2)+((R1.x - TargetX)**2))
			if target_distance > 0.05:
				move = Twist()
				move.linear.x = 0.5
				if robot_ahead:
					move.linear.x = 0.1
				move.angular.z = 0.0
				move.linear.y = 0.0
				pub = rospy.Publisher("/robot1/cmd_vel", Twist, queue_size=2)
				pub.publish(move)
				print(target_distance)
			if target_distance <= 0.05:
				move = Twist()
				move.linear.x = 0.0
				move.angular.z = 0.0
				move.linear.y = 0.0
				pub = rospy.Publisher("/robot1/cmd_vel", Twist, queue_size=2)
				pub.publish(move)
				print("reached target")
	if orders == "6":
		assigned = False
		Ready = False
		for item in all_spots:
			item.occupied = False
		right = False
		for item in all_spots:
			if R1.spot == item.number:  # the robot needs to point to its assigned position.
				angle = atan2(0 - item.y, 0 - item.x) - np.pi
		if angle < -3.1415:
			angle += (2*np.pi)  # important line of code to make sure the angle value stays in the correct range of values
		if (angle-0.1) < R1.heading < (angle + 0.1):
			Ready = True
			move = Twist()
			move.linear.x = 0.0
			move.angular.z = 0.0
			move.linear.y = 0.0
			pub = rospy.Publisher("/robot1/cmd_vel", Twist, queue_size=2)
			pub.publish(move)
		if not Ready:
			if 0 <= R1.heading < (np.pi/2):  # get the rover to turn right in some cases, depending on its quadrant
				if (np.pi/-2) <= angle <= 0:
					right = True
			if (np.pi/-2) <= R1.heading < 0:
				if (np.pi*-1) <= angle <= (np.pi/-2):
					right = True
			if (np.pi/-1) <= R1.heading < (np.pi/-2):
				if (np.pi/2) <= angle <= np.pi:
					right = True
			if (np.pi/2) <= R1.heading < np.pi:
				if 0 <= angle <= (np.pi/2):
					right = True
				move = Twist()
				move.linear.x = 0.0
				move.angular.z = 1.0
				move.linear.y = 0.0
				if right:
					move.angular.z = -1.0
				pub = rospy.Publisher("/robot1/cmd_vel", Twist, queue_size=2)
				pub.publish(move)
				print("must face target")
			if object_ahead:
				move = Twist()
				move.linear.x = 0.0
				move.angular.z = 0.0
				move.linear.y = 0.0
				pub = rospy.Publisher("/robot1/cmd_vel", Twist, queue_size=2)
				pub.publish(move)
		if Ready and not object_ahead:
			right = False
			move = Twist()
			move.linear.x = 0.5
			if robot_ahead:
				move.linear.x = 0.1
			move.angular.z = 0.0
			move.linear.y = 0.0
			pub = rospy.Publisher("/robot1/cmd_vel", Twist, queue_size=2)
			pub.publish(move)
		if object_ahead:
			move = Twist()
			move.linear.x = 0.0
			move.angular.z = 0.0
			move.linear.y = 0.0
			pub = rospy.Publisher("/robot1/cmd_vel", Twist, queue_size=2)
			pub.publish(move)


def signal_handler(signal, frame):
	move = Twist()
	move.linear.x = 0.0
	move.angular.z = 0.0
	move.linear.y = 0.0
	pub = rospy.Publisher("/robot1/cmd_vel", Twist, queue_size=2)
	pub.publish(move)
	print("exiting program")
	sys.exit(0)


def listener():
	rospy.init_node('listener')
	rospy.Subscriber("/chatter", String, update_orders)
	rospy.Subscriber("/robot1/scan", LaserScan, check_laser)
	rospy.Subscriber("/robot1/odom", Odometry, get_rotation_pos)
	rospy.Subscriber("/robot2/odom", Odometry, check_robot2)
	rospy.Subscriber("/robot3/odom", Odometry, check_robot3)
	rospy.Subscriber("/robot1/odom", Odometry, get_rotation_pos)
	rospy.Subscriber("/robot1/odom", Odometry, go)
	rospy.spin()


if __name__ == '__main__':
	listener()
