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
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_from_euler
MyX = float(0)
MyY = float(0)
R1X = float(0)
R1Y = float(0)
R2X = float(0)
R2Y = float(0)
object_ahead = bool(False)
robot_ahead = bool(False)
roll = pitch = yaw = 0.0
MyHeading = 0
distance = 10
orders = " "
Ready = bool(False)
position1 = bool(False)
position2 = bool(False)
position3 = bool(False)
assigned = bool(False)
TargetX = float(0)
TargetY = float(0)
right = False


def update_orders(msg):
	global orders
	orders = msg.data
	print(orders)


def get_rotation_pos(msg):
	global roll, pitch, yaw
	global MyX
	global MyY
	global MyHeading
	orientation_q = msg.pose.pose.orientation
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
	MyHeading = yaw
	MyX = float(msg.pose.pose.position.x)
	MyY = float(msg.pose.pose.position.y)


def check_laser(msg):
	global object_ahead
	object_ahead = False
	object_ahead_distance = min(msg.ranges[0], msg.ranges[10], msg.ranges[20], msg.ranges[350], msg.ranges[340])
	if object_ahead_distance < 0.8:
		object_ahead = True


def check_robot1(msg):
	global robot_ahead
	global R1X
	global R1Y
	robot_ahead = False
	R1X = float(msg.pose.pose.position.x)
	R1Y = float(msg.pose.pose.position.y)
	distance = math.sqrt((R1X-MyX)**2 + (R1Y-MyY)**2)
	if distance < 1:  # only do any of this if robot2 is close
		Xdistance = R1X-MyX
		Ydistance = R1Y-MyY
		if (-0.25*np.pi) < MyHeading <= (0.75*np.pi):  # check northwest
			if Xdistance >= 0 and Ydistance >= 0:
				robot_ahead = True
		if (0.25*np.pi) < MyHeading <= (1.25*np.pi):  # check southwest
			if Xdistance <= 0 and Ydistance >= 0:
				robot_ahead = True
		if (-1.25*np.pi) < MyHeading <= (-0.25*np.pi):  # check southeast
			if Xdistance <= 0 and Ydistance <= 0:
				robot_ahead = True
		if (-0.75*np.pi) < MyHeading <= (0.25*np.pi):  # check northeast
			if Xdistance >= 0 and Ydistance <= 0:
				robot_ahead = True


def check_robot2(msg):
	global robot_ahead
	global R2X
	global R2Y
	R2X = float(msg.pose.pose.position.x)
	R2Y = float(msg.pose.pose.position.y)
	distance = math.sqrt((R2X-MyX)**2 + (R2Y-MyY)**2)
	if distance < 1:  # only do any of this if robot3 is close
		Xdistance = R2X-MyX
		Ydistance = R2Y-MyY
		if (-0.25*np.pi) < MyHeading <= (0.75*np.pi):  # check northwest
			if Xdistance >= 0 and Ydistance >= 0:
				robot_ahead = True
		if (0.25*np.pi) < MyHeading <= (1.25*np.pi):  # check southwest
			if Xdistance <= 0 and Ydistance >= 0:
				robot_ahead = True
		if (-1.25*np.pi) < MyHeading <= (-0.25*np.pi):  # check southeast
			if Xdistance <= 0 and Ydistance <= 0:
				robot_ahead = True
		if (-0.75*np.pi) < MyHeading <= (0.25*np.pi):  # check northeast
			if Xdistance >= 0 and Ydistance <= 0:
				robot_ahead = True


def check_assignment(msg):
	global assigned
	global position1
	global position2
	global position3
	global TargetX
	global TargetY
	assignment = msg.data
	print(assignment)
	if "R3spot1" in assignment:
		position1 = True
		assigned = True
		print("I am first")
		if orders == "2":
			TargetX = 0
			TargetY = 0
		if orders == "3":
			TargetX = 2
			TargetY = 2
		if orders == "5":
			TargetX = 2
			TargetY = 1.5
	if "R3spot2" in assignment:
		position2 = True
		assigned = True
		print("I am second")
		if orders == "2":
			TargetX = 1
			TargetY = 0
		if orders == "3":
			TargetX = 1
			TargetY = 1
		if orders == "5":
			TargetX = 1.25
			TargetY = 1.2
	if "R3spot3" in assignment:
		position3 = True	
		assigned = True	 
		print("I am third")
		if orders == "2":
			TargetX = 2
			TargetY = 0
		if orders == "3":
			TargetX = 2
			TargetY = 0
		if orders == "5":
			TargetX = 1.25
			TargetY = 1.7


def go(msg):
	global object_ahead
	global orders
	global Ready
	global assigned
	global position1
	global position2
	global position3
	global right
	if orders == "1":
		assigned = False
		position1 = False
		position2 = False
		position3 = False
		if object_ahead or robot_ahead:
			move = Twist()
			move.linear.x = 0.0
			move.angular.z = 1.0
			move.linear.y = 0.0
			pub = rospy.Publisher("/robot3/cmd_vel", Twist, queue_size=2)
			pub.publish(move)
		if not object_ahead and not robot_ahead:
			move = Twist()
			move.linear.x = 0.5
			move.angular.z = 0.0
			move.linear.y = 0.0
			pub = rospy.Publisher("/robot3/cmd_vel", Twist, queue_size=2)
			pub.publish(move)
	if orders == "0":
		position1 = False
		position2 = False
		position3 = False
		assigned = False
		move = Twist()
		move.linear.x = 0.0
		move.angular.z = 0.0
		move.linear.y = 0.0
		pub = rospy.Publisher("/robot3/cmd_vel", Twist, queue_size=2)
		pub.publish(move)
	if orders == "2":
		Ready = False
		if assigned:
			if position1:
				angle = atan2(MyY, MyX) - np.pi
			if position2:
				angle = atan2(MyY, (MyX-1)) - np.pi
			if position3:
				angle = atan2(MyY, (MyX-2)) - np.pi
			if angle < -3.1415:
				angle += (2*np.pi)
			if (angle - 0.3) < MyHeading < (angle + 0.3):
				Ready = True
				move = Twist()
				move.linear.x = 0.0
				move.angular.z = 0.0
				move.linear.y = 0.0
				pub = rospy.Publisher("/robot3/cmd_vel", Twist, queue_size=2)
				pub.publish(move)
			if not Ready:
				target_distance = math.sqrt(((MyY-TargetY)**2)+((MyX-TargetX)**2))
				if target_distance > 0.1:
					if 0 <= MyHeading < (np.pi/2):  # get the rover to turn right in some cases, depending on its quadrant
						if (np.pi/-2) <= angle <= 0:
							right = True
					if (np.pi/-2) <= MyHeading < 0:
						if (np.pi*-1) <= angle <= (np.pi/-2):
							right = True
					if (np.pi/-1) <= MyHeading < (np.pi/-2):
						if (np.pi/2) <= angle <= np.pi:
							right = True
					if (np.pi/2) <= MyHeading < np.pi:
						if 0 <= angle <= (np.pi/2):
							right = True
					move = Twist()
					move.linear.x = 0.0
					move.angular.z = 1.0
					move.linear.y = 0.0
					if right:
						move.angular.z = -1.0
					pub = rospy.Publisher("/robot3/cmd_vel", Twist, queue_size=2)
					pub.publish(move)
					print("must face target")
				if target_distance <= 0.1:
					move = Twist()
					move.linear.x = 0.0
					move.angular.z = 0.0
					move.linear.y = 0.0
					pub = rospy.Publisher("/robot3/cmd_vel", Twist, queue_size=2)
					pub.publish(move)
					print("reached target")
			if Ready:
				right = False
				target_distance = math.sqrt(((MyY-TargetY)**2)+((MyX-TargetX)**2))
				if target_distance > 0.1:			
					move = Twist()
					move.linear.x = 0.5
					if robot_ahead:
						move.linear.x = 0.1
					move.angular.z = 0.0
					move.linear.y = 0.0
					pub = rospy.Publisher("/robot3/cmd_vel", Twist, queue_size=2)
					pub.publish(move)
					print("target distance: {}".format(target_distance)) 
				if target_distance <= 0.1:			
					move = Twist()
					move.linear.x = 0.0
					move.angular.z = 0.0
					move.linear.y = 0.0
					pub = rospy.Publisher("/robot3/cmd_vel", Twist, queue_size=2)
					pub.publish(move)
					print("reached target")
	if orders == "3":  # flyingV
		Ready = False
		if assigned:
			if position1:
				angle = atan2((MyY-2), (MyX-2)) - np.pi
			if position2:
				angle = atan2((MyY-1), (MyX-1)) - np.pi
			if position3:
				angle = atan2(MyY, (MyX-2)) - np.pi
			if angle < -3.1415:
				angle += (2*np.pi)
			if (angle - 0.3) < MyHeading < (angle + 0.3):
				Ready = True
				move = Twist()
				move.linear.x = 0.0
				move.angular.z = 0.0
				move.linear.y = 0.0
				pub = rospy.Publisher("/robot3/cmd_vel", Twist, queue_size=2)
				pub.publish(move)
			if not Ready:
				target_distance = math.sqrt(((MyY-TargetY)**2)+((MyX-TargetX)**2))
				if target_distance > 0.1:
					if 0 <= MyHeading < (np.pi/2):  # get the rover to turn right in some cases, depending on its quadrant
						if (np.pi/-2) <= angle <= 0:
							right = True
					if (np.pi/-2) <= MyHeading < 0:
						if (np.pi*-1) <= angle <= (np.pi/-2):
							right = True
					if (np.pi/-1) <= MyHeading < (np.pi/-2):
						if (np.pi/2) <= angle <= np.pi:
							right = True
					if (np.pi/2) <= MyHeading < np.pi:
						if 0 <= angle <= (np.pi/2):
							right = True
					move = Twist()
					move.linear.x = 0.0
					move.angular.z = 1.0
					move.linear.y = 0.0
					if right:
						move.angular.z = -1.0
					pub = rospy.Publisher("/robot3/cmd_vel", Twist, queue_size=2)
					pub.publish(move)
					print("must face target")
				if target_distance <= 0.1:
					move = Twist()
					move.linear.x = 0.0
					move.angular.z = 0.0
					move.linear.y = 0.0
					pub = rospy.Publisher("/robot3/cmd_vel", Twist, queue_size=2)
					pub.publish(move)
					print("reached target")
			if Ready:
				right = False
				target_distance = math.sqrt(((MyY-TargetY)**2)+((MyX-TargetX)**2))
				if target_distance > 0.1:			
					move = Twist()
					move.linear.x = 0.5
					if robot_ahead:
						move.linear.x = 0.1
					move.angular.z = 0.0
					move.linear.y = 0.0
					pub = rospy.Publisher("/robot3/cmd_vel", Twist, queue_size=2)
					pub.publish(move)
					print("target distance: {}".format(target_distance)) 
				if target_distance <= 0.1:			
					move = Twist()
					move.linear.x = 0.0
					move.angular.z = 0.0
					move.linear.y = 0.0
					pub = rospy.Publisher("/robot3/cmd_vel", Twist, queue_size=2)
					pub.publish(move)
					print("reached target")
# placeholder for orders 3 and orders 4
	if orders == "5":
		Ready = False
		if assigned:
			if position1:
				angle = atan2(MyY-1.5, MyX-2) - np.pi
			if position2:
				angle = atan2(MyY-1.2, (MyX-1.25)) - np.pi
			if position3:
				angle = atan2(MyY-1.7, (MyX-1.25)) - np.pi
			if angle < -3.1415:
				angle += (2*np.pi)
			if (angle-0.3) < MyHeading < (angle + 0.3):
				Ready = True
				move = Twist()
				move.linear.x = 0.0
				move.angular.z = 0.0
				move.linear.y = 0.0
				pub = rospy.Publisher("/robot2/cmd_vel", Twist, queue_size=2)
				pub.publish(move)
			if not Ready:
				target_distance = math.sqrt(((MyY-TargetY)**2)+((MyX-TargetX)**2))
				if target_distance > 0.1:
					if 0 <= MyHeading < (np.pi/2):  # get the rover to turn right in some cases, depending on its quadrant
						if (np.pi/-2) <= angle <= 0:
							right = True
					if (np.pi/-2) <= MyHeading < 0:
						if (np.pi*-1) <= angle <= (np.pi/-2):
							right = True
					if (np.pi/-1) <= MyHeading < (np.pi/-2):
						if (np.pi/2) <= angle <= np.pi:
							right = True
					if (np.pi/2) <= MyHeading < np.pi:
						if 0 <= angle <= (np.pi/2):
							right = True
					move = Twist()
					move.linear.x = 0.0
					move.angular.z = 1.0
					move.linear.y = 0.0
					if right:
						move.angular.z = -1.0
					pub = rospy.Publisher("/robot3/cmd_vel", Twist, queue_size=2)
					pub.publish(move)
					print("must face target")
				if target_distance <= 0.1:
					move = Twist()
					move.linear.x = 0.0
					move.angular.z = 0.0
					move.linear.y = 0.0
					pub = rospy.Publisher("/robot3/cmd_vel", Twist, queue_size=2)
					pub.publish(move)
					print("reached target")
			if Ready:
				right = False
				target_distance = math.sqrt(((MyY-TargetY)**2)+((MyX-TargetX)**2))
				if target_distance > 0.1:
					move = Twist()
					move.linear.x = 0.5
					if robot_ahead:
						move.linear.x = 0.1
					move.angular.z = 0.0
					move.linear.y = 0.0
					pub = rospy.Publisher("/robot3/cmd_vel", Twist, queue_size=2)
					pub.publish(move)
					print("target distance: {}".format(target_distance)) 
				if target_distance <= 0.1:
					move = Twist()
					move.linear.x = 0.0
					move.angular.z = 0.0
					move.linear.y = 0.0
					pub = rospy.Publisher("/robot3/cmd_vel", Twist, queue_size=2)
					pub.publish(move)
					print("reached target")


def signal_handler(signal, frame):
	move = Twist()
	move.linear.x = 0.0
	move.angular.z = 0.0
	move.linear.y = 0.0
	pub = rospy.Publisher("/robot3/cmd_vel", Twist, queue_size=2)
	pub.publish(move)
	print("exiting program")
	sys.exit(0)


def listener():
	rospy.init_node('listener')
	rospy.Subscriber("/chatter", String, update_orders)
	rospy.Subscriber("/robot3/scan", LaserScan, check_laser)
	rospy.Subscriber("/robot3/odom", Odometry, get_rotation_pos)
	rospy.Subscriber("/robot1/odom", Odometry, check_robot1)
	rospy.Subscriber("/robot2/odom", Odometry, check_robot2)
	rospy.Subscriber("/robot3/odom", Odometry, get_rotation_pos)
	rospy.Subscriber("/robot1/chatter", String, check_assignment)
	rospy.Subscriber("/robot3/odom", Odometry, go)
	rospy.spin()


if __name__ == '__main__':
	listener()
