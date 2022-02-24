#!/usr/bin/env python

import rospy
import sys, select, termios, tty, signal
import threading
from std_msgs.msg import String
key = "x"
msg = """
Command the swarm!
1- drive around randomly
2- form up in corner
3- form up in V
4- pivot the formation
5- cluster in center
6- spread out from center
0- stop
"""

def talker():
	global key
	pub = rospy.Publisher('chatter', String, queue_size=10)
	rospy.init_node('commander', anonymous=True)
	rate = rospy.Rate(10)
	pub.publish(key)

def getKey(key_timeout):
	global key
	tty.setraw(sys.stdin.fileno())
	rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
	if rlist:
		key = sys.stdin.read(1)
	else:
		key = ''
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

def signal_handler(signal, frame):
	print("exiting program")
	sys.exit(0)

if __name__ == '__main__':
	settings = termios.tcgetattr(sys.stdin)
	repeat = rospy.get_param("~repeat_rate", 0.0)
	key_timeout = rospy.get_param("~key_timeout", 0.0)
	if key_timeout == 0.0:
		key_timeout = None
	status=0
	try:
		print(msg)
		while(1):
			key = getKey(key_timeout)
			if key == "1":
				print("swarm will drive around randomly")
				talker()
			if key == "2":
				print("swarm will form up")
				talker()
			if key == "3":
				print("swarm will form V")
				talker()
			if key == "4":
				print("swarm will pivot")
				talker()
			if key == "5":
				print("swarm will cluster in center")
				talker()
			if key == "6":
				print("swarm will spread out")
				talker()
			if key == "0":
				print("swarm will stop")
				talker()
	except rospy.ROSInterruptException:
		pass
	finally:
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

