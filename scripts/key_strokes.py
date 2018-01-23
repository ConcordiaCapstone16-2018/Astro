#!/usr/bin/env python

import rospy
import serial
import time
import keyboard as kb

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

publish_rate = 30
fwd_val = 10
bwd_val = 5
yaw_val = 7

def key_strokes():

	#Inititalize node called key_strokes.
	rospy.init_node('key_strokes')

	#Create a publisher object.
	pub = rospy.Publisher('keys', String ,queue_size = 10)
	pub2 = rospy.Publisher('yellow_robot/cmd_vel', Twist, queue_size = 10)
	#Control the rate at which the publisher publishes
	rate = rospy.Rate(publish_rate)

	# This is a standard rospy construct. Check if rospy is
	# cancelled (ctrl-c), if not, keep goin.
	
	while not rospy.is_shutdown():

		# Get the key being pressed.
		if(kb.is_pressed('up')):
			str = 'U'
			yaw = 0
			vel = -fwd_val
		elif(kb.is_pressed('down')):
			str = 'D'
			yaw = 0
			vel = fwd_val
		elif(kb.is_pressed('left')):
			str = 'L'
			yaw = -yaw_val
			if(kb.is_pressed('up')):
				vel=fwd_val
			else:
				vel = 0
		
		elif(kb.is_pressed('right')):
			str = 'R'
			yaw = yaw_val
			if(kb.is_pressed('up')):
				vel = fwd_val
			else:
				 vel = 0	
		else:
			str = '0'
			yaw = 0
			vel = 0

		# Pack it
		data = Twist(Vector3(vel,0,0),Vector3(0,0,yaw))

		# Publish it.
		pub.publish(str)
		pub2.publish(data)

		# Print it to logger
		rospy.loginfo(str)

		# Sleep for time required to maintain rate.
		rate.sleep()


if __name__ == '__main__':
	try:
		key_strokes()
	except rospy.ROSInterruptException:
		pass

