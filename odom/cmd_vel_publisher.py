#! /usr/bin/env python

import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3


print("\nThis script allows a user to input the desired speed of the Astro autonomous toolcart.")

print("\nThere are two modes - sraight motion and yaw motion. Enter the mode and then speed separated by ONE space. \n\nFor forward straight motion, enter: X 0.5\nFor backwrd straight motion, enter: X -0.1  \nFor left yaw, enter:                L 0.25\nFor right yaw, enter:               R 1.0\n\nDon't mess it up because there is no input error handling...lol change the numerical values to change the speed") 
  

def cmd_vel_publisher():

	rospy.init_node('cmd_vel_publisher')
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
	

	while not rospy.is_shutdown():

		inp = raw_input('\nMode  Speed\n')
		inp = inp.split(' ')

		if(inp[0]=='X' or inp[0]=='x'):
			vel = float(inp[1])
			yaw = 0
	
		elif(inp[0]=='L' or inp[0]=='l'):
			vel = 0
			yaw = -float(inp[1])

		elif(inp[0]=='R' or inp[0]=='r'):
			vel = 0
			yaw = float(inp[1])
		else:
			vel = 0
			yaw = 0

		data = Twist(Vector3(vel,0,0), Vector3(0,0,yaw))
		pub.publish(data)


if __name__ == '__main__':
	try:
		cmd_vel_publisher()
	except rospy.ROSInterruptException:
		pass
	
	
