#!/usr/bin/env python

import rospy
import serial
import time
import keyboard as kb

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3



def callback(ticks):
	if ticks.linear.x == 0 and ticks.angular.z == 0 and ticks.angular.x == 0:
		for i in range(0,5):
			pub.publish(Twist(Vector3(0,0,0), Vector3(1,0,0)))
							
pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
sub = rospy.Subscriber("cmd_vel", Twist, callback)

def repeater():
	rospy.init_node("repeater")
	

	

		
	rospy.spin()



if __name__ == '__main__':
	try:
		repeater()
	except rospy.ROSInterruptException:
		pass

