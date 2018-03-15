#! /usr/bin/env python

# Capstone team 16 - Astro Autonomous Toolcart
# 
# This script repeatedly publishes encoder ticks for testing purposes. 
#
#

import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from custom_msgs.msg import ticks


hz = int(raw_input("This script publishes desired left and right encoder counts at a desired rate. \n\nEnter desired publication rate (hz): "))

 
def ticks_publisher():

	rospy.init_node('ticks_publisher')

	pub = rospy.Publisher('ticks', ticks, queue_size=1)
	
	rate = rospy.Rate(hz)

	while not rospy.is_shutdown():	
		
		n = int(raw_input("\nEnter total number of times to publish: "))
		
		arr = ticks()	

		data = raw_input("\nEnter the L and R ticks, respectively:\nL R\n")

		data = data.split(" ")
		
		data[0] = float(data[0])
		data[1] = float(data[1])
		data.insert(0,hz)
	
		t1 = rospy.Time.now() #in nanoseconds
		
		arr.ticks_data[0].data = data[0]
		arr.ticks_data[1].data = data[1]
		arr.ticks_data[2].data = float(data[2])
		

		for i in range(0,n):
				
			
			pub.publish(arr)
			rate.sleep()


if __name__ == '__main__':
	try:
		ticks_publisher()
	except rospy.ROSInterruptException:
		pass
	
	
