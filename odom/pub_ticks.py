#! /usr/bin/env python

# Capstone team 16 - Astro Autonomous Toolcart
# 
# This script repeatedly publishes encoder ticks for testing purposes. 
#
#

import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int16MultiArray


hz = int(raw_input("This script publishes desired left and right encoder counts at a desired rate. \n\nEnter desired publication rate (hz): "))

 
def ticks_publisher():

	rospy.init_node('ticks_publisher')

	pub = rospy.Publisher('publish_ticks', Int16MultiArray, queue_size=1)
	
	rate = rospy.Rate(hz)

	while not rospy.is_shutdown():	
		
		n = int(raw_input("\nEnter total number of times to publish: "))
		
		arr = Int16MultiArray()	

		data = raw_input("\nEnter the L and R ticks, respectively:\nL R\n")

		data = data.split(" ")
		data[0] = int(data[0])
		data[1] = int(data[1])
		data.insert(0,hz)
	
		t1 = rospy.Time.now() #in nanoseconds

		for i in range(0,n):

			t2 = rospy.Time.now()
			dt = t2-t1
			t1 = t2		
			print(dt/1000000.0)
				
			data[0] = int(1000.0/hz)
			arr.data = data
			pub.publish(arr)
			rate.sleep()


if __name__ == '__main__':
	try:
		ticks_publisher()
	except rospy.ROSInterruptException:
		pass
	
	
