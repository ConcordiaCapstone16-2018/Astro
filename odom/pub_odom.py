#! usr/bin/env python

# Capstone team 16 - Astro autonomous tool cart
# 
# Compute and Publish Odomotry for Astro
#
#
# 1. receive encoder ticks (time stamped?)
# 2. convert them to wheel velocities
# 3. integrate to get x-y coordinates
# 4. publish


# Coordinates of the thing: 
# Forward direction of the robot is x axis. 
# Initial orientation of 0 means global x axis is aligned with 
# robot's local x axis to begin with. 



import rospy
import math
import tf

from std_msgs.msg import Int16MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3

wheel_radius = 0.0619
wheel_separation = 0.4
ticks_rev = 497.0
x0 = 0
y0 = 0
th0 = 0
x_vel = 0
yaw_rate = 0


def pub_odom():
	print("\nBeginning to publish odometry data...\n")


	
	global pub,odom_tf,sub

	rospy.init_node('pub_odom')
	pub = rospy.Publisher('pub_odom', Odometry, queue_size=10)
	odom_tf = tf.TransformBroadcaster()
	sub = rospy.Subscriber('publish_ticks',Int16MultiArray,callback)

	
	rospy.spin()

		

def callback(ticks):
	
		print("Topic received. processing data")			

		# 1. Receive ticks
		#    dt will have to be read from consecutive stamped messages  
		global x0,y0,th0,wheel_radius,wheel_separation,yaw_rate,x_vel
		print(x0)
		
		dt = ticks.data[0]
		ticks_l = ticks.data[1]
		ticks_r = ticks.data[2]
		

			


	
		# 2. Calculate wheel vel (twist)
		w_r = 2*math.pi*ticks_r / (ticks_rev * dt)
		w_l = 2*math.pi*ticks_l / (ticks_rev * dt)
	
		v_r = w_r * wheel_radius
		v_l = w_l * wheel_radius

		# Turning left is positive yaw
		yaw_rate = (v_r - v_l) / wheel_separation

	
		# 3. Calculate x, y and orientation (th)
	
		try:
 
			turning_radius = (wheel_separation/2.0) * (v_l + v_r) / ( v_r - v_l)	
			ICCx = x0 - turning_radius * math.sin(th0)
			ICCy = y0 + turning_radius * math.cos(th0)
			x = math.cos(yaw_rate*dt) * (x0 - ICCx) - math.sin(yaw_rate*dt) * (y0 - ICCy) + ICCx
			y = math.sin(yaw_rate*dt) * (x0 - ICCx) + math.cos(yaw_rate*dt) * (y0 - ICCy) + ICCy
				
			th = (th0 + (yaw_rate*dt))

			print(x_vel)

		except ZeroDivisionError:
			
			yaw_rate = 0
			x_vel = w_r * wheel_radius 			
		 	x = x0 + x_vel*dt*math.cos(th0)
			y = y0 + x_vel*dt*math.sin(th0)
			th = th0
			print(x_vel)

		# pack up the Odometry object
	

		quat = tf.transformations.quaternion_from_euler(0,0,th)
	
		odom_tf.sendTransform(
			(x,y,0.0),
			quat,
			rospy.Time.now(), 
			"Astro/base_link",
			"odom"
		)

		odom = Odometry()
		odom.header.stamp = rospy.Time.now()
		odom.header.frame_id = "odom"
		odom.child_frame_id = "Astro/base_link"
		odom.pose.pose = Pose(Point(x,y,0),Quaternion(0,0,0,quat[3])) 
		odom.twist.twist = Twist(Vector3(x_vel,0,0),Vector3(0,0,yaw_rate))
		x0 = x
		y0 = y
		th0 = th

		
		pub.publish(odom)
		



if __name__ == '__main__':

	try:
		pub_odom()
	except rospy.ROSInterruptException:
		pass
