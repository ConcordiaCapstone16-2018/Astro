#!/usr/bin/env python

import roslib
import rospy
import tf
import rospy


br = tf.TransformBroadcaster()


#rate = rospy.Rate(30)

if __name__ == '__main__':
	
	rospy.init_node('broadcast_tf')

	br.sendTransform((0,0,0),tf.transformations.quaternion_from_euler(0,0,0),rospy.Time.now(),'turtlename','world')	 
	rospy.spin();




	
