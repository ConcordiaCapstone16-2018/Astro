#!/usr/bin/env python

import roslib
import rospy
import tf



def publish():
	br = tf.TransformBroadcaster()
	br.sendTransform((0.381,0,.531), 
					tf.transformations.quaternion_from_euler(0, 0, 0),
					rospy.Time.now(),
					'Astro/base_link', 
					"Astro/hokuyo_link")
	print("Transform from base to laser published.")



if __name__ == '__main__':
		
	rospy.init_node('tf_broadcaster')
	
	publish() 
	
	rospy.spin()
