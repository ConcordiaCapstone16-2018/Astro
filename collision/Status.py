#!/usr/bin/env python
import rospy

from std_msgs.msg import String
from actionlib_msgs.msg import GoalID

measure = None
msg = GoalID()
Stop = rospy.Publisher("move_base/cancel", GoalID, queue_size = 10)


def callback(data):
    global measure
    measure = data.data
    if measure == '1':
      msg.id = ''
      Stop.publish(msg)
      print "Obstacle Detected: Front"
      rospy.sleep(0.01)
    
    if measure == '2': 
      msg.id = ''
      Stop.publish(msg)
      print "Obstacle Detected: Back"   
      rospy.sleep(0.01)
    
    if measure == '3':  
      msg.id = ''
      Stop.publish(msg)
      print "Obstacle Detected: Left"   
      rospy.sleep(0.01)
           	
            
    else:
      print "No Obstacles Detected"

    #rospy.spin()

def StatusDetection():
    
    rospy.init_node("StatusDetection", anonymous=True)
    rospy.Subscriber("Status", String, callback)
    
    
    rate = rospy.Rate(10)

    rospy.sleep(1)

    rospy.spin()


if __name__ == '__main__':
    try:
        StatusDetection()

    except rospy.ROSInterruptException:
        pass 
