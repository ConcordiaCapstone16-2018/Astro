#!/usr/bin/env python
import rospy
from  std_msgs.msg import Float32
from actionlib_msgs.msg import GoalID

measure = None


def callback(data):
    global measure
    measure = data.data


def DistanceListener():
    
    rospy.init_node("DistanceListener", anonymous=True)
    rospy.Subscriber("Distance", Float32, callback)
    
    Stop = rospy.Publisher("move_base/cancel", GoalID, queue_size = 10)
    rate = rospy.Rate(10)
   
    
    #data = Float32()
    msg = GoalID()
    rospy.sleep(1)
    while not rospy.is_shutdown():
                
     
	  print  "Distance = "
          if measure < 150.0:
           msg.id = ''
           Stop.publish(msg)
           print "Msg Published"
	   rospy.sleep(0.01)
           	
            
          else:
           print measure
                 
           #rospy.sleep(2)
  
    rospy.spin()	
    #stop = rospy.Publisher("move_base/cancel", GoalID, queue_size = 10)

    



        	
     


if __name__ == '__main__':
    try:
        DistanceListener()

    except rospy.ROSInterruptException:
        pass 
	
