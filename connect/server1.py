import socket
import rospy
from std_msgs.msg import String

HOST = '127.0.0.1'
PORT = 9880
ADDR = (HOST,PORT)
BUFSIZE = 4096

serv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

serv.bind(ADDR)
serv.listen(5)


rospy.init_node("publish_string")
pub = rospy.Publisher("publish_string", String, queue_size = 10)
data = String();
data = "1"
rate = rospy.Rate(10)


print ('listening ...')

while 1:

	pub.publish("SUP")
	rospy.spin()

while True:
  conn, addr = serv.accept()
  print ('client connected ... '), addr
  myfile = open('testfile.txt', 'w')
	
  while True:
    data = conn.recv(BUFSIZE)
    if not data: break
    myfile.write(data)
    print ('writing file ....')
    



  myfile.close()
  print ('finished writing file')
  conn.close()
print ('client disconnected')
