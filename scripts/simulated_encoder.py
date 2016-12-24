#!/usr/bin/env python
import rospy
import serial
import string
import sys


from datetime import datetime

from std_msgs.msg import String

def encoder(left, right):
	global ser

	pub = rospy.Publisher('encoder', String, queue_size = 10)
	rospy.init_node('encoder', anonymous=True)
	rate = rospy.Rate(20)

	rospy.loginfo("Started encoder")
	while not rospy.is_shutdown():
		bytesToPublish = '%d %d' % (left, right)
		pub.publish(str(bytesToPublish))
		rate.sleep()

if __name__ == '__main__':
	left_encode   	= 0 
	right_encode 	= 0
	try:
		if len(sys.argv) == 3:
  	      		left_encode = int(sys.argv[1])
          		right_encode = int(sys.argv[2])
		encoder(left_encode, right_encode)
	except rospy.ROSInterruptException:
		#ser.close()  #this doesn't seem to work well
		pass
		
		
