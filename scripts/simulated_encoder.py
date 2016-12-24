#!/usr/bin/env python
import rospy
import serial
import string

from datetime import datetime

from std_msgs.msg import String

left_encode = 0
right_encode = 0


def encoder():
	global ser
	global left_encode
	global right_encode 

	pub = rospy.Publisher('encoder', String, queue_size = 10)
	rospy.init_node('encoder', anonymous=True)
	rate = rospy.Rate(20)

	rospy.loginfo("Started encoder")
	while not rospy.is_shutdown();
		bytesToPublish = '%d %d' % (left_encode, right_encode)
		pub.publish(str(bytesToPublish))
		rate.sleep()



if __name__ == '__main__':
	try:
		global left_encode
		global right_encode 
		if len(sys.argv) == 2:
  	      left_encode = int(sys.argv[1])
          right_encode = int(sys.argv[2])
		encoder()
	except rospy.ROSInterruptException:
		#ser.close()  #this doesn't seem to work well
		pass
		
		
