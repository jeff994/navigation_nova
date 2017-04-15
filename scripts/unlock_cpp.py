#!/usr/bin/env python
import rospy
import string
from std_msgs.msg import String

def send_unlock_command():
	pub = rospy.Publisher('unlock_msg', String, queue_size=10)
	rospy.init_node('unlock_cpp', anonymous=True)
	rate = rospy.Rate(1)

	stringToSend = 'unlock_please\0'

	for i in range (0,5):
		if not rospy.is_shutdown():
                	pub.publish(stringToSend)
		rate.sleep()
	
if __name__ == '__main__':
	try:
		send_unlock_command()
	except rospy.ROSInterruptException:
		pass
