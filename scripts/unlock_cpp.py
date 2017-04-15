#!/usr/bin/env python
import rospy
import string
from std_msgs.msg import String

def send_unlock_command():
	pub = rospy.Publisher('unlock_msg', String, queue_size=10)
	rospy.init_node('unlock_cpp', anonymous=True)

	stringToSend = 'iap_jump_app\r\n'
	if not rospy.is_shutdown():
		pub.publish(stringToSend)

if __name__ == '__main__':
	try:
		send_unlock_command()
	except rospy.ROSInterruptException:
		pass
