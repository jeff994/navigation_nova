#!/usr/bin/env python

import rospy
import serial
import string
from std_msgs.msg import String


def KeyControl():
	choice = 0
	pub = rospy.Publisher('keyboard', String, queue_size = 100)
	rospy.init_node('keyboard', anonymous=True)
	rate = rospy.Rate(10)
	is_valid = 0
	print (30 * '-')
	print ("   M A I N - M E N U")
	print (30 * '-')
	print ("1. Re-Init job")
	print ("2. Stop robot from moving")
	print ("3. Get a new job")
	print ("4. Quit")
	print (30 * '-')
	#rospy.loginfo(str("test"))
	while True:
		is_valid = 0
		while not is_valid:
			#rospy.loginfo(str("Reading data from serial port"))
			try :
	            		choice = int ( raw_input('Enter your choice [1-4] : ') )
	            		is_valid = 1 ## set it to 1 to validate input and to terminate the while..not loop
	        	except ValueError, e :
	            		print ("'%s' is not a valid integer." % e.args[0].split(": ")[1])
		if choice == 1:
			print ("Init Job For Robot")
	        	pub.publish('Reset')
		elif choice == 2:
			print ("Clear all the remaining tasks, stop the robot")
		    	pub.publish('Stop')
		elif choice == 3:
			print ("Get a new job")
		elif choice == 4: 
			print ("Stop the control...")
			break; 
		else:
		    	print ("Invalid number. Try again...")
	rate.sleep()

if __name__ == '__main__':
	try:
		KeyControl()
	except rospy.ROSInterruptException:
		#ser.close()  #this doesn't seem to work well
		pass
		
		
