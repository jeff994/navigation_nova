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
	print ("1. Move 1 meter forward")
	print ("2. Move 1 meter back")
	print ("3. Turn left")
	print ("4. Turn right")
	print ("5. Speed increse")
	print ("6. Speed decrease")
	print ("8. Stop robot from moving")
	print ("9. Quit form keyboard control")
	print (30 * '-')
	#rospy.loginfo(str("test"))
	while True:
		is_valid = 0
		while not is_valid:
			#rospy.loginfo(str("Reading data from serial port"))
			try :
	            		choice = int ( raw_input('Enter your choice [1-9] : ') )
	            		is_valid = 1 ## set it to 1 to validate input and to terminate the while..not loop
	        	except ValueError, e :
	            		print ("'%s' is not a valid integer." % e.args[0].split(": ")[1])
		if choice == 1:
	        	pub.publish('Forward')
	    	elif choice == 2:
			pub.publish('Back')
		elif choice == 3:
			pub.publish('Turn_Left')
		elif choice == 4: 
			pub.publish('Turn_Right')
		elif choice == 5: 
			pub.publish('Faster')
		elif choice == 6:
			pub.publish('Slower')
		elif choice == 8:
			print ("Clear all the remaining tasks, stop the robot")
		    	pub.publish('Stop')
		elif choice == 9:
			break 
		else:
		    	print ("Invalid/Not defined number. Try again...")
	rate.sleep()

if __name__ == '__main__':
	try:
		KeyControl()
	except rospy.ROSInterruptException:
		#ser.close()  #this doesn't seem to work well
		pass
		
		
