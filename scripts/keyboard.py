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
	print ("0. Start|Stop robot")
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
	while not rospy.is_shutdown():
		choice = raw_input('Enter your choice [0-9] : ') 
		if(len(choice) > 1):
			print("You're supposed to enter only one key, try again...")
			continue 
		if choice == '0':
			pub.publish('Switch')
		elif choice == '1':
	        	pub.publish('Forward')
	    	elif choice == '2':
			pub.publish('Back')
		elif choice == '3':
			pub.publish('Turn_Left')
		elif choice == '4': 
			pub.publish('Turn_Right')
		elif choice == '5': 
			pub.publish('Faster')
		elif choice == '6':
			pub.publish('Slower')
		elif(choice == '7'):
			pub.publish('Demo')
		elif choice == '8':
		    	pub.publish('Stop')
		elif choice == '9':
			pub.publish('Test')
		else:
		    	print ("Invalid/Not defined number. Try again...")
	rate.sleep()

if __name__ == '__main__':
	try:
		KeyControl()
	except rospy.ROSInterruptException:
		#ser.close()  #this doesn't seem to work well
		pass
		
		
