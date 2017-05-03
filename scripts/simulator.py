#!/usr/bin/env python
# The class is to simulate the robot b

import rospy
import serial
import string
import sys


from datetime import datetime

from std_msgs.msg import String

encoder_pub = rospy.Publisher('encoder', String, queue_size = 10)

left_encode   	= 0 
right_encode 	= 0

def executor_simulator(data):
	global left_encode
	global right_encode
	command_str = str(data.data)
	rospy.loginfo("recieved command %s|", command_str)
        if  (command_str == 'SF000006E\n'):
            left_encode = 1700
            right_encode  = 1700
	elif (command_str == 'SL000006E\n'):
		left_encode = -2400
		right_encode = 2400
    	elif (command_str == 'SF000005E\n'):
    		left_encode = 1444
    		right_encode  = 1444
    	elif(command_str == 'SF000004E\n'):
    		left_encode = 1108
    		right_encode = 1108
    	elif(command_str == 'SF000003E\n'):
    		left_encode = 867
    		right_encode = 867
        elif(command_str == 'SB000006E\n'):
            left_encode = -2000
            right_encode  = -2000
    	elif(command_str == 'SB000005E\n'):
    		left_encode = -1444
    		right_encode  = -1444
    	elif(command_str == 'SB000004E\n'):
    		left_encode = -1108
    		right_encode = -1108
    	elif(command_str == 'SB000003E\n'):
    		left_encode = -867
    		right_encode = -867
        elif(command_str == 'SL000006E\n'):
            left_encode = -2000
            right_encode  = 2000
    	elif(command_str == 'SL000005E\n'):
    		left_encode = -1444
    		right_encode  = 1444
    	elif(command_str == 'SL000004E\n'):
    		left_encode = -1108
    		right_encode = 1108
    	elif(command_str == 'SL000003E\n'):
    		left_encode = -879
    		right_encode = 879
        elif(command_str == 'SR000006E\n'):
            left_encode = 2000
            right_encode  = -2017
    	elif(command_str == 'SR000005E\n'):
    		left_encode = 1108
    		right_encode = -1105
    	elif(command_str == 'SR000004E\n'):
    		left_encode = 1108
    		right_encode = -1105
    	elif(command_str == 'SR000003E\n'):
    		left_encode = 879
    		right_encode = -879
    	else:
    		left_encode = 0
    		right_encode = 0
    	rospy.loginfo("I heard %s: %d:%d",command_str, left_encode, right_encode)

def encoder_simulator():
	global encoder_pub	
	bytesToPublish = '%d %d' % (left_encode, right_encode)
	if(left_encode != 0  or right_encode != 0):
	 	rospy.loginfo(bytesToPublish)
	encoder_pub.publish(str(bytesToPublish))

def simulator():
	rospy.init_node('simulator', anonymous=True)
    	rospy.Subscriber("command", String, executor_simulator)
    	rate = rospy.Rate(10)
    	while not rospy.is_shutdown():
        	encoder_simulator()
        	rate.sleep()

    # spin() simply keeps python from exiting until this node is stopped
    	rospy.spin()

if __name__ == '__main__':
	try:
		simulator()
	except rospy.ROSInterruptException:
		#ser.close()  #this doesn't seem to work well
		pass
		
		
