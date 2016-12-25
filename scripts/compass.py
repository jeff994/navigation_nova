#!/usr/bin/env python

import rospy
import serial
import string
from std_msgs.msg import String

ser = serial.Serial()



# Real robot port
ser.port = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Uno_7553335363735161B1D1-if00"
# Testing port
#ser.port = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Uno_7553335363735161B1D1-if00"

ser.baudrate = 9600
ser.open()

def compass():
	pub = rospy.Publisher('compass', String, queue_size = 100)
	rospy.init_node('compass', anonymous=True)
	rate = rospy.Rate(100)
	#rospy.loginfo(str("test"))
	while ser.isOpen():
		#rospy.loginfo(str("Reading data from serial port"))
		bytesToRead = ser.readline()
		#rospy.loginfo(bytesToRead)
		bytesToRead = bytesToRead.strip('\n')
		if len(bytesToRead)  > 0: 
			#removes all alphabets turns into integer to remove 00
			all = string.maketrans('','')
			nodigs = all.translate(all, string.digits)
			bytesToRead = bytesToRead.translate(all, nodigs)
			bytesToPublish = int(bytesToRead)
			#publishing data in string for standardization
			#rospy.loginfo(str(bytesToRead))
			pub.publish(str(bytesToPublish))

		rate.sleep()

if __name__ == '__main__':
	try:
		compass()
	except rospy.ROSInterruptException:
		#ser.close()  #this doesn't seem to work well
		pass
		
		
