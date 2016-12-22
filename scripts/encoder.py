#!/usr/bin/env python
import rospy
import serial
import string
from datetime
#add a timer to measure the serial port read line waiting time
start = 0.0
end = 0.0

from std_msgs.msg import String

ser = serial.Serial()

# Testing port
#ser.port = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Uno_75439333335351412220-if00"
# Real robot encoder port
ser.port = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Uno_75533353637351616171-if00"
ser.baudrate = 9600
ser.open()

def open_serial():
	global ser
	if ser.isOpen():
		return 1 
	#real robot port
	ser.port = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Uno_75435363138351A09171-if00"
	#testing port
	#ser.port = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Uno_75439333335351412220-if00"
	ser.baudrate = 9600
	ser.open()
	if ser.isOpen():
		return 1
	return 0 

def encoder():
	pub = rospy.Publisher('encoder', String, queue_size = 10)
	rospy.init_node('encoder', anonymous=True)
	rate = rospy.Rate(20)
	
	rospy.loginfo("Started encoder")

	while open_serial():
		start = datetime.now()
		bytesToRead = ser.readline()
		end = datetime.now()
		delta = end-start
		if(delta.seconds > 0.3):
			print str(delta)
			rospy.logwarn("Time gap between data received is more than 0.3 sec")
		
		#rospy.loginfo(str(bytesToRead))
		bytesToRead = bytesToRead.strip('\n')
		if len(bytesToRead)  == 17: 
			#separates the data into readable things
                      	#rospy.loginfo(str(bytesToRead))
			r_encoder, r_direction, l_encoder, l_direction = bytesToRead.split(" ")
			nr_encoder = int(r_encoder)
			nl_encoder = int (l_encoder)
			if r_direction == "1" :
				nr_encoder = -int(nr_encoder)
			else:
				nr_encoder = int(nr_encoder)
			if l_direction == "1": 
				nl_encoder = -int(nl_encoder)
			else :
				nl_encoder = int(nl_encoder)

		#turning them into strings
                                #real data 
			bytesToPublish = '%d %d' % (nl_encoder, nr_encoder)
				#simulated testing data 
                                #bytesToPublish = '-1234 1234'
                                #publishing data in string for standardization
                                #rospy.loginfo(str(bytesToPublish))
			if(nl_encoder != 0  or nr_encoder != 0):
				rospy.loginfo("------------------")
				rospy.loginfo("------------------")
				rospy.loginfo(bytesToPublish)
			pub.publish(str(bytesToPublish))
		else:
			rospy.logwarn("Found data which is not in a required format")
			rospy.logwarn(bytesToRead)
		ser.flushInput()
		ser.flushOutput()
		ser.flush()
		rate.sleep()

if __name__ == '__main__':
	try:
		encoder()
	except rospy.ROSInterruptException:
		#ser.close()  #this doesn't seem to work well
		pass
		
		
