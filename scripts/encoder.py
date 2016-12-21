#!/usr/bin/env python
import rospy
import serial
import string
from std_msgs.msg import String

ser = serial.Serial()

# Testing port
#ser.port = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Uno_75439333335351412220-if00"
# Real robot encoder port
ser.port = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Uno_75533353637351616171-if00"
ser.baudrate = 9600
ser.open()

def encoder():
	pub = rospy.Publisher('encoder', String, queue_size = 10)
	rospy.init_node('encoder', anonymous=True)
	rate = rospy.Rate(20)
	
	rospy.loginfo("Started encoder")

	while ser.isOpen():
		bytesToRead = ser.readline()
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
		
		
