#!/usr/bin/env python
import rospy
import serial
import string
from std_msgs.msg import String

#-------------------------------------------------------#
#	Robot drive module									#
#-------------------------------------------------------#

encode_to_mm = 69.00 	#1000 encoding signals = 1 mm travelled
turn_radius = 395 		#radius when turning in mm (half distance between the middle point of two wheels) 
desired_speed = 3			# Global robot moving spped, 3 - 5
speed_now = 3		# Robot moving speed now
robot_on_mission = 0	#set an indicator that robot's on a job right now 

initial_bearing = 0 	#set as north for now
############################################################

compass_data = 0		#degrees, true north is 0 degrees
x_now = 0  				#mm
y_now = 0				#mm
r = 350 				#mm, distance between center of robot to wheel
x_target = 0			#mm
y_target = 0 			#mm, should always be 0, because we will be moving in a straight line
bearing_target = 0 		#degrees

#defining serial port to write to (the commands)
ser = serial.Serial()


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

# Helper function which can send commands to robot 
def send_command(command_string, speed):
	#sending the string
	if(open_serial() == 0):
		rospy.loginfo("Commanding Serial port not connected, failed to execute the command")
		return 

	#handle the format of the string
	stringToSend = 'S%s00000%dE\n' % (command_string, speed) #might need to add \n behind the E
	#rospy.loginfo(str(stringToSend))
	
	if ser.isOpen():
		ser.write(stringToSend)
	else:
		rospy.loginfo("Commanding Serial port not connected")
