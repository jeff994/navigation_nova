#!/usr/bin/env python
import rospy
import serial
import string
from std_msgs.msg import String

#-------------------------------------------------------#
#	Robot drive module									#
#-------------------------------------------------------#

encode_to_mm = 69.00 	# 1000 encoding signals = 1 mm travelled
turn_radius = 378 		# radius when turning in mm (half distance between the middle point of two wheels) 
desired_speed = 5		# Global robot moving spped, 3 - 5
speed_now = 5			# Robot moving speed now
robot_on_mission = 0	# set an indicator that robot's on a job right now 
robot_enabled = 0 		# A switch to enable or disable robot from execuing any jobs 


initial_bearing = 0 	#set as north for now
############################################################

lon_now = 31.205718
lat_now  = 121.628857
bearing_now =  0

lon_target = 0.0
lat_target = 0.0
bearing_target = 0 		#degrees

pub_command = rospy.Publisher('command', String, queue_size=10)
pub_gps	= rospy.Publisher('gps', String, queue_size=10)

def init_gps():
	lon_now = 31.205718
	lat_now  = 121.628857
	bearing_now =  0
	lon_target = 0.0
	lat_target = 0.0
	bearing_target = 0              #degrees


# Helper function which can send commands to robot 
def send_command(command_string, speed):
	global ser;
	#sending the string
	#handle the format of the string
	stringToSend = 'S%s00000%dE\n' % (command_string, speed) #might need to add \n behind the E
	pub_command.publish(stringToSend)
	rospy.loginfo(str(stringToSend))
	
