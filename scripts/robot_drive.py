#!/usr/bin/env python
import rospy
import serial
import string
from std_msgs.msg import String

#-------------------------------------------------------#
#	Robot drive module									#
#-------------------------------------------------------#

encode_to_mm 		= 69.00 	# 1000 encoding signals = 1 mm travelled
turn_radius 		= 378 		# radius when turning in mm (half distance between the middle point of two wheels) 
desired_speed 		= 5			# Global robot moving spped, 3 - 5
speed_now 			= 5			# Robot moving speed now
robot_on_mission 	= 0			# set an indicator that robot's on a job right now 
robot_enabled 		= 0 		# A switch to enable or disable robot from execuing any jobs 
robot_on_obstancle 	= 0			# If robot is on obstancle avoidence, then set it to be 1
robot_over_obstacle = 0			# The robot finished obstance avodence 
move_direction 		= 'F'		# Global robot walking/turning direction
robot_initialized	= 0 		# Confirm whehter the robot has initialized the true north 
robot_moving 		= 0		# based on the encoder data to know whether the robot's moving

initial_bearing 	= 0 	#set as north for now
############################################################

lon_now 			= 121.635139
lat_now  			= 31.2112262
bearing_now 		= 0

lon_target 			= 121.635139
lat_target 			= 31.2112262
bearing_target 		= 0 		#degrees

step_angle 			= 0
step_distance		= 0

pub_command 		= rospy.Publisher('command', 	String, queue_size=10)
pub_gps				= rospy.Publisher('gps', 		String, queue_size=10)

def init_gps():
	lon_now 		= 121.6
	lat_now  		= 31.2
	bearing_now 	= 0
	lon_target 		= 121.6
	lat_target 		= 31.2
	bearing_target 	= 0              #degrees

# start the robot movement or turning 
def start():
	global speed_now
	global move_direction
	send_command(move_direction, speed_now)

# just send a command to stop robot 
def stop_robot():
	speed_now = 0
	desired_speed = 0
	# move_direction = 'S'
	# updated the stop command from 'S' to 'P'
	move_direction = 'P'
	send_command(move_direction,speed_now)

def unlock_robot():
	# send a command to unlock robot after obstacle avoidece 
	pub_command.publish('SB00000BE\n')
	rospy.loginfo('Unlock robot after avoidense ')

# change speed
def change_speed():
	global move_direction
	global speed_now 
	global desired_speed 
	send_command(move_direction, desired_speed)
	speed_now  = desired_speed
	distpub = 'Robot speed changed from %d to %d' % (speed_now, desired_speed)
	rospy.loginfo(distpub)

# Helper function which can send commands to robot 
def send_command(command_string, speed):
	#sending the string
	#handle the format of the string
	stringToSend = 'S%s00000%dE\n' % (command_string, speed) #might need to add \n behind the E
	pub_command.publish(stringToSend)
	rospy.loginfo(str(stringToSend))
	

#@yuqing_toggleobstaclemode
def enter_no_obstacle():
	pub_command.publish('SW00000WE\n')
	rospy.loginfo('SW00000WE enter no obstacle mode')

#@yuqing_toggleobstaclemode
def enter_obstacle():
	pub_command.publish('SO00000OE\n')
	rospy.loginfo('SO00000OE enter no obstacle mode')
