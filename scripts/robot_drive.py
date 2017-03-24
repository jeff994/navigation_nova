#!/usr/bin/env python
import rospy
import serial
import string
import time
import robot_publisher 

from std_msgs.msg import String
#-------------------------------------------------------#
#	Robot drive module									#
#-------------------------------------------------------#

encode_to_mm 		= 69.00 	# 1000 encoding signals = 1 mm travelled
turn_radius 		= 378 		# radius when turning in mm (half distance between the middle point of two wheels) 
desired_speed 		= 6			# Global robot moving spped, 3 - 5
speed_now 			= 6			# Robot moving speed now
robot_on_mission 	= 0			# set an indicator that robot's on a job right now 
robot_enabled 		= 0 		# A switch to enable or disable robot from execuing any jobs
robot_paused 		= 0 		# A flag to indicate enable robot to execute job  
robot_on_obstancle 	= 0			# If robot is on obstancle avoidence, then set it to be 1
robot_over_obstacle = 0			# The robot finished obstance avodence 
move_direction 		= 'F'		# Global robot walking/turning direction
robot_initialized	= 1 		# Confirm whehter the robot has initialized the true north 
robot_moving 		= 0		# based on the encoder data to know whether the robot's moving
robot_turning		= 0
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

#yuqing_obstaclemodeconfirm
#1: obstacle mode 
#0: no-obstalce mode
obstacle_mode = 1
#yuqing_unlockconfirm
isunlockdone = 0

def init_gps():
	global lon_now, lat_now, bearing_now
	global bearing_target, lon_target, lat_target

	lon_now 		= 121.635139
	lat_now  		= 31.2112262
	bearing_now 	= 0
	lon_target 		= 121.635139
	lat_target 		= 31.2112262
	bearing_target 	= 0              #degrees
	rospy.loginfo("Robot reset to original position")

# start the robot movement or turning 
def start():
	global speed_now
	global move_direction
	robot_publisher.publish_command(move_direction, speed_now)

# Simple conversion, get all the encoder not processed, then convert them to the distance 
def encoder_to_distance(encoder_data, encoder_received, encoder_processed):
	left_encode = 0
	right_encode = 0
	#in case data received is faster than the processing time 
	if(encoder_received > encoder_processed): 
		for x in range(encoder_processed, encoder_received):
			left_encode += encoder_data[2 * x]
			right_encode += encoder_data[2 * x +1]

	if(encoder_received < encoder_processed): 
		for x in range(encoder_processed, 1000):
			left_encode 	+= encoder_data[2 * x]
			right_encode 	+= encoder_data[2 * x + 1]
			for x in range(0, encoder_received):
				left_encode 	+= encoder_data[2 * x]
				right_encode 	+= encoder_data[2 * x + 1]
	return left_encode, right_encode 

# just send a command to stop robot 
def stop_robot():
	speed_now = 0
	desired_speed = 0
	# move_direction = 'S'
	# updated the stop command from 'S' to 'P'
	move_direction = 'P'
	robot_publisher.publish_command(move_direction,speed_now)

def unlock_robot():
	# send a command to unlock robot after obstacle avoidece 
	robot_publisher.pub_command.publish('SB00000BE\n')
	rospy.loginfo('Unlock robot after avoidense ')

# change speed
def change_speed():
	global move_direction, speed_now, desired_speed 
	robot_publisher.publish_command(move_direction, desired_speed)
	speed_now  = desired_speed
	distpub = 'Robot speed changed from %d to %d' % (speed_now, desired_speed)
	rospy.loginfo(distpub)

#@yuqing_toggleobstaclemode
def enter_no_obstacle():
	robot_publisher.pub_command.publish('SW00000WE\n')
	rospy.loginfo('SW00000WE enter no obstacle mode')

#@yuqing_toggleobstaclemode
def enter_obstacle():
	robot_publisher.pub_command.publish('SO00000OE\n')
	rospy.loginfo('SO00000OE enter no obstacle mode')
	#yuqing_obstaclemodeconfirm
	#remove unlock
	
