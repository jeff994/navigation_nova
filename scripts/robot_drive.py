#!/usr/bin/env python
import rospy
import serial
import string
import time
import robot_publisher 
import robot_correction
import math

from std_msgs.msg import String
from math import radians, cos, sin, asin, sqrt, atan2, degrees
#-------------------------------------------------------#
#	Robot drive module									#
#-------------------------------------------------------#

encode_to_mm 		= 27.8 		# 1000 encoding signals = 1 mm travelled
turn_radius 		= 307.0
lengthBetweenTwoWheels = 614.0
last_night_turn_radius 		= 370.0 			# radius when turning in mm (half distance between the middle point of two wheels) 
speed_full			= 6
speed_lower			= 5
speed_lowest		= 4
speed_desired 		= speed_full	# Global robot moving spped, 3 - 5
speed_now 			= speed_full	# Robot moving speed now
robot_on_mission 	= False			# set an indicator that robot's on a job right now 
robot_enabled 		= False 		# A switch to enable or disable robot from execuing any jobs
robot_paused 		= False 		# A flag to indicate enable robot to execute job  
move_direction 		= 'F'			# Global robot walking/turning direction
robot_initialized	= True 			# Confirm whehter the robot has initialized the true north 
robot_moving 		= False			# based on the encoder data to know whether the robot's moving
robot_turning		= False
initial_bearing 	= 0.0 			#set as north for now

speed_6 			= 6
speed_5 			= 5
speed_4 			= 4
speed_3 			= 3
speed_2 			= 2

############################################################

lon_now 			= 121.635139
lat_now  			= 31.2112262
bearing_now 		= 0.0

lon_target 			= 121.635139
lat_target 			= 31.2112262
bearing_target 		= 0.0 		#degrees

x 					= 0.0
y 					= 0.0
th 					= 0.0

vx 					= 0.0
vy 					= 0.0
vth 				= 0.0

roll 				= 0.0
pitch 				= 0.0
yaw 				= 0.0

past_step_time 			= 0.0

step_angle 			= 0.0
step_distance		= 0.0

direction 			= 0


#yuqing_obstaclemodeconfirm
#1: obstacle mode 
#0: no-obstalce mode
obstacle_mode = True
#yuqing_unlockconfirm
isunlockdone = False
# Jianbo: Burn mode and normal mode 
# True for burn mode when pwer up 
# False for normal mode 
burn_mode = True

def init_gps():
	global lon_now, lat_now, bearing_now
	global bearing_target, lon_target, lat_target

	lon_now 		= 121.635139
	lat_now  		= 31.2112262
	bearing_now 				= 0.0
	lon_target 		= 121.635139
	lat_target 		= 31.2112262
	bearing_target 				= 0.0              #degrees
	rospy.loginfo("Robot reset to original position")

# start the robot movement or turning 
def start():
	global speed_now
	global move_direction
	robot_publisher.publish_command(move_direction, speed_now)

# Get all the encoder not processed 
def accum_encoder_data(encoder_data, encoder_received, encoder_processed):
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

def get_step():
	global past_step_time
	global x
	global y	
	global bearing_now
	global bearing_target
	global vx 	#encode/s
	global vy 	#encode/s
	global vth 	#encode/mm*s
	global step_distance
	global encode_to_mm

	current_step_time 	= rospy.get_time()
	dt 			 	 	= current_step_time - past_step_time

	#th is relative to each job
	th 					= (bearing_target - bearing_now) 	
	if (th < -180.0):
		th 				= th + 360.0
	elif (th > 180.0):
		th 				= th - 360.0

	#calculate step deltas
	th_radian 			= th * 3.14159265 / 180.0
	delta_x 			= vx * cos(th_radian) * dt / encode_to_mm #mm
	delta_y 			= vx * sin(th_radian) * dt / encode_to_mm #mm
	delta_th 			= vth * dt / encode_to_mm #radians

	#coordinates relative to job
	x  		 			= x + delta_x
	y 					= y + delta_y

	#relative to step
	bearing_now 		= (bearing_now + (delta_th * 180.0 / 3.14159265) + 360.0)%360.0
	step_distance 		= sqrt((delta_x ** 2.0) + (delta_y ** 2.0))

	#setting past_time for next iteration
	past_step_time  	= current_step_time

	#update gps, this should be called for every step
	robot_correction.update_robot_gps_new(step_distance, bearing_now)
	time.sleep(0.02)


# just send a command to stop robot 
def stop_robot():
	speed_now = 0
	desired_speed = 0
	move_direction = 'P' #P
	# move_direction = 'S'
	# updated the stop command from 'S' to 'P'
	#while True:
	#	#step 1, send the stop command every 10 milli seoncs
	#	if(robot_moving or robot_turning):
	#		robot_publisher.publish_command(move_direction,speed_now)
	#		time.sleep(0.01)
	#	else: 
	#		break
	robot_publisher.publish_command(move_direction, speed_now)

def unlock_robot():
	# send a command to unlock robot after obstacle avoidece 
	robot_publisher.pub_command.publish('SB00000BE\n')
	#robot_publisher.pub_command.publish('obstacle unlock')
	rospy.loginfo('Unlock robot after avoidancee ')

# change speed
def change_speed():
	global move_direction, speed_now, speed_desired 
	robot_publisher.publish_command(move_direction, speed_desired)
	speed_now = speed_desired
	distpub = 'Robot speed changed from %d to %d' % (speed_now, speed_desired)
	rospy.loginfo(distpub)

#@yuqing_toggleobstaclemode
def enter_no_obstacle():
	robot_publisher.pub_command.publish('SW00000WE\n')
	#robot_publisher.pub_command.publish('no obstacle mode')
	rospy.loginfo('SW00000WE enter no obstacle mode')

#@yuqing_toggleobstaclemode
def enter_obstacle():
	robot_publisher.pub_command.publish('SO00000OE\n')
	#robot_publisher.pub_command.publish('obstacle mode')
	rospy.loginfo('SO00000OE enter obstacle mode')
	#yuqing_obstaclemodeconfirm
	#remove unlock

#normal and burn mode's commands are written in serial_handler_node
def enter_normal_mode():
	#for i in range (0,3):
	robot_publisher.pub_command.publish('normal')
	rospy.loginfo('Switch to normal mode')

def enter_burn_mode():
	#for i in range (0,5):
	robot_publisher.pub_command.publish('burn')
	rospy.loginfo('Switch to burn mode')
