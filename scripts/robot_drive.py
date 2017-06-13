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

linear_encode_to_mm 		 	= 0.0
turning_encode_to_mm  			= 0.0
turn_radius 					= 0.0

lon_now 						= 0.0
lat_now  						= 0.0
bearing_now 					= 0.0

lon_target 						= 0.0
lat_target 						= 0.0
bearing_target 					= 0.0 		#degrees

speed_6 			= 6
speed_5 			= 5
speed_4 			= 4
speed_3 			= 3
speed_2 			= 2

roll 				= 0.0
pitch 				= 0.0
yaw 				= 0.0
past_yaw 			= 0.0

step_angle 			= 0.0
step_distance		= 0.0

direction 			= "stop"
past_direction 		= "stop"

#-----------------FLAGS------------------#

speed_desired 		= speed_6	# Global robot moving spped, 3 - 5
speed_now 			= speed_6	# Robot moving speed now
robot_on_mission 	= False			# set an indicator that robot's on a job right now
robot_enabled 		= False 		# A switch to enable or disable robot from execuing any jobs
robot_paused 		= False 		# A flag to indicate enable robot to execute job
move_direction 		= 'F'			# Global robot walking/turning direction
robot_initialized	= True 			# Confirm whehter the robot has initialized the true north
robot_moving 		= False			# based on the encoder data to know whether the robot's moving
robot_turning		= False

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

############################################################

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

# just send a command to stop robot
def stop_robot():
	speed_now = 0
	desired_speed = 0
	move_direction = 'B' #P
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
	distpub = 'Robot speed changed from %d to %d' % (speed_now, speed_desired)
	speed_now = speed_desired
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

def read_system_config():
	# Read configure path
	print("Read configuration file")
	config_path = os.path.dirname(os.path.abspath(__file__)) + '/robot.cfg'
	size_para 	= 23
	ret 		= [None] * size_para

	# Now reading configurable parameters
	# [tuning parameters] related
	ret[0], robot_drive.linear_encode_to_mm 			= robot_configure.read_config_float(config_path, 'tuning', 'linear_encode_to_mm')
	ret[1], robot_drive.turning_encode_to_mm			= robot_configure.read_config_float(config_path, 'tuning', 'turning_encode_to_mm')
	# [mechanical related]
	ret[2], robot_drive.turn_radius 					= robot_configure.read_config_float(config_path, 'mechanic', 'turn_radius')
	# [correction]
	ret[3], robot_correction.min_correction_distance 	= robot_configure.read_config_float(config_path, 'correction', 'min_correction_distance')
	ret[4], robot_correction.min_correction_angle 		= robot_configure.read_config_float(config_path, 'correction', 'min_correction_angle')
	ret[5], robot_correction.max_correction_runs 		= robot_configure.read_config_float(config_path, 'correction', 'max_correction_runs')
	# [move]
	ret[6], robot_move.dist_to_correct 					= robot_configure.read_config_float(config_path, 'move', 'dist_to_correct')
	ret[7], robot_move.dist_lower_speed 				= robot_configure.read_config_float(config_path, 'move', 'dist_lower_speed')
	ret[8], robot_move.dist_lowest_speed 				= robot_configure.read_config_float(config_path, 'move', 'dist_lowest_speed')
	ret[9], robot_move.linear_full_speed 				= robot_configure.read_config_float(config_path, 'move', 'linear_full_speed')
	ret[10], robot_move.linear_lower_speed 				= robot_configure.read_config_float(config_path, 'move', 'linear_lower_speed')
	ret[11], robot_move.linear_lowest_speed 			= robot_configure.read_config_float(config_path, 'move', 'linear_lowest_speed')
	# [turn]
	ret[12], robot_turn.angle_lower_speed 				= robot_configure.read_config_float(config_path, 'turn', 'angle_lower_speed')
	ret[13], robot_turn.angle_lowest_speed 				= robot_configure.read_config_float(config_path, 'turn', 'angle_lowest_speed')
	ret[14], robot_turn.turn_full_speed 				= robot_configure.read_config_float(config_path, 'turn', 'turn_full_speed')
	ret[15], robot_turn.turn_lower_speed 				= robot_configure.read_config_float(config_path, 'turn', 'turn_lower_speed')
	ret[16], robot_turn.turn_lowest_speed 				= robot_configure.read_config_float(config_path, 'turn', 'turn_lowest_speed')

	# [init]
	ret[17], robot_drive.obstacle_mode 					= robot_configure.read_config_float(config_path, 'init', 'obstacle_mode')
	ret[18], robot_drive.robot_enabled 					= robot_configure.read_config_float(config_path, 'init', 'robot_enabled')
	ret[19], robot_drive.robot_paused 					= robot_configure.read_config_float(config_path, 'init', 'robot_paused')
	ret[20], robot_job.init_lon 						= robot_configure.read_config_float(config_path, 'init', 'init_lon')
	ret[21], robot_job.init_lat 						= robot_configure.read_config_float(config_path, 'init', 'init_lat')
	ret[22], robot_job.init_bearing 					= robot_configure.read_config_float(config_path, 'init', 'init_bearing')

	robot_drive.lon_now  		= robot_job.init_lon
	robot_drive.lat_now  		= robot_job.init_lat
	robot_drive.lon_target  	= robot_job.init_lon
	robot_drive.lat_target  	= robot_job.init_lat
	robot_drive.bearing_now 	= robot_job.init_bearing
	robot_drive.bearing_target 	= robot_job.init_bearing

	# check whether the reading is successful or not
	for index in range(size_para):
		if not ret[index]:
			print("The ",index,"parameter is not correct")

	print("Finished read configure file")
	return
