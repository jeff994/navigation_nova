#!/usr/bin/env python
import rospy
import string
import time 
import os
import robot_obstacle
import robot_job
import robot_correction 
import robot_publisher
import robot_listener 
import robot_drive 
import robot_move
import robot_turn 
import robot_configure

from std_msgs.msg import String


# The main progream process the robot logic 
def main_commander():
	# ----------------------------------------------------------------------------------------#
	#  code to check and use the ecnoder data                     							  #
	# ----------------------------------------------------------------------------------------#
	# publish parameters to the web server 
	robot_publisher.publish_parameters()

	# Very important error handling: 
	# If Not any new data comming, waiting for next data, 
	# if waiting too long need to issue warning or error	 
	if(robot_listener.encoder_received == robot_listener.encoder_processed):
		robot_listener.process_encoder_delay()
		return
	
	# It process all the encoder data received - regardless robot status etc .  
	# Including dynamically update robot GPS etc 
	robot_listener.process_encoder_data()

	# ----------------------------------------------------------------------------------------#
	#  code to close robot when required	                    							  #
	# ----------------------------------------------------------------------------------------#
	# If robot not enabled, just need to disable the robot 
	if not robot_drive.robot_enabled: 
		robot_job.disable_robot()
		return

	# ----------------------------------------------------------------------------------------#
	#  Codes for obstacle avoidence handling                     							  #
	# ----------------------------------------------------------------------------------------#
	# The flat would be set by hardware, we cannot do anything but blankly calculate the gps coordinates  
	if robot_obstacle.robot_on_obstacle:
		rospy.loginfo("Robot on obstacle avoidence, please wait") 
		return 

	# Robot obstancle avoidence is over, now resume to normal operation 
	if robot_obstacle.robot_over_obstacle:
		robot_obstacle.complete_obstacle_avoidence()
		return 

	# handle robot paused conidtions 
	if robot_drive.robot_paused:
		rospy.loginfo("Pause robot") 
		robot_job.pause_robot(); 
		return; 


	# ----------------------------------------------------------------------------------------#
	#  Codes for robot normal jobs like walking and turning       							  #
	# ----------------------------------------------------------------------------------------#
	# Check whether if there's any job left for the robot
	# If no jobs, make sure robot stopped moving, we cannot leave robot moving there 
	if not robot_job.has_jobs_left():
		robot_job.process_no_job()
		return

	job_completed = robot_job.process_job()
	
	# ----------------------------------------------------------------------------------------#
	#  Robot's doing the initialization jobs, not normal jobs      							  #
	# ----------------------------------------------------------------------------------------# 
	if not robot_drive.robot_initialized:
		robot_job.complete_init_compass(compass_data[compass_index])
		return

	# ----------------------------------------------------------------------------------------#
	#  Error compensation after current job completed      									  #
	# ----------------------------------------------------------------------------------------#
	if job_completed:
		rospy.loginfo("Complete a job")
		robot_job.complete_current_job()
		#robot_correction.angle_correction()
		robot_correction.dist_correction_normal()

def read_system_config():
	# Read configure path 
	config_path = os.path.dirname(os.path.abspath(__file__)) + '/robot.cfg'
	size_para 	= 17
	ret 		= [None] * size_para
	
	# Now reading configurable parameters 
	# [speed] related
	ret[0], robot_drive.speed_full 						= robot_configure.read_config_int(config_path, 'speed', 'speed_full')
	ret[1], robot_drive.speed_lower 					= robot_configure.read_config_int(config_path, 'speed', 'speed_lower')
	ret[2], robot_drive.speed_lowest 					= robot_configure.read_config_int(config_path, 'speed', 'speed_lowest')
	# [architectural]
	ret[3], robot_drive.encoder_to_mm 					= robot_configure.read_config_int(config_path, 'mechanic', 'encode_to_mm')
	ret[4], robot_drive.turn_radius 					= robot_configure.read_config_int(config_path, 'mechanic', 'turn_radius')
	# [correction]
	ret[5], robot_correction.min_correction_distance 	= robot_configure.read_config_int(config_path, 'correction', 'min_correction_angle')
	ret[6], robot_correction.min_correction_angle 		= robot_configure.read_config_int(config_path, 'correction', 'min_correction_distance')
	ret[7], robot_correction.max_correction_runs 		= robot_configure.read_config_int(config_path, 'correction', 'max_correction_runs')
	# [walk]
	ret[8], robot_move.dist_lower_speed 				= robot_configure.read_config_int(config_path, 'move', 'dist_lower_speed')
	ret[9], robot_move.dist_lowest_speed 				= robot_configure.read_config_int(config_path, 'move', 'dist_lowest_speed')
	ret[10], robot_move.dist_to_correct 				= robot_configure.read_config_int(config_path, 'move', 'dist_to_correct')
	ret[11], robot_move.dist_end_point_check 			= robot_configure.read_config_int(config_path, 'move', 'dist_end_point_check')
	ret[12], robot_move.angle_lower_speed 				= robot_configure.read_config_int(config_path, 'move', 'angle_lower_speed')
	ret[13], robot_move.angle_lowest_speed 				= robot_configure.read_config_int(config_path, 'move', 'angle_lowest_speed')
	# [init]c
	ret[14], robot_drive.obstacle_mode 					= robot_configure.read_config_int(config_path, 'init', 'obstacle_mode')
	ret[15], robot_drive.robot_enabled 					= robot_configure.read_config_int(config_path, 'init', 'robot_enabled')
	ret[16], robot_drive.robot_paused 					= robot_configure.read_config_int(config_path, 'init', 'robot_paused')
	
	# check whether the reading is successful or not 
	for index in range(size_para):
		if not ret[index]: 
			rospy.loginfo("The no %d configure parameter reading is wrong", index)
	return


#subscribes to different topic 
def main_listener():
	rospy.init_node('commander')
	rospy.Subscriber('compass', String, robot_listener.compass_callback)
	rospy.Subscriber('encoder', String, robot_listener.encoder_callback)
	rospy.Subscriber('keyboard', String, robot_listener.keyboard_callback)
	rospy.Subscriber('rc_sensor_f', String, robot_listener.rc_sensor_f_callback)
	rospy.Subscriber('rc_sensor_b', String, robot_listener.rc_sensor_b_callback)
	#@yuqing_obstacledriverread
	rospy.Subscriber('driver_obstacle', String, robot_listener.driver_obstacle_callback)
	rospy.Subscriber('job', String, robot_listener.job_callback)
	rospy.Subscriber('control', String, robot_listener.control_callback)
	while not rospy.is_shutdown():
		main_commander()
	rospy.spin()

if __name__ == '__main__':
	try:
		# AAron's initial one for final testing
		#job_generator(initial_bearing, loops)
		read_system_config()
		robot_listener.init_encoder_buffer()
		robot_listener.init_compass_buffer()
		main_listener()
	except rospy.ROSInterruptException:
		pass
