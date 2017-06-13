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
import robot_status
import execute_command    #only here to init command buffer
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from serial_handler.msg import Status   #getting the msg file from the serial_handler package

# The main progream process the robot logic
def main_commander():
	# ----------------------------------------------------------------------------------------#
	#  code to check and use the ecnoder data                     							  #
	# ----------------------------------------------------------------------------------------#
	# publish parameters to the web server
	#robot_publisher.publish_parameters()

	if robot_drive.burn_mode:
		if not robot_drive.robot_enabled:
			rospy.loginfo("Robot is on burn mode")
			time.sleep(0.1)
			return
		else:
			rospy.loginfo("Robot enabled, switch to normal mode")
			robot_drive.enter_normal_mode()
	else:
		robot_publisher.publish_parameters()

	# Very important error handling:
	# If Not any new data comming, waiting for next data,
	# if waiting too long need to issue warning or error
	if(robot_listener.encoder_received == robot_listener.encoder_processed): 	#aaron comment
		robot_listener.process_encoder_delay() 									#aaron comment
		return 																	#aaron comment

	# It process all the encoder data received - regardless robot status etc .
	#Including dynamically update robot GPS etc
	robot_listener.process_encoder_data() 										#aaron comment

	# ----------------------------------------------------------------------------------------#
	#  code to close robot when required	                    							  #
	# ----------------------------------------------------------------------------------------#
	# If robot not enabled, just need to disable the robot
	if not robot_drive.robot_enabled:
		rospy.loginfo("Robot is disabled")
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
		robot_obstacle.complete_obstacle_avoidance()
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
	#if not robot_drive.robot_initialized:
	#	robot_job.complete_init_compass(compass_data[compass_index])
	#	return

	# ----------------------------------------------------------------------------------------#
	#  Error compensation after current job completed      									  #
	# ----------------------------------------------------------------------------------------#
	if job_completed:
		no_correction_jobs = robot_job.no_correction_jobs()
		robot_job.complete_current_job()
		if no_correction_jobs == 0:
			rospy.loginfo("Complete a normal job, check whether correction is needed")
			robot_correction.dist_correction_normal()
		elif no_correction_jobs == 1: # the job is last correction job
			rospy.loginfo("Complete all correction jobs, check whether further correction is needed")
			robot_correction.dist_correction_correction();



#subscribes to different topic
def main_listener():
	rospy.init_node('commander')
	#rospy.Subscriber('compass', String, robot_listener.compass_callback) 			#aaron comment
	rospy.Subscriber('encoder', String, robot_listener.encoder_callback) 			#aaron comment
	rospy.Subscriber('keyboard', String, robot_listener.keyboard_callback)
	#rospy.Subscriber('rc_sensor_f', String, robot_listener.rc_sensor_f_callback)  #write the callbacks properly
	#rospy.Subscriber('rc_sensor_b', String, robot_listener.rc_sensor_b_callback)  #once we can get the correct data formats
	#ADD SUBSCRIBERS FOR GYRO DATA, MAKE CALLBACKS TO HANDLE THEM

	#@yuqing_obstacledriverread
	rospy.Subscriber('obstacle_status', String, robot_listener.obstacle_status_callback) #was previously driver_obstacle
	rospy.Subscriber('job', String, robot_listener.job_callback)
	rospy.Subscriber('control', String, robot_listener.control_callback)
	#rospy.Subscriber('velocity', Vector3, robot_listener.velocity_callback)
	rospy.Subscriber('IMU', Vector3, robot_listener.IMU_callback) #not yet done
	rospy.Subscriber('battery', String, robot_listener.battery_callback)
	rospy.Subscriber('direction', String, robot_listener.direction_callback)
	rospy.Subscriber('hardware_status', Status, robot_listener.status_callback)

	while not rospy.is_shutdown():
		main_commander()
	rospy.spin()

if __name__ == '__main__':
	try:
		# AAron's initial one for final testing
		#job_generator(initial_bearing, loops)
		robot_drive.read_system_config()
		robot_listener.init_encoder_buffer()
		robot_listener.init_compass_buffer()
		main_listener()
	except rospy.ROSInterruptException:
		pass
