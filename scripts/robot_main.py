#!/usr/bin/env python
import rospy
import string
import time 
import robot_obstacle
import robot_job
import robot_correction 
import robot_publisher
import robot_listener 
import robot_drive 

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
		robot_listener.init_encoder_buffer()
		robot_listener.init_compass_buffer()
		main_listener()
	except rospy.ROSInterruptException:
		pass
