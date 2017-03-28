#!/usr/bin/env python
import rospy
import serial
import string
import math
import gpsmath
import robot_drive
import robot_job 
import robot_correction 

robot_on_obstacle 		= 0 # if robot is on obstacle avoidence, it would be set to 1
robot_over_obstacle 	= 0 # it's effective if the robot_on_obstacle 

needForward = False
justStop = False

from math import radians, cos, sin, asin, sqrt, atan2, degrees

#-------------------------------------------------------#
#	Robot error correction module						#
#-------------------------------------------------------#
# while robot's moving, dynamically update robot gps
# still need to handle more scenarios 

def is_on_obstacle_avoidence(first, second, third, forth):
	if(first == 0 or first == 1 or first == 2 or first == 3):
		return 1
	if(second ==0 or second == 2 or second ==3 or second == 1):
		return 2 
	if(third ==0 or third == 2 or third ==3 or third == 1):
		return 3 
	if(forth ==0 or forth == 2 or forth ==3 or forth == 1):
		return 4 
	return 0

def start_obstacle_avidence():
	rospy.loginfo('start_obstacle_avidence')
	global robot_on_obstacle
	global robot_over_obstacle
	robot_on_obstacle 		= True
	robot_over_obstacle 	= False
	#yuqing_unlockconfirm
	robot_drive.isunlockdone = False

# if obstacle avoidence is over
def obstacle_is_over():
	rospy.loginfo('obstacle_is_over')
	global robot_on_obstacle
	global robot_over_obstacle
	robot_on_obstacle 	= False
	robot_over_obstacle = True

def unlock_from_obstacle():
 	rospy.loginfo('unlock_from_obstacle')
	robot_drive.unlock_robot()
	global robot_over_obstacle
	robot_over_obstacle = False

	# Get the last four digit of the reverse car sendor data 
def rc_sensor_data(rc_sensor_value):
	first_digit = rc_sensor_value % 16
	rc_sensor_value = rc_sensor_value / 16 
	second_digit = rc_sensor_value % 16
	rc_sensor_value = rc_sensor_value / 16 
	third_digit = rc_sensor_value % 16
	rc_sensor_value = rc_sensor_value / 16 
	forth_digit = rc_sensor_value % 16
	return first_digit, second_digit, third_digit, forth_digit

# Complete the obstacle avoidence after we get a signal from the robot base  
def complete_obstacle_avoidence(): 
	# step 1: Waiting for robot stop moving 
	if (robot_drive.robot_turning or robot_drive.robot_moving):
		rospy.loginfo("waiting robot to stop")
		return

	# step 2: Once robot stopped moving, send unlock signal to the robot 
	if (not robot_drive.isunlockdone):
		rospy.loginfo("unlock robot for further processing")
		robot_drive.unlock_robot()
		return

	# step 3: Once robot is unlocked, resume control to the program 
	# Need to perform necessary correction 
	rospy.loginfo("Resume the robot from obstacle avoidence") 
	# First get ready the robot for normal walking 
	#yuqing_unlockconfirm 
	#robot_obstacle.unlock_from_obstacle()
	# Remove the un-finished job 
	job_executing = robot_job.current_job()
	if robot_drive.robot_on_mission:
		current_job_type = job_executing.classfication; 
		if(current_job_type == 'N'):
			rospy.loginfo("oRbot met obstacle during normal job, pefrorm correction")
			robot_correction.correction_count 	= 0
			robot_job.complete_current_job()
			# Re-calculate and send the corretion job 
			#robot_correction.distance_correction(dist_forward_after_obstacle)
			#@yuqing_forwardafterobstacle
			#forward distance by angle from sensor
			#rospy.loginfo("forward 0.5m after obstacle") 
		elif(current_job_type == 'C'): 
			if(robot_correction.correction_count  > robot_correction.max_correction_run):
				rospy.loginfo("Robot has tried to move to %f, %f for %d times, failed")
				while (current_job_type == 'C'):
					robot_job.complete_current_job()
					if len(robot_job.job_type) > 0:
						job_executing 		= robot_job.current_job()
						current_job_type 	= job_executing.classfication; 
					else:
						rospy.loginfo("The last job in the queue")
						break; 

				rospy.loginfo("Cleared all the correction jobs")
				if len(robot_job.job_type) > 0:
					job_executing 		= robot_job.current_job()
					current_job_type 	= job_executing.classfication; 
					rospy.loginfo("Add correction for next %s job", current_job_type)
					robot_job.complete_current_job()
			else:
				rospy.loginfo("Robot meet a obstacle while peforming correction job")
				robot_correction.correction_count = robot_correction.correction_count + 1
				rospy.loginfo("Robot failed correction job for %d time", robot_correction.correction_count )
				#discard all correction jobs 
				while (current_job_type == 'C'):
					robot_job.complete_current_job()
					if len(robot_job.job_type) > 0:
						job_executing 		= robot_job.current_job()
						current_job_type 	= job_executing.classfication; 
					else:
						rospy.loginfo("The last job in the queue")
						break; 
		else:
			rospy.logerr("Invalid job_type found")

		if robot_obstacle.needForward:
			robot_correction.distance_correction_obstacle(robot_job.dist_forward_after_obstacle)		
		else:
			robot_correction.distance_correction()
			rospy.loginfo("no need to forward 0.5m after obstacle")

		robot_drive.isunlockdone = False
		robot_obstacle.robot_over_obstacle = False

