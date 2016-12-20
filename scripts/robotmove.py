#!/usr/bin/env python
import rospy
import serial
import string
from std_msgs.msg import String
import robotdrive

#-------------------------------------------------------#
#	Robot moving module									#
#-------------------------------------------------------#

move_direction = 'F'
dist_completed = 0

# Starts the robot for moving, put the control variables into proper value 
def start_move(move_speed):
	global dist_completed
	global move_direction
	rospy.loginfo('Robot moving job started')
	robotdrive.robot_on_mission = 1 
	dist_completed = 0
	robotdrive.send_command(move_direction, move_speed)

# Roboet complet a moving job 
def stop_move():	
	global dist_completed
	dist_completed = 0
	robotdrive.robot_on_mission = 0 
	robotdrive.send_command('S',0)
	rospy.loginfo('Robot completed a moving job')

# Update robot speed as required new speed 
def change_move_speed(speed_now, disired_speed):
	global move_direction
	robotdrive.send_command(move_direction, desired_speed)
	speed_now = desired_speed 

# main function to control the robot movement 
def move_distance(dist_to_run, left_encode, right_encode, speed_now, desired_speed):
	global move_direction 
	global dist_completed
	# if robot received a meaning less job, just signal, clear the job and return 
	if (dist_to_run == 0):
		rospy.loginfo('Robot received a meaning less moving job')
		stop_move()
		return 1

	if (dist_to_run < 0):
		move_direction = 'B'
		dist_to_run = -dist_to_run
	else: 
		move_direction = 'F'

	# Mission started, let robot start moving 
	if (robotdrive.robot_on_mission == 0): 
		start_move(speed_now)
		return 0

	# Exception handling, make sure robot wheels is moving the same direction 
	if (left_encode > 10 and right_encode < 10):
		rospy.loginfo('Robot wheel not moving as expected, step current job')
		stop_move()
		return 1 

	# Accumulate the running distance and check 
	# Get each step of the distance 
	dist_step = (left_encode + right_encode)/(2.0 * robotdrive.encode_to_mm)
	# accumulate the distance to the completed distance 
	dist_completed = dist_completed + dist   #this is in mm
	#distance travelled threshold (put 2 mm thresh hold before stopping)
	dist_threshold = dist_to_run - 2 	#0 mm, I can choose -50mm, but since there will be inefficiencies, 0 error threshold might be good enough
	
	distpub = 'dist-travelled: %f dist-total:%f dist-step:%f' % (dist_completed,dist_threshold,dist)
	rospy.loginfo(distpub)

	if (dist_threshold - dist_completed > 0) :
		#just continue moving of job not completed and no change of speed command received 
		if(move_speed  == move_speed_now):
			rospy.loginfo('Still moving at the same speed...')
		else :
			change_move_speed(speed_now, disired_speed)
		return  0
	else :
		stop_move()
       	return 1; 
       	#clean current job 
	
	return 0; 
