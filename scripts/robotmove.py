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

# Starts the robot for moving, put the control variables into proper value 
def start_move(dist_completed, robot_on_mission, move_direction, move_speed):
	rospy.loginfo('Robot moving job started')
	robot_on_mission = 1 
	dist_completed = 0
	robotdrive.send_command(move_direction, move_speed)

# Roboet complet a moving job 
def complete_move(dist_completed, robot_on_mission):	
	dist_travelled = 0
	robot_on_mission = 0 
	robotdrive.send_command('S',0)
	rospy.loginfo('Robot completed a moving job')

# Update robot speed as required new speed 
def update_move_speed(move_direction, speed_now, disired_speed):
	robotdrive.send_command(move_direction, desired_speed)
	speed_now = desired_speed 

# main function to control the robot movement 
def move_distance(dist_to_run, dist_completed, robot_on_missin, left_encode, right_encode, speed_now, desired_speed):
	global move_direction 
	# if robot received a meaning less job, just signal, clear the job and return 
	if (dist_to_run == 0):
		rospy.loginfo('Robot received a meaning less moving job')
		complete_move(dist_completed, robot_on_mission)
		return 

	if (dist_to_run < 0):
		move_direction = 'B'
	else 
		move_direction = 'F'


	# Mission started, let robot start moving 
	if (robot_on_mission == 0): 
		start_move(dist_completed, robot_on_mission, move_direction, move_speed)
		return

	# Exception handling, make sure robot wheels is moving the same direction 
	if (left_encode > 10 and right_encode < 10):
		rospy.loginfo('Robot wheel not moving as expected, step current job')
		complete_move(dist_completed, robot_on_mission)
		return

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
			update_move_speed(move_direction, speed_now, disired_speed)
		return
	else :
		complete_move(dist_completed, robot_on_mission)
       	 	#clean current job 
