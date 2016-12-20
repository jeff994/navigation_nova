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
def start_move():
	global dist_completed
	global move_direction
	rospy.loginfo('Robot moving job started')
	robotdrive.robot_on_mission = 1 
	dist_completed = 0
	robotdrive.send_command(move_direction, robotdrive.speed_now)

# Roboet complet a moving job 
def stop_move():	
	global dist_completed
	dist_completed = 0
	robotdrive.robot_on_mission = 0 
	robotdrive.send_command('S',0)
	rospy.loginfo('Robot completed a moving job')

# Update robot speed as required new speed 
def continue_move():
	global move_direction
	if(robotdrive.speed_now  == robotdrive.desired_speed):
		rospy.loginfo('Still moving at the same speed...')
	else:
		robotdrive.send_command(move_direction, desired_speed)
		robotdrive.speed_now  = robotdrive.desired_speed
		distpub = 'Robot move speed changed from %d to %d' % (robotdrive.speed_now, robotdrive.desired_speed)
		rospy.loginfo(distpub)
# main function to control the robot movement 
def move_distance(dist_to_run, left_encode, right_encode):
	global move_direction 
	global dist_completed
	# if robot received a meaning less job, just signal, clear the job and return 
	if (dist_to_run == 0):
		rospy.logwarn('Robot received a meaningless moving job')
		stop_move()
		return 1

	if (dist_to_run < 0):
		move_direction = 'B'
		dist_to_run = -dist_to_run
	else: 
		move_direction = 'F'

	# Mission started, let robot start moving 
	if (robotdrive.robot_on_mission == 0): 
		start_move()
		return 0

	if(move_direction == 'F' and left_encode < -10 and right_encode < -10):
		rospy.logwarn('Robot supposed to move forward, actually moveing backward, stop the job')
		stop_move()
		return 1 

	if(move_direction == 'B' and left_encode > 10 and right_encode > 10):
		rospy.logwarn('Robot supposed to move backward, acutally moveing forward, stop the job')
		stop_move()
		return 1 

	# Exception handling, make sure robot wheels is moving the same direction 
	if (left_encode > 10 and right_encode < 10):
		rospy.logwarn('Robot wheel not moving as expected, stop current job')
		stop_move()
		return 1 

	# Accumulate the running distance and check 
	# Get each step of the distance 
	dist_step = (left_encode + right_encode)/(2.0 * robotdrive.encode_to_mm)
	# accumulate the distance to the completed distance 
	dist_completed = dist_completed + dist_step   #this is in mm
	#distance travelled threshold (put 2 mm thresh hold before stopping)
	dist_threshold = dist_to_run - 2 	#0 mm, I can choose -50mm, but since there will be inefficiencies, 0 error threshold might be good enough
	
	distpub = 'dist-travelled: %f dist-total:%f dist-step:%f' % (dist_completed,dist_threshold,dist_step)
	rospy.loginfo(distpub)

	if (dist_threshold - dist_completed > 0) :
		#just continue moving of job not completed and no change of speed command received 
		#if speed changed, then just change the move speed 
		continue_move(); 
		return  0
	else :
		stop_move()
       	return 1; 
       	#clean current job 
	
	return 0; 
