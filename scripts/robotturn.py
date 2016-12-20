#!/usr/bin/env python
import rospy
import serial
import string
from std_msgs.msg import String
import robotdrive
import math

#-------------------------------------------------------#
# Robot turning module 									#
#-------------------------------------------------------#
turn_direction 	='L'
degree_turned = 0

# start a turn job 
def start_turn():
	global degree_turned
	global turn_direction
	rospy.loginfo('Robot starts to execute a turn job')
	robotdrive.robot_on_mission = 1
	degree_turned = 0
	robotdrive.send_command(turn_direction, robotdrive.speed_now)

# tell the robot to complete it's turning job 
def stop_turn():
	global degree_turned
	robotdrive.robot_on_mission = 0
	degree_turned = 0
	robotdrive.send_command('S',0)
	rospy.loginfo('Robot completed a turn job')

# change the speed of turing 
def continue_turn():
	global turn_direction
	if(robotdrive.desired_speed == robotdrive.speed_now ): 
			rospy.loginfo('Continue turning at same speed...')
	else:
		robotdrive.send_command(turn_direction, robotdrive.desired_speed)
		distpub = 'Robot turning speed changed from %d to %d' % (robotdrive.speed_now, robotdrive.desired_speed)
		robotdrive.speed_now = robotdrive.desired_speed
		rospy.loginfo(distpub)

# let robot performs a turning job of certain degree 
def turn_degree(degree_to_turn, left_encode, right_encode): 
 	global turn_direction
 	global degree_turned 

 	# The degree passed is not correct, just log and return 
	if(degree_to_turn == 0): 
		#No turn is required, clear current job and rerun 
		rospy.logwarn('Robot has been assigned a meaning less 0 degree turn task')
		stop_turn()
		return 1

 	if (degree_to_turn < 0): #ccw turning 
 		degree_to_turn = - degree_to_turn
 		turn_direction = 'L'
 	else:  #cw turning 
 		turn_direction = 'R'

	#robot has not started turning, just start the turning 
	if(robotdrive.robot_on_mission == 0):
		start_turn()
		return 0
	
	if(turn_direction == 'R' and left_encode <- 10 and right_encode > 10):
		rospy.logwarn('Robot wheel moving revered to the turn right command');
		stop_turn()
		return 1

	if(turn_direction == 'L' and left_encode > 10 and right_encode < -10):
		rospy.logwarn('Robot wheel moving revered to the turn left command');
		stop_turn()
		return 1

	if((left_encode > 10 and right_encode > 10) or (left_encode < -10 and right_encode < -10)): 
		rospy.logwarn('Robot wheel not moving as expected, clear current task')
		stop_turn()
		return 1

	#Get the turned angle and then calculate 
	distance = (abs(left_encode) + abs(right_encode))/(2.0 * robotdrive.encode_to_mm)
	step_angle = (360 * distance) / (math.pi * 2 * robotdrive.turn_radius)   
	degree_turned = degree_turned + step_angle
		
	#simple log for tracing 
	distpub = 'Required angle:%f turned angle:%f step angle: %f' % (degree_to_turn, degree_turned, step_angle)
        rospy.loginfo(distpub)

	if(degree_turned < degree_to_turn): 
		continue_turn()
		return 0
	else: 
		#finishe the turning 
		stop_turn()
		return 1
	return 0 
