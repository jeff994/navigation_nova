#!/usr/bin/env python
import rospy
import serial
import string
from std_msgs.msg import String
import robotdrive

#-------------------------------------------------------#
# Robot turning module 									#
#-------------------------------------------------------#
turn_direction 	='L'

# start a turn job 
def start_turn(robot_on_mission, degree_turned, turn_direction, move_speed):
	rospy.loginfo('Robot starts to execute a turn job')
	robot_on_mission = 1
	degree_turned = 0
	robotdrive.send_command(turn_direction, move_speed)

# tell the robot to complete it's turning job 
def complete_turn(degree_turned, robot_on_mission):
	robot_on_mission = 0
	degree_turned = 0
	robotdrive.send_command('S',0)
	rospy.loginfo('Robot completed a turn job')

# change the speed of turing 
def update_turn_speed(turn_direction, move_speed, move_speed_now):
	robotdrive.send_command(turn_direction, move_speed)
	move_speed_now = move_speed

# let robot performs a turning job of certain degree 
def turn_degree(degree_to_turn, degree_turned, robot_on_mission, left_encode, right_encode, move_speed_now, move_speed): 
 	global turn_direction
 	if (degree < 0): #ccw turning 
 		turn_direction = 'L'
 	else:
 		turn_direction = 'R'

 	# The degree passed is not correct, just log and return 
	if(degree == 0): 
		#No turn is required, clear current job and rerun 
		rospy.loginfo('Robot has been assigned a meaning less 0 degree turn task')
		complete_turn(degree_turned, robot_on_mission)
		return

	#robot has not started turning, just start the turning 
	if(robot_on_mission == 0):
		start_turn(robot_on_mission, degree_turned, turn_direction, move_speed)
		return

	if((left_encode > 10 and right_encode > 10) or (left_encode < -10 and right_encode < -10)): 
		rospy.loginfo('Robot wheel not moving as expected, clear current task')
		complete_turn(degree_turned, robot_on_mission)
		return

	#Get the turned angle and then calculate 
	distance = (abs(left_encode) + abs(right_encode))/(2.0 * robotdrive.encode_to_mm)
	step_angle = (360 * distance) / (math.pi * 2 * robotdrive.turn_radius)   
	degree_turned = degree_turned + step_angle
		
	#simple log for tracing 
	distpub = 'Required angle:%f turned angle:%f' % (degree, degree_turned)
        rospy.loginfo(distpub)


	if(degree_turned < abs(degree)): 
		#continure turning and no need issue new command
		if(move_speed == move_speed_now): 
			rospy.loginfo('Continue turning at same speed...')
		else:
			rospy.loginfo('Change speed of turning...')
			update_turn_speed(turn_direction, move_speed, move_speed_now)
		return;
	else: 
		#finishe the turning 
		complete_turn(degree_turned, robot_on_mission)
		return
