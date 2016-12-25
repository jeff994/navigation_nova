#!/usr/bin/env python
import rospy
import serial
import string
from std_msgs.msg import String
import robot_drive
import math

#-------------------------------------------------------#
# Robot turning module 									#
#-------------------------------------------------------#

turn_direction 	='L'
degree_turned = 0
degree_to_turn = 0 

# just make sure the angle is between [0-360)
def correct_angle(angle):
	if(angle < -360 or angle >= 720):
		rospy.logerr('Waring Angle not supposed to appear')
		return 

	if(angle < 0):
		angle = angle + 360 
	elif(angle >= 360): 
		angle = angle - 360 
	return angle

# start a turn job 
def start_turn(bearing_now):
	global degree_turned
	global turn_direction
	global degree_to_turn

	robot_drive.bearing_now = bearing_now
	degree_to_turn = robot_drive.bearing_target - robot_drive.bearing_now 	
	if(degree_to_turn > 180): 
		degree_to_turn = degree_to_turn - 360
	elif(degree_to_turn < -180):
		degree_to_turn = degree_to_turn + 360
	
	if (degree_to_turn < 0): #Left turning 
 		turn_direction = 'L'
 	else:  #Right turning 
 		turn_direction = 'R'

	# put more detailed spped definitioan 
	if(abs(degree_to_turn) < 4):
		robot_drive.speed_now = 3
		robot_drive.desired_speed = 3 
	elif(abs(degree_to_turn) < 10):
		robot_drive.speed_now = 4
		robot_drive.desired_speed = 4 
	else:
		robot_drive.speed_now = 5
		robot_drive.desired_speed = 5 
	rospy.loginfo('Robot starts to execute a turn job')
	robot_drive.robot_on_mission = 1
	degree_turned = 0
	robot_drive.send_command(turn_direction, robot_drive.speed_now)
	rospy.loginfo("Start: Degree turned %d, degree to turn %d, bearing_now %d, bearing_target %d", degree_turned, degree_to_turn, robot_drive.bearing_now, robot_drive.bearing_target)

# tell the robot to complete it's turning job 
def stop_turn():
	global degree_turned
	global degree_to_turn 
	robot_drive.robot_on_mission = 0	
	degree_turned = 0
	degree_to_turn = 0
	robot_drive.send_command('S',0)
	rospy.loginfo('Robot completed a turn job')

# change the speed of turing 
def continue_turn(step_angle):
	global degree_turned
	global turn_direction
	global degree_to_turn
	
	
	if(abs(degree_to_turn) - abs(degree_turned) < 4):
		robot_drive.send_command(turn_direction, 3)
		robot_drive.speed_now = 3
		robot_drive.desired_speed = 3
		rospy.loginfo("Only 2 degrees left, redusing turning speed to 3")
	elif(abs(degree_to_turn) - abs(degree_turned) < 10):
		robot_drive.send_command(turn_direction, 4)
		robot_drive.speed_now = 4
		robot_drive.desired_speed = 4
		rospy.loginfo("Only 5 degrees left, redusing turning speed to 4")
	
	#dynamically update robot bearing 
	#robot_drive.bearing_now  = correct_angle(robot_drive.bearing_now)
	
	if(robot_drive.desired_speed == robot_drive.speed_now ): 
			rospy.loginfo('Continue turning at same speed...')
	else:
		robot_drive.send_command(turn_direction, robot_drive.desired_speed)
		distpub = 'Robot turning speed changed from %d to %d' % (robot_drive.speed_now, robot_drive.desired_speed)
		robot_drive.speed_now = robot_drive.desired_speed
		rospy.loginfo(distpub)

# let robot performs a turning job of certain degree 
def turn_degree(bearing_now, left_encode, right_encode): 
 	global turn_direction
 	global degree_turned 
 	global degree_to_turn 

	#robot has not started turning, just start the turning 
	if(robot_drive.robot_on_mission == 0):
		start_turn(bearing_now)
		return 0
	
	# convered from angle to required turn angles  
 	# calculate the obsolute anlge 

 	# The degree passed is not correct, just log and return 
	if(degree_to_turn == 0): 
		#No turn is required, clear current job and rerun 
		rospy.logwarn('Robot has been assigned a meaning less 0 degree turn task')
		stop_turn()
		return 1

	if(turn_direction == 'L' and left_encode <- 10 and right_encode > 10):
		rospy.logwarn('Robot wheel moving revered to the turn right command')
		#stop_turn()
		return 0

	if(turn_direction == 'R' and left_encode > 10 and right_encode < -10):
		rospy.logwarn('Robot wheel moving revered to the turn left command')
		#stop_turn()
		return 0

	if((left_encode > 10 and right_encode > 10) or (left_encode < -10 and right_encode < -10)): 
		rospy.logwarn('Robot wheel not moving as expected, clear current task')
		#stop_turn()
		return 0

	#Get the turned angle and then calculate 
	distance = (abs(left_encode) + abs(right_encode))/(2.0 * robot_drive.encode_to_mm)
	step_angle = (180 * distance) / (math.pi * robot_drive.turn_radius)   
	if(turn_direction == 'L'): 
		step_angle = - step_angle

	#robot_drive.bearing_now = robot_drive.bearing_now + step_angle
	rospy.loginfo("Degree turned %d, degree to turn %d, bearing_now %d, bearing_target %d", degree_turned, degree_to_turn, robot_drive.bearing_now, robot_drive.bearing_target)

	degree_turned = degree_turned + abs(step_angle)
	degree_threshold = abs(degree_to_turn)
	#simple log for tracing 
	distpub = 'Required angle:%f turned angle:%f step angle: %f' % (degree_to_turn, degree_turned, step_angle)
        rospy.loginfo(distpub)

	if(degree_turned < degree_threshold): 
		continue_turn(step_angle)
		return 0
	else: 
		#finishe the turning 
		stop_turn()
		return 1
	return 0 
