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
degree_turned = 0
degree_to_turn = 0 
status_pub = rospy.Publisher('status', String, queue_size = 100)

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
def start_turn():
	global degree_turned
	global degree_to_turn

	robot_drive.lon_target	= robot_drive.lon_now
	robot_drive.lat_target 	= robot_drive.lat_now
	
	degree_to_turn = robot_drive.bearing_target - robot_drive.bearing_now 	
	if(degree_to_turn > 180): 
		degree_to_turn = degree_to_turn - 360
	elif(degree_to_turn < -180):
		degree_to_turn = degree_to_turn + 360
	
	if (degree_to_turn < 0): #Left turning 
 		robot_drive.move_direction = 'L'
 	else:  #Right turning 
 		robot_drive.move_direction = 'R'

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
	if robot_drive.robot_moving == 1:
		robot_drive.robot_on_mission = 1
	degree_turned = 0
	status_pub.publish("enabled 1")
	robot_drive.start()
	rospy.loginfo("Start: Degree turned %d, degree to turn %d, bearing_now %d, bearing_target %d", degree_turned, degree_to_turn, robot_drive.bearing_now, robot_drive.bearing_target)

# tell the robot to complete it's turning job 
def stop_turn():
	global degree_turned
	global degree_to_turn 
	if robot_drive.robot_moving == 0:
		robot_drive.robot_on_mission = 0	
		degree_turned = 0
		degree_to_turn = 0
		rospy.loginfo('Robot completed a turn job')
		status_pub.publish("enabled 0")
	robot_drive.stop_robot()

# change the speed of turing 
def continue_turn(step_angle):
	global degree_turned
	global degree_to_turn
	
	if robot_drive.robot_moving == 0:
		rospy.loginfo("Robot stopped during the mission, start to turn again")
        robot_drive.start()

	if(abs(degree_to_turn) - abs(degree_turned) < 10):
		robot_drive.desired_speed = 3
		rospy.loginfo("Only 2 degrees left, redusing turning speed to 3")
	elif(abs(degree_to_turn) - abs(degree_turned) < 20):
		robot_drive.desired_speed = 4
		rospy.loginfo("Only 5 degrees left, redusing turning speed to 4")
	
	#dynamically update robot bearing 
	#robot_drive.bearing_now  = correct_angle(robot_drive.bearing_now)
	
	if(robot_drive.desired_speed == robot_drive.speed_now ): 
			rospy.loginfo('Continue turning at same speed...')
	else:
		robot_drive.change_speed()

# let robot performs a turning job of certain degree 
def turn_degree(): 
 	global degree_turned 
 	global degree_to_turn 

	#robot has not started turning, just start the turning 
	if(robot_drive.robot_on_mission == 0):
		start_turn()
		return 0
	
	# convered from angle to required turn angles  
 	# calculate the obsolute anlge 

 	# The degree passed is not correct, just log and return 
	if(degree_to_turn == 0): 
		#No turn is required, clear current job and rerun 
		rospy.logwarn('Robot has been assigned a meaning less 0 degree turn task')
		stop_turn()
		return 1

	#Get the turned angle and then calculate 
	step_angle = robot_drive.step_angle  
	#robot_drive.bearing_now = robot_drive.bearing_now + step_angle
	rospy.loginfo("Degree turned %f, degree to turn %f, bearing_now %f, bearing_target %f", degree_turned, abs(degree_to_turn), robot_drive.bearing_now, robot_drive.bearing_target)

	degree_turned = degree_turned + abs(step_angle)
	degree_threshold = abs(degree_to_turn) - 0.7
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
