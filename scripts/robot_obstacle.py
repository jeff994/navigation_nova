#!/usr/bin/env python
import rospy
import serial
import string
import math
import gpsmath
import robot_drive
import robot_job 

robot_on_obstacle 		= 0 # if robot is on obstacle avoidence, it would be set to 1
robot_over_obstacle 	= 0 # it's effective if the robot_on_obstacle 

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
	robot_on_obstace 	= 1
	robot_over_obstacle 	= 0

# if obstacle avoidence is over
def obstacle_is_over():
	rospy.loginfo('obstacle_is_over')
	global robot_on_obstacle
	global robot_over_obstacle
	robot_on_obstacle 	= 0 
	robot_over_obstacle 	= 1

def unlock_from_obstacle():
 	rospy.loginfo('unlock_from_obstacle')
	robot_drive.unlock_robot()
	global robot_over_obstacle
	robot_over_obstacle = 0

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
