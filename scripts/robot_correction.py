#!/usr/bin/env python
import rospy
import serial
import string
import math
import gpsmath
import robot_drive


from math import radians, cos, sin, asin, sqrt, atan2, degrees

#-------------------------------------------------------#
#	Robot error correction module						#
#-------------------------------------------------------#
# while robot's moving, dynamically update robot gps 
def update_robot_gps(left_dist, right_dist): 
	lon1 = robot_drive.lon_now
	lat1 =  robot_drive.lat_now
	initial_bearing = robot_drive.bearing_now
	
	# robto moving perfectly straight 
	if(left_dist == right_dist):
		new_gps = gpsmath.get_gps(lon1, lat1, initial_bearing, left_encode)
		robot_drive.lon_now = new_gps[0]
		robot_drive.lat_now = new_gps[1]
		return
	
	# robot not so perfectly walking 
	alpha = 0.0
	R = 0.0 
	total_dist = left_dist + right_dist
	alpha =  total_dist / (2.0 * robot_drive.turn_radius)
	if(left_dist > right_dist):	
		R = total_dist * robot_drive.turn_radius / (left_dist - right_dist)
	elif(right_dist > left_dist): 
		R = total_dist * robot_drive.turn_radius / (right_dist - left_dist)


	half_alpha = alpha / 2.0
	bearing = initial_bearing + half_alpha
	distance = R * sin (half_alpha)
	new_gps = gpsmath.get_gps(lon1, lat1, bearing, distance)
	
	robot_drive.bearing_now = bearing
	robot_drive.lon_now = new_gps[0]
	robot_drive.lat_now = new_gps[1]
