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
# still need to handle more scenarios 

def update_robot_gps(left_dist, right_dist): 
	#scenario 1, robot not moving 
	if(left_dist == 0 and right_dist == 0):
		#no updating of information 
		return

	lon1 = robot_drive.lon_now
	lat1 =  robot_drive.lat_now
	initial_bearing = robot_drive.bearing_now
	
	# scenario 02 robot moving perfectly straight, bearing won't change, while lan and lon need to be updated 
	if(left_dist == right_dist):
		if(right_dist > 0 ):
			new_gps = gpsmath.get_gps(lon1, lat1, initial_bearing, left_dist)
			robot_drive.lon_now = new_gps[0]
			robot_drive.lat_now = new_gps[1]
			return
		if(right_dist < 0):
			new_gps = gpsmath.get_gps(lon1, lat1, -initial_bearing, left_dist)
			robot_drive.lon_now = new_gps[0]
			robot_drive.lat_now = new_gps[1]
			return

	
	alpha = 0.0
	total_dist = abs(left_dist) + abs(right_dist)
	alpha = (abs(left_dist) + abs(right_dist)) / (2.0 * robot_drive.turn_radius)
	#convert to degree 
	half_alpha = 180 * alpha / (2.0 * math.pi)  
	bearing = 0.0
	distance = 0.0 
	R = 0.0; 
	# scenario 03 robot moving forward 
	# robot not so perfectly walking forward, eigher left wheel is faster or right wheel is faster 
	if(left_dist > 0 and right_dist > 0):
		# a little bot of right turning 
		if(left_dist > right_dist):	
			R = total_dist * robot_drive.turn_radius / (left_dist - right_dist)
			bearing = initial_bearing + half_alpha
		elif(right_dist > left_dist): 
			R = total_dist * robot_drive.turn_radius / (right_dist - left_dist)
			bearing = initial_bearing - half_alpha
	# scenario 04 robot moving backward
	elif(left_dist < 0 and right_dist < 0):
		if(left_dist < right_dist):
			R = total_dist * robot_drive.turn_radius / (right_dist - left_dist)
			bearing  = initial_bearing + 180 - half_alpha
		elif(right_dist < left_dist):
			R = total_dist * robot_drive.turn_radius / (left_dist - right_dist)
			bearing  = initial_bearing + 180 + half_alpha
	#for robot two wheels not moving at the same direction or once of the thing not moving 
	# forwaring with rotation 
	else:
		r1 = abs(left_dist) / alpha
		r2 = abs(right_dist) / alpha 
		R = abs(r1 - r2) 
		if(left_dist >0 or (left_dist == 0 and  right_dist <0)): 
			bearing = initial_bearing + half_alpha
		else:
			bearing = initial_bearing - half_alpha

	bearing = gpsmath.format_bearing(bearing)
	distance = R * sin (half_alpha) * 2
	new_gps = gpsmath.get_gps(lon1, lat1, bearing, distance)		
	robot_drive.bearing_now = bearing
	robot_drive.lon_now = new_gps[0]
	robot_drive.lat_now = new_gps[1]

