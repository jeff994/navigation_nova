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

def update_robot_gps(left_encode, right_encode): 
	#scenario 1, robot not moving 
	if(left_encode == 0 and right_encode == 0):
		#no updating of information 
		return

	left_dist = left_encode / robot_drive.encode_to_mm
	right_dist = right_encode / robot_drive.encode_to_mm

	lon1 = robot_drive.lon_now
	lat1 =  robot_drive.lat_now
	initial_bearing = robot_drive.bearing_now	
	
	#rospy.loginfo("Bearing now %f,lon_now %f, lat_now %f", robot_drive.bearing_now, robot_drive.lon_now, robot_drive.lat_now)
	# scenario 02 robot moving perfectly straight, bearing won't change, while lan and lon need to be updated 
	if(left_dist == right_dist):
		if(right_dist > 0 ):
			robot_drive.lon_now, robot_drive.lat_now = gpsmath.get_gps(lon1, lat1, initial_bearing, left_dist)
		if(right_dist < 0):
			robot_drive.lon_now, robot_drive.lat_now = gpsmath.get_gps(lon1, lat1, -initial_bearing, left_dist)
		
		#rospy.loginfo("Bearing now %f,lon_now %f, lat_now %f", robot_drive.bearing_now, robot_drive.lon_now, robot_drive.lat_now)
		stringToSend = '%f %f %f' % (robot_drive.lon_now, robot_drive.lat_now, robot_drive.bearing_now)
		robot_drive.pub_gps.publish(stringToSend)
		return	

	alpha = 0.0
	total_dist = abs(left_dist) + abs(right_dist)
	#convert to degree 
	
	bearing = 0.0
	distance = 0.0 
	R = 0.0; 
	# scenario 03 robot moving forward 
	# robot not so perfectly walking forward, eigher left wheel is faster or right wheel is faster 
	if(left_dist > 0 and right_dist > 0):
		# a little bot of right turning
		alpha = (left_dist - right_dist)/(2.0 * robot_drive.turn_radius) 
		R = total_dist * robot_drive.turn_radius / (left_dist - right_dist)
		bearing = initial_bearing + degrees(alpha)/2
	# scenario 04 robot moving backward
	elif(left_dist < 0 and right_dist < 0):
		alpha = (left_dist - right_dist)/(2.0 * robot_drive.turn_radius) 
		R = total_dist * robot_drive.turn_radius / (right_dist - left_dist)
		bearing  = initial_bearing + degrees(alpha)/2
	#for robot two wheels not moving at the same direction or once of the thing not moving 
	# forwaring with rotation 
	else:
		alpha = (abs(left_dist) + abs(right_dist)) / (2.0 * robot_drive.turn_radius)
		r1 = abs(left_dist) / alpha
		r2 = abs(right_dist) / alpha 
		R = abs(r1 - r2) 
		if(left_dist >0 or (left_dist == 0 and  right_dist <0)): 
			bearing = initial_bearing + degrees(alpha)/2
		else:
			bearing = initial_bearing + degrees(alpha)/2
	
	bearing = gpsmath.format_bearing(bearing)
	dist = R * sin(degrees(abs(alpha))/2) * 2
	rospy.loginfo("Distance moved %f, step_angle %f, R %f", dist, degrees(abs(alpha)), R) 
	robot_drive.lon_now, robot_drive.lat_now  = gpsmath.get_gps(lon1, lat1, bearing, distance)		
	robot_drive.bearing_now = bearing
	stringToSend = '%f %f %f' % (robot_drive.lon_now, robot_drive.lat_now, robot_drive.bearing_now) #might need to add \n behind the E
	robot_drive.pub_gps.publish(stringToSend)
	rospy.loginfo("Bearing now %f,lon_now %f, lat_now %f", robot_drive.bearing_now, robot_drive.lon_now, robot_drive.lat_now)

