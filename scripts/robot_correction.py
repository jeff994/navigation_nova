#!/usr/bin/env python
import rospy
import serial
import string
import math
import gpsmath
import robot_drive
import robot_job 
import robot_publisher 

from math import radians, cos, sin, asin, sqrt, atan2, degrees

#-------------------------------------------------------#
#	Robot error correction module						#
#-------------------------------------------------------#
# while robot's moving, dynamically update robot gps
# still need to handle more scenarios 

def update_robot_gps(left_encode, right_encode): 
	robot_drive.step_angle = 0.0
	robot_drive.step_distance = 0.0

	#scenario 1, robot not moving 
	if(left_encode == 0 and right_encode == 0):
		#no updating of information 
				#@yuqing_continueturn
		return

	# loacal vaiables 
	left_dist 	= left_encode / robot_drive.encode_to_mm
	right_dist 	= right_encode / robot_drive.encode_to_mm
	alpha 		= 0.0
	total_dist 	= abs(left_dist) + abs(right_dist) 
	R 			= 0.0

	# global vaiables 
	robot_drive.step_angle 		= 0.0
	robot_drive.step_distance 	= (left_dist + right_dist) / 2

	rospy.loginfo("Bearing now %f,lon_now %f, lat_now %f", robot_drive.bearing_now, robot_drive.lon_now, robot_drive.lat_now)
	# scenario 01, 02 robot moving perfectly straight, bearing won't change, while lan and lon need to be updated 
	if(left_dist == right_dist):
		if(right_dist > 0 ):
			robot_drive.lon_now, robot_drive.lat_now = gpsmath.get_gps(robot_drive.lon_now, robot_drive.lat_now , right_dist, robot_drive.bearing_now)
		if(right_dist < 0):
			robot_drive.lon_now, robot_drive.lat_now = gpsmath.get_gps(robot_drive.lon_now, robot_drive.lat_now , right_dist, -robot_drive.bearing_now)
		#rospy.loginfo("Bearing now %f,lon_now %f, lat_now %f", robot_drive.bearing_now, robot_drive.lon_now, robot_drive.lat_now)
		stringToSend = '%f %f %f' % (robot_drive.lon_now, robot_drive.lat_now, robot_drive.bearing_now)
		robot_drive.pub_gps.publish(stringToSend)
		return	
	# scenario 02 robot moving forward with slight 
	# robot not so perfectly walking forward, eigher left wheel is faster or right wheel is faster 
	elif(left_dist > 0 and right_dist > 0):
		# a little bit of right turning
		alpha 	= (left_dist - right_dist) / (2.0 * robot_drive.turn_radius) 
		R 	= total_dist * robot_drive.turn_radius / abs(left_dist - right_dist)
	# scenario 04 robot moving backward
	elif(left_dist < 0 and right_dist < 0):
		alpha 	= (left_dist - right_dist) / (2.0 * robot_drive.turn_radius) 
		R 	= -total_dist * robot_drive.turn_radius / abs(right_dist - left_dist)
	# for robot two wheels not moving at the same direction or once of the thing not moving 
	# forwaring with rotation 
	else:
		alpha 	= total_dist / (2.0 * robot_drive.turn_radius)
		r1 		= abs(left_dist) / alpha
		r2 		= abs(right_dist) / alpha 
		R 		= abs(r1 - r2)

		#right turn 
		if(left_dist > 0 or (left_dist == 0 and right_dist < 0)): 
			alpha = alpha 
		else:
			alpha = -alpha 

	robot_drive.step_angle 	= degrees(alpha)
	# covnert to degree
	bearing 				= robot_drive.bearing_now + robot_drive.step_angle / 2
	# convert between [0 - 360)
	bearing 				= gpsmath.format_bearing(bearing)
	dist 					= R * sin(abs(alpha/2)) * 2
	rospy.loginfo("Step Distance moved %fmm, Step_angle %f degree, R %f mm, Step_distance %f mm", dist, robot_drive.step_angle, R, robot_drive.step_distance) 
	robot_drive.lon_now, robot_drive.lat_now 	= gpsmath.get_gps(robot_drive.lon_now, robot_drive.lat_now, dist, bearing)		
	robot_drive.bearing_now 					= gpsmath.format_bearing(robot_drive.bearing_now + robot_drive.step_angle)
	robot_publisher.publish_gps()
	rospy.loginfo("Bearing now %f,lon_now %f, lat_now %f", robot_drive.bearing_now, robot_drive.lon_now, robot_drive.lat_now)

def dist_correction_normal():
	distance_correction(robot_drive.lon_now, robot_drive.lat_now, robot_drive.bearing_now, robot_drive.lon_target, robot_drive.lat_target, robot_drive.bearing_target)

# correct robot every time by comapring the lat_now, lon_now with target position
def distance_correction(lon_now, lat_now, bearing_now, lon_target, lat_target, bearing_target):
	rospy.loginfo("**************correction jobs**************")
	distance 	= gpsmath.haversine(lon_now, lat_now, lon_target, lat_target)
	bearing 	= gpsmath.bearing(lon_now, lat_now, lon_target, lat_target)
	# check the bearing now and bearing target 
	rospy.loginfo("GPS now [%f, %f], GPS target: [%f, %f]", lon_now, lat_now, lon_target, lat_target)
	rospy.loginfo("Bearing move %f, Bearing now %f, bearing target %f", bearing, bearing_now, bearing_target)
	
	diff_angle = abs(bearing_target - bearing_now)
	
	if(bearing > 90 and bearing < 270):
		distance = -distance 
		bearing = (bearing+180)%360

	rospy.loginfo("There's a %f mm distance error, %f angle difference", distance, diff_angle)

	if abs(distance) > 100 or (diff_angle > 5 and diff_angle < 355):
		rospy.loginfo("Add jobs to correction.")
		robot_job.insert_compensation_jobs(lon_now, lat_now, lon_target, lat_target)
	else: 
		rospy.loginfo("no need to compensate errors")
	
# Correct a robot with obstancles by inserting a job to move the robot forward for 1m
def distance_correction_obstacle(dist):
	lon_new, lat_new = gpsmath.get_gps(robot_drive.lon_now, robot_drive.lat_now, dist, robot_drive.bearing_now)
	#fist distance correction 
	distance_correction(lon_new, lat_new, robot_drive.bearing_now, robot_drive.lon_target, robot_drive.lat_target, robot_drive.bearing_target)
	#rospy.loginfo("There's a %f mm distance error, %f angle difference", distance, diff_angle)
	rospy.loginfo("Add a job to move forward %d mm", robot_job.dist_forward_after_obstacle)
	robot_job.insert_compensation_jobs(robot_drive.lon_now, robot_drive.lat_now, lat_new, lon_new, lat_new)
