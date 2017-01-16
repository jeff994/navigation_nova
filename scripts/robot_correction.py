#!/usr/bin/env python
import rospy
import serial
import string
import math
import gpsmath
import robot_drive
import robot_job 


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
                #@yuqing_continueturn
                robot_drive.step_angle = 0.0
                robot_drive.step_distance = 0.0
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
		R 	= total_dist * robot_drive.turn_radius / abs(right_dist - left_dist)
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
	rospy.loginfo("Distance moved %f, step_angle %f, R %f, step_distance %f", dist, robot_drive.step_angle, R, robot_drive.step_distance) 
	robot_drive.lon_now, robot_drive.lat_now 	= gpsmath.get_gps(robot_drive.lon_now, robot_drive.lat_now, dist, bearing)		
	robot_drive.bearing_now 			= gpsmath.format_bearing(robot_drive.bearing_now + robot_drive.step_angle)
	stringToSend 					= '%f %f %f' % (robot_drive.lon_now, robot_drive.lat_now, robot_drive.bearing_now) #might need to add \n behind the E
	robot_drive.pub_gps.publish(stringToSend)
	rospy.loginfo("Bearing now %f,lon_now %f, lat_now %f", robot_drive.bearing_now, robot_drive.lon_now, robot_drive.lat_now)

# before this add a correction job if angle is more than 3 degrees 
def distance_correction():
	distance 	= gpsmath.haversine(robot_drive.lon_now, robot_drive.lat_now, robot_drive.lon_target, robot_drive.lat_target)
	bearing 	= gpsmath.bearing(robot_drive.lon_now, robot_drive.lat_now, robot_drive.lon_target, robot_drive.lat_target)
	# check the bearing now and bearing target 
	rospy.loginfo("GPS now [%f, %f], GPS target: [%f, %f]", robot_drive.lon_now, robot_drive.lat_now, robot_drive.lon_target,robot_drive.lat_target)

	rospy.loginfo("Bearing move %f, Bearing now %f, bearing target %f", bearing, robot_drive.bearing_now, robot_drive.bearing_target)

	if(bearing > 90 and bearing < 270):
		distance = -distance 

	if(distance > 100):
		robot_job.add_correction_turn(bearing)
		robot_job.add_correction_move(distance)
		diff_angle 	= abs(robot_drive.bearing_target - bearing)
		if(diff_angle > 2 and  diff_angle < 358):
			robot_job.add_correction_turn(robot_drive.bearing_target)

	diff_angle = abs(robot_drive.bearing_target - robot_drive.bearing_now)
	if(diff_angle > 2  and diff_angle < 358): 
		robot_job.add_correction_turn(robot_drive.bearing_target)
        rospy.loginfo("**************************************")
        rospy.loginfo("**************correction**************")
	rospy.loginfo("There's a %f mm distance error, %f angle difference", distance, diff_angle)

# def angle_correction(): 
# 	rospy.loginfo("bearing now calculated: %f, bearing target: %f", robot_drive.bearing_now, robot_drive.bearing_target)
# 	diff_angle = gpsmath.format_bearing(robot_drive.bearing_now - robot_drive.bearing_target)
# 	robot_drive.bearing_now = robot_drive.bearing_target;
# 	if diff_angle> 2.0:
# 		robot_job.generate_turn(robot_drive.bearing_target);
	#rospy.loginfo("bearing now calculated: %d, compass _data: %d", robot_drive.bearing_now, compass_data[compass_index])
	#if(distance > 50):	
		#robot_job.generate_move(distance , direction)
		#redefine a move job 
		#return
