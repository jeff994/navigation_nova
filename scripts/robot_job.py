#!/usr/bin/env python
import rospy
import serial
import string
import math 
import gpsmath
import robot_drive

#-------------------------------------------------------#
#	Robot jobs module									#
#-------------------------------------------------------#

from std_msgs.msg import String
from math import radians, cos, sin, asin, sqrt, atan2, degrees

###################### EDIT HERE ###########################
#defining or acquiring the GPS coordinates
gps_num 			= 5
gps_lon 			= [103.962386,103.962389,103.962456,103.962461,103.962381] #S,A,B,C,D
gps_lat 			= [1.340549,1.3407,1.340696,1.340589,1.340599]
job_des 			= []			#could be 'T' or 'F'
job_num 			= []			#if job is 'T', the number is the angle of robot need to face of the job else it's the distance in mm 
job_lon_target 		= []
job_lat_target 		= []
job_bearing_target 	= []
loops 				= 2 				#how many rounds to go


# generate job from pre-defined gps 
def job_generator(init_bearing):	
	global gps_num 
	global loops 
	
	#handles from start to first point
	gps_num = len(gps_lon) 
	#handles how many loops
	for i in range (loops) :
		for k in range (gps_num):
			ne_k = (k + 1) % gps_num
			generate_job(k, ne_k)
	#final turn to init_bearing
	generate_turn(init_bearing)

#based on the gps coordinates of the two location, generate jobs
def generate_job(first_point, second_point):
	global gps_lon
	global gps_lat
	#handles from first_point to second_point
	generate_job_from_gps(gps_lon[first_point],gps_lat[first_point],gps_lon[second_point],gps_lat[second_point]) #km 

# set target gps and bearing 
def set_target_gps(lon, lat, bearing):
	global job_lon_target
	global job_lat_target
	global job_bearing_target
	job_lon_target.extend([lon])
	job_lat_target.extend([lat])
	job_bearing_target.extend([bearing])
	rospy.loginfo("Added a target gps %d", len(job_lon_target))

# based on two gps corrdinates, generate a turn job and a move job 
def generate_job_from_gps(lon1, lat1, lon2, lat2):
	angle_next 	= gpsmath.bearing(lon1, lat1, lon2, lat2)  	# the angle that the robot must face before it moves 
	distance 	= gpsmath.haversine(lon1, lat1, lon2, lat2)	# the distance that the robot have to move after the angle corrected
	generate_turn(angle_next)
	set_target_gps(lon1, lat1, angle_next)
	generate_move(distance, 'F')
	set_target_gps(lon2, lat2, angle_next)

# clear current job
def remove_current_job():
	global job_des
	global job_num 
	global job_lon_target
	global job_lat_target
	global job_bearing_target 
	# Reset the robot job status 
	robot_drive.robot_on_mission = 0
	if(len(job_des) > 0): 
		del job_des[0]
	if(len(job_num) > 0):
		del job_num[0]
	if len(job_lat_target) > 0: 
		del job_lat_target[0]
	if len(job_lon_target) > 0: 
		del job_lon_target[0]
	if len(job_bearing_target) > 0: 
		del job_bearing_target[0]

# Clear jobs 
def clear_jobs():
	global job_des
	global job_num 
	global job_lon_target
	global job_lat_target
	global job_bearing_target
	del job_des[:]
	del job_num[:]
	del job_lon_target[:]
	del job_lat_target[:]
	del job_bearing_target[:]

def generate_move( distance, direction):
	global job_des
	global job_num 
	job_num.extend([distance]) 
	job_des.extend([direction])
	rospy.loginfo("Generated a job move %s with distance %f mm", direction, distance)

def generate_turn(angle):
	global job_des
	global job_num 
	job_num.extend([angle]) 
	job_des.extend(['T'])

	rospy.loginfo("Generated a job turn to angle %f", angle)

def add_correction_turn(angle ): 
	global job_des
	global job_num 
	job_des.insert(0, 'T')
	job_num.insert(0, angle)
	rospy.loginfo("Inserted a job turn to angle %f", angle)

def add_correction_move(distance):
	global job_des 
	global job_num 
	if(distance == 0):
		return
	job_num.insert(0, abs(distance))
	direction = 'F'
	if(distance < 0):
		job_des.insert(0, 'B')
	else:
		job_des.insert(0, 'F')
	rospy.loginfo("Inserted a correction job move %s with distance %f mm", direction, distance)

def add_target_gps(lon, lat, bearing):
	global job_lon_target
	global job_lat_target
	global job_bearing_target
	job_lon_target.insert(0, lon)
	job_lat_target.insert(0, lat)
	job_bearing_target.insert(0, bearing)

# based on two gps corrdinates, generate a turn job and a move job 
def add_job_from_gps(lon1, lat1, lon2, lat2):
	angle_next 	= gpsmath.bearing(lon1, lat1, lon2, lat2)  	# the angle that the robot must face before it moves 
	distance 	= gpsmath.haversine(lon1, lat1, lon2, lat2)	# the distance that the robot have to move after the angle corrected
	add_correction_turn(angle_next)
	add_target_gps(lon1, lat1, angle_next)
	add_correction_move(distance, 'F')
	add_target_gps(lon2, lat2, angle_next)

# as discovred, the digital compass would be accurabte in true north, so to initialize the robot,
# we need to turn one round to identify the true north 
def initialize_job():
	lon_new, lat_new  = simple_job_turn(90, robot_drive.lon_now, robot_drive.lat_now)
	lon_new, lat_new  = simple_job_turn(180, lon_new, lat_new )
	lon_new, lat_new  = simple_job_turn(270, lon_new, lat_new )
	lon_new, lat_new  = simple_job_turn(0, lon_new, lat_new )
	lon_new, lat_new  = simple_job_turn(90, lon_new, lat_new )


def simple_job_move(dist, dire, lon, lat, bearing):
	generate_move(dist, dire)
	lon_new, lat_new  = gpsmath.get_gps(lon, lat, dist, bearing)
	set_target_gps(lon_new, lat_new, bearing)
	return lon_new, lat_new

def simple_job_turn(bearing_target, lon_now, lat_now):
	generate_turn(bearing_target)
	set_target_gps(lon_now, lat_now, bearing_target)
	return bearing_target

def simple_job():
	lon_new, lat_new 	= simple_job_move(10000, 'F', robot_drive.lon_now, robot_drive.lat_now, robot_drive.bearing_now)
	bearing_target 		= simple_job_turn(90, lon_new, lat_new)
	lon_new, lat_new 	=simple_job_move(20000, 'F', lon_new, lat_new, bearing_target)
	bearing_target 		= simple_job_turn(180, lon_new, lat_new)
	lon_new, lat_new 	=simple_job_move(10000, 'F', lon_new, lat_new, bearing_target)
	bearing_target 		= simple_job_turn(270, lon_new, lat_new)
	lon_new, lat_new 	=simple_job_move(20000, 'F', on_new, lat_new, bearing_target)
	bearing_target 		= simple_job_turn(0, lon_new, lat_new)





