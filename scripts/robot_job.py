#!/usr/bin/env python
import rospy
import serial
import string
import math 
import gpsmath

#-------------------------------------------------------#
#	Robot jobs module									#
#-------------------------------------------------------#

from std_msgs.msg import String
from math import radians, cos, sin, asin, sqrt, atan2, degrees

###################### EDIT HERE ###########################
#defining or acquiring the GPS coordinates
gps_num = 4
start 	= [103.962386,1.340549]
gps_lon = [103.962386,103.962389,103.962456,103.962461,103.962381] #S,A,B,C,D
gps_lat = [1.340549,1.3407,1.340696,1.340589,1.340599]
job_des = []			#could be 'T' or 'F'
job_num = []			#if job is 'T', the number is the angle of robot need to face of the job else it's the distance in mm 
loops = 1 				#how many rounds to go


def job_generator(init_bearing):	
	global gps_lon
	global gps_lat
	global job_des
	global job_num 
	global gps_num 
	global loops 
	#handles from start to first point
	job = job_details(0, 1)
	robot_job.job_des.append('T')
	robot_job.job_des.append('F')
	robot_job.job_num.extend([job[0],job[1]])   #in the form of target bearing and distance
		
	#handles how many loops
	for i in range (loops) :
		for k in range (gps_num):
			if k < gps_num - 1 :
				job = job_details(k + 1, k + 2)
				robot_job.job_des.extend(['T','F'])
				robot_job.job_num.extend([job[0],job[1]])
				
			else : 
				job = job_details(k + 1, 1)
				robot_job.job_des.extend(['T','F'])
				robot_job.job_num.extend([job[0],job[1]])
	
	#handles closing loop, going back to start
	job = job_details(1, 0)
	robot_job.job_des.extend(['T','F'])
	robot_job.job_num.extend([job[0],job[1]])
	#final turn to init_bearing
	robot_job.job_des.append('T')
	robot_job.job_num.append(init_bearing)
	#ending_turn_angle = init_angle - angle_now
	#if ending_turn_angle > 180 :
	#	ending_turn_angle = ending_turn_angle - 360.0
	#robot_job.job_num.append(ending_turn_angle)

#based on the gps coordinates of the two location, generate jobs
def generate_one_job(first_point, second_point):
	global gps_lon
	global gps_lat
	#handles from first_point to second_point
	distance 	= gpsmath.haversine(gps_lon[first_point],gps_lat[first_point],gps_lon[second_point],gps_lat[second_point]) #km
	angle_next 	= gpsmath.bearing(gps_lon[first_point],gps_lat[first_point],gps_lon[second_point],gps_lat[second_point]) #deg  
		
	#handles turning angle		
	#turn_angle = angle_next - angle_now
	#if turn_angle > 180.0 :
	#	turn_angle = turn_angle - 360.0
	#handles forward distance in mm
	distance = distance * 1000.0 * 1000.0
	return ([round(angle_next), distance])

# clear current job
def remove_current_job():
	global job_des
	global job_num 
	if(len(job_des) > 0): 
		del job_des[0]
	if(len(job_num) > 0):
		del job_num[0]

# Clear jobs 
def clear_jobs():
	global job_des
	global job_num 
	del job_des[:]
	del job_num[:]

def generate_move( distance, direction):
	global job_des
	global job_num 
	job_num.extend([distance]) 
	job_des.extend([direction])

def generate_turn(angle):
	global job_des
	global job_num 
	job_num.extend([angle]) 
	job_des.extend(['T'])


def simple_job():
	generate_move(2000)
	generate_turn(90)
	generate_move(5000)
	generate_turn(270)
	generate_move(5000)
	generate_turn(-90)
	generate_move(2000)


