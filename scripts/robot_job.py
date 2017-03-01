#!/usr/bin/env python
import rospy
import serial
import string
import math 
import gpsmath
import robot_drive
import robot_turn
import robot_move 

#-------------------------------------------------------#
#	Robot jobs module									#
#-------------------------------------------------------#
from std_msgs.msg import String
from math import radians, cos, sin, asin, sqrt, atan2, degrees

###################### EDIT HERE ###########################
#defining or acquiring the GPS coordinates
init_lon			= 103.962386
init_lat			= 1.340549
init_bearing		= 0
gps_lon 			= [103.962389,103.962456,103.962461,103.962381] #S,A,B,C,D
gps_lat 			= [1.3407,1.340696,1.340589,1.340599]
loops 				= 1 			#how many rounds to go
correction_count 	= 0
max_correction_run 	= 15
job_lists 			= [] 

# if not jobs in the sytem 
def process_no_job():
	robot_drive.robot_on_mission = 0
	if(robot_drive.robot_moving == 1):
		rospy.logwarn('warning: robot is not fully stopped even though a top command issued')
		robot_drive.stop_robot()
		time.sleep(0.05)
		return

# Process all kinds of robot job as required 
def process_job():
	job_completed = 0 
	global job_lists
	if (job_lists[0].description == 'T') : 
		#rospy.loginfo("Bearing now %f, bearing target %f", robot_drive.bearing_now, robot_drive.bearing_target)
		#if(robot_drive.robot_on_mission == 0): 
		robot_drive.bearing_target  = job_lists[0].value
		# Pre-steps of turning jobs starts: calculate the required angle to turn 
		# start the job 
		job_completed = robot_turn.turn_degree()
	elif (job_lists[0].description == 'F' or job_lists[0].description == 'B'):
		if(job_lists[0].description == 'B'):
			job_lists[0].value  = -abs(job_lists[0].value)
		#rospy.loginfo("process_job move......")
		job_completed =robot_move.move_distance(job_lists[0].value)
		#rospy.loginfo("Bearing target before correction %f", robot_drive.bearing_target)
	else :
		rospy.logwarn('job_des %s:%d', job_lists[0].description, job_lists[0].value)
		rospy.logwarn('warning: illegal job description found, not peform any actions')
	#rospy.loginfo("Bearing target before correction %f", robot_drive.bearing_target)
	return job_completed

# Complete init compass
def complete_init_compass(compass_value):
	if abs(compass_value) <=2 or abs(compass_value) >= 358:
		#Clear the remainging initialization jobs 
		robot_drive.bearing_now = compass_value
		clear_job_list()
		robot_drive.robot_initialized = 1
		rospy.loginfo("Robot initlization completed")

# disable robot if emergency stop button clicked (0)
def disable_robot():
	while True:
		#step 1, send the stop command every 10 milli seoncs
		if(robot_drive.robot_moving == 1 or robot_drive.robot_turning == 1):
			robot_drive.stop_robot()
			time.sleep(0.01)
		else: 
			# Clear all the reamining jobs
			clear_job_list()
			#robot_drive.stop_robot()
			break
			#robot_drive.robot_enabled = 1

# class to define i
class Job:
	classfication 	= 'N'
	lon_target		= 0.0
	lat_target 		= 0.0 
	bearing_target  = 0
	description 	= ''
	value 			= 0
	index 			= 0

	# constructor
	def __init__(self, lon, lat, bearing, classify, description, value):
		self.lon_target 	= lon
		self.lat_target 	= lat 
		self.bearing_target = bearing
		self.classfication	= classify
		self.value = value 
		self.description = description

#@yuqing_forwardafterobstacle
dist_forward_after_obstacle = 1000

#---------------------------re-factored job lists ------------------------------
# init robot initial poition 
def init_robot_gps(lon, lat, bearing):
	global init_lat, init_lon, init_bearing 
	init_lon 		= lon
	init_lat 		= lat
	init_bearing 	= bearing 

# generate robot jobs based on robot gps route 
def generate_jobs_from_gps():
	#step 1: Move from initial point to the loop start point 
	global init_lat, init_lon, init_bearing
	global loops, gps_lon, gps_lat 
	global job_lists
	append_regular_jobs(init_lat, init_lon, gps_lon[0],gps_lat[0]);
	#step 2: Start loop jobs
	#handles from start to first point
	gps_num = len(gps_lon) 
	rospy.loginfo("jobs created for init %d, loop control points: %d", len(job_lists), len(gps_lon)) 
	#handles how many loops
	if(gps_num > 1):
		for i in range (loops) :
			for k in range (gps_num):
				ne_k = (k + 1) % gps_num
				append_regular_jobs(gps_lon[k],gps_lat[k],gps_lon[ne_k],gps_lat[ne_k])
	#move to init position
	rospy.loginfo("Number of jobs %d", len(job_lists))
	append_regular_jobs(gps_lon[0],gps_lat[0], init_lat, init_lon)
	turn_job 	= Job(init_lat, init_lon, 0, 'N', 'T', 0)
	job_lists.extend([turn_job])

# generate regualr jobs from point to point()
def append_regular_jobs(lon_source, lat_source, lon_target, lat_target):
	global job_lists
	bearing 	= gpsmath.bearing(lon_source, lat_source, lon_target, lat_target)
	distance 	= gpsmath.haversine(lon_source, lat_source, lon_target, lat_target)
	turn_job 	= Job(lon_source, lat_source, bearing, 'N', 'T', bearing)
	move_job 	= Job(lon_target, lat_target, bearing, 'N', 'F', distance) 
	job_lists.extend([turn_job])
	job_lists.extend([move_job])

def has_jobs_left():
	global job_lists 
	#rospy.loginfo("No of jobs left %d", len(job_lists)) 
	return len(job_lists) <= 0

def current_job():
	global job_lists
	return job_lists[0]

def clear_job_list():
	global job_lists
	del job_lists[:]

def complete_current_job():
	global job_lists
	robot_drive.lon_target 		= job_lists[0].lon_target;
	robot_drive.lat_target 		= job_lists[0].lat_target; 
	robot_drive.bearing_target 	= job_lists[0].bearing_target; 
	del job_lists[0]

# list of test jobs: Not from gps, but just like move, turn etc
# ---------------------------------------------------------------------------------
def simple_move(distance, bearing, direction):
	rospy.loginfo("Added a turn job T, %d", bearing)
	simple_turn(bearing)
	# add a move job to move 10 meters 
	lon_new, lat_new  = append_regular_job(robot_drive.lon_now, robot_drive.lat_now, distance, bearing)

def simple_turn(bearing):
	append_turn_job(robot_drive.lon_now, robot_drive.lat_now, bearing)

# ----------------------------------------------------------------------------------
# A list of operations for regular jobs 

def append_turn_job(lon_target, lat_target, bearing_target):
	turn_job 	= Job(lon_target, lat_target, bearing_target, 'N', 'T', bearing_target)
	job_lists.extend([turn_job])

def append_regular_job(lon_now, lat_now, distance, bearing):
	# Get new GPS 
	lon_new, lat_new  = gpsmath.get_gps(lon_now, lat_now, distance, bearing)
	append_regular_jobs(lon_now, lat_now, lon_new, lat_new)
	return lon_new, lat_new

def define_test_job():
	# add a turn job to turn to 0 degree 
	append_turn_job(robot_drive.lon_now, robot_drive.lat_now, 0)
	# add a move job to move 10 meters 
	lon_new, lat_new  = append_regular_job(robot_drive.lon_now, robot_drive.lat_now, 10000, 0)
	# now turn to 90 
	append_turn_job(lon_new, lat_new , 90)
	# move another 10 meters 
	lon_new, lat_new  = append_regular_job(lon_new, lat_new, 10000, 90)
	# now turn to 180 
	append_turn_job(lon_new, lat_new , 180)
	# move another 10 meters 
	lon_new, lat_new  = append_regular_job(lon_new, lat_new, 10000, 180)
	# now turn to 270 
	append_turn_job(lon_new, lat_new , 270)
	# move another 10 meters 
	lon_new, lat_new  = append_regular_job(lon_new, lat_new, 10000, 270)
	# now turn to 270 
	append_turn_job(robot_drive.lon_now, robot_drive.lat_now, 0)

# The job used to initialize the compass
def define_initialize_job():
	append_turn_job(robot_drive.lon_now, robot_drive.lat_now, 0)
	append_turn_job(robot_drive.lon_now, robot_drive.lat_now, 90)
	append_turn_job(robot_drive.lon_now, robot_drive.lat_now, 180)
	append_turn_job(robot_drive.lon_now, robot_drive.lat_now, 270)
	append_turn_job(robot_drive.lon_now, robot_drive.lat_now, 0)
	append_turn_job(robot_drive.lon_now, robot_drive.lat_now, 90)

#--------------------------------------------------------------------------------
# a list of operations for the correction jobs 
def insert_compensation_jobs(lon_source, lat_source, lon_target, lat_target):
	global job_lists
	bearing 	= gpsmath.bearing(lon_source, lat_source, lon_target, lat_target)
	distance 	= gpsmath.haversine(lon_source, lat_source, lon_target, lat_target)
	turn_job 	= Job(lon_source, lat_source, bearing, 'C', 'T', bearing)
	move_job 	= Job(lon_target, lat_target, bearing, 'C', 'F', distance) 
	job_lists.insert(0, move_job)
	job_lists.insert(0, turn_job)
	


#------------------------- end of re-factoring ----------------------------------
