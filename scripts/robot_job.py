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
gps_num 			= 5

init_lon			= 103.962386
init_lat			= 1.340549
init_bearing		= 0

gps_lon 			= [103.962389,103.962456,103.962461,103.962381] #S,A,B,C,D
gps_lat 			= [1.3407,1.340696,1.340589,1.340599]
job_des 			= []			#could be 'T' or 'F'
job_num 			= []			#if job is 'T', the number is the angle of robot need to face of the job else it's the distance in mm 
job_lon_target 		= []
job_lat_target 		= []
job_bearing_target 	= []
job_type			= []			#define job type check whether it's a correction job or normal job 
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
		if(robot_drive.robot_moving == 1):
			robot_drive.stop_robot()
			time.sleep(0.01)
		else: 
			# Clear all the reamining jobs
			clear_jobs()
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
	append_regular_jobs(init_lat, init_lon, gps_lon[0],gps_lat[0]);
	#step 2: Start loop jobs
	#handles from start to first point
	gps_num = len(gps_lon) 
	#handles how many loops
	for i in range (loops) :
		for k in range (gps_num):
			ne_k = (k + 1) % gps_num
			append_regular_jobs(gps_lon[k],gps_lat[k],gps_lon[ne_k],gps_lat[ne_k])
	#move to init position
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

def insert_compensation_jobs(lon_source, lat_source, lon_target, lat_target):
	global job_lists
	bearing 	= gpsmath.bearing(lon_source, lat_source, lon_target, lat_target)
	distance 	= gpsmath.haversine(lon_source, lat_source, lon_target, lat_target)
	turn_job 	= Job(lon_source, lat_source, bearing, 'C', 'T', bearing)
	move_job 	= job(lon_target, lat_target, bearing, 'C', 'F', distance) 
	job_lists.insert(0, turn_job)
	job_lists.insert(0, move_job)

def has_jobs_left():
	global job_lists 
	#rospy.loginfo("No of jobs left %d", len(job_lists)) 
	return len(job_lists) <= 0

def current_job():
	global job_lists
	return job_lists[0]

def complete_current_job():
	global job_lists
	robot_drive.lon_target 		= job_lists[0].lon_target;
	robot_drive.lat_target 		= job_lists[0].lat_target; 
	robot_drive.bearing_target 	= job_lists[0].bearing_target; 
	global job_lists
	del job_lists[0]

def clear_job_list():
	global job_lists
	del job_lists[:]

def simple_move(distance, bearing, direction):
	rospy.loginfo("Added a turn job T, %d", bearing)
	turn_job 	= Job(robot_drive.lon_now, robot_drive.lat_now, 0, 'N', 'T', bearing)
	job_lists.extend([turn_job])
	# add a move job to move 10 meters 
	rospy.loginfo("Added a move job %s, %d", direction, distance)
	lon_new, lat_new  = gpsmath.get_gps(robot_drive.lon_now, robot_drive.lat_now, distance, bearing)
	append_regular_jobs(robot_drive.lon_now, robot_drive.lat_now, lon_new, lat_new)

def simple_trun(bearing):
	turn_job 	= Job(robot_drive.lon_now, robot_drive.lat_now, 0, 'N', 'T', bearing)
	job_lists.extend([turn_job])


def define_test_job():
	# add a turn job to turn to 0 degree 
	turn_job 	= Job(robot_drive.lon_now, robot_drive.lat_now, 0, 'N', 'T', 0)
	job_lists.extend([turn_job])
	# add a move job to move 10 meters 
	lon_new, lat_new  = gpsmath.get_gps(robot_drive.lon_now, robot_drive.lat_now, 10000, 0)
	append_regular_jobs(robot_drive.lon_now, robot_drive.lat_now, lon_new, lat_new)
	# now turn to 90 
	turn_job 	= Job(lon_new, lat_new , 90, 'N', 'T', 90)
	job_lists.extend([turn_job])
	# move another 10 meters 
	lon_new, lat_new  = gpsmath.get_gps(lon_new, lat_new , 10000, 90)
	append_regular_jobs(robot_drive.lon_now, robot_drive.lat_now, lon_new, lat_new)
	# now turn to 180 
	turn_job 	= Job(lon_new, lat_new , 180, 'N', 'T', 180)
	job_lists.extend([turn_job])
	# move another 10 meters 
	lon_new, lat_new  = gpsmath.get_gps(lon_new, lat_new , 10000, 180)
	append_regular_jobs(robot_drive.lon_now, robot_drive.lat_now, lon_new, lat_new)
	# now turn to 270 
	turn_job 	= Job(lon_new, lat_new , 270, 'N', 'T', 270)
	job_lists.extend([turn_job])
	# move another 10 meters 
	lon_new, lat_new  = gpsmath.get_gps(lon_new, lat_new , 10000, 270)
	append_regular_jobs(robot_drive.lon_now, robot_drive.lat_now, lon_new, lat_new)
	# now turn to 270 
	turn_job 	= Job(lon_new, lat_new , 0, 'N', 'T', 0)
	job_lists.extend([turn_job])


def define_initialize_job():
	turn_job 	= Job(robot_drive.lon_now, robot_drive.lat_now, 0, 'N', 'T', 0)
	job_lists.extend([turn_job])
	turn_job 	= Job(robot_drive.lon_now, robot_drive.lat_now, 90, 'N', 'T', 90)
	job_lists.extend([turn_job])
	turn_job 	= Job(robot_drive.lon_now, robot_drive.lat_now, 180, 'N', 'T', 180)
	job_lists.extend([turn_job])
	turn_job 	= Job(robot_drive.lon_now, robot_drive.lat_now, 270, 'N', 'T', 270)
	job_lists.extend([turn_job])
	turn_job 	= Job(robot_drive.lon_now, robot_drive.lat_now, 0, 'N', 'T', 0)
	job_lists.extend([turn_job])
	turn_job 	= Job(robot_drive.lon_now, robot_drive.lat_now, 90, 'N', 'T', 90)
	job_lists.extend([turn_job])


#------------------------- end of re-factoring ----------------------------------
# generate job from gps lists 
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
	set_target_gps(robot_drive.lon_now, robot_drive.lat_now, init_bearing)

#based on the gps coordinates of the two location, generate jobs
def generate_job(first_point, second_point):
	global gps_lon
	global gps_lat
	#handles from first_point to second_point
	generate_job_from_gps(gps_lon[first_point],gps_lat[first_point],gps_lon[second_point],gps_lat[second_point]) #km 


# based on two gps corrdinates, generate a turn job and a move job 
def generate_job_from_gps(lon1, lat1, lon2, lat2):
	angle_next 	= gpsmath.bearing(lon1, lat1, lon2, lat2)  	# the angle that the robot must face before it moves 
	distance 	= gpsmath.haversine(lon1, lat1, lon2, lat2)	# the distance that the robot have to move after the angle corrected
	apeend_regular_turn_job(angle_next, lon1, lat1, angle_next)
	append_move_regular_job(distance, 'F', lon2, lat2, angle_next)

# append a regular move job on the back 
def append_regular_move_job(distance, direction, target_lon, target_lat, target_bearing):
	generate_move(distance, direction)
	set_target_gps(target_lon, target_lat, target_bearing)

# append a regular turn job on the back 
def apeend_regular_turn_job(target_lon, target_lat, target_bearing):
	generate_turn(target_bearing)
	set_target_gps(target_lon, target_lat, target_bearing)

# set target gps and bearing 
def set_target_gps(lon, lat, bearing):
	global job_lon_target
	global job_lat_target
	global job_bearing_target
	job_lon_target.extend([lon])
	job_lat_target.extend([lat])
	job_bearing_target.extend([bearing])
	rospy.loginfo("Added a target gps %d", len(job_lon_target))

# clear current job
def remove_current_job():
	global job_des
	global job_num 
	global job_lon_target
	global job_lat_target
	global job_bearing_target 
	global job_type
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
	if len(job_type) > 0:
		del job_type[0]

# Clear jobs 
def clear_jobs():
	global job_des
	global job_num 
	global job_lon_target
	global job_lat_target
	global job_bearing_target
	global job_type
	del job_des[:]
	del job_num[:]
	del job_lon_target[:]
	del job_lat_target[:]
	del job_bearing_target[:]
	del job_type[:]

def generate_move( distance, direction):
	global job_des
	global job_num 
	global job_type
	job_num.extend([distance]) 
	job_des.extend([direction])
	job_type.extend(['N'])
	rospy.loginfo("Generated a normal job move %s with distance %f mm", direction, distance)

def generate_turn(angle):
	global job_des
	global job_num 
	global job_type
	job_num.extend([angle]) 
	job_des.extend(['T'])
	job_type.extend(['N'])
	rospy.loginfo("Generated a normal job turn to angle %f", angle)


# all the correction related jobs need to be inserted on the front 

def add_correction_turn(angle ): 
	global job_des
	global job_num 
	global job_type
	job_des.insert(0, 'T')
	job_num.insert(0, angle)
	job_type.insert(0, 'C')
	rospy.loginfo("Inserted a correction job turn to angle %f", angle)

def add_correction_move(distance):
	global job_des 
	global job_num 
	global job_type
	if(distance == 0):
		return
	job_num.insert(0, abs(distance))
	direction = 'F'
	if(distance < 0):
		job_des.insert(0, 'B')
	else:
		job_des.insert(0, 'F')
	job_type.insert(0, 'C')
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
	add_correction_move(distance)
	add_target_gps(lon2, lat2, angle_next)
	add_correction_turn(angle_next)
	add_target_gps(lon1, lat1, angle_next)

# as discovred, the digital compass would be accurabte in true north, so to initialize the robot,
# we need to turn one round to identify the true north 
def initialize_job():
	bearing  = simple_job_turn(90, robot_drive.lon_now, robot_drive.lat_now)
	bearing  = simple_job_turn(180, lon_new, lat_new )
	bearing  = simple_job_turn(270, lon_new, lat_new )
	bearing  = simple_job_turn(0, lon_new, lat_new )
	bearing  = simple_job_turn(90, lon_new, lat_new )





