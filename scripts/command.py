#!/usr/bin/env python
import rospy
import serial
import string
import math 
import gpsmath
import robotjob 
import robotdrive
import robotmove
import robotturn

from std_msgs.msg import String
from math import radians, cos, sin, asin, sqrt, atan2, degrees

###################### EDIT HERE ###########################
#defining or acquiring the GPS coordinates
gps_num = 4
start = [103.962386,1.340549]
gps_lon = [103.962386,103.962389,103.962456,103.962461,103.962381] #S,A,B,C,D
gps_lat = [1.340549,1.3407,1.340696,1.340589,1.340599]

initial_bearing = 0 	#set as north for now
loops = 1 				#how many rounds to go
############################################################

keyboard_data  = ''		#keyboard control data

compass_data = 0		#degrees, true north is 0 degrees
dist_travelled = 0		#mm
degree_turned = 0 		#angle inturns of degree

x_now = 0  				#mm
y_now = 0				#mm
r = 350 				#mm, distance between center of robot to wheel
x_target = 0			#mm
y_target = 0 			#mm, should always be 0, because we will be moving in a straight line
bearing_target = 0 		#degrees
job_des = []			#could be 'T' or 'F'
job_num = []			#if job is 'T', the number is the angle of robot need to face of the job else it's the distance in mm 


#based on the gps coordinates of the two location, generate jobs
def job_details(first_point, second_point):
	global gps_lon
	global gps_lat
	#handles from first_point to second_point
	distance 	= gpsmath.haversine(gps_lon[first_point],gps_lat[first_point],gps_lon[second_point],gps_lat[second_point]) #km
	angle_next 	= gpsmath.bearing(gps_lon[first_point],gps_lat[first_point],gps_lon[second_point],gps_lat[second_point]) #deg  
		
	#handles turning angle		
	#turn_angle = angle_next - angle_now
	#if turn_angle > 180.0 :
	#	turn_angle = turn_angle - 360.0
	generate_move
	#handles forward distance in mm
	distance = distance * 1000.0 * 1000.0
	return ([round(angle_next), distance])

def job_generator(init_bearing, loops):
	global job_des
	global job_num
	
	#handles from start to first point
	job = job_details(0, 1)
	job_des.append('T');
	job_des.append('F');
	job_num.extend([job[0],job[1]])   #in the form of target bearing and distance
		
	#handles how many loops
	for i in range (loops) :
		for k in range (gps_num):
			if k < gps_num - 1 :
				job = job_details(k + 1, k + 2)
				job_des.extend(['T','F'])
				job_num.extend([job[0],job[1]])
				
			else : 
				job = job_details(k + 1, 1)
				job_des.extend(['T','F'])
				job_num.extend([job[0],job[1]])
	
	#handles closing loop, going back to start
	job = job_details(1, 0)
	job_des.extend(['T','F'])
	job_num.extend([job[0],job[1]])
	#final turn to init_bearing
	job_des.append('T')
	job_num.append(init_bearing)
	#ending_turn_angle = init_angle - angle_now
	#if ending_turn_angle > 180 :
	#	ending_turn_angle = ending_turn_angle - 360.0
	#job_num.append(ending_turn_angle)

# Subscriber to keyboard topic and peform actions based on the command get  
def keyboard_callback(data):
	global keyboard_data
	global turn_direction
	global job_des
	global job_num

	keyboard_data = data.data
	if (keyboard_data == 'Forward'):
		rospy.loginfo("Command received: Start to move forward 1 m")
		robotjob.generate_move(job_des, job_num, 1000, 'F')
	if (keyboard_data == 'Back'):
		rospy.loginfo("Command received: Start to move back 1 m")
		robotjob.generate_move(job_des, job_num, -1000, 'B')
	elif (keyboard_data == 'Turn_Left'):
		rospy.loginfo("Left turn received"); 
		robotjob.generate_turn(job_des, job_num, -90)
	elif (keyboard_data == 'Turn_Right'): 
		rospy.loginfo('Right turn received')
		robotjob.generate_turn(job_des, job_num, 90)
	elif (keyboard_data == 'Stop'):
		rospy.loginfo("Comamnd received, clear all jobs") 
		robotjob.clear_jobs(job_des, job_num)
	elif (keyboard_data == 'Faster'):
		if(robotdrive.move_speed < 5): 
			robotdrive.move_speed = robotdrive.move_speed + 1  
	elif (keyboard_data == 'Slower'):
		if(robotdrive.move_speed > 3): 
			robotdrive.move_speed = move_speed - 1  
	else: 
		rospy.loginfo(keyboard_data)
		rospy.loginfo("Not recognizing command receivied")

# Real time get compass data 
def compass_callback(data):
	global compass_data
	#update compass_data global variable
	compass_data = int(data.data)
	#rospy.loginfo("compass : %s", data.data)

# The main call back, getting encoder data and make decision for the next move 
def encoder_callback(data):
	#accumulate encoder data
	global r
	global compass_data
	global job_des
	global job_num
	global x_now
	global y_now
	global dist_travelled
	global move_speed
	global move_speed_now

	#Step 1: Get encoder data and convert them to number for later use 
	#Get left encoder and right encoder 
	data_string = data.data
	left_encode, right_encode = data_string.split(" ")

	#convert encoder number to floading point number, make sure all subsquent calculation is on floating point mode 
	left_encode  = float(left_encode)
    	right_encode = float(right_encode)

    	# Step 2: Check whether if there's any job left for the robot
    	# If no jobs, make sure robot stopped moving, we cannot leave robot moving there 
	if(len(job_des) < 1 or len(job_num) < 1):
		#rospy.loginfo('Not any jobs left')
		# Make sure robt stop   
		if(left_encode >=1 or right_encode >=1):
			rospy.logwarn('warning: robot is not fully stopped')
			robotdrive.send_command('S',0)
        	return

     # Step 3: Perform actually turning and moving 
	#Peform turning job 
	job_completed = 0; 
	if (job_des[0] == 'T') : 	#used for temporally disable the truning part  
		#bearing thresholds
		job_completed =robotturn.turn_degree(job_num[0], left_encode, right_encode, robotdrive.move_speed_now, robotdrive.move_speed)
	#FSM moving of dirction
	elif (job_des[0] == 'F' or job_des[0] == 'B') :
		job_completed =robotmove.move_distance(job_num[0], left_encode, right_encode, robotdrive.move_speed_now, robotdrive.move_speed)
	else :
		rospy.logwarn('warning: illegal job description found, not peform any actions')
	
	if job_completed == 1: 
		del job_des[0]
		del job_num[0]

#subscribes to different topic 
def main_listener():
	rospy.init_node('commander')
	rospy.Subscriber('compass', String, compass_callback)
	rospy.Subscriber('encoder', String, encoder_callback)
	rospy.Subscriber('keyboard', String, keyboard_callback)
	rospy.spin()

if __name__ == '__main__':
	try:
		# AAron's initial one for final testing
		#job_generator(initial_bearing, loops)
		#job_generator_move_1m();
		main_listener()
	except rospy.ROSInterruptException:
		pass
