#!/usr/bin/env python
import rospy
import serial
import string
import math 

from std_msgs.msg import String
from math import radians, cos, sin, asin, sqrt, atan2, degrees

###################### EDIT HERE ###########################
#defining or acquiring the GPS coordinates
gps_num = 4
start = [103.962386,1.340549]
gps_lon = [103.962386,103.962389,103.962456,103.962461,103.962381] #S,A,B,C,D
gps_lat = [1.340549,1.3407,1.340696,1.340589,1.340599]

initial_bearing = 0 	#set as north for now
loops = 1 		#how many rounds to go
robot_on_mission = 0	#set an indicator that robot's on a job right now 
encode_to_mm = 68.3 	#1000 encoding signals = 1 mm travelled
turn_radius = 312 	#radius when turning in mm (half distance between the middle point of two wheels) 

############################################################

keyboard_data  = ''	#keyboard control data
move_direction ='F' # robot moving direction: F or B, F for forwarding, B for back 
move_speed = 3		# Global robot moving spped, 3 - 5
turn_direction='L'  # Global robot turning direction

compass_data = 0	#degrees, true north is 0 degrees
dist_travelled = 0	#mm
degree_turned = 0 	#angle inturns of degree

move_speed_now = 3;	# Robot moving speed now
x_now = 0  		#mm
y_now = 0		#mm
r = 350 		#mm, distance between center of robot to wheel
x_target = 0		#mm
y_target = 0 		#mm, should always be 0, because we will be moving in a straight line
bearing_target = 0 	#degrees
job_des = []		#could be 'T' or 'F'
job_num = []		#if job is 'T', the number is the angle of robot need to face of the job else it's the distance in mm 

#defining serial port to write to (the commands)
ser = serial.Serial()
#real robot port
ser.port = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Uno_75435363138351A09171-if00"
#testing port
#ser.port = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Uno_75439333335351412220-if00"
ser.baudrate = 9600
ser.open()

# Calculate distance between two gps coordinates 
def haversine(lon1, lat1, lon2, lat2):
	#convert to radians
	lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1,lon2, lat2])
	#haversine
	dlon = lon2 - lon1
	dlat = lat2 - lat1
	a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
	c = 2 * asin(sqrt(a))
	r = 6371 #radius of earth in kilometers

	return c * r

# Calculate angle between two diffent gps positions 
def bearing(lon1, lat1, lon2, lat2):  #from position 1 to 2
	#convert to radians
	lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])
	
	bearing = atan2(sin(lon2-lon1)*cos(lat2), cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(lon2-lon1))
	bearing = degrees(bearing)
	bearing = (bearing + 360) % 360

	return bearing

#based on the gps coordinates of the two location, generate jobs
def job_details(first_point, second_point):
	global gps_lon
	global gps_lat
	#handles from first_point to second_point
	distance = haversine(gps_lon[first_point],gps_lat[first_point],gps_lon[second_point],gps_lat[second_point]) #km
	angle_next = bearing(gps_lon[first_point],gps_lat[first_point],gps_lon[second_point],gps_lat[second_point]) #deg  
		
	#handles turning angle		
	#turn_angle = angle_next - angle_now
	#if turn_angle > 180.0 :
	#	turn_angle = turn_angle - 360.0
		
	#handles forward distance in mm
	distance = distance * 1000.0 * 1000.0

	return ([round(angle_next), distance])

# Clear jobs 
def clear_jobs():
	global job_des 
	global job_num
	del job_des[:]
	del job_num[:]

#Generate differnet jobs 
def job_generator_move_1m(direction):
	global job_des
	global job_num
	global move_direction 

	move_direction = direction
	job_num.extend([1000]) 
	job_des.extend([move_direction])

def job_generator_turn_90_left():
	global job_des
	global job_num
	job_num.extend([-90]) 
	job_des.extend(['T'])

def job_generator_turn_90_right():
	global job_des
	global job_num
	job_num.extend([90]) 
	job_des.extend(['T'])

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
	keyboard_data = data.data
	if (keyboard_data == 'Forward'):
		rospy.loginfo("Command received: Start to move forward 1 m")
		job_generator_move_1m('F')
	if (keyboard_data == 'Back'):
		rospy.loginfo("Command received: Start to move forward 1 m")
		job_generator_move_1m('B')
	elif (keyboard_data == 'Turn_Left'):
		rospy.loginfo("Left turn received"); 
		job_generator_turn_90_left('B')
	elif (keyboard_data == 'Turn_Right'): 
		rospy.loginfo('Right turn received')
		job_generator_turn_90_right() 
	elif (keyboard_data == 'Stop'):
		rospy.loginfo("Comamnd received, clear all jobs") 
		clear_jobs()
	elif (keyboard_data == 'Faster'):
		if(move_speed < 5): 
			move_speed = move_speed + 1  
	elif (keyboard_data == 'Slower'):
		if(move_speed > 3): 
			move_speed = move_speed - 1  
	else: 
		rospy.loginfo(keyboard_data)
		rospy.loginfo("Not recognizing command receivied")

# Real time get compass data 
def compass_callback(data):
	global compass_data
	#update compass_data global variable
	compass_data = int(data.data)
	#rospy.loginfo("compass : %s", data.data)

#-------------------------------------------------------#
#	Robot moving module									#
#-------------------------------------------------------#
# Roboet complet a moving job 
def complete_move_job():
	global dist_travelled 
	global job_num
	global job_des
	global robot_on_mission 	
	dist_travelled = 0
	robot_on_mission = 0 
	del job_des[0]
	del job_num[0]
	rospy.loginfo('Robot completed a moving job')

def start_move_job():
	global dist_travelled 
	global robot_on_mission 
	global move_direction
	global move_speed
	rospy.loginfo('Robot moving job started')
	robot_on_mission = 1 
	dist_travelled = 0
	send_command(move_direction, move_speed)

def update_move_speed()
	global move_direction
	global move_speed
	global move_speed_now 
	send_command(move_direction, move_speed)
	move_speed_now = move_speed
	
def move_distance(dist_to_run, left_encode, right_encode):
	global dist_travelled 
	global job_num
	global job_des
	global robot_on_mission 
	global move_speed 
	global move_speed_now

	# if robot received a meaning less job, just signal, clear the job and return 
	if (dist_to_run <= 0):
		rospy.loginfo('Robot received a meaning less moving job')
		complete_move_job()
		return 

	# Mission started, let robot start moving 
	if(robot_on_mission == 0): 
		start_move_job()
		return
	# Exception handling, make sure robot wheels is moving the same direction 
	if(left_encode > 10 and right_encode < 10):
		rospy.loginfo('Robot wheel not moving as expected, step current job')
		complete_move_job()
		return

	# Accumulate the running distance and check 
	dist = (left_encode + right_encode)/(2.0 * encode_to_mm)
	dist_travelled = dist_travelled + dist   #this is in mm
	#distance travelled threshold
	dist_threshold = dist_to_run - 0 	#0 mm, I can choose -50mm, but since there will be inefficiencies, 0 error threshold might be good enough
	
	distpub = 'dist-travelled: %f dist-total:%f dist-step:%f' % (dist_travelled,dist_threshold,dist)
        rospy.loginfo(distpub)

	if (dist_threshold - dist_travelled > 2) :
		#just continue moving 
		
		if(move_speed  == move_speed_now):
			rospy.loginfo('Still moving at the same speed...')
		else :
			update_move_speed()
		return
	else :
		complete_move_job()
        	send_command('S',0)
       	 	#clean current job 
       	 	
#-------------------------------------------------------#
# Robot turning module 									#
#-------------------------------------------------------#
# clear a turn job 
def complete_turn_job():
	global degree_turned 
 	global job_des 
 	global job_num
 	global robot_on_mission
	del job_des[0]
	del job_num[0]
	robot_on_mission = 0
	degree_turned = 0
	rospy.loginfo('Robot completed a turn job')

# start a turn job 
def start_turn_job():
	rospy.loginfo('Robot starts to execute a turn job')
	global degree_turned 
	global robot_on_mission
	global move_speed
	robot_on_mission = 1
	degree_turned = 0
	send_command(turn_direction, move_speed)

def update_turn_speed()
	global turn_direction
	global move_speed
	send_command(turn_direction, move_speed)
	move_speed_now = move_speed


# let robot performs a turning job of certain degree 
def turn_degree(degree, left_encode, right_encode): 
	global turn_radius
	global degree_turned 
 	global job_des 
 	global job_num
 	global robot_on_mission
 	global turn_direction

 	if (degree < 0): #ccw turning 
 		turn_direction = 'R'
 	else:
 		turn_direction = 'L'

 	# The degree passed is not correct, just log and return 
	if(degree == 0): 
		#No turn is required, clear current job and rerun 
		rospy.loginfo('Robot has been assigned a meaning less 0 degree turn task')
		complete_turn_job()
		return

	#robot has not started turning, just start the turning 
	if(robot_on_mission == 0):
		start_turn_job()
		return

	if((left_encode > 10 and right_encode > 10) or (left_encode < -10 and right_encode < -10)): 
		rospy.loginfo('Robot wheel not moving as expected, clear current task')
		complete_turn_job()
		return

	#Get the turned angle and then calculate 
	distance = (abs(left_encode) + abs(right_encode))/(2.0 * encode_to_mm)
	step_angle = (360 * distance) / (math.pi * 2 * turn_radius)   
	degree_turned = degree_turned + step_angle
		
	#simple log for tracing 
	distpub = 'Required angle:%f turned angle:%f' % (degree, degree_turned)
        rospy.loginfo(distpub)


	if(degree_turned < abs(degree)): 
		#continure turning and no need issue new command
		if(move_speed == move_speed_now) 
			rospy.loginfo('Continue turning at same speed...')
		else 

		return;
	else: 
		#finishe the turning 
		complete_turn_job()
		send_command('S',0)
		return

def turn():
	global compass_data
	global job_num
	global job_des
	

	high_threshold = (job_num[0] + 2 + 360) % 360
	low_threshold = (job_num[0] - 2 + 360) % 360
	
	if (compass_data != low_threshold or compass_data != high_threshold or compass_data != job_num[0]) : #boundary of plus minus 1 degree
		#it is still outside the boundary, continue turning
		d_angle = job_num[0] - compass_data
		if (d_angle > 0) :
			if (d_angle > 180) :
				send_command('L',4)     #test with slowest speed first
			else :
				send_command('R',4)		#after testing will use speed feedback
		elif (d_angle < 0) :
			if (d_angle < -180) :
				send_command('R',4)
			else :
				send_command('L',4)
	#once turn till target, delete job, considered job done
	if (compass_data == low_threshold or compass_data == high_threshold or compass_data == job_num[0]) :
		send_command('S',0);
		del job_des[0]
		del job_num[0]

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

	#Step 1: Get encoder data and convert them to number for later use 
	#Get left encoder and right encoder 
	data_string = data.data
	left_encode, right_encode = data_string.split(" ")

	#convert encoder number to floading point number, make sure all subsquent calculation is on floating point mode 
	left_encoder_n  = float(left_encode)
    	right_encoder_n = float(right_encode)

    	# Step 2: Check whether if there's any job left for the robot
    	# If no jobs, make sure robot stopped moving, we cannot leave robot moving there 
	if(len(job_des) < 1 or len(job_num) < 1):
		#rospy.loginfo('Not any jobs left')
		# Make sure robt stop   
		if(left_encoder_n >=1 or right_encoder_n >=1):
			rospy.logwarn('warning: robot is not fully stopped')
			send_command('S',0)
        	return

     # Step 3: Perform actually turning and moving 
	#Peform turning job 
	if (job_des[0] == 'T') : 	#used for temporally disable the truning part  
		#bearing thresholds
		turn_degree(job_num[0], left_encoder_n, right_encoder_n)
	#FSM moving of dirction
	elif (job_des[0] == 'F' or job_des[0] == 'B') :
		move_distance(job_num[0], left_encoder_n, right_encoder_n)
	else 
		rospy.logwarn('warning: illegal job description found, not peform any actions')

# Helper function which can send commands to robot 
def send_command(command_string, speed):
	global job_des
	global job_num
	#handle the format of the string
	stringToSend = 'S%s00000%dE\n' % (command_string, speed) #might need to add \n behind the E
	#sending the string
	#rospy.loginfo(str(stringToSend))
	if ser.isOpen():
		ser.write(stringToSend)
	else:
		rospy.loginfo("Commanding Serial port not connected")

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
