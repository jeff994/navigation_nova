#!/usr/bin/env python
import rospy
import string
import time 
import gpsmath
import json 
import robot_obstacle
import robot_job 
import robot_drive
import robot_move
import robot_turn
import robot_correction 
import robot_publisher
import robot_listener 
import json
from datetime import datetime
from std_msgs.msg import String

#used to hold the encoder data received, init with some value  
encoder_buffer_size = 1000 	# The buffer size for the encoder
encoder_data 		= []	# pairs of encoder data , l, r etc
compass_size 		= 10 	# The compas data buffer size 
compass_data 		= [] 	# Hold the compass dat which [0 - 359] 
# two index indicate the processing and receiving index of the encoder data 
encoder_received	= 0 	# The index of the encoder received 
encoder_processed 	= 0		# The index of the encoder signal processed 
compass_index 		= 0 	# current compass index
last_process_time 	= 0 	# last processing time 
max_delay 			= 1.0	# max delay allowed for not receiving any signal from encooder 
last_received_time 	= 0.0 	# the time of receiving the last encoer data 

#@yuqing_publishparam

# init the the encoder buffer with some empty data when system starts 
def init_encoder_buffer( size=2000 ):
	global encoder_data 
	if(len(encoder_data) == size):
		return 
	for i in range(size - len(encoder_data)):
		encoder_data.append(0)

# init the compass buffer with some empty data when sytem states 
def init_compass_buffer(size = 10):
	global compass_data 
	if(len(compass_data) == size):
		return 
	for i in range(size):
		compass_data.append(0)

# For each encoder signal received, log the time, if the next signal not coming for a long time, need to process 
def process_encoder_delay():
	global last_received_time  
	time_now = datetime.now()
	#rospy.loginfo(time_now)
	#rospy.loginfo("----------------------")
	#rospy.loginfo(last_received_time)
	if(last_received_time != 0): 
		delta = time_now - last_received_time
		delay_seconds = delta.seconds + delta.microseconds / 1000000.0 
		if(delay_seconds >  max_delay):
			bytesToLog = 'Error: Not receiving data for %f seconds: Stopping robot immediately' % (max_delay)
			rospy.logerr(bytesToLog)
			robot_drive.stop_robot()
		else:
			time.sleep(0.05)
	else:
		time.sleep(0.05)

# Very import step, based on the encoder data, we do the conversion and calcuation 
def process_encoder_data():
	global encoder_data 
	global encoder_received
	global encoder_processed
	# convert the encoder data to distance 
	left_encode, right_encode = robot_drive.encoder_to_distance(encoder_data, encoder_received, encoder_processed)
	# After process, update the proccessed index the same as received index 
	encoder_processed = encoder_received
	# dynamically calculate and update the gps data, step_angle, step_distance etc while the robot moving 
	robot_correction.update_robot_gps(left_encode, right_encode)

# Subscriber to keyboard topic and peform actions based on the command get  
def keyboard_callback(data):
	keyboard_data = ''
	keyboard_data = data.data
	robot_drive.speed_now = 6
	robot_drive.desired_speed = 6 
	if (keyboard_data == 'Init'):
		rospy.loginfo("Testing init job")
		robot_job.initialize_job()
	elif (keyboard_data == 'Forward'):
		rospy.loginfo("Command received: Start to move forward 1 m")
		robot_job.simple_move(10000, 0, 'F')
	elif (keyboard_data == 'Back'):
		rospy.loginfo("rospeived: Start to move back 1 m")
		robot_job.simple_move(-10000, 0, 'B')
	elif (keyboard_data == 'Turn_West'):
		rospy.loginfo("Command received: turn to 270 (WEST)") 
		#robot_drive.bearing_now = compass_data[compass_index] 
		robot_job.simple_turn(270)
	elif (keyboard_data == 'Turn_East'): 
		rospy.loginfo('Command received: turn to 90 (EAST)')
		#robot_drive.bearing_now = compass_data[compass_index]
		robot_job.simple_turn(90)
	elif (keyboard_data == 'Stop'):
		rospy.loginfo("Comamnd received: Clear all jobs") 
		robot_job.clear_jobs()
		robot_drive.robot_on_mission =  0
	elif (keyboard_data == "Switch"):
		if(robot_drive.robot_enabled == 1):
			rospy.loginfo("robot disabled")  
			robot_drive.robot_enabled = 0
			robot_drive.init_gps()
		else: 
			robot_drive.init_gps()
			rospy.loginfo("robot enabled")  
			robot_drive.robot_enabled = 1 
	elif (keyboard_data == 'Faster'):
		rospy.loginfo('Command received: Try to increase robot speed')
		if(robot_drive.desired_speed < 6): 
			rospy.loginfo('Robot speed increased')
			robot_drive.desired_speed = robot_drive.desired_speed + 1  
		else:
			rospy.loginfo('Robot speed already maximized')
	elif (keyboard_data == 'Slower'):
		rospy.loginfo('Command received, trying to reduce robot speed')
		if(robot_drive.desired_speed > 3): 
			rospy.loginfo('Robot speed reduced')
			robot_drive.desired_speed = robot_drive.desired_speed - 1  
		else: 
			rospy.loginfo('Robot speed already minimized')
	elif (keyboard_data == "Demo"):
		#robot_drive.bearing_now = 0
		rospy.loginfo("Simple job")
		robot_job.simple_job()
	elif (keyboard_data == "Test"):
		robot_job.job_generator(0)
	elif (keyboard_data == "No_obstacle"):#@yuqing_toggleobstaclemode
		rospy.loginfo('keyboard No_obstacle')
		robot_drive.enter_no_obstacle()
	elif (keyboard_data == "Obstacle"):#@yuqing_toggleobstaclemode
		rospy.loginfo('keyboard Obstacle')
		robot_drive.enter_obstacle()
	elif (keyboard_data == "30m"):#@yuqing_toggleobstaclemode
		rospy.loginfo('forward 30m')
		robot_job.simple_job_move(10000, 'F', robot_drive.lon_now, robot_drive.lat_now, robot_drive.bearing_now)
	elif (keyboard_data == "180"):#@yuqing_toggleobstaclemode
		rospy.loginfo('turn 180')
		robot_job.simple_job_turn(180, robot_drive.lon_now, robot_drive.lat_now)
	elif (keyboard_data == "zero"):#@yuqing_toggleobstaclemode
		rospy.loginfo('turn 0')
		robot_job.simple_job_turn(0, robot_drive.lon_now, robot_drive.lat_now)
	else:
		rospy.loginfo(keyboard_data)
		rospy.loginfo("Not recognizing command receivied")

#@yuqing_obstacledriverread
#read obstacle finish data thro driver node
def driver_obstacle_callback(data):
	string = data.data
	string = string.strip('\r\n')
	rospy.loginfo('driver callback: %s, length: %d ',  string, len(string))
	rospy.loginfo('robot_on_obstacle: %d', robot_obstacle.robot_on_obstacle)
	
	if(string == 'FINISH'):
		if robot_obstacle.robot_on_obstacle > 0: 
			rospy.loginfo('first finish')
			robot_obstacle.obstacle_is_over()
		else: 
			rospy.loginfo('Received one more finish')
			#robot_drive.unlock_robot()
	else:
		#yuqing_obstaclemodeconfirm
		if (string == "LOWER CONTROL"):
			rospy.loginfo("robot enter no obstacle mode")
			robot_drive.obstacle_mode = 0
		elif (string == "UPPER CONTROL"):
			rospy.loginfo("robot enter obstacle mode")
			robot_drive.obstacle_mode = 1
		#yuqing_unlockconfirm
		elif (string == "UNLOCK"):
			robot_drive.isunlockdone = 1
		else:
			index = string.find('OBSTACLE')
			# if current state is no obstacle but recevited obstacel, starts obstacle avidentce 
			if (robot_obstacle.robot_on_obstacle == 0):
				if(index != -1):
					robot_obstacle.start_obstacle_avidence()
					robot_obstacle.needForward = False
					robot_obstacle.justStop = False

			if(index != -1):
				rospy.loginfo('callback: obstacle start %s', string)
				direction = string[-1]
				if ((direction == 'L') or (direction =='R')):
					robot_obstacle.needForward = True
				else:
					robot_obstacle.justStop = True
			else:
					rospy.loginfo("string: %s", string)
	return
# handle the data from the front reverse car sensor
def rc_sensor_f_callback(data):
	str_right = data.data[-5:]
	str_right = str_right.strip('E')
	#@yuqing_obstacledriverread
	#if robot_obstacle.robot_on_obstacle: 
		#if(str_right == 'CESO'):
			#robot_obstacle.obstancle_is_over()
	#else:
	#rc_sensor_front = int(str_right, 16)
	#first, second, third, forth = robot_obstacle.rc_sensor_data(rc_sensor_front)
	#ret = robot_obstacle.is_on_obstacle_avoidence(first, second, third, forth)
	#if ret > 0:
	#	robot_obstacle.start_obstacle_avidence()
	return

# handle the data from the back reverse car sensor
def rc_sensor_b_callback(data):
	str_right = data.data[-5:]
	str_right = str_right.strip('E')
	#@yuqing_obstacledriverread
	#if robot_obstacle.robot_on_obstancle: 
		#if(str_right == 'CESO'):
			#robot_obstacle.obstancle_is_over()
	#else:
	#rc_sensor_front = int(str_right, 16)

	#first, second, third, forth = robot_obstacle.rc_sensor_data(rc_sensor_front)
	#ret = robot_obstacle.is_on_obstacle_avoidence(first, second, third, forth)
	#if ret > 0:
	#	robot_obstacle.start_obstacle_avidence()
	return

# handle the data from the job creator from our website, based on the gps corrdicates provided, 
# Generate a list of jobs 
def job_callback(data):
	json_str = str(data.data)
	# Clear the lontiude and latitude list 
	del robot_job.gps_lon[:]
	del robot_job.gps_lat[:]
	try:
		decoded = json.loads(json_str)
		# pretty printing of json-formatted strin
		list_route  = decoded['route']
		for item in list_route:
			lon = float(item.get(u'lon'))
			lat = float(item.get(u'lat'))
			robot_job.gps_lon.extend([lon])
			robot_job.gps_lat.extend([lat])
		robot_job.clear_jobs()
		# after parsing the gps corrdinates, now generate robot jobs 
		robot_job.job_generator(robot_drive.initial_bearing)
	except (ValueError, KeyError, TypeError):
		rospy.loginfo('JSON format error:')
		rospy.loginfo(json_str)

# Real time get compass data 
def compass_callback(data):
	global compass_data
	global compass_index
	#update compass_data global variable
	compass_data[compass_index] = int(data.data)
	#rospy.loginfo("compass index: %d, angle: %d", compass_index, compass_data[compass_index])
	compass_index = (compass_index + 1) % compass_size

# The main call back, getting encoder data and make decision for the next move 
def encoder_callback(data):
	global encoder_data
	global encoder_received 
	#accumulate encoder data
	#Step 1: Get encoder data and convert them to number for later use 
	#Get left encoder and right encoder 
	data_string = data.data
	left_encode, right_encode = data_string.split(" ")

	left_encode  = int(left_encode)
	right_encode = int(right_encode)
	if(left_encode == 0 or right_encode == 0):
		#rospy.loginfo("encoder 0,0")
		robot_drive.robot_moving = 0
		robot_drive.robot_turning = 0
	elif (left_encode * right_encode > 0):
		robot_drive.robot_moving = 1
		robot_drive.robot_turning = 0
	else:
		robot_drive.robot_turning = 1
		robot_drive.robot_moving =0 		

	index = encoder_received * 2
	encoder_data[encoder_received * 2] = float(left_encode)
	encoder_data[encoder_received * 2 + 1] = float(right_encode)

		#bytesToLog = 'Encoder sequence %d received' % (encoder_received)
		#rospy.loginfo(str(bytesToLog))

	encoder_received = (encoder_received + 1) % 1000
	#convert encoder number to floading point number, make sure all subsquent calculation is on floating point mode 
	#if (robot_drive.robot_on_mission ==1 ):
	#	rospy.loginfo(str(data_string))

# The main progream process the robot logic 
def main_commander():
	# ----------------------------------------------------------------------------------------#
	#  code to check and use the ecnoder data                     							  #
	# ----------------------------------------------------------------------------------------#
	global encoder_received, encoder_processed
	global compass_data, compass_index
	
	# publish parameters to the web server 
	robot_publisher.publish_parameters()

	# Very important error handling: 
	# If Not any new data comming, waiting for next data, 
	# if waiting too long need to issue warning or error	 
	if(encoder_received == encoder_processed):
		process_encoder_delay()
		return
	
	# It process all the encoder data received - regardless robot status etc .  
	# Including dynamically update robot GPS etc 
	process_encoder_data()

	# ----------------------------------------------------------------------------------------#
	#  code to close robot when required	                    							  #
	# ----------------------------------------------------------------------------------------#
	# If robot not enabled, just need to disable the robot 
	if(robot_drive.robot_enabled == 0): 
		robot_job.disable_robot()
		return

	# ----------------------------------------------------------------------------------------#
	#  Codes for obstacle avoidence handling                     							  #
	# ----------------------------------------------------------------------------------------#
	# The flat would be set by hardware, we cannot do anything but blankly calculate the gps coordinates  
	if(robot_obstacle.robot_on_obstacle > 0):
		rospy.loginfo("Robot on obstacle avoidence, please wait") 
		return 

	# Robot obstancle avoidence is over, now resume to normal operation 
	if(robot_obstacle.robot_over_obstacle > 0):
		robot_obstacle.complete_obstacle_avoidence()
		return 

	# ----------------------------------------------------------------------------------------#
	#  Codes for robot normal jobs like walking and turning       							  #
	# ----------------------------------------------------------------------------------------#
	# Check whether if there's any job left for the robot
	# If no jobs, make sure robot stopped moving, we cannot leave robot moving there 
	if robot_job.has_jobs_left():
		robot_job.process_no_job()
		return

	job_completed = robot_job.process_job()
	
	# ----------------------------------------------------------------------------------------#
	#  Robot's doing the initialization jobs, not normal jobs      							  #
	# ----------------------------------------------------------------------------------------# 
	if robot_drive.robot_initialized == 0:
		robot_job.complete_init_compass(compass_data[compass_index])
		return

	# ----------------------------------------------------------------------------------------#
	#  Error compensation after current job completed      									  #
	# ----------------------------------------------------------------------------------------#
	if job_completed == 1:
		robot_job.complete_current_job()
		#robot_correction.angle_correction()
		robot_correction.distance_correction()

#subscribes to different topic 
def main_listener():
	rospy.init_node('commander')
	rospy.Subscriber('compass', String, compass_callback)
	rospy.Subscriber('encoder', String, robot_listener.encoder_callback)
	rospy.Subscriber('keyboard', String, keyboard_callback)
	rospy.Subscriber('rc_sensor_f', String, rc_sensor_f_callback)
	rospy.Subscriber('rc_sensor_b', String, rc_sensor_b_callback)
	#@yuqing_obstacledriverread
	rospy.Subscriber('driver_obstacle', String, driver_obstacle_callback)
	rospy.Subscriber('job', String, job_callback)
	while not rospy.is_shutdown():
		main_commander()
	rospy.spin()

if __name__ == '__main__':
	try:
		# AAron's initial one for final testing
		#job_generator(initial_bearing, loops)
		init_encoder_buffer()
		init_compass_buffer()
		main_listener()
	except rospy.ROSInterruptException:
		pass
