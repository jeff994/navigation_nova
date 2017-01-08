#!/usr/bin/env python
import rospy
import string
import robot_job 
import robot_drive
import robot_move
import robot_turn
import time 
import robot_correction 
import gpsmath
import json 
import robot_obstacle

from datetime import datetime

from std_msgs.msg import String

#used to hold the encoder data received, init with some value  
buffer_size = 1000 
#pairs of encoder data , l, r, l, r etc
encoder_data = []

compass_size = 10 
compass_data = [] 

# two index indicate the processing and receiving index of the encoder data 
encoder_received = 0  # value range is from 0 - buffer_size -  
encoder_processed = 0
compass_index = 0 	#current compass index

last_process_time = 0 #last processing time 
max_delay = 1.0
last_received_time = 0.0 
robot_moving = 0

def init_encoder_buffer( size=2000 ):
    global encoder_data 
    if(len(encoder_data) == size):
    	return 
    for i in range(size - len(encoder_data)):
        encoder_data.append(0)

def init_compass_buffer(size = 10):
	global compass_data 
	if(len(compass_data) == size):
		return 
	for i in range(size):
		compass_data.append(0)

# Subscriber to keyboard topic and peform actions based on the command get  
def keyboard_callback(data):
	keyboard_data = '';
	keyboard_data = data.data
	robot_drive.speed_now = 5
	robot_drive.desired_speed = 5 
	if (keyboard_data == 'Forward'):
		rospy.loginfo("Command received: Start to move forward 1 m")
		robot_job.generate_move(1000, 'F')
	elif (keyboard_data == 'Back'):
		rospy.loginfo("rospeived: Start to move back 1 m")
		robot_job.generate_move(-1000, 'B')
	elif (keyboard_data == 'Turn_West'):
		rospy.loginfo("Command received: turn to 270 (WEST)") 
		#robot_drive.bearing_now = compass_data[compass_index] 
		robot_job.generate_turn(270)
	elif (keyboard_data == 'Turn_East'): 
		rospy.loginfo('Command received: turn to 90 (EAST)')
		#robot_drive.bearing_now = compass_data[compass_index]
		robot_job.generate_turn(90)
	elif (keyboard_data == 'Stop'):
		rospy.loginfo("Comamnd received: Clear all jobs") 
		robot_job.clear_jobs()
		robot_drive.robot_on_mission =  0
	elif (keyboard_data == "Switch"):
		if(robot_drive.robot_enabled == 1):
			rospy.loginfo("robot disabled")  
			robot_drive.robot_enabled = 0
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
		robot_job.simple_job(); 
	elif (keyboard_data == "Test"):
		robot_job.job_generator(0)
	else:
		rospy.loginfo(keyboard_data)
		rospy.loginfo("Not recognizing command receivied")


# handle the data from the front reverse car sensor
def rc_sensor_f_callback(data):
	str_right = data.data[-4:]
	if robot_obstacle.robot_on_obstacle: 
		if(str_right == 'CESO'):
			robot_obstacle.obstancle_is_over()
	else:
		rc_sensor_front = int(str_right, 16)
		first, second, third, forth = robot_obstacle.rc_sensor_data(rc_sensor_front)
		ret = robot_obstacle.is_on_obstacle_avidence(first, second, third, forth)
		if ret > 0:
			robot_obstacle.start_obstacle_avidence()
	return

# handle the data from the back reverse car sensor
def rc_sensor_b_callback(data):
	str_right = data.data[-4:]
	if robot_obstacle.robot_on_obstancle: 
		if(str_right == 'CESO'):
			robot_obstacle.obstancle_is_over()
	else:
		rc_sensor_front = int(str_right, 16)

		first, second, third, forth = robot_obstacle.rc_sensor_data(rc_sensor_front)
		ret = robot_obstacle.is_on_obstacle_avidence(first, second, third, forth)
		if ret > 0:
			robot_obstacle.start_obstacle_avidence()
	return

# handle the data from the job creator from our website, based on the gps corrdicates provided, 
# Generate a list of jobs 
def job_callback(data):
	json_str = data.data
	rospy.loginfo(json_str)
	# Clear the lontiude and latitude list 
	robot_drive.robot_enabled = 0
	del robot_job.gps_lon[:]
	del robot_job.gps_lat[:]
	try:
		decoded = json.loads(json_str)
	    	# pretty printing of json-formatted strin
	    	list_route  = decoded['route']
	    	for k in range(len(list_route)):
	 		gps_pair = list_route[k]
	    		lon = float(gps_pair['lon'])
	    		lat = float(gps_pair['lat'])
	    		robot_job.gps_lon.extend(lon)
	    		robot_job.gps_lat.extend(lat)
		robot_drive.clear_jobs()
		# after parsing the gps corrdinates, now generate robot jobs 
		robot_drive.job_generator(robot_drive.initial_bearing)
	except (ValueError, KeyError, TypeError):
    		print "JSON format error"
	return 

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
	global robot_moving 
	#accumulate encoder data
	#Step 1: Get encoder data and convert them to number for later use 
	#Get left encoder and right encoder 
	data_string = data.data
	left_encode, right_encode = data_string.split(" ")

	left_encode  = int(left_encode)
    	right_encode = int(right_encode)
	if(left_encode == 0 and right_encode == 0):
		robot_moving = 0
	else:
		robot_moving = 1 		

    	index = encoder_received * 2
    	encoder_data[encoder_received * 2] = float(left_encode)
    	encoder_data[encoder_received * 2 + 1] = float(right_encode)

    	#bytesToLog = 'Encoder sequence %d received' % (encoder_received)
    	#rospy.loginfo(str(bytesToLog))

	encoder_received = (encoder_received + 1) % 1000
	#convert encoder number to floading point number, make sure all subsquent calculation is on floating point mode 
	#if (robot_drive.robot_on_mission ==1 ):
	#	rospy.loginfo(str(data_string))

def process_encoder_delay():
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
	
def process_no_job(left_encode, right_encode):
	robot_drive.robot_on_mission = 0
	if(left_encode !=0 or right_encode !=0):
		rospy.logwarn('warning: robot is not fully stopped even though a top command issued')
		robot_drive.stop_robot()
		time.sleep(0.05)
		return

def process_encoder_data(encoder_received, encoder_processed):
	global encode_data 
	left_encode = 0
	right_encode = 0
	#in case data received is faster than the processing time 
    	if(encoder_received > encoder_processed): 
    		for x in range(encoder_processed, encoder_received):
    			left_encode += encoder_data[2 * x]
    			right_encode += encoder_data[2 * x +1]

   	if(encoder_received < encoder_processed): 
   		for x in range(encoder_processed, 1000):
   			left_encode 	+= encoder_data[2 * x]
    			right_encode 	+= encoder_data[2 * x + 1]
    		for x in range(0, encoder_received):
    			left_encode 	+= encoder_data[2 * x]
    			right_encode 	+= encoder_data[2 * x + 1]
	return left_encode, right_encode 

def disable_robot():
	global encoder_data 
	global encoder_received
	global encoder_processed
	global robot_moving
 
	while True:
		#step 1, send the stop command every 10 milli seoncs
		if(robot_moving == 1):
			robot_drive.stop_robot()
			time.sleep(0.01)
		else: 
			# Clear all the reamining jobs
			robot_job.clear_jobs()
			#robot_drive.stop_robot()
			break; 
			#robot_drive.robot_enabled = 1

def main_commander():
	#rospy.loginfo("starting main commander")

	global encoder_data 
	global encoder_received
	global encoder_processed
	global last_received_time  

	job_completed = 0 
	left_encode = 0
	right_encode = 0 
	
	# Not any new data comming, waiting for next data, if waiting too long need to issue warning or error	 
	#bytsToLog = "encoder received %d, processed %d" % (encoder_received,encoder_processed)
	#rospy.loginfo(bytsToLog)
	if(encoder_received == encoder_processed):
		process_encoder_delay()
		return
	
	# get new data received 
	#last_received_time = datetime.now()
	#rospy.loginfo(str(last_received_time))
	#rospy.loginfo("Processing encoder delay %d, %d", encoder_received, encoder_processed)

	# calculate the correct encode data for further proces 
	#rospy.loginfo("Processing encoder data")
	left_encode, right_encode = process_encoder_data(encoder_received, encoder_processed)
	
	encoder_processed = encoder_received
	
	robot_correction.update_robot_gps(left_encode, right_encode)

	# add a handle to stop the robot from current task 
	if(robot_drive.robot_enabled == 0): 
		disable_robot()
		return

	# The flat would be set by hardware, we cannot do anything but blankly calculate the gps coordinates  
	if(robot_obstacle.robot_on_obstacle > 0):
		rospy.loginfo("Robot on obstacle avoidence, please wait"); 
		return 

	if(robot_obstacle.robot_over_obstacle > 0):
		# First get ready the robot for normal walking  
		robot_obstacle.unlock_from_obstacle()
		# Need to perform necessary correction 
		rospy.loginfo("Robot on obstacle avoidence, please wait"); 
		return 

	# Check whether if there's any job left for the robot
    	# If no jobs, make sure robot stopped moving, we cannot leave robot moving there 
	if(len(robot_job.job_des) < 1 or len(robot_job.job_num) < 1):
		process_no_job(left_encode, right_encode)
		return		
     	   	 
    	if (robot_job.job_des[0] == 'T') : 
    		#rospy.loginfo("Bearing now %f, bearing target %f", robot_drive.bearing_now, robot_drive.bearing_target)
		#if(robot_drive.robot_on_mission == 0): 
		robot_drive.bearing_target  = robot_job.job_num[0]
		# Pre-steps of turning jobs starts: calculate the required angle to turn 
		# start the job 
		job_completed =robot_turn.turn_degree(compass_data[compass_index], left_encode, right_encode)
	
	elif (robot_job.job_des[0] == 'F' or robot_job.job_des[0] == 'B'):
		if(robot_job.job_des[0] == 'B'):
			robot_job.job_num[0] = -  abs(robot_job.job_num[0])
		job_completed =robot_move.move_distance(robot_job.job_num[0], left_encode, right_encode) 
	else :
		rospy.logwarn('warning: illegal job description found, not peform any actions')

	if job_completed == 1: 
		robot_job.remove_current_job()
		#robot_correction.angle_correction()
		robot_correction.distance_correction()

#subscribes to different topic 
def main_listener():
	rospy.init_node('commander')
	rospy.Subscriber('compass', String, compass_callback)
	rospy.Subscriber('encoder', String, encoder_callback)
	rospy.Subscriber('keyboard', String, keyboard_callback)
	rospy.Subscriber('rc_sensor_f', String, rc_sensor_f_callback)
	rospy.Subscriber('rc_sensor_b', String, rc_sensor_b_callback)
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
