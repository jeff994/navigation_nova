#!/usr/bin/env python
import rospy
import string
import robot_job 
import robot_drive
import robot_move
import robot_turn

from std_msgs.msg import String

# Subscriber to keyboard topic and peform actions based on the command get  
def keyboard_callback(data):
	keyboard_data = data.data
	if (keyboard_data == 'Forward'):
		rospy.loginfo("Command received: Start to move forward 1 m")
		robot_job.generate_move(1000, 'F')
	if (keyboard_data == 'Back'):
		rospy.loginfo("Command received: Start to move back 1 m")
		robot_job.generate_move(-1000, 'B')
	elif (keyboard_data == 'Turn_Left'):
		rospy.loginfo("Left turn received") 
		robot_job.generate_turn(-90)
	elif (keyboard_data == 'Turn_Right'): 
		rospy.loginfo('Right turn received')
		robot_job.generate_turn(90)
	elif (keyboard_data == 'Stop'):
		rospy.loginfo("Comamnd received, clear all jobs") 
		robot_job.clear_jobs()
	elif (keyboard_data == 'Faster'):
		if(robot_drive.move_speed < 5): 
			robot_drive.desired_speed = robot_drive.desired_speed + 1  
	elif (keyboard_data == 'Slower'):
		if(robot_drive.move_speed > 3): 
			robot_drive.desired_speed = robot_drive.desired_speed - 1  
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
	#Step 1: Get encoder data and convert them to number for later use 
	#Get left encoder and right encoder 
	data_string = data.data
	left_encode, right_encode = data_string.split(" ")

	#convert encoder number to floading point number, make sure all subsquent calculation is on floating point mode 
	left_encode  = float(left_encode)
    	right_encode = float(right_encode)

    	# Step 2: Check whether if there's any job left for the robot
    	# If no jobs, make sure robot stopped moving, we cannot leave robot moving there 
	if(len(robot_job.job_des) < 1 or len(robot_job.job_num) < 1):
		#rospy.loginfo('Not any jobs left')
		# Make sure robt stop   
		if(left_encode >=1 or right_encode >=1):
			rospy.logwarn('warning: robot is not fully stopped')
			robot_drive.send_command('S',0)
        	return

     # Step 3: Perform actually turning and moving 
	#Peform turning job 
	job_completed = 0 
	if (robot_job.job_des[0] == 'T') : 	#used for temporally disable the truning part  
		#bearing thresholds
		job_completed =robot_turn.turn_degree(robot_job.job_num[0], left_encode, right_encode)
	#FSM moving of dirction
	elif (robot_job.job_des[0] == 'F' or robot_job.job_des[0] == 'B') :
		job_completed =robot_move.move_distance(robot_job.job_num[0], left_encode, right_encode)
	else :
		rospy.logwarn('warning: illegal job description found, not peform any actions')
	
	if job_completed == 1: 
		robot_job.remove_current_job()

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
		#job_generator_move_1m()
		main_listener()
	except rospy.ROSInterruptException:
		pass
