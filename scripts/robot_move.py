#!/usr/bin/env python
import rospy
import serial
import string
from std_msgs.msg import String
import robot_drive
import robot_correction 
import gpsmath
import robot_job 

#-------------------------------------------------------#
#	Robot moving module									#
#-------------------------------------------------------#

dist_completed 			= 0.0
dist_to_run 			= 0.0
#@yuqing_correctionper10m
dist_to_correct 		= 10000.0
dist_lowest_speed 		= 500.0
dist_lower_speed 		= 1000.0
dist_end_point_check 	= 600.0 #was 600

dist_4_speed 			= 2000.0
dist_3_speed 			= 250.0
dist_2_speed 			= 250.0


status_pub = rospy.Publisher('status', String, queue_size = 100)
# Starts the robot for moving, put the control variables into proper value 
def start_move():
	global dist_completed
	global dist_to_run 
	global dist_lowest_speed
	global dist_lower_speed

	global dist_4_speed
	global dist_3_speed
	global dist_2_speed

	# if the task is a short distance task, then start with a lower speed 
	if abs(dist_to_run) < dist_3_speed:
		robot_drive.speed_now 		= 2
		robot_drive.speed_desired 	= 2
	elif abs(dist_to_run) < dist_lowest_speed:
		robot_drive.speed_now  		= 3 
		robot_drive.speed_desired 	= 3
	elif abs(dist_to_run) < dist_lower_speed: 
		robot_drive.speed_now  		= 4
		robot_drive.speed_desired 	= 4 
   	else:
		robot_drive.speed_now 		= 5 
		robot_drive.speed_desired 	= 5 

	#changed by aaron to handle slow communication
	#if abs(dist_to_run) < dist_2_speed:
	#	robot_drive.speed_now 		= robot_drive.speed_2
	#	robot_drive.speed_desired 	= robot_drive.speed_2
	#elif abs(dist_to_run) < dist_3_speed:
	#	robot_drive.speed_now 		= robot_drive.speed_3
	#	robot_drive.speed_desired 	= robot_drive.speed_3
	#elif abs(dist_to_run) < dist_4_speed:
	#	robot_drive.speed_now 		= robot_drive.speed_4
	#	robot_drive.speed_desired 	= robot_drive.speed_4
	#else:
	#	robot_drive.speed_now 		= robot_drive.speed_6
	#	robot_drive.speed_now 		= robot_drive.speed_6

    # only if the robot starts to move then change the status
	if robot_drive.robot_moving:
		robot_drive.robot_on_mission = True 
		dist_completed = 0.0
		rospy.loginfo('Started a moving job')
	else:
	  	robot_drive.start()
		rospy.loginfo('Starting a moving job')

# Roboet complet a moving job 
def stop_move():	
	global dist_completed
	if not robot_drive.robot_moving :
		dist_completed = 0.0
		robot_drive.robot_on_mission = False
		rospy.loginfo('Robot completed a moving job')
	else:
		rospy.loginfo('Robot still moving, stopping robot')
		robot_drive.stop_robot()

# Update robot speed as required new speed 
def continue_move():
	global dist_to_run
	global dist_completed
	global dist_lowest_speed
	global dist_lower_speed
	global dist_3_speed

	# if robot's on mission and somehow it's stopped, need to restart the robot 
	if not robot_drive.robot_moving:
        	rospy.loginfo("Robot stopped during the mission, start to move again")
        	robot_drive.start()

    # if robot is approaching the destination, then need to decrease speed 
	if (abs(dist_to_run) - abs(dist_completed) < dist_3_speed):
		robot_drive.speed_desired = 2
		rospy.loginfo('Reduce speed to 2, very close to target position')
	elif(abs(dist_to_run) - abs(dist_completed) < dist_lowest_speed):
		robot_drive.speed_desired = 3
		rospy.loginfo('Reduce speed to 3, very close to target position')
	elif(abs(dist_to_run) - abs(dist_completed) < dist_lower_speed):
		robot_drive.speed_desired = 4
		rospy.loginfo('Reduce speed to 4 only 20 cm to target position')

	#if (abs(dist_to_run) - abs(dist_completed) < dist_2_speed):
	#	#robot_drive.speed_now 		= robot_drive.speed_2
	#	robot_drive.speed_desired 	= robot_drive.speed_2
	#elif (abs(dist_to_run) - abs(dist_completed) < dist_3_speed):
	#	#robot_drive.speed_now 		= robot_drive.speed_3
	#	robot_drive.speed_desired 	= robot_drive.speed_3
	#elif (abs(dist_to_run) - abs(dist_completed) < dist_4_speed):
	#	#robot_drive.speed_now 		= robot_drive.speed_4
	#	robot_drive.speed_desired 	= robot_drive.speed_4
	#else:
	#	#robot_drive.speed_now 		= robot_drive.speed_6
	#	robot_drive.speed_now 		= robot_drive.speed_6


    # if change of speed request is received 
	if(robot_drive.speed_now  != robot_drive.speed_desired):
        	robot_drive.change_speed()

# main function to control the robot movement 
def move_distance(dist):
	global dist_completed
	global dist_to_run 

	dist_to_run = dist
	# if robot received a meaning less job, just signal, clear the job and return 
	if (abs(dist_to_run) < robot_correction.min_correction_distance):
		stop_move()
		rospy.logwarn('Robot received a meaningless moving job')
		return True

	#check the move direction 
	if (dist_to_run < 0.0):
		robot_drive.move_direction = 'B'
	else: 
		robot_drive.move_direction = 'F'

	# Mission started, let robot start moving 
	if not robot_drive.robot_on_mission:
		rospy.loginfo("robot move start move") 
		start_move()
		return False

	# Accumulate the running distance and check 
	# Get each step of the distance 
	dist_step = robot_drive.step_distance
	# accumulate the distance to the completed distance 
	dist_completed = dist_completed + abs(dist_step)   #this is in mm
	# robot is with in the range, then we condidered robot reached the position 
	dist_threshold = abs(dist_to_run) - robot_correction.min_correction_distance/2 	#0 mm, I can choose -50mm, but since there will be inefficiencies, 0 error threshold might be good enough
	
	distpub = 'Dist-travelled: %f dist-total:%f dist-step:%f' % (dist_completed, abs(dist_to_run) ,dist_step)
	rospy.loginfo(distpub)

	#@yuqing_correctionper10m
	#if travel over 10m, job_completed to 1, start to correct
	if (dist_completed >= dist_to_correct):
		rospy.loginfo("-----------------dist_completed: %f, start to correct", dist_completed)
		stop_move()
		return not robot_drive.robot_on_mission
	
	dist_remain = dist_threshold - dist_completed;
	# very near to the target position, we could check whether the robot is closer to the position or the remaining value to move
	# If it's closer, we just move the the position 
	if(abs(dist_remain) < dist_end_point_check):
		dist_temp = robot_job.left_gps_distance()
		if(dist_temp < abs(dist_remain)):  # Robot is closer to end gps position then calculated task distance ##aaron: dist_remain less than 0 if overshoot
			stop_move()
			return not robot_drive.robot_on_mission
		else:
			continue_move()

	if (dist_remain > 0.0) :
		#just continue moving of job not completed and no change of speed command received 
		#if speed changed, then just change the move speed 
		continue_move() 
		return  False
	else :
		stop_move()
        # make sure the robot is stopped before next job
        return not robot_drive.robot_on_mission 
       	#clean current job 
	return False 
