#!/usr/bin/env python
import rospy
import serial
import string
from std_msgs.msg import String
import robot_drive
import robot_correction 
import gpsmath

#-------------------------------------------------------#
#	Robot moving module									#
#-------------------------------------------------------#

dist_completed = 0.0
dist_to_run = 0.0
#@yuqing_correctionper10m
dist_to_correct = 10000.0

status_pub = rospy.Publisher('status', String, queue_size = 100)
# Starts the robot for moving, put the control variables into proper value 
def start_move():
	global dist_completed
	global dist_to_run 

	if abs(dist_to_run) < 100:
		robot_drive.speed_now  = 4
        	robot_drive.desired_speed = 4
	elif abs(dist_to_run) < 200: 
    		robot_drive.speed_now  = 5
        	robot_drive.desired_speed = 5
        else:
		robot_drive.speed_now = 6
		robot_drive.desired_speed = 6

	rospy.loginfo('Start a move job')

    # only if the robot starts to move then change the status
	if(robot_drive.robot_moving == 1):
   		robot_drive.robot_on_mission = 1 
   	    dist_completed = 0
        rospy.loginfo('Robot moving job started')
    else:
	   robot_drive.start()

# Roboet complet a moving job 
def stop_move():	
	global dist_completed
	if robot_drive.robot_moving == 0:
        	dist_completed = 0
        	robot_drive.robot_on_mission = 0
        	rospy.loginfo('Robot completed a moving job')
	else:
        	rospy.loginfo('Robot still moving, stopping robot')
        	robot_drive.stop_robot()

# Update robot speed as required new speed 
# Update robot speed as required new speed 
def continue_move():
    global dist_to_run
    global dist_completed

    if robot_drive.robot_moving == 0:
        rospy.loginfo("Robot stopped during the mission, start to move again")
        robot_drive.start()

    if(abs(dist_to_run) - abs(dist_completed) < 50):
            robot_drive.desired_speed = 4
            rospy.loginfo('Reduce speed to 4, very close to target position')
    elif(abs(dist_to_run) - abs(dist_completed) < 200):
            robot_drive.desired_speed = 5
            rospy.loginfo('Reduce speed to 5, only 20 cm to target position')

    if(robot_drive.speed_now  == robot_drive.desired_speed):
            rospy.loginfo('Still moving at the same speed...')
    else:
            robot_drive.change_speed()

# main function to control the robot movement 
def move_distance(dist):
	global dist_completed
	global dist_to_run 

	dist_to_run = dist
	# if robot received a meaning less job, just signal, clear the job and return 
	if (dist_to_run == 0):
		rospy.logwarn('Robot received a meaningless moving job')
          	stop_move()
		return not robot_drive.robot_on_mission

	if (dist_to_run < 0):
		robot_drive.move_direction = 'B'
	else: 
		robot_drive.move_direction = 'F'

	# Mission started, let robot start moving 
	if (robot_drive.robot_on_mission == 0):
		rospy.loginfo("robot move start move") 
		start_move()
		return 0

	# Accumulate the running distance and check 
	# Get each step of the distance 
	dist_step = robot_drive.step_distance
	# accumulate the distance to the completed distance 
	dist_completed = dist_completed + abs(dist_step)   #this is in mm
	#distance travelled threshold (put 2 mm thresh hold before stopping)
	dist_threshold = abs(dist_to_run) - abs(dist_step) 	#0 mm, I can choose -50mm, but since there will be inefficiencies, 0 error threshold might be good enough
	
	distpub = 'Dist-travelled: %f dist-total:%f dist-step:%f' % (dist_completed, abs(dist_to_run) ,dist_step)
	rospy.loginfo(distpub)

	#@yuqing_correctionper10m
	#if travel over 10m, job_completed to 1, start to correct
	if (dist_completed > dist_to_correct):
		rospy.loginfo("-----------------dist_completed: %f, start to correct", dist_completed)
		stop_move()
		return not robot_drive.robot_on_mission
		
	if (dist_threshold - dist_completed > 0) :
		#just continue moving of job not completed and no change of speed command received 
		#if speed changed, then just change the move speed 
		continue_move() 
		return  0
	else :
		stop_move()
        	# make sure the robot is stopped before next job
        	return not robot_drive.robot_on_mission 
       	#clean current job 
	
	return 0 
