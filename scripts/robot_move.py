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

move_direction = 'F'
dist_completed = 0.0
dist_to_run = 0.0

# Starts the robot for moving, put the control variables into proper value 
def start_move():
	global dist_completed
	global move_direction
	global dist_to_run 
	
	lon1 = robot_drive.lon_now
	lat1 =  robot_drive.lat_now
	

	initial_bearing = robot_drive.bearing_now
	target_gps = gpsmath.get_gps(lon1, lat1, initial_bearing, dist_to_run)
	robot_drive.lon_target = target_gps[0]
	robot_drive.lat_target = target_gps[1]

	if abs(dist_to_run)<20:
		robot_drive.speed_now  = 3
        robot_drive.desired_speed = 3
    elif abs(dist_to_run) < 50 
    	robot_drive.speed_now  = 4
        robot_drive.desired_speed = 4
    else:
		robot_drive.speed_now = 5
		robot_drive.desired_speed = 5

	rospy.loginfo('Robot moving job started')
	robot_drive.robot_on_mission = 1 
	dist_completed = 0
	robot_drive.send_command(move_direction, robot_drive.speed_now)

# Roboet complet a moving job 
def stop_move():	
	global dist_completed
	dist_completed = 0
	robot_drive.robot_on_mission = 0 
	robot_drive.send_command('S',0)
	rospy.loginfo('Robot completed a moving job')

# Update robot speed as required new speed 
# Update robot speed as required new speed 
def continue_move(left_dist, right_dist):
    global move_direction
    global dist_to_run
    global dist_completed

    if(abs(dist_to_run) - abs(dist_completed) < 20):
            robot_drive.send_command(move_direction, 3)
            robot_drive.speed_now  = 3
            robot_drive.desired_speed = 3
            rospy.loginfo('Reduce speed to 3, very close to target position')
            return
    elif(abs(dist_to_run) - abs(dist_completed) < 50):
            robot_drive.send_command(move_direction, 4)
            robot_drive.speed_now  = 4
            robot_drive.desired_speed = 4
            rospy.loginfo('Reduce speed to 4, only 5 cm to target position')
            return

    robot_correction.update_robot_gps(left_dist, right_dist)
    if(robot_drive.speed_now  == robot_drive.desired_speed):
            rospy.loginfo('Still moving at the same speed...')
    else:
            robot_drive.send_command(move_direction, robot_drive.desired_speed)
            robot_drive.speed_now  = robot_drive.desired_speed
            distpub = 'Robot move speed changed from %d to %d' % (robot_drive.speed_now, robot_drive.desired_speed)
            rospy.loginfo(distpub)




# main function to control the robot movement 
def move_distance(dist, left_encode, right_encode):
	global move_direction 
	global dist_completed
	global dist_to_run 

	dist_to_run = dist
	# if robot received a meaning less job, just signal, clear the job and return 
	if (dist_to_run == 0):
		rospy.logwarn('Robot received a meaningless moving job')
		stop_move()
		return 1

	if (dist_to_run < 0):
		move_direction = 'B'
	else: 
		move_direction = 'F'

	# Mission started, let robot start moving 
	if (robot_drive.robot_on_mission == 0): 
		start_move()
		return 0

	if(move_direction == 'F' and left_encode < -10 and right_encode < -10):
		rospy.logwarn('Robot supposed to move forward, actually moveing backward, stop the job')
		#stop_move()
		return 0 

	if(move_direction == 'B' and left_encode > 10 and right_encode > 10):
		rospy.logwarn('Robot supposed to move backward, acutally moveing forward, stop the job')
		#stop_move()
		return 0 

	# Exception handling, make sure robot wheels is moving the same direction 
	if (left_encode > 10 and right_encode < 10):
		rospy.logwarn('Robot wheel not moving as expected, stop current job')
		#stop_move()
		return 0 

	# Accumulate the running distance and check 
	# Get each step of the distance 
	left_dist = left_encode / robot_drive.encode_to_mm
	right_dist = right_encode / robot_drive.encode_to_mm

	dist_step = (left_dist + right_dist) / 2.0
	# accumulate the distance to the completed distance 
	dist_completed = dist_completed + abs(dist_step)   #this is in mm
	#distance travelled threshold (put 2 mm thresh hold before stopping)
	dist_threshold = abs(dist_to_run) - 2 	#0 mm, I can choose -50mm, but since there will be inefficiencies, 0 error threshold might be good enough
	
	distpub = 'dist-travelled: %f dist-total:%f dist-step:%f' % (dist_completed,dist_threshold,dist_step)
	rospy.loginfo(distpub)

	if (dist_threshold - dist_completed > 0) :
		#just continue moving of job not completed and no change of speed command received 
		#if speed changed, then just change the move speed 
		continue_move(left_dist, right_dist) 
		return  0
	else :
		stop_move()
       	return 1 
       	#clean current job 
	
	return 0 
