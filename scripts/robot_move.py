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

status_pub = rospy.Publisher('status', String, queue_size = 100)
# Starts the robot for moving, put the control variables into proper value 
def start_move():
	global dist_completed
	global dist_to_run 
	
	lon1 = robot_drive.lon_now
	lat1 =  robot_drive.lat_now
	
	robot_drive.bearing_target = robot_drive.bearing_now 
	rospy.loginfo("Bearing target  %f", robot_drive.bearing_target)
	lon, lat = gpsmath.get_gps(lon1, lat1,abs(dist_to_run), robot_drive.bearing_target)
	robot_drive.lon_target = lon
	robot_drive.lat_target = lat
	rospy.loginfo("Lon target %f, lat target %f", lon, lat)
	
	if abs(dist_to_run) < 50:
		robot_drive.speed_now  = 3
        	robot_drive.desired_speed = 3
	elif abs(dist_to_run) < 200: 
    		robot_drive.speed_now  = 4
        	robot_drive.desired_speed = 4
        else:
		robot_drive.speed_now = 5
		robot_drive.desired_speed = 5

	rospy.loginfo('Robot moving job started')

    # only if the robot starts to move then change the status
	if(robot_drive.robot_moving == 1):
   		robot_drive.robot_on_mission = 1 

        status_pub.publish("enabled 1")
   	dist_completed = 0
	robot_drive.start()

# Roboet complet a moving job 
def stop_move():	
	global dist_completed
	dist_completed = 0
        if robot_drive.robot_moving == 0:
	   	robot_drive.robot_on_mission = 0 
       	rospy.loginfo('Robot completed a moving job')
	robot_drive.stop_robot()
        status_pub.publish("enabled 0")

# Update robot speed as required new speed 
# Update robot speed as required new speed 
def continue_move():
    global dist_to_run
    global dist_completed

    if robot_drive.robot_moving == 0:
        rospy.loginfo("Robot stopped during the mission, start to move again")
        robot_drive.start()

    if(abs(dist_to_run) - abs(dist_completed) < 50):
            robot_drive.desired_speed = 3
            rospy.loginfo('Reduce speed to 3, very close to target position')
    elif(abs(dist_to_run) - abs(dist_completed) < 200):
            robot_drive.desired_speed = 4
            rospy.loginfo('Reduce speed to 4, only 20 cm to target position')

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
        if robot_drive.robot_moving: 
          stop_move()
		return 1

	if (dist_to_run < 0):
		robot_drive.move_direction = 'B'
	else: 
		robot_drive.move_direction = 'F'

	# Mission started, let robot start moving 
	if (robot_drive.robot_on_mission == 0): 
		start_move()
		return 0

	# Accumulate the running distance and check 
	# Get each step of the distance 
	dist_step = robot_drive.step_distance
	# accumulate the distance to the completed distance 
	dist_completed = dist_completed + abs(dist_step)   #this is in mm
	#distance travelled threshold (put 2 mm thresh hold before stopping)
	dist_threshold = abs(dist_to_run) - 7 	#0 mm, I can choose -50mm, but since there will be inefficiencies, 0 error threshold might be good enough
	
	distpub = 'dist-travelled: %f dist-total:%f dist-step:%f' % (dist_completed, abs(dist_to_run) ,dist_step)
	rospy.loginfo(distpub)

	if (dist_threshold - dist_completed > 0) :
		#just continue moving of job not completed and no change of speed command received 
		#if speed changed, then just change the move speed 
		continue_move() 
		return  0
	else :
        if robot_drive.robot_moving: 
		  stop_move()
        return robot_drive.robot_on_mission
       	#clean current job 
	
	return 0 
