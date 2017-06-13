#!/usr/bin/env python
import rospy
import string

from std_msgs.msg import String

battery_level = 100

#status callback variables
on_obstacle  				= 0
manual_mode 	 			= 0
obstacle_avoidance_mode 	= 0
has_obstacle 				= 0
interaction_mode 			= 0

motor_1_ok 	 				= 1
motor_2_ok 	 	 			= 1
encoder_ok 					= 1
gyroscope_ok 				= 1
reverse_sensor_ok 		 	= 1
distance_sensor_ok 			= 1

