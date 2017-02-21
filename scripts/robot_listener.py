# holds all kinds of call back etc 
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
import json
from datetime import datetime
from std_msgs.msg import String


# The main call back, getting encoder data and make decision for the next move 
def encoder_callback(data):
	rospy.loginfo("test")