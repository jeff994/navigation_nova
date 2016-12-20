#!/usr/bin/env python
import rospy

# Clear jobs 
def clear_jobs(job_des, job_num):
	del job_des[:]
	del job_num[:]

def generate_move(job_des, job_num, distance, direction)
	job_num.extend([distance]) 
	job_des.extend([direction])

def generate_turn(job_des, job_num, angle)
	job_num.extend([angle]) 
	job_des.extend(['T'])

