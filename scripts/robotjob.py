#!/usr/bin/env python
import rospy

# Clear jobs 
def clear_jobs(job_des, job_num):
	del job_des[:]
	del job_num[:]
