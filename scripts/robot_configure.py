#!/usr/bin/env python
import rospy
import string
import ConfigParser

from std_msgs.msg import String

def read_config(config_file_path, field, key): 
    cf = ConfigParser.ConfigParser()
    ret = True 
    result = None
    try:
        cf.read(config_file_path)
        result = cf.get(field, key)
    except:
    	ret = False
        rospy.loginfo("Failed to read %s-%s", field, key)
    return ret, result

def read_config_int(config_file_path, field, key): 
    cf = ConfigParser.ConfigParser()
    ret = True 
    val = 0
    try:
        cf.read(config_file_path)
        result = cf.get(field, key)
        val = int(result)
    except:
        ret = False
        rospy.loginfo("Failed to read %s-%s", field, key)
    return ret, val

def write_config(config_file_path, field, key, value):
    cf = ConfigParser.ConfigParser()
    ret = True
    try:
        cf.read(config_file_path)
        cf.set(field, key, value)
        cf.write(open(config_file_path,'w'))
    except:
    	ret = False
        rospy.loginfo("Failed to write parameters")
    return ret 
