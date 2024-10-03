#! /usr/bin/env python

import sys
import rospy 
import time
import os
import csv
from std_srvs.srv import *
from UI_nodes_pkg.srv import *
from UI_nodes_pkg.msg import *

rospy.init_node('config_node', anonymous=True)
get_config_service = '/UI/get_config'
set_config_service = '/UI/set_config'
path_current = os.path.dirname(__file__)
path_config = os.path.join(path_current, '../files/config.csv')

config_dict = {}
fieldnames = ['prop', 'value']

with open(path_config) as csvfile:
    reader = csv.DictReader(csvfile)
    for row in reader:
        config_dict[row['prop']]=row['value']


def get_config_callback(req): 
	"""
	Get config values
	"""
	resp=GetConfigResponse()
	resp.success = True
	try:
		path_config_srv = os.path.join(path_current, req.file_name)
		with open(path_config_srv) as csvfile:
			reader = csv.DictReader(csvfile)
			for row in reader:
				resp_el = configProp()
				resp_el.prop = row['prop']
				resp_el.value = row['value']
				resp.data.append(resp_el)
	except:
		resp.success = False
	return resp

rospy.Service(get_config_service, GetConfig, get_config_callback) #Change srv type


def set_config_callback(req): 
	"""
	Overwrite config values
	"""
	resp=SetConfigResponse()
	resp.success = True
	try:
		for el in req.data:
			config_dict[el.prop] = el.value

		rows=[]
		for prop in config_dict:
			rows.append({'prop': prop, 'value': config_dict[prop]})

		path_config_srv = os.path.join(path_current, req.file_name)
		with open(path_config_srv, 'w') as f:
			writer = csv.DictWriter(f, fieldnames=fieldnames)
			writer.writeheader()
			writer.writerows(rows)
	except:
		resp.success = False
	return resp

rospy.Service(set_config_service, SetConfig, set_config_callback) #Change srv type


rospy.spin()