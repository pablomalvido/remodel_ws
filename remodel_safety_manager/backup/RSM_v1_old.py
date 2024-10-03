#!/usr/bin/env python3

import omron
import rospy
import numpy as np
import math 
import time
import os
import copy
from std_msgs.msg import Bool
from remodel_safety_manager.msg import *


#Initialize ROS node 
rospy.init_node('RSM_PLC_connection') 
rate = rospy.Rate(10)
publisher_safety_ok = rospy.Publisher('RSM/safety_ok', Bool, queue_size=1)
publisher_RSM_problems = rospy.Publisher('RSM/problems', RSM_problems, queue_size=1)
publisher_reset = rospy.Publisher('RSM/reset', Bool, queue_size=1)
ROS_reset = False
ROS_UI_problem = False
ROS_vision_problem = False
ROS_problem = False

problems = {'PLC_robot_problem': False, 'PLC_Estop_problem': False, 'PLC_door_problem': False, 'PLC_curtain_problem': False, 'ROS_UI_problem': False, 'ROS_vision_problem': False}
published_problems = problems.copy()
publish_problems = True
published_PLC_safety_ok = False
publish_safety = True
published_PLC_reset = False
publish_reset = True
load_page = True


#Establish the EIP connection with the PLC
EIP_instance = omron.n_series.NSeriesEIP()
EIP_instance.connect_explicit('192.168.1.1')
EIP_instance.register_session()
EIP_instance.update_variable_dictionary()


#Subscriptors
def ROS_reset_cb(info):
    global ROS_reset
    ROS_reset = True

subs_ROS_Reset = rospy.Subscriber('/RSM/ROS_reset', Bool, ROS_reset_cb)


def ROS_UI_problem_cb(info):
    global problems
    problems['ROS_UI_problem'] = info.data

subs_ROS_UI_problem = rospy.Subscriber('/RSM/ROS_UI_problem', Bool, ROS_UI_problem_cb)


def ROS_vision_problem_cb(info):
    global problems
    problems['ROS_vision_problem'] = info.data

subs_ROS_vision_problem = rospy.Subscriber('/RSM/ROS_vision_problem', Bool, ROS_vision_problem_cb)


def load_page_cb(info):
    global load_page
    load_page = True

subs_ROS_vision_problem = rospy.Subscriber('/RSM/load_page', Bool, load_page_cb)


while not rospy.is_shutdown(): 
	#Read PLC data
	PLC_safety_ok = EIP_instance.read_variable('PLC_safety_ok')
	problems['PLC_robot_problem'] = EIP_instance.read_variable('PLC_robot_problem')
	problems['PLC_Estop_problem'] = not EIP_instance.read_variable('PLC_Estop_problem') #Inverted true
	problems['PLC_door_problem'] = not EIP_instance.read_variable('PLC_door_problem') #Inverted true
	problems['PLC_curtain_problem'] = not EIP_instance.read_variable('PLC_curtain_problem') #Inverted true
	PLC_reset = EIP_instance.read_variable('PLC_reset')

	print(str(PLC_safety_ok))

	#Decides what to publish
	if problems != published_problems or load_page:
		publish_problems = True
		published_problems = problems.copy()
	else:
		publish_problems = False

	if PLC_safety_ok != published_PLC_safety_ok or load_page:
		publish_safety = True
		published_PLC_safety_ok = PLC_safety_ok
	else:
		publish_safety = False

	if PLC_reset != published_PLC_reset or load_page:
		publish_reset = True
		published_PLC_reset = PLC_reset
		load_page = False #Just in the last one
	else:
		publish_reset = False

	#Publish PLC data
	if publish_safety:
		safety_message = Bool()
		safety_message.data = PLC_safety_ok
		publisher_safety_ok.publish(safety_message) #Sends info to planner and UI

	if publish_problems:
		problems_message = RSM_problems()
		problems_message.robot = problems['PLC_robot_problem']
		problems_message.Estop = problems['PLC_Estop_problem']
		problems_message.door = problems['PLC_door_problem']
		problems_message.curtain = problems['PLC_curtain_problem']
		problems_message.UI = problems['ROS_UI_problem']
		problems_message.vision = problems['ROS_vision_problem']
		publisher_RSM_problems.publish(problems_message) #Updates the Alarms page in the UI (maintains trues (set))

	if publish_reset:
		reset_message = Bool()
		reset_message.data = PLC_reset
		publisher_reset.publish(reset_message) #Resets the Alarms page in the UI
	
	#ROS safety
	if problems['ROS_UI_problem'] or problems['ROS_vision_problem']:
		ROS_problem = True
		#ROS_UI_problem = False
		problems['ROS_vision_problem'] = False
		print("Error UI")
	else:
		ROS_problem = False

	#Send data to PLC
	if ROS_reset:
		EIP_instance.write_variable('ROS_reset', True)
		ROS_reset = False
		print("Reset UI")
	else:
		EIP_instance.write_variable('ROS_reset', False)
	if ROS_problem:
		EIP_instance.write_variable('ROS_problem', True)
		ROS_problem = False
	else:
		EIP_instance.write_variable('ROS_problem', False)

	rate.sleep()
