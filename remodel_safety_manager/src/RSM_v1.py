#!/usr/bin/env python3

from aphyt import omron
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

#Global variables
ROS_reset = False
ROS_UI_problem = False
ROS_vision_problem = False
ROS_problem = False
problems = {'PLC_robot_problem': False, 'PLC_Estop_problem': False, 'PLC_door_problem': False, 'PLC_curtain_problem': False, 'ROS_UI_problem': False, 'ROS_vision_problem': False}
display_problems = {"robot": {"detected": False, "active": False}, "Estop": {"detected": False, "active": False}, "curtain": {"detected": False, "active": False}, "door": {"detected": False, "active": False}, "UI": {"detected": False, "active": False}, "vision": {"detected": False, "active": False}}
published_problems = copy.deepcopy(display_problems)
publish_problems = True
published_PLC_safety_ok = False
publish_safety = True
published_PLC_reset = False
publish_reset = True
load_page = True


#Establish the EIP connection with the PLC
#EIP_instance = omron.n_series.NSeriesEIP()
EIP_instance = omron.n_series.NSeries()
EIP_instance.connect_explicit('10.0.0.50')
EIP_instance.register_session()
EIP_instance.update_variable_dictionary()


#Subscriptors receiving info from other subsystems (UI, vision, planer...)
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


def get_safety_cb(info):
    global load_page
    load_page = True
    print("Page loading")

subs_ROS_status_request = rospy.Subscriber('/RSM/get_safety', Bool, get_safety_cb)


while not rospy.is_shutdown(): 
	#Read PLC data
	PLC_safety_ok = EIP_instance.read_variable('PLC_safety_ok')
	problems['PLC_robot_problem'] = EIP_instance.read_variable('PLC_robot_problem')
	problems['PLC_Estop_problem'] = not EIP_instance.read_variable('PLC_Estop_problem') #Inverted true
	problems['PLC_door_problem'] = not EIP_instance.read_variable('PLC_door_problem') #Inverted true
	problems['PLC_curtain_problem'] = not EIP_instance.read_variable('PLC_curtain_problem') #Inverted true
	PLC_reset = EIP_instance.read_variable('PLC_reset')

	#print(str(PLC_safety_ok))

	#Updates problems. Detected and not active mean that even if the alarm is not active, it was and it haven't been reseted yet
	if problems['PLC_Estop_problem']:
		display_problems["Estop"]["detected"] = True
		display_problems["Estop"]["active"] = True
	else:
		display_problems["Estop"]["active"] = False
	
	if problems['PLC_door_problem']:
		display_problems["door"]["detected"] = True
		display_problems["door"]["active"] = True
	else:
		display_problems["door"]["active"] = False

	if problems['PLC_curtain_problem']:
		display_problems["curtain"]["detected"] = True
		display_problems["curtain"]["active"] = True
	else:
		display_problems["curtain"]["active"] = False

	if problems['PLC_robot_problem']:
		display_problems["robot"]["detected"] = True
		display_problems["robot"]["active"] = True
	else:
		display_problems["robot"]["active"] = False

	if problems['ROS_UI_problem']:
		display_problems["UI"]["detected"] = True
		display_problems["UI"]["active"] = True
	else:
		display_problems["UI"]["active"] = False

	if problems['ROS_vision_problem']:
		display_problems["vision"]["detected"] = True
		display_problems["vision"]["active"] = True
	else:
		display_problems["vision"]["active"] = False

	#Decides what to publish, when there is something to update
	if ((display_problems != published_problems) or load_page):
		publish_problems = True
		published_problems = copy.deepcopy(display_problems)
		print("Problems changed")
	else:
		publish_problems = False

	if ((PLC_safety_ok != published_PLC_safety_ok) or load_page):
		publish_safety = True
		published_PLC_safety_ok = PLC_safety_ok
	else:
		publish_safety = False

	if ((PLC_reset != published_PLC_reset) or load_page):
		publish_reset = True
		published_PLC_reset = PLC_reset
		load_page = False #Just in the last one
	else:
		publish_reset = False

	#Publish PLC data so other subsystems can receive it and start corrective/safety actions, such as the UI and planner
	if publish_safety:
		safety_message = Bool()
		safety_message.data = PLC_safety_ok
		publisher_safety_ok.publish(safety_message) #Sends info to planner and UI
		rate.sleep()

	if publish_problems:
		problems_message = RSM_problems()
		problems_message.robot_det = display_problems["robot"]["detected"]
		problems_message.Estop_det = display_problems["Estop"]["detected"]
		problems_message.door_det = display_problems["door"]["detected"]
		problems_message.curtain_det = display_problems["curtain"]["detected"]
		problems_message.UI_det = display_problems["UI"]["detected"]
		problems_message.vision_det = display_problems["vision"]["detected"]
		problems_message.robot_act = display_problems["robot"]["active"]
		problems_message.Estop_act = display_problems["Estop"]["active"]
		problems_message.door_act = display_problems["door"]["active"]
		problems_message.curtain_act = display_problems["curtain"]["active"]
		problems_message.UI_act = display_problems["UI"]["active"]
		problems_message.vision_act = display_problems["vision"]["active"]
		publisher_RSM_problems.publish(problems_message) #Updates the Alarms page in the UI (maintains trues (set))
		rate.sleep()

	if publish_reset:
		reset_message = Bool()
		if PLC_reset:
			display_problems = {"robot": {"detected": False, "active": False}, "Estop": {"detected": False, "active": False}, "curtain": {"detected": False, "active": False}, "door": {"detected": False, "active": False}, "UI": {"detected": False, "active": False}, "vision": {"detected": False, "active": False}}
		reset_message.data = PLC_reset
		publisher_reset.publish(reset_message) #Resets the Alarms page in the UI
		rate.sleep()
	
	#ROS safety. Alarms coming from ROS (not real time stop) and not from the PLC or safety devices
	if problems['ROS_UI_problem'] or problems['ROS_vision_problem']:
		ROS_problem = True
		#ROS_UI_problem = False
		problems['ROS_vision_problem'] = False
		print("Error UI")
	else:
		ROS_problem = False

	#Send data to PLC (Omron in this case)
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
