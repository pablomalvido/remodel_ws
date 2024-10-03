#! /usr/bin/env python

import sys
import rospy 
import time
import os
import csv
from std_srvs.srv import *
from UI_nodes_pkg.srv import *
from UI_nodes_pkg.msg import *

rospy.init_node('record_traj', anonymous=True)
traj_record_service = '/trajectory_record'
get_trajs_service = '/get_trajectories'
exec_traj_service = '/execute_trajectory'
del_traj_service = '/delete_trajectory'
rate=rospy.Rate(0.25) #20Hz

path_current = os.path.dirname(__file__)
traj_dir = os.path.join(path_current, '../files/saved_trajectories/')


def record_traj_cb(req): 
	"""
	Add new wapoint/instruction to the routine
	"""
	resp = RecordTrajResponse()
	resp.success = True
	try:
		if req.recordPose:
			resp.result = "Pose recorded"
		elif req.moveGripper:
			resp.result = "Move gripper recorded"
		elif req.closeGripper:
			resp.result = "Grasp recorded"
		elif req.tape:
			resp.result = "Tape recorded"
		elif req.saveTraj:
			resp.result = "Saved traj: " + str(req.fileName)
	except:
		resp.success = False
		resp.result = "Error"
	return resp

rospy.Service(traj_record_service, RecordTraj, record_traj_cb)


def get_traj_cb(req): 
	"""
	Retrieves the list of saved trajectories
	"""
	resp = GetTrajResponse()
	resp.success = True
	try:
		resp.data = os.listdir(traj_dir)
	except:
		resp.success = False
	return resp

rospy.Service(get_trajs_service, GetTraj, get_traj_cb)


def exec_traj_cb(req): 
	"""
	Simulated trajectory execution. This should be replaced by the actual logics for moving the robot according to the recorded plan
	"""
	resp = ExecTrajResponse()
	resp.success = True
	try:
		rospy.sleep(3)
		resp.result = str(req.data) + " executed successfully"
	except:
		resp.success = False
		resp.result = "Error"
	return resp

rospy.Service(exec_traj_service, ExecTraj, exec_traj_cb)


def del_traj_cb(req):
	"""
	Deletes a saved trajectory
	""" 
	resp = ExecTrajResponse()
	resp.success = True
	try:
		resp.result = str(req.data) + " deleted successfully"
	except:
		resp.success = False
		resp.result = "Error"
	return resp

rospy.Service(del_traj_service, ExecTraj, del_traj_cb)


rospy.spin()