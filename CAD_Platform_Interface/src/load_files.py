#!/usr/bin/env python

import rospy, os
import numpy as np
from UI_nodes_pkg.srv import *

#Initialize ROS node 
rospy.init_node('CAD_loader') 
rate = rospy.Rate(10)

user_name = os.getlogin()

def get_usb_cb(req):
    global user_name
    resp = GetListResponse()
    resp.success = True
    try:
        arr = os.listdir('/media/' + str(user_name) + '/')
        resp.data = arr
    except:
        resp.success = False
    return resp

rospy.Service('/UI/get_usbs', GetList, get_usb_cb)


def get_files_cb(req):
    global user_name
    resp = GetListResponse()
    resp.success = True
    try:
        arr = os.listdir('/media/' + str(user_name) + '/' + req.data)
        resp.data = arr
    except:
        resp.success = False
    return resp

rospy.Service('/UI/get_files_usb', GetList, get_files_cb)

def move_file_cb(req):
    global user_name
    resp = MoveFileResponse()
    resp.success = True
    try:
        #dest_dir = os.path.join(os.path.dirname(__file__), '../data_UI/')
        dest_dir = '/home/remodel/catkin_ws/src/elvez_pkg/data_UI/'
        os.system('rm -r ' + dest_dir + req.destination)
        os.system('cp -r /media/' + str(user_name) + '/' + req.source +  ' ' + dest_dir + req.destination)
        print('cp -r /media/' + str(user_name) + '/' + req.source +  ' ' + dest_dir + req.destination)
    except:
        resp.success = False
    return resp

rospy.Service('/UI/copy_file', MoveFile, move_file_cb)

rospy.spin()