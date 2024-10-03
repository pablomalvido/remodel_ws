#! /usr/bin/env python

import sys
import rospy 
import time
import os
import csv
from UI_nodes_pkg.msg import *
import moveit_commander
from moveit_msgs.msg import *
import math
import random

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('fake_sensors')
forceR_pub = rospy.Publisher('/right_norbdo/forces', sensorT, queue_size=1)
forceL_pub = rospy.Publisher('/left_norbdo/forces', sensorT, queue_size=1)
tactileR_pub = rospy.Publisher('sensors/tactile_right', sensorXY, queue_size=1)
tactileL_pub = rospy.Publisher('sensors/tactile_left', sensorXY, queue_size=1)

rate=rospy.Rate(10) #20Hz
force_msgR = forces()
force_msgL = forces()
tactile_msgR = tactile()
tactile_msgL = tactile()
sensorT_msg = sensorT()
sensorXY_msg = sensorXY()
old_aL=0.5
old_bL=0.5
old_cL=2.5
old_aR=0.5
old_bR=0.5
old_cR=2.5

while not rospy.is_shutdown():
	force_msgR.Fx = max(min(force_msgR.Fx + random.randint(-19,19), 99),-99)
	force_msgR.Fy = max(min(force_msgR.Fy + random.randint(-19,19), 99),-99)
	force_msgR.Fz = max(min(force_msgR.Fz + random.randint(-19,19), 99),-99)
	force_msgR.Tx = max(min(force_msgR.Tx + random.randint(-19,19), 99),-99)
	force_msgR.Ty = max(min(force_msgR.Ty + random.randint(-19,19), 99),-99)
	force_msgR.Tz = max(min(force_msgR.Tz + random.randint(-19,19), 99),-99)
	sensorT_msg.data = [force_msgR.Fx/10, force_msgR.Fy/10, force_msgR.Fz/10, force_msgR.Tx/10, force_msgR.Ty/10, force_msgR.Tz/10]
	sensorT_msg.labels = ['Fx','Fy','Fz','Tx','Ty','Tz']
	forceR_pub.publish(sensorT_msg)

	force_msgL.Fx = max(min(force_msgL.Fx + random.randint(-19,19), 99),-99)
	force_msgL.Fy = max(min(force_msgL.Fy + random.randint(-19,19), 99),-99)
	force_msgL.Fz = max(min(force_msgL.Fz + random.randint(-19,19), 99),-99)
	force_msgL.Tx = max(min(force_msgL.Tx + random.randint(-19,19), 99),-99)
	force_msgL.Ty = max(min(force_msgL.Ty + random.randint(-19,19), 99),-99)
	force_msgL.Tz = max(min(force_msgL.Tz + random.randint(-19,19), 99),-99)
	sensorT_msg.data = [force_msgL.Fx/10, force_msgL.Fy/10, force_msgL.Fz/10, force_msgL.Tx/10, force_msgL.Ty/10, force_msgL.Tz/10]
	sensorT_msg.labels = ['Fx','Fy','Fz','Tx','Ty','Tz']
	forceL_pub.publish(sensorT_msg)

	tactile_msgR.x = []
	tactile_msgR.y = []
	a=max(min(old_aR + (float(random.randint(-10,10)))/20, 1),-1)
	b=max(min(old_bR + (float(random.randint(-20,20)))/20, 3),-3)
	c=max(min(old_cR + (float(random.randint(-60,60)))/100, 5.0),1.0)
	old_aR=a
	old_bR=b
	old_cR=c
	for i in range(60):
		tactile_msgR.x.append(float(i)/10)
		tactile_msgR.y.append(a*tactile_msgR.x[-1]**2 + b*tactile_msgR.x[-1] + c)
	sensorXY_msg.x = tactile_msgR.x
	sensorXY_msg.y = tactile_msgR.y
	tactileR_pub.publish(sensorXY_msg)

	tactile_msgL.x = []
	tactile_msgL.y = []
	a=max(min(old_aL + (float(random.randint(-10,10)))/20, 1),-1)
	b=max(min(old_bL + (float(random.randint(-20,20)))/20, 3),-3)
	c=max(min(old_cL + (float(random.randint(-60,60)))/100, 5.0),1.0)
	old_aL=a
	old_bL=b
	old_cL=c
	for i in range(60):
		tactile_msgL.x.append(float(i)/10)
		tactile_msgL.y.append(a*tactile_msgL.x[-1]**2 + b*tactile_msgL.x[-1] + c)
	sensorXY_msg.x = tactile_msgL.x
	sensorXY_msg.y = tactile_msgL.y
	tactileL_pub.publish(sensorXY_msg)

	rate.sleep()