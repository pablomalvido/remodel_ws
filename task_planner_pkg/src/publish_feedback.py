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

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('process_feedback')
feedback_publisher = rospy.Publisher('/UI/feedback', configProp, queue_size=1)
rate=rospy.Rate(20) #20Hz

moveit_groups = {}
moveit_groups["arm_left"] = moveit_commander.MoveGroupCommander("arm_left")
moveit_groups["arm_right"] = moveit_commander.MoveGroupCommander("arm_right")

def compute_distance(pose1, pose2):
        dist = math.sqrt((pose1.position.x - pose2.position.x)**2 + (pose1.position.y - pose2.position.y)**2 + (pose1.position.z - pose2.position.z)**2)
        return dist

publish_right_speed_msg = configProp()
publish_right_speed_msg.prop = "speed_right"
publish_left_speed_msg = configProp()
publish_left_speed_msg.prop = "speed_left"

last_pose_left = moveit_groups["arm_left"].get_current_pose().pose
last_pose_right = moveit_groups["arm_right"].get_current_pose().pose
last_time = time.time()

while not rospy.is_shutdown():
	pose_left = moveit_groups["arm_left"].get_current_pose().pose
	pose_right = moveit_groups["arm_right"].get_current_pose().pose
	time_diff = time.time() - last_time
	last_time = time.time()
	speed_left = (compute_distance(pose_left, last_pose_left)*1000)/(time_diff)
	speed_right = (compute_distance(pose_right, last_pose_right)*1000)/(time_diff)
	last_pose_left = pose_left
	last_pose_right = pose_right
        
	publish_right_speed_msg.value = str(round(speed_right,1)) + " mm/s"
	feedback_publisher.publish(publish_right_speed_msg)
	publish_left_speed_msg.value = str(round(speed_left,1)) + " mm/s"
	feedback_publisher.publish(publish_left_speed_msg)

	rate.sleep()