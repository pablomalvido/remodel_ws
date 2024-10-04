#! /usr/bin/env python
import sys
import os
import copy
import rospy
from std_msgs.msg import *
from std_srvs.srv import *
import actionlib
from task_planner_pkg.msg import gripper_ActFeedback, gripper_ActResult, gripper_ActAction, gripper_ActGoal
from task_planner_pkg.msg import EEFActAction, EEFActResult, EEFActFeedback, EEFActGoal
from task_planner_pkg.srv import *
from ROS_UI_backend.msg import configProp
from industrial_msgs.msg import RobotStatus


rospy.init_node('end_effectors_node', anonymous=True)
feedback_publisher = rospy.Publisher('/UI/feedback', configProp, queue_size=1)
execute_grippers = 0
init = False
stop_mov = False

#ROBOT SAFETY
def callback_robot_status(msg):
    global stop_mov
    if str(msg.e_stopped) == "val: 1":
        if not stop_mov:
            print("STOPPING")
        stop_mov = True

robot_status_subscriber = rospy.Subscriber('/robot_status', RobotStatus, callback_robot_status) 

#INITIALIZE GRIPPER ACTIONS
def feedback_cb_left_move(fb):
        global left_move_error
        global gripper_left_finish
        left_move_error = fb.error
        gripper_left_finish = fb.finish

def feedback_cb_right_grasp(fb):
        global left_move_error
        global gripper_right_finish
        right_grasp_error = fb.error
        gripper_right_finish = fb.finish

def feedback_cb_right_move(fb):
        global right_move_error
        global gripper_right_finish
        right_move_error = fb.error
        gripper_right_finish = fb.finish

def feedback_cb_left_grasp(fb):
        global left_move_error
        global gripper_left_finish
        left_grasp_error = fb.error
        gripper_left_finish = fb.finish

gripper_left_active = True
gripper_right_active = True
gripper_both_active = True
config = {}

def init_grippers(req):
        global execute_grippers
        global init
        global client_left_move
        global client_left_grasp
        global client_right_move
        global client_right_grasp
        global gripper_right_distance
        global gripper_left_distance
        global config

        gripper_right_distance = 0
        gripper_left_distance = 0
        config['grasp_distance'] = req.grasp_distance
        config['slide_distance'] = req.slide_distance
        execute_grippers = req.exe
        init = True
        if execute_grippers:
                if gripper_left_active or gripper_both_active:
                        client_left_move = actionlib.SimpleActionClient('/left_wsg/action/move', gripper_ActAction)
                        client_left_move.wait_for_server()
                        client_left_grasp = actionlib.SimpleActionClient('/left_wsg/action/grasp', gripper_ActAction)
                        client_left_grasp.wait_for_server()
                if gripper_right_active or gripper_both_active:
                        client_right_move = actionlib.SimpleActionClient('/right_wsg/action/move', gripper_ActAction)
                        client_right_move.wait_for_server()
                        client_right_grasp = actionlib.SimpleActionClient('/right_wsg/action/grasp', gripper_ActAction)
                        client_right_grasp.wait_for_server()
        resp = EEFInitResponse()
        resp.success = True
        return resp

rospy.Service("/EEF/init", EEFInit, init_grippers)


def actuate_grippers(distance, speed, arm, grasp=False):
        global execute_grippers
        global gripper_left_finish
        global gripper_right_finish
        global gripper_right_distance
        global gripper_left_distance
        global client_left_move
        global client_left_grasp
        global client_right_move
        global client_right_grasp
        global stop_mov
        global config
        global EEF_action

        if not init:
                print("You must first initialize the grippers")
                return

        if execute_grippers and not stop_mov:
                goal = gripper_ActGoal()
                if arm=="left":
                       distance = distance - 9
                goal.width = distance
                goal.speed = speed
                if grasp:
                        if (arm == "right" or arm == "both") and (gripper_right_active or gripper_both_active):
                                client_right_grasp.send_goal(goal, feedback_cb = feedback_cb_right_grasp)
                        if (arm == "left" or arm == "both") and (gripper_left_active or gripper_both_active):
                                client_left_grasp.send_goal(goal, feedback_cb = feedback_cb_left_grasp)
                else:
                        if (arm == "right" or arm == "both") and (gripper_right_active or gripper_both_active):
                                client_right_move.send_goal(goal, feedback_cb = feedback_cb_right_move)
                        if (arm == "left" or arm == "both") and (gripper_left_active or gripper_both_active):
                                client_left_move.send_goal(goal, feedback_cb = feedback_cb_left_move)
                
                count_i = 0
                counter_period = 0.1
                counter_limit = 0
                if (arm == "left" or arm == "both") and (gripper_left_active or gripper_both_active):
                        if gripper_left_distance>0:
                                counter_limit = ((abs(gripper_left_distance - distance)/speed)/counter_period)+10
                        else:
                                counter_limit = 3.0/counter_period
                        while not gripper_left_finish:
                                rospy.sleep(counter_period)
                                if count_i > counter_limit:
                                       break
                                count_i+=1
                        gripper_left_finish = False
                if (arm == "right" or arm == "both") and (gripper_right_active or gripper_both_active):  
                        if gripper_right_distance>0:
                                counter_limit = ((abs(gripper_right_distance - distance)/speed)/counter_period)+10
                        else:
                                counter_limit = 3.0/counter_period      
                        while not gripper_right_finish:
                                rospy.sleep(counter_period)
                                if count_i > counter_limit:
                                       break
                                count_i+=1
                        gripper_right_finish = False

        if arm == "right" or arm == "both":
                gripper_right_distance = distance #AND PUBLISH TO UI
                right_gripper_msg = configProp()
                right_gripper_msg.prop = "eef_right_status"
                if distance > config['slide_distance']:
                        right_gripper_msg.value = "Gripper open"
                elif distance > config['grasp_distance']:
                        right_gripper_msg.value = "Gripper slide"
                else:
                        right_gripper_msg.value = "Gripper closed"    
                feedback_publisher.publish(right_gripper_msg)    
        if arm == "left" or arm == "both":
                gripper_left_distance = distance
                left_gripper_msg = configProp()
                left_gripper_msg.prop = "eef_left_status"
                if distance > config['slide_distance']:
                        left_gripper_msg.value = "Gripper open"
                elif ((distance+9) > config['grasp_distance']):
                        left_gripper_msg.value = "Gripper slide"
                else:
                        left_gripper_msg.value = "Gripper closed"    
                feedback_publisher.publish(left_gripper_msg)   

        fb = EEFActFeedback()
        fb.done = True
        EEF_action.publish_feedback(fb) 


def actuate_gun():
        global execute_gun
        global stop_mov
        if execute_gun and not stop_mov:
                rospy.wait_for_service('/gun/tape')
                tape_srv = rospy.ServiceProxy('gun/tape', Trigger)
                tape_res = tape_srv(TriggerRequest())
        return 1


def goal_callback(goal):
        if goal.type==0: #Gripper
                actuate_grippers(goal.distance, goal.speed, goal.arm, goal.grasp)
        else:
                actuate_gun()
        res = EEFActResult()
        res.success = True
        return res

EEF_action = actionlib.SimpleActionServer("EEF/actuate", EEFActAction, goal_callback, False) #Defines the action server: Name of the server, type and callback function
EEF_action.start() #Starts the action server, so it can be called from the client