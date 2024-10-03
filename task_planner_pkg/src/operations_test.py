#! /usr/bin/env python
import sys
import os
import copy
import rospy
import  rospkg
import csv
import PyKDL 
import moveit_commander
from moveit_msgs.msg import *
from moveit_msgs.srv import *
import geometry_msgs.msg
from std_msgs.msg import *
from std_srvs.srv import *
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
import math
from sensor_msgs.msg import *
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import xml.etree.ElementTree as ET
import tf
from elvez_pkg.msg import *
from elvez_pkg.srv import *
from task_planner_pkg.msg import *
from task_planner_pkg.srv import *
from ROS_UI_backend.msg import *
from ROS_UI_backend.srv import *
import actionlib
from actionlib_msgs.msg import GoalStatusArray
from task_planner_pkg.msg import gripper_ActFeedback, gripper_ActResult, gripper_ActAction, gripper_ActGoal
from task_planner_pkg.msg import process_UIFeedback, process_UIResult, process_UIAction, process_UIGoal
from vision_pkg_full_demo.srv import *
from rosapi.srv import *
from norbdo_force_sensor.msg import forces
from motoman_msgs.srv import WriteSingleIO
from motoman_msgs.srv import WriteSingleIORequest
from industrial_msgs.msg import RobotStatus
from industrial_msgs.msg import TriState
from utils import *
from end_effectors import *


def EC(op, step2=0, config=[], route_group=""):
        global pick_grasp_offset
        global z_offset_pick
        global process_actionserver
        global fast_speed_execution
        global slow_speed_execution
        global speed_execution
        global grasping_cables

        step1_increase = 0
        grasping_cables = False
        print("Pick connector")
        print("Step2: "+str(step2))
        if op['spot']['side'] == "R":
                pick_arm = "right"
                fingers_size = ATC1.EEF_dict[ATC1.EEF_right].fingers_dim #GET fingers_size with a srv
        else:
                pick_arm = "left"
                fingers_size = ATC1.EEF_dict[ATC1.EEF_left].fingers_dim
        main_arm = 'arm_'+pick_arm

        prepick_pose = 'arm_'+pick_arm+'_prepick'
        if op["spot"]["name"] == "WH3":
                pick_pose = get_shifted_pose(op["spot"]["pose_corner"], [op["spot"]['width']/2, - fingers_size[0]/2 - pick_grasp_offset[op["spot"]["name"]] - config['grasp_offset'], op["spot"]["height"]/2 + 0.01, 0, 0, 0])
        else:
                pick_pose = get_shifted_pose(op["spot"]["pose_corner"], [op["spot"]['width']/2, - fingers_size[0]/2 - pick_grasp_offset[op["spot"]["name"]] - config['grasp_offset'], op["spot"]["height"]/2, 0, 0, 0])
        pick_pose = get_shifted_pose(pick_pose, [0, 0, 0, 0, 0, -1.5708])
        pick_pose_offset = get_shifted_pose(pick_pose, [0, 0, op["spot"]["height"]/2 + z_offset_pick, 0, 0, 0])

        if step2 == 0:
                #Move to predefined dual-arm config (pointing down)
                set_named_target('arms', "arms_platform_5")
                go('arms')
                step2=1
                rospy.sleep(0.5)
        
        if step2 == 1:
                #Move torso to config
                set_named_target('torso', "torso_combs")
                go('torso')
                step2=2
                rospy.sleep(0.5)

        if step2 == 2:
                #Move pick_arm to predefined config (orientation for picking)
                set_named_target(main_arm, prepick_pose)
                step2+=move_group_async(main_arm)
                rospy.sleep(0.5)

        if step2 == 3:
                #Move arm to grasp cable (with approach+retract)
                waypoints_EC1 = []
                init_pose = get_current_pose(main_arm).pose
                waypoints_EC1.append(init_pose)
                waypoints_EC1.append(correctPose(pick_pose_offset, pick_arm, rotate = True, ATC_sign = -1, picking_app = True))
                plan, success = compute_cartesian_path_velocity_control([waypoints_EC1], [fast_speed_execution], arm_side=pick_arm)
                if success:
                        step2+=execute_plan_async(route_group, plan)

        if step2 == 4:
                actuate_grippers(config['open_distance'], config['gripper_speed'], pick_arm, config, grasp=False)
                waypoints_EC2 = []
                init_pose = get_current_pose(main_arm).pose
                waypoints_EC2.append(init_pose)
                waypoints_EC2.append(correctPose(pick_pose, pick_arm, rotate = True, ATC_sign = -1, picking_app = True))
                plan, success = compute_cartesian_path_velocity_control([waypoints_EC2], [slow_speed_execution], arm_side=pick_arm)
                if success:
                        step2+=execute_plan_async(route_group, plan)

        if step2 == 5:
                actuate_grippers(config['grasp_distance'], config['gripper_speed'], pick_arm, config, grasp=True)
                rospy.sleep(1.0)
                waypoints_EC3 = []
                init_pose = get_current_pose(main_arm).pose
                waypoints_EC3.append(init_pose)
                waypoints_EC3.append(correctPose(pick_pose_offset, pick_arm, rotate = True, ATC_sign = -1, picking_app = True))
                plan, success = compute_cartesian_path_velocity_control([waypoints_EC3], [slow_speed_execution], arm_side=pick_arm)
                if success:
                        step2+=execute_plan_async(route_group, plan)

        if step2 == 6:
                #Move to predefined dual-arm config
                set_named_target(main_arm, prepick_pose)
                step2+=move_group_async('arm_'+pick_arm)
                rospy.sleep(0.5)

        if step2 == 7:
                set_named_target('arms', "arms_platform_5")
                move_group_async("arms")
                step1_increase=1
                rospy.sleep(0.5)
                #PUBLISH FEEDBACK AFTER FINISHING EACH OPERATION
        
        return step2, step1_increase

########################## ADVANCED MANIPULATION SERVICES #######################################

def set_named_target(group, target):
        rospy.wait_for_service('/adv_manip/set_named_target')
        set_named_target_srv = rospy.ServiceProxy('/adv_manip/set_named_target', MotionGroupsCommand)
        req = MotionGroupsCommandRequest()
        req.group = group; req.target = target
        set_named_target_srv(req)

def go(group, wait=True):
        rospy.wait_for_service('/adv_manip/go')
        go_srv = rospy.ServiceProxy('/adv_manip/go', MotionGroupsCommand)
        req = MotionGroupsCommandRequest()
        req.group = group; req.wait = wait
        go_srv(req)

def correctPose():
        pass

def get_current_pose():
        pass #return pose_stamped

def compute_cartesian_path_velocity_control():
        pass

def move_group_async():
        pass #return step_inc

def execute_plan_async():
        pass

def get_fingers_size(side):
        pass


##########################################################

br = tf.TransformBroadcaster() 

def broadcastTransform(br, frame, frame_id, parent_frame, time=rospy.get_rostime()): 
    """
    Get the tf between 2 frames
    """
    br.sendTransform((frame.p.x(), frame.p.y(), frame.p.z()), 
        frame.M.GetQuaternion(), 
        time, 
        frame_id, 
        parent_frame)

def visualize_keypoints_simple(poses, parent_frame = "/torso_base_link"):
        """
        Visualize a list of poses [Pose] with respect to the parent_frame
        """
        while not rospy.is_shutdown():
                number = 2000
                for waypoint in poses:
                        current_time = rospy.get_rostime()
                        waypoint_frame = pose_to_frame(waypoint)
                        broadcastTransform(br, waypoint_frame, str(number), parent_frame, time=current_time)
                        number += 1