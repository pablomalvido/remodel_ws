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
from UI_nodes_pkg.msg import *
from UI_nodes_pkg.srv import *
from ROS_UI_backend.msg import *
from ROS_UI_backend.srv import *
import actionlib
from actionlib_msgs.msg import GoalStatusArray
from UI_nodes_pkg.msg import gripper_ActFeedback, gripper_ActResult, gripper_ActAction, gripper_ActGoal
from UI_nodes_pkg.msg import process_UIFeedback, process_UIResult, process_UIAction, process_UIGoal
from vision_pkg_full_demo.srv import *
from rosapi.srv import *
from norbdo_force_sensor.msg import forces
from motoman_msgs.srv import WriteSingleIO
from motoman_msgs.srv import WriteSingleIORequest
from industrial_msgs.msg import RobotStatus
from industrial_msgs.msg import TriState
from utils import *
from advanced_manipulation import *

def route_cables(op, connector_op, route_arm="left"):
        global rot_center
        global step1
        global step2
        global stop_mov
        global process_actionserver
        global larger_slide
        global force_limit_cable
        global force_limit
        global grasping_cables

        grasping_cables = False
        force_limit = copy.deepcopy(force_limit_cable)
        #IMPORTANT!!! We need to have sticks as corners too when the direction between guides is not the direction of the next guide
        min_dist_guides = 0.05
        max_angle_diff = 10.0*(math.pi/180.0)

        index = 0
        corrected_guides = copy.deepcopy(op)
        routing_operations = []
        
        for guide in op["spot"]:
                if index == 0:
                        prev_guide_end = get_shifted_pose(connector_op["spot"]["pose_corner"], [connector_op["spot"]['width'], connector_op["spot"]['gap']/2, 0, 0, 0, 0])
                        rot_center = copy.deepcopy(prev_guide_end)
                        next_guide_beginning = get_shifted_pose(guide['pose_corner'], [0, guide['gap']/2, 0, 0, 0, 0])
                        next_guide_end = get_shifted_pose(guide['pose_corner'], [guide['width'], guide['gap']/2, 0, 0, 0, 0])
                        if compute_distance(prev_guide_end, next_guide_end) < compute_distance(prev_guide_end, next_guide_beginning):
                                #rotate next guide
                                corrected_guides["spot"][0]["pose_corner"] = get_shifted_pose(guide["pose_corner"], [guide['width'], guide['gap'], 0, 0, 0, math.pi])

                        next_guide_beginning_corrected = get_shifted_pose(corrected_guides["spot"][0]["pose_corner"], [0, guide['gap']/2, 0, 0, 0, 0])
                        next_guide_end_corrected = get_shifted_pose(corrected_guides["spot"][0]["pose_corner"], [guide['width'], guide['gap']/2, 0, 0, 0, 0])
                        dist_guides = compute_distance(prev_guide_end, next_guide_beginning_corrected)

                        #Check corners
                        next_guide_dir = math.atan2(next_guide_end_corrected.position.y - next_guide_beginning_corrected.position.y, next_guide_end_corrected.position.x - next_guide_beginning_corrected.position.x)
                        cable_dir_angle = math.atan2(next_guide_beginning_corrected.position.y - prev_guide_end.position.y, next_guide_beginning_corrected.position.x - prev_guide_end.position.x)
                        if abs(next_guide_dir - cable_dir_angle) < max_angle_diff:
                                routing_operations.append({'op':"first_lift", 'next_guide': corrected_guides["spot"][0], 'prev_guide': connector_op["spot"]})
                        else:
                                routing_operations.append({'op':"first_corner", 'next_guide': corrected_guides["spot"][0], 'prev_guide': connector_op["spot"]})
                        index += 1
                        continue

                prev_guide_beginning = get_shifted_pose(corrected_guides["spot"][index-1]["pose_corner"], [0, corrected_guides["spot"][index-1]['gap']/2, 0, 0, 0, 0])
                prev_guide_end = get_shifted_pose(corrected_guides["spot"][index-1]["pose_corner"], [corrected_guides["spot"][index-1]['width'], corrected_guides["spot"][index-1]['gap']/2, 0, 0, 0, 0])
                next_guide_beginning = get_shifted_pose(guide['pose_corner'], [0, guide['gap']/2, 0, 0, 0, 0])
                next_guide_end = get_shifted_pose(guide['pose_corner'], [guide['width'], guide['gap']/2, 0, 0, 0, 0])
                if compute_distance(prev_guide_end, next_guide_end) < compute_distance(prev_guide_end, next_guide_beginning):
                        #rotate next guide
                        corrected_guides["spot"][index]["pose_corner"] = get_shifted_pose(guide["pose_corner"], [guide['width'], guide['gap'], 0, 0, 0, math.pi])

                #Check distance to next guide
                next_guide_beginning_corrected = get_shifted_pose(corrected_guides["spot"][index]["pose_corner"], [0, guide['gap']/2, 0, 0, 0, 0])
                next_guide_end_corrected = get_shifted_pose(corrected_guides["spot"][index]["pose_corner"], [guide['width'], guide['gap']/2, 0, 0, 0, 0])
                dist_guides = compute_distance(prev_guide_end, next_guide_beginning_corrected)

                #Check corners (cable and guides direction in xy plane)
                #prev guide
                prev_guide_dir = math.atan2(prev_guide_end.position.y - prev_guide_beginning.position.y, prev_guide_end.position.x - prev_guide_beginning.position.x)
                #next guide
                next_guide_dir = math.atan2(next_guide_end_corrected.position.y - next_guide_beginning_corrected.position.y, next_guide_end_corrected.position.x - next_guide_beginning_corrected.position.x)
                #cable direction
                cable_dir_angle = math.atan2(next_guide_beginning_corrected.position.y - prev_guide_end.position.y, next_guide_beginning_corrected.position.x - prev_guide_end.position.x)

                if abs(prev_guide_dir - next_guide_dir) < max_angle_diff and abs(prev_guide_dir - cable_dir_angle) < max_angle_diff:
                        #A1==A2==AL, No change of direction
                        if dist_guides < min_dist_guides: #Check also if there is a corner, in this case insert too and evaluate if it is possible to grasp
                                routing_operations.append({'op':"route_top", 'guides': [corrected_guides["spot"][index]]})
                        else:
                                routing_operations.append({'op':"insert_lift", 'next_guide': corrected_guides["spot"][index], 'prev_guide': corrected_guides["spot"][index-1]})

                elif abs(prev_guide_dir - next_guide_dir) > max_angle_diff and abs(next_guide_dir - cable_dir_angle) < max_angle_diff:
                        #A2==AL!=A1, Needs insert
                        routing_operations.append({'op':"insert_corner", 'next_guide': corrected_guides["spot"][index], 'prev_guide': corrected_guides["spot"][index-1]})
                else:
                        #A1==A2!=AL or A1==AL!=A2 or A1!=A2!=AL, Needs insert and corner
                        routing_operations.append({'op':"insert_grasp_corner", 'next_guide': corrected_guides["spot"][index], 'prev_guide': corrected_guides["spot"][index-1]})
                index += 1

        #Last guide insertion
        routing_operations.append({'op':"insert_final", 'next_guide': '', 'prev_guide': corrected_guides["spot"][-1]})

        #join operations together
        routing_operations2 = []
        for op in routing_operations:
                if len(routing_operations2)>0:
                        if op['op'] == "route_top" and routing_operations2[-1]['op'] == "route_top":
                                routing_operations2[-1]['guides'].append(op['guides'][0])
                                continue
                routing_operations2.append(op)

        #execute tasks of the created routing plan
        i=0
        if True:
                larger_slide = True
        else:
                larger_slide = False
        for op in routing_operations2:
                if i == step1:
                        print(op['op'])
                        if op['op'] == "first_lift":
                                RC_first_lift(op, route_arm)
                        elif op['op'] == "first_corner":
                                RC_first_corner(op, route_arm)
                        elif op['op'] == "route_top":
                                RC_route_top(op, route_arm)
                        elif op['op'] == "insert_lift":
                                RC_insert_lift(op, route_arm)
                        elif op['op'] == "insert_corner":
                                RC_insert_corner(op, route_arm)
                        elif op['op'] == "insert_grasp_corner":
                                RC_insert_grasp_corner(op, route_arm)
                        elif op['op'] == "insert_final":
                                RC_insert_final(op, route_arm)

                        if not stop_mov:
                                step1 += 1
                                step2 = 0
                                process_actionserver.publish_feedback()
                        else:
                               break
                i+=1


def RC_insert(op, route_arm):
        global step2
        global rot_center
        global speed_execution
        global speed_tension
        global force_controlled
        global rot_center_up
        global stop_mov_force
        global count
        global larger_slide

        waypoints1 = []
        if route_arm == "left":
                arm = arm_left
                EEF_route = ATC1.EEF_dict[ATC1.EEF_left].fingers_dim
                arm2 = arm_right
                arm2_side = "right"
        elif route_arm == "right":
                arm = arm_right
                EEF_route = ATC1.EEF_dict[ATC1.EEF_right].fingers_dim
                arm2 = arm_left
                arm2_side = "left"

        #insert_guide_center = get_shifted_pose(op["prev_guide"]["pose_corner"],[op["prev_guide"]['width']/2, op["prev_guide"]['gap']/2, EEF_route[2], 0, 0, 0])
        insert_guide_center = get_shifted_pose(op["prev_guide"]["pose_corner"],[op["prev_guide"]['width']/2, op["prev_guide"]['gap']/2, 0, 0, 0, 0])

        if step2==0:
                #Apply tension
                init_pose = arm.get_current_pose().pose
                if larger_slide:
                        top_guide_end = get_shifted_pose(op["prev_guide"]["pose_corner"],[op["prev_guide"]['width'] + EEF_route[0]/2 + 0.003, op["prev_guide"]['gap']/2, op["prev_guide"]['height'] + z_offset + EEF_route[2], 0, 0, 0]) #THE GUIDE WIDTH SHOULD BE DIVIDED BY 2, NOT BY 4
                else:
                        top_guide_end = get_shifted_pose(op["prev_guide"]["pose_corner"],[op["prev_guide"]['width']/4 + EEF_route[0]/2, op["prev_guide"]['gap']/2, op["prev_guide"]['height'] + z_offset + EEF_route[2], 0, 0, 0]) #THE GUIDE WIDTH SHOULD BE DIVIDED BY 2, NOT BY 4
                waypoints1.append(init_pose)
                waypoints1.append(ATC1.correctPose(top_guide_end, route_arm, rotate = True, ATC_sign = -1, routing_app = True))
                plan, success = compute_cartesian_path_velocity_control([waypoints1], [speed_execution], arm_side=route_arm)
                if success:
                        execute_plan_async(route_group, plan)
                        #arm.execute(plan, wait=True)
        
        if step2==1:
                actuate_grippers(grasp_distance, gripper_speed, route_arm, grasp=True)

                waypoints12 = []
                if larger_slide:
                        top_guide_end = get_shifted_pose(op["prev_guide"]["pose_corner"],[op["prev_guide"]['width'] + EEF_route[0]/2 + 0.003, op["prev_guide"]['gap']/2, op["prev_guide"]['height'] + z_offset + EEF_route[2], 0, 0, 0]) #THE GUIDE WIDTH SHOULD BE DIVIDED BY 2, NOT BY 4
                else:
                        top_guide_end = get_shifted_pose(op["prev_guide"]["pose_corner"],[op["prev_guide"]['width']/4 + EEF_route[0]/2, op["prev_guide"]['gap']/2, op["prev_guide"]['height'] + z_offset + EEF_route[2], 0, 0, 0]) #THE GUIDE WIDTH SHOULD BE DIVIDED BY 2, NOT BY 4
                waypoints12.append(ATC1.correctPose(top_guide_end, route_arm, rotate = True, ATC_sign = -1, routing_app = True))
                top_guide_forward = get_shifted_pose(top_guide_end,[force_offset, 0, 0, 0, 0, 0])
                waypoints12.append(ATC1.correctPose(top_guide_forward, route_arm, rotate = True, ATC_sign = -1, routing_app = True))
                #Move with force_control
                print("WAITING FOR SERVICE")
                if force_control_active:
                        rospy.wait_for_service('/right_norbdo/tare')
                        print("WAITING FOR TARING")
                        tare_forces_srv = rospy.ServiceProxy('right_norbdo/tare', Trigger)
                        tare_res = tare_forces_srv(TriggerRequest())
                        tare_forces_srvL = rospy.ServiceProxy('left_norbdo/tare', Trigger)
                        tare_res = tare_forces_srvL(TriggerRequest())
                rospy.sleep(0.5) #REDUCE. Original 0.5
                print("TARED CORRECTLY")
                stop_mov_force = False #reset this value before activating the force_controlled
                force_controlled = True
                plan, success = compute_cartesian_path_velocity_control([waypoints12], [speed_tension], arm_side=route_arm)
                if success:
                        success = execute_force_control(arm_side = route_arm, plan = plan)
                rospy.sleep(0.5) #REDUCE. Original 0.5
                force_controlled = False
                rospy.sleep(2.5) #REDUCE. Original 2.5

        if step2==2:
                if rot_center_up:
                        waypoints2_1 = []
                        waypoints2_2 = []
                        trash_pose = arm.get_current_pose().pose
                        init_pose1 = arm.get_current_pose().pose
                        waypoints2_1.append(init_pose1)
                        init_pose_guides = ATC1.antiCorrectPose(init_pose1, route_arm, routing_app = True) 
                        init_pose2 = arm2.get_current_pose().pose
                        waypoints2_2.append(init_pose2)
                        init_pose_guides2 = ATC1.antiCorrectPose(init_pose2, arm2_side, routing_app = True, route_arm = False) 
                        insert_pose_guides_1 = init_pose_guides
                        insert_pose_guides_1.position.z = insert_guide_center.position.z
                        waypoints2_1.append(ATC1.correctPose(insert_pose_guides_1, route_arm, rotate = True, ATC_sign = -1, routing_app = True))
                        insert_pose_guides_2 = init_pose_guides2
                        insert_pose_guides_2.position.z = insert_guide_center.position.z
                        waypoints2_2.append(ATC1.correctPose(insert_pose_guides_2, arm2_side, rotate = True, ATC_sign = -1, routing_app = True, route_arm = False))
                        #visualize_keypoints_simple([waypoints2_1[-1], waypoints2_2[-1]], parent_frame="/base_link")
                        if route_arm == "left":
                                plan, success = dual_arm_cartesian_plan([waypoints2_1], [speed_execution], [waypoints2_2], [speed_execution], ATC1= ATC1, sync_policy=1)
                        elif route_arm == "right":
                                plan, success = dual_arm_cartesian_plan([waypoints2_2], [speed_execution], [waypoints2_1], [speed_execution], ATC1= ATC1, sync_policy=1)
                        if success:
                                execute_plan_async("arms", plan)
                                #arms.execute(plan, wait=True)
                        rot_center = insert_pose_guides_2

                        if step2 == 3:
                               rot_center_up = False
                        return insert_pose_guides_1

                else:
                        print("###########TEST CIRC###########")
                        rot_center_up = False
                        #Circ motion
                        waypoints2 = []
                        trash_pose = arm.get_current_pose().pose
                        init_pose = arm.get_current_pose().pose
                        print("INIT POSE: " + str(init_pose))
                        waypoints2.append(copy.deepcopy(init_pose))
                        init_pose_guides = ATC1.antiCorrectPose(init_pose, route_arm, routing_app = True)
                        #init_pose_check = ATC1.correctPose(init_pose_guides, route_arm, rotate = True, ATC_sign = -1, routing_app = True)
                        #visualize_keypoints_simple([init_pose_guides, init_pose_check, init_pose], parent_frame = "/base_link")

                        radius_rot = compute_distance(rot_center, init_pose_guides)
                        #final_point = in the direction rot_center --> guide a distance of radius_rot
                        final_cable_direction = get_axis(insert_guide_center, rot_center)
                        final_point = copy.deepcopy(insert_guide_center) #Same orientation
                        final_point.position.x = rot_center.position.x + final_cable_direction[0]*radius_rot
                        final_point.position.y = rot_center.position.y + final_cable_direction[1]*radius_rot
                        final_point.position.z = rot_center.position.z + final_cable_direction[2]*radius_rot #Maybe delete?

                        #Evaluate collisions in final_point, if there are, there slide a bit more distance and evaluate again. Check also when the distance to the next guide is already too small, so then it will just do a slide_top
                        #With the correct final_point, calculates the rotation knowing, center, initial and final arc points.

                        guide_yaw_axis = get_axis_from_RM(pose_to_frame(rot_center).M, "Y")
                        rot_axis = get_ort_axis(guide_yaw_axis, final_cable_direction)[2]
                        rot_axis = [-element for element in rot_axis]
                        dist_z = compute_distance_relative(final_point, init_pose_guides, guide_yaw_axis) #from circle to guide in z axis
                        # print(dist_z)
                        # print(radius_rot)
                        rot_degree = (math.asin(-dist_z/radius_rot))*180.0/math.pi
                        print(rot_degree)

                        center_waypoints = []
                        circle_waypoints = []
                        success, center_waypoints, circle_waypoints = circular_trajectory(rot_center, init_pose_guides, rot_degree, rot_axis, center_waypoints, circle_waypoints, step = 2, rot_gripper = False)
                        for wp in circle_waypoints:
                                waypoints2.append(ATC1.correctPose(wp, route_arm, rotate = True, ATC_sign = -1, routing_app = True))
                                
                        #plan, fraction = arm.compute_cartesian_path(waypoints2, 0.01, 0.0)
                        #arm.execute(plan, wait=True)
                        plan, success = compute_cartesian_path_velocity_control([waypoints2], [speed_execution], arm_side=route_arm)
                        if success:
                                execute_plan_async(route_group, plan)
                                #arm.execute(plan, wait=True)
                        print("###########TEST CIRC END###########")
                        return circle_waypoints[-1]


def RC_push_inserted(push_arm):
        global step2
        global routed_guides
        global holding_cables
        global holding_comeback_pose_corrected
        routed_guides_copy = copy.deepcopy(routed_guides)
        for guide in routed_guides_copy:
                top_guide_beginning_backward = get_shifted_pose(guide["pose_corner"],[-(guide['width']/2 + EEF_route[0]/2 + grasp_offset), guide['gap']/2, guide['height'] + z_offset + EEF_route[2], 0, 0, 0])
                bottom_guide_beginning_backward = get_shifted_pose(guide["pose_corner"],[-(guide['width']/2 + EEF_route[0]/2 + grasp_offset), guide['gap']/2, guide['height']/4, 0, 0, 0])
                
                if step2==3:
                        if holding_cables:
                                actuate_grippers(slide_distance, gripper_speed, push_arm, grasp=False)
                                init_pose = arm2.get_current_pose().pose
                                waypoints_RCI0 = [init_pose, holding_comeback_pose_corrected]
                                plan, success = compute_cartesian_path_velocity_control([waypoints_RCI0], [speed_execution], arm_side=push_arm)
                                if success:
                                        execute_plan_async(group2, plan)
                                holding_cables = False
                        else:
                                step2+=1

                if step2==4:
                        #Push down with second arm
                        actuate_grippers(open_distance, gripper_speed, push_arm, grasp=False)
                        waypoints_RCI1 = []
                        init_pose = arm2.get_current_pose().pose
                        waypoints_RCI1.append(init_pose)
                        up_pose = copy.deepcopy(init_pose)
                        top_guide_beginning_backward_corrected = ATC1.correctPose(top_guide_beginning_backward, push_arm, rotate = True, ATC_sign = -1, routing_app = True, route_arm = False)
                        up_pose.position.z = top_guide_beginning_backward_corrected.position.z
                        waypoints_RCI1.append(up_pose)
                        plan, success = compute_cartesian_path_velocity_control([waypoints_RCI1], [speed_execution], arm_side=push_arm)
                        if success:
                                execute_plan_async(group2, plan)

                if step2==5:
                        waypoints_RCI2 = []
                        init_pose = arm2.get_current_pose().pose
                        waypoints_RCI2.append(init_pose)
                        waypoints_RCI2.append(ATC1.correctPose(top_guide_beginning_backward, push_arm, rotate = True, ATC_sign = -1, routing_app = True, route_arm = False))
                        plan, success = compute_cartesian_path_velocity_control([waypoints_RCI2], [fast_speed_execution], arm_side=push_arm)
                        if success:
                                execute_plan_async(group2, plan)

                if step2==6:
                        actuate_grippers(grasp_distance, gripper_speed, push_arm, grasp=False)
                        waypoints_RCI3 = []
                        init_pose = arm2.get_current_pose().pose
                        waypoints_RCI3.append(init_pose)
                        waypoints_RCI3.append(ATC1.correctPose(bottom_guide_beginning_backward, push_arm, rotate = True, ATC_sign = -1, routing_app = True, route_arm = False, secondary_frame=True))
                        plan, success = compute_cartesian_path_velocity_control([waypoints_RCI3], [slow_speed_execution], arm_side=push_arm)
                        if success:
                                execute_plan_async(group2, plan)

                if step2==7:
                        waypoints_RCI4 = []
                        init_pose = arm2.get_current_pose().pose
                        waypoints_RCI4.append(init_pose)
                        waypoints_RCI4.append(ATC1.correctPose(top_guide_beginning_backward, push_arm, rotate = True, ATC_sign = -1, routing_app = True, route_arm = False))
                        plan, success = compute_cartesian_path_velocity_control([waypoints_RCI4], [speed_execution], arm_side=push_arm)
                        if success:
                                execute_plan_async(group2, plan)
                        routed_guides.pop(0) #remove the guide that has been pushed from the list
                        step2 = 3

        if len(routed_guides)==0:
                step2 = 8                                
       

def RC_first_lift(op, route_arm):
        global step2
        global speed_execution
        global slide_distance
        global gripper_speed
        global routed_guides

        waypoints = []
        if route_arm == "left":
                arm = arm_left
                EEF_route = ATC1.EEF_dict[ATC1.EEF_left].fingers_dim
        elif route_arm == "right":
                arm = arm_right
                EEF_route = ATC1.EEF_dict[ATC1.EEF_right].fingers_dim

        if step2==0:
                actuate_grippers(slide_distance, gripper_speed, route_arm, grasp=False)
                rospy.sleep(0.5)
                init_pose = arm.get_current_pose().pose
                waypoints.append(init_pose)
                next_guide_top_beginning = get_shifted_pose(op["next_guide"]["pose_corner"],[- EEF_route[0]/2, op["next_guide"]['gap']/2, op["next_guide"]['height'] + z_offset + EEF_route[2], 0, 0, 0])
                next_guide_top = get_shifted_pose(op["next_guide"]["pose_corner"],[op["next_guide"]['width']/2, op["next_guide"]['gap']/2, op["next_guide"]['height'] + z_offset + EEF_route[2], 0, 0, 0])
                waypoints.append(ATC1.correctPose(next_guide_top_beginning, route_arm, rotate = True, ATC_sign = -1, routing_app = True))
                #plan, success = compute_cartesian_path_velocity_control([waypoints], [speed_execution], arm_side=route_arm)
                plan, success, motion_group_plan = compute_cartesian_path_velocity_control_arms_occlusions([waypoints], [speed_execution], arm_side=route_arm)
                if success:
                        execute_plan_async(motion_group_plan, plan)
                        #arm.execute(plan, wait=True)
        if step2==1:
                waypoints2 = []
                init_pose = arm.get_current_pose().pose
                waypoints2.append(init_pose)
                #waypoints2.append(waypoints[-1])
                waypoints.append(ATC1.correctPose(next_guide_top, route_arm, rotate = True, ATC_sign = -1, routing_app = True))
                plan, success = compute_cartesian_path_velocity_control([waypoints2], [speed_execution], arm_side=route_arm)
                if success:
                        execute_plan_async(route_group, plan)
                        #arm.execute(plan, wait=True)

        if step2==2:
                routed_guides.append(op['next_guide'])


def RC_first_corner(op, route_arm):
        global step2
        global rot_center
        global speed_execution
        global speed_tension
        global rot_center_up
        global slide_distance
        global grasp_distance
        global gripper_speed
        #Apply tension (Move forward 3cm max (stopped by "force controller"))
        #Insert (Circular motion)
        #Slide and lift (till x_offset)
        #Apply tension to cable
        #Bring second hand (lift from wherever it is, move to offset, go to the corner cable position to grasp)
        #Move first arm to the end of the top of the guide
        #Save in a global variable that in the next insert, it must be done with both arms and linear motion (Instead of circular)

        if route_arm == "left":
                arm = arm_left
                EEF_route = ATC1.EEF_dict[ATC1.EEF_left].fingers_dim
                arm2 = arm_right
                arm2_side = "right"
                EEF_route2 = ATC1.EEF_dict[ATC1.EEF_right].fingers_dim
        elif route_arm == "right":
                arm = arm_right
                EEF_route = ATC1.EEF_dict[ATC1.EEF_right].fingers_dim
                arm2 = arm_left
                arm2_side = "left"
                EEF_route2 = ATC1.EEF_dict[ATC1.EEF_left].fingers_dim

        
        #Slide
        waypoints3 = []
        init_pose = arm.get_current_pose().pose
        waypoints3.append(init_pose)
        next_guide_beginning = get_shifted_pose(op["next_guide"]["pose_corner"],[-EEF_route[0]/2, op["next_guide"]['gap']/2, z_offset2, 0, 0, 0])
        next_guide_top = get_shifted_pose(op["next_guide"]["pose_corner"],[op["next_guide"]['width']/2, op["next_guide"]['gap']/2, op["next_guide"]['height'] + z_offset + EEF_route[2], 0, 0, 0])
        next_guide_top_beginning = get_shifted_pose(op["next_guide"]["pose_corner"],[-EEF_route[0]/2, op["next_guide"]['gap']/2, op["next_guide"]['height'] + z_offset + EEF_route[2], 0, 0, 0])
        next_guide_top_beginning_route_offset1 = get_shifted_pose(op["next_guide"]["pose_corner"],[-max(EEF_route[0]/2 + x_offset, EEF_route[1]/2), op["next_guide"]['gap']/2, op["next_guide"]['height'] + z_offset + EEF_route[2], 0, 0, 0])        
        prev_guide_end = get_shifted_pose(op["prev_guide"]["pose_corner"],[EEF_route[0]/2, op["next_guide"]['gap']/2, 0, 0, 0, 0])
        
        cable_dir_angle = math.atan2(next_guide_top_beginning_route_offset1.position.y - prev_guide_end.position.y, next_guide_top_beginning_route_offset1.position.x - prev_guide_end.position.x)
        next_guide_top_beginning_route_offset1.orientation = get_quaternion_in_Z(cable_dir_angle).orientation
        xy_dist = math.sqrt((next_guide_top_beginning_route_offset1.position.x - prev_guide_end.position.x)**2 + (next_guide_top_beginning_route_offset1.position.y - prev_guide_end.position.y)**2)
        z_increase = (x_offset + EEF_route[0])*((next_guide_top_beginning_route_offset1.position.z - prev_guide_end.position.z)/(xy_dist))
        next_guide_top_beginning_route_offset2 = get_shifted_pose(next_guide_top_beginning_route_offset1,[x_offset + EEF_route[0], 0, z_increase, 0, 0, 0])
        waypoints3.append(ATC1.correctPose(next_guide_top_beginning_route_offset2, route_arm, rotate = True, ATC_sign = -1, routing_app = True))

        if step2==0:        
                actuate_grippers(slide_distance, gripper_speed, route_arm, grasp=False)
                plan, success = compute_cartesian_path_velocity_control([waypoints3], [speed_execution], arm_side=route_arm)
                if success:
                        execute_plan_async(route_group, plan)
                        #arm.execute(plan, wait=True)

        if step2==1:
                #Apply tension ToDo

                #Bring second hand
                waypoints4 = []
                init_pose = arm2.get_current_pose().pose
                waypoints4.append(init_pose)
                grasp_pose_up = get_shifted_pose(next_guide_top_beginning_route_offset1, [0, 0, z_offset+ EEF_route2[2], 0, 0, 0])
                grasp_pose_up_corrected = ATC1.correctPose(grasp_pose_up, arm2_side, rotate = True, ATC_sign = -1, routing_app = True, route_arm = False)
                init_pose_up = copy.deepcopy(init_pose)
                init_pose_up.position.z = grasp_pose_up_corrected.position.z
                waypoints4.append(init_pose_up)
                waypoints4.append(ATC1.correctPose(grasp_pose_up, arm2_side, rotate = True, ATC_sign = -1, routing_app = True, route_arm = False))
                waypoints4.append(ATC1.correctPose(next_guide_top_beginning_route_offset1, arm2_side, rotate = True, ATC_sign = -1, routing_app = True, route_arm = False))
                #visualize_keypoints_simple([grasp_pose_up, next_guide_top_beginning_route_offset1, next_guide_top_beginning_route_offset2])
                #visualize_keypoints_simple([waypoints4[-1], waypoints3[-1]], parent_frame = "/base_link")
                plan, success = compute_cartesian_path_velocity_control([waypoints4], [speed_execution], arm_side=arm2_side)
                if success:
                        execute_plan_async(group2, plan)
                        #arm2.execute(plan, wait=True)

        if step2==2:
                #Apply tension movement. ToDo
                actuate_grippers(grasp_distance, gripper_speed, route_arm, grasp=True)

                #Move arm down 
                waypoints5 = []
                init_pose = arm.get_current_pose().pose
                init_pose_guides = ATC1.antiCorrectPose(init_pose, route_arm, routing_app = True)
                waypoints5.append(init_pose)
                pose_down = copy.deepcopy(init_pose_guides)
                pose_down.position.z = next_guide_top_beginning_route_offset1.position.z
                waypoints5.append(ATC1.correctPose(pose_down, route_arm, rotate = True, ATC_sign = -1, routing_app = True))
                plan, success = compute_cartesian_path_velocity_control([waypoints5], [speed_execution], arm_side=route_arm)
                if success:
                        execute_plan_async(route_group, plan)
                        #arm.execute(plan, wait=True)

        if step2==3:
                #Rotate arms
                # waypoints6_circ_wrist = []
                # waypoints6_center_wrist = []
                # waypoints6_circ = []
                # waypoints6_center = []
                # init_pose = arm.get_current_pose().pose
                # init_pose_guides = ATC1.antiCorrectPose(init_pose, route_arm, routing_app = True)

                # corner_center = next_guide_top_beginning_route_offset1
                # radius_rot = compute_distance(rot_center, init_pose_guides)
                
                # line_center = get_shifted_pose(corner_center, [1,0,0,0,0,0])
                # angle_center = math.atan2(line_center.position.y - corner_center.position.y, line_center.position.x - corner_center.position.x)
                # line_guide = get_shifted_pose(next_guide_top, [1,0,0,0,0,0])
                # angle_guide = math.atan2(line_guide.position.y - next_guide_top.position.y, line_guide.position.x - next_guide_top.position.x)
                # rot_degree = angle_guide - angle_center 

                # center_waypoints = []
                # circle_waypoints = []
                # success, waypoints6_center_wrist, waypoints6_circ_wrist = circular_trajectory(rot_center, init_pose_guides, rot_degree, [0,0,1], waypoints6_center_wrist, waypoints6_circ_wrist, step = 2, rot_gripper = False)
                # for wp in waypoints6_circ_wrist:
                #         waypoints6_circ.append(ATC1.correctPose(wp, route_arm, rotate = True, ATC_sign = -1, routing_app = True))
                # for wp in waypoints6_center_wrist:
                #         waypoints6_center.append(ATC1.correctPose(wp, arm2_side, rotate = True, ATC_sign = -1, routing_app = True, route_arm = False))

                # if route_arm == "left":
                #         plan, success = dual_arm_cartesian_plan([waypoints6_circ], [speed_execution], [waypoints6_center], [speed_execution], ATC1= ATC1, sync_policy=1)
                # elif route_arm == "right":
                #         plan, success = dual_arm_cartesian_plan([waypoints6_center], [speed_execution], [waypoints6_circ], [speed_execution], ATC1= ATC1, sync_policy=1)
                # arms.execute(plan)
                waypoints6_wrist = []
                init_pose = arm.get_current_pose().pose
                init_pose_guides = ATC1.antiCorrectPose(init_pose, route_arm, routing_app = True)
                success, waypoints6_wrist = interpolate_trajectory(initial_pose = init_pose_guides, final_pose = next_guide_top, step_pos_min = 0.01, step_deg_min = 5, n_points_max = 20)
                if not success:
                        return
                waypoints6 = []
                for wp in waypoints6_wrist:
                        waypoints6.append(ATC1.correctPose(wp, route_arm, rotate = True, ATC_sign = -1, routing_app = True))
                plan, success = master_slave_plan(waypoints6, ATC1, 30.0, route_arm, type=1)
                execute_plan_async("arms", plan)
                #arms.execute(plan, wait=True)

                rot_center_jigs = arm2.get_current_pose().pose
                rot_center = ATC1.antiCorrectPose(rot_center_jigs, arm2_side, routing_app = True, route_arm = False)
                rot_center_up = True

        if step2==4:
                routed_guides.append(op['next_guide'])


def RC_route_top(op, route_arm):
        global step2
        global speed_execution
        global routed_guides
        waypoints = []
        if route_arm == "left":
                arm = arm_left
                EEF_route = EEF_route = ATC1.EEF_dict[ATC1.EEF_left].fingers_dim
        elif route_arm == "right":
                arm = arm_right
                EEF_route = EEF_route = ATC1.EEF_dict[ATC1.EEF_right].fingers_dim

        if step2==0:
                waypoints.append(arm.get_current_pose().pose)
                for guide in op["guides"][1:]:
                        top_guide = get_shifted_pose(guide["pose_corner"],[guide['width']/2, guide['gap']/2, guide['height'] + z_offset + EEF_route[2], 0, 0, 0])
                        waypoints.append(ATC1.correctPose(top_guide, route_arm, rotate = True, ATC_sign = -1, routing_app = True))

                plan, success = compute_cartesian_path_velocity_control([waypoints], [speed_execution], arm_side=route_arm)
                if success:
                        execute_plan_async(route_group, plan)
                        #arm.execute(plan, wait=True)

        routed_guides.pop(-1)
        if step2==1:
                for guide in op["guides"]:
                        routed_guides.append(guide)


def RC_insert_lift(op, route_arm):
        global rot_center
        global speed_execution
        global fast_speed_execution
        global speed_tension
        global step2
        global slide_distance
        global grasp_distance
        global open_distance
        global gripper_speed
        global routed_guides
        global holding_cables
        global holding_comeback_pose_corrected
        #Apply tension (Move forward 3cm max (stopped by "force controller"))
        #Insert (Circular motion)
        #Slide (till x_offset)
        #Bring second hand (lift from wherever it is, move to offset, go down and grasp)
        #Lift (beginning and forward)

        if route_arm == "left":
                arm = arm_left
                EEF_route = ATC1.EEF_dict[ATC1.EEF_left].fingers_dim
                arm2 = arm_right
                arm2_side = "right"
                EEF_route2 = ATC1.EEF_dict[ATC1.EEF_right].fingers_dim
        elif route_arm == "right":
                arm = arm_right
                EEF_route = ATC1.EEF_dict[ATC1.EEF_right].fingers_dim
                arm2 = arm_left
                arm2_side = "left"
                EEF_route2 = ATC1.EEF_dict[ATC1.EEF_left].fingers_dim
        
        if step2<3:
                #Apply tension and insert
                final_insert_wp = RC_insert(op, route_arm)
        
        if step2>=3 and step2<8:
                RC_push_inserted(arm2_side)

        if step2>=8 and step2<=12:
                #Slide
                actuate_grippers(slide_distance, gripper_speed, route_arm, grasp=False)

                waypoints3_wrist = []
                init_pose1 = arm.get_current_pose().pose
                final_insert_wp = ATC1.antiCorrectPose(init_pose1, route_arm, routing_app = True) 
                waypoints3_wrist.append(final_insert_wp)
                prev_guide_end = get_shifted_pose(op["prev_guide"]["pose_corner"],[op["prev_guide"]['width']/2 + EEF_route[0]/2, op["prev_guide"]['gap']/2, z_offset2, 0, 0, 0])
                next_guide_beginning = get_shifted_pose(op["next_guide"]["pose_corner"],[-EEF_route[0]/2, op["next_guide"]['gap']/2, z_offset2, 0, 0, 0])
                next_guide_top_beginning = get_shifted_pose(op["next_guide"]["pose_corner"],[-EEF_route[0], op["next_guide"]['gap']/2, op["next_guide"]['height'] + z_offset + EEF_route[2], 0, 0, 0])
                next_guide_top = get_shifted_pose(op["next_guide"]["pose_corner"],[op["next_guide"]['width']/2, op["next_guide"]['gap']/2, op["next_guide"]['height'] + z_offset + EEF_route[2], 0, 0, 0])
                if compute_distance(prev_guide_end, next_guide_beginning) > x_offset:
                        if step2==8:
                                next_guide_offset = get_shifted_pose(op["next_guide"]["pose_corner"],[-x_offset - EEF_route[0]/2, op["next_guide"]['gap']/2, z_offset2, 0, 0, 0])
                                waypoints3_wrist.append(next_guide_offset)
                                waypoints3_fingers = []
                                for wp in waypoints3_wrist:
                                        waypoints3_fingers.append(ATC1.correctPose(wp, route_arm, rotate = True, ATC_sign = -1, routing_app = True))
                                #print(waypoints3_fingers)
                                waypoints3_fingers[0] = init_pose1
                                plan, success = compute_cartesian_path_velocity_control([waypoints3_fingers], [speed_execution], arm_side=route_arm, step = 0.001)
                                if success:
                                        execute_plan_async(route_group, plan)
                                        #arm.execute(plan, wait=True)

                        if step2==9:
                                #Bring second hand
                                if holding_cables:
                                        actuate_grippers(slide_distance, gripper_speed, arm2_side, grasp=False)
                                        init_pose = arm2.get_current_pose().pose
                                        waypoints40 = [init_pose, holding_comeback_pose_corrected]
                                        plan, success = compute_cartesian_path_velocity_control([waypoints40], [speed_execution], arm_side=arm2_side)
                                        if success:
                                                execute_plan_async(group2, plan)
                                else:
                                        step2+=1
                        
                        if step2==10:
                                actuate_grippers(open_distance, gripper_speed, arm2_side, grasp=False)
                                waypoints4 = []
                                prev_guide_forward = get_shifted_pose(op["prev_guide"]["pose_corner"],[op["prev_guide"]['width'] + grasp_offset + EEF_route2[0]/2, op["prev_guide"]['gap']/2, z_offset2, 0, 0, 0])
                                prev_guide_forward_up = get_shifted_pose(prev_guide_forward, [0, 0, op["prev_guide"]['height'] + z_offset*2 + EEF_route2[2], 0, 0, 0])
                                prev_guide_forward_up_corrected = ATC1.correctPose(prev_guide_forward_up, arm2_side, rotate = True, ATC_sign = -1, routing_app = True, route_arm = False)
                                init_pose = arm2.get_current_pose().pose
                                waypoints4.append(init_pose)
                                init_pose_up = copy.deepcopy(init_pose)
                                init_pose_up.position.z = prev_guide_forward_up_corrected.position.z
                                waypoints4.append(init_pose_up)
                                waypoints4.append(ATC1.correctPose(prev_guide_forward_up, arm2_side, rotate = True, ATC_sign = -1, routing_app = True, route_arm = False))
                                plan, success = compute_cartesian_path_velocity_control([waypoints4], [fast_speed_execution], arm_side=arm2_side)
                                if success:
                                        execute_plan_async(group2, plan)
                                        #arm2.execute(plan, wait=True)
                                #Apply tension movement. ToDo

                        if step2==11:
                                waypoints5 = []
                                init_pose = arm2.get_current_pose().pose
                                waypoints5.append(init_pose)
                                prev_guide_forward = get_shifted_pose(op["prev_guide"]["pose_corner"],[op["prev_guide"]['width'] + grasp_offset + EEF_route2[0]/2, op["prev_guide"]['gap']/2, z_offset2, 0, 0, 0])
                                waypoints5.append(ATC1.correctPose(prev_guide_forward, arm2_side, rotate = True, ATC_sign = -1, routing_app = True, route_arm = False))
                                plan, success = compute_cartesian_path_velocity_control([waypoints5], [speed_execution], arm_side=arm2_side)
                                if success:
                                        execute_plan_async(group2, plan)
                                rot_center = copy.deepcopy(prev_guide_forward)

                        if step2==12:
                                actuate_grippers(grasp_distance, gripper_speed, arm2_side, grasp=True)
                                step2=13

                else:
                        prev_guide_center = get_shifted_pose(op["prev_guide"]["pose_corner"],[op["prev_guide"]['width']/2, op["prev_guide"]['gap']/2, op["prev_guide"]['height']/2, 0, 0, 0])
                        rot_center = copy.deepcopy(prev_guide_center)
                        step2=13

        if step2==13:
                #Slide to the top
                waypoints5 = []
                init_pose = arm.get_current_pose().pose
                waypoints5.append(init_pose)
                waypoints5.append(ATC1.correctPose(next_guide_top_beginning, route_arm, rotate = True, ATC_sign = -1, routing_app = True))
                plan, success = compute_cartesian_path_velocity_control([waypoints5], [speed_execution], arm_side=route_arm)
                if success:
                        execute_plan_async(route_group, plan)
                        #arm.execute(plan, wait=True)

        if step2==14:
                waypoints52 = []
                waypoints52.append(ATC1.correctPose(next_guide_top_beginning, route_arm, rotate = True, ATC_sign = -1, routing_app = True))
                waypoints52.append(ATC1.correctPose(next_guide_top, route_arm, rotate = True, ATC_sign = -1, routing_app = True))
                plan, success = compute_cartesian_path_velocity_control([waypoints52], [speed_execution], arm_side=route_arm)
                if success:
                        execute_plan_async(route_group, plan)
                        #arm.execute(plan, wait=True)

        if step2==15:
                routed_guides.append(op['next_guide'])


def RC_insert_corner(op, route_arm):
        global step2
        global rot_center
        global speed_execution
        global speed_tension
        global slide_distance
        global grasp_distance
        global gripper_speed

        if route_arm == "left":
                arm = arm_left
                EEF_route = ATC1.EEF_dict[ATC1.EEF_left].fingers_dim
                arm2 = arm_right
                arm2_side = "right"
                EEF_route2 = ATC1.EEF_dict[ATC1.EEF_right].fingers_dim
        elif route_arm == "right":
                arm = arm_right
                EEF_route = ATC1.EEF_dict[ATC1.EEF_right].fingers_dim
                arm2 = arm_left
                arm2_side = "left"
                EEF_route2 = ATC1.EEF_dict[ATC1.EEF_left].fingers_dim
        
        if step2<3:
                #Apply tension and insert
                final_insert_wp = RC_insert(op, route_arm)  

        if step2>=3 and step2<8:
                RC_push_inserted(arm2_side)

        if step2==8:
                #Slide
                actuate_grippers(slide_distance, gripper_speed, route_arm, grasp=False)

                #Move up
                init_pose1 = arm.get_current_pose().pose
                final_insert_wp = ATC1.antiCorrectPose(init_pose1, route_arm, routing_app = True) 
                # prev_guide_top = get_shifted_pose(op["prev_guide"]["pose_corner"],[op["next_guide"]['width']/2, op["next_guide"]['gap']/2, op["next_guide"]['height'] + z_offset + EEF_route[2], 0, 0, 0])
                # waypoints2 = []
                # waypoints2.append(init_pose1)
                # insert_up_wp = final_insert_wp
                # insert_up_wp.position.z = prev_guide_top.position.z
                # waypoints2.append(ATC1.correctPose(insert_up_wp, route_arm, rotate = True, ATC_sign = -1, routing_app = True))
                waypoints2 = []
                waypoints2.append(init_pose1)
                next_guide = get_shifted_pose(op["next_guide"]["pose_corner"],[-x_offset - EEF_route[0], op["next_guide"]['gap']/2, op["next_guide"]['height']/2, 0, 0, 0])
                waypoints2.append(ATC1.correctPose(next_guide, route_arm, rotate = True, ATC_sign = -1, routing_app = True))
                plan, success = compute_cartesian_path_velocity_control([waypoints2], [speed_execution], arm_side=route_arm)
                if success:
                        execute_plan_async(route_group, plan)
                        #arm.execute(plan, wait=True)

        # if step2==8:
        #         #Move to next guide
        #         prev_guide_top = get_shifted_pose(op["next_guide"]["pose_corner"],[op["next_guide"]['width']/2, op["next_guide"]['gap']/2, op["next_guide"]['height'] + z_offset + EEF_route[2], 0, 0, 0])
        #         waypoints3 = []
        #         #waypoints3.append(waypoints2[-1])
        #         init_pose = arm.get_current_pose().pose
        #         waypoints3.append(init_pose)
        #         waypoints3.append(ATC1.correctPose(prev_guide_top, route_arm, rotate = True, ATC_sign = -1, routing_app = True))
        #         plan, success = compute_cartesian_path_velocity_control([waypoints3], [speed_execution], arm_side=route_arm)
        #         if success:
        #                 execute_plan_async(route_group, plan)
        #                 #arm.execute(plan, wait=True)

        #         prev_guide_center = get_shifted_pose(op["prev_guide"]["pose_corner"],[op["prev_guide"]['width']/2, op["prev_guide"]['gap']/2, op["prev_guide"]['height']/2, 0, 0, 0])
        #         rot_center = copy.deepcopy(prev_guide_center)

        # if step2==9:
        #         routed_guides.append(op['next_guide'])

        if step2==9:
                #Bring second hand
                waypoints3 = []
                prev_guide_backward = get_shifted_pose(op["prev_guide"]["pose_corner"],[- grasp_offset - EEF_route2[0]/2, op["prev_guide"]['gap']/2, z_offset2, 0, 0, 0])
                prev_guide_backward_up = get_shifted_pose(prev_guide_backward, [0, 0, op["prev_guide"]['height'] + z_offset*2 + EEF_route2[2], 0, 0, 0])
                prev_guide_backward_up_corrected = ATC1.correctPose(prev_guide_backward_up, arm2_side, rotate = True, ATC_sign = -1, routing_app = True, route_arm = False)
                init_pose = arm2.get_current_pose().pose
                waypoints3.append(init_pose)
                init_pose_up = copy.deepcopy(init_pose)
                init_pose_up.position.z = prev_guide_backward_up_corrected.position.z
                waypoints3.append(init_pose_up)
                waypoints3.append(ATC1.correctPose(prev_guide_backward_up, arm2_side, rotate = True, ATC_sign = -1, routing_app = True, route_arm = False))
                plan, success = compute_cartesian_path_velocity_control([waypoints3], [fast_speed_execution], arm_side=arm2_side)
                if success:
                        execute_plan_async(group2, plan)
                        #arm2.execute(plan, wait=True)
                #Apply tension movement. ToDo

        if step2==10:
                actuate_grippers(open_distance, gripper_speed, arm2_side, grasp=False)
                waypoints4 = []
                init_pose = arm2.get_current_pose().pose
                waypoints4.append(init_pose)
                prev_guide_backward = get_shifted_pose(op["prev_guide"]["pose_corner"],[- grasp_offset - EEF_route2[0]/2, op["prev_guide"]['gap']/2, z_offset2, 0, 0, 0])
                waypoints4.append(ATC1.correctPose(prev_guide_backward, arm2_side, rotate = True, ATC_sign = -1, routing_app = True, route_arm = False))
                plan, success = compute_cartesian_path_velocity_control([waypoints4], [speed_execution], arm_side=arm2_side)
                if success:
                        execute_plan_async(group2, plan)
                rot_center = copy.deepcopy(prev_guide_backward)

        if step2==11:
                actuate_grippers(grasp_distance, gripper_speed, arm2_side, grasp=True)
                step2=12

        if step2==12:
                #Slide to the top
                waypoints5 = []
                init_pose = arm.get_current_pose().pose
                waypoints5.append(init_pose)
                next_guide_top_beginning = get_shifted_pose(op["next_guide"]["pose_corner"],[-EEF_route[0], op["next_guide"]['gap']/2, op["next_guide"]['height'] + z_offset + EEF_route[2], 0, 0, 0])
                waypoints5.append(ATC1.correctPose(next_guide_top_beginning, route_arm, rotate = True, ATC_sign = -1, routing_app = True))
                plan, success = compute_cartesian_path_velocity_control([waypoints5], [speed_execution], arm_side=route_arm)
                if success:
                        execute_plan_async(route_group, plan)
                        #arm.execute(plan, wait=True)

        if step2==13:
                waypoints6 = []
                next_guide_top_beginning = get_shifted_pose(op["next_guide"]["pose_corner"],[-EEF_route[0], op["next_guide"]['gap']/2, op["next_guide"]['height'] + z_offset + EEF_route[2], 0, 0, 0])
                next_guide_top = get_shifted_pose(op["next_guide"]["pose_corner"],[op["next_guide"]['width']/2, op["next_guide"]['gap']/2, op["next_guide"]['height'] + z_offset + EEF_route[2], 0, 0, 0])
                waypoints6.append(ATC1.correctPose(next_guide_top_beginning, route_arm, rotate = True, ATC_sign = -1, routing_app = True))
                waypoints6.append(ATC1.correctPose(next_guide_top, route_arm, rotate = True, ATC_sign = -1, routing_app = True))
                plan, success = compute_cartesian_path_velocity_control([waypoints6], [speed_execution], arm_side=route_arm)
                if success:
                        execute_plan_async(route_group, plan)
                        #arm.execute(plan, wait=True)

        if step2==14:
                routed_guides.append(op['next_guide'])


def RC_insert_grasp_corner(op, route_arm):
        global step2
        global rot_center
        global speed_execution
        global speed_tension
        global rot_center_up
        global slide_distance
        global grasp_distance
        global gripper_speed
        global force_offset
        #Apply tension (Move forward 3cm max (stopped by "force controller"))
        #Insert (Circular motion)
        #Slide and lift (till x_offset)
        #Apply tension to cable
        #Bring second hand (lift from wherever it is, move to offset, go to the corner cable position to grasp)
        #Move first arm to the end of the top of the guide
        #Save in a global variable that in the next insert, it must be done with both arms and linear motion (Instead of circular)

        if route_arm == "left":
                arm = arm_left
                EEF_route = ATC1.EEF_dict[ATC1.EEF_left].fingers_dim
                arm2 = arm_right
                arm2_side = "right"
                EEF_route2 = ATC1.EEF_dict[ATC1.EEF_right].fingers_dim
        elif route_arm == "right":
                arm = arm_right
                EEF_route = ATC1.EEF_dict[ATC1.EEF_right].fingers_dim
                arm2 = arm_left
                arm2_side = "left"
                EEF_route2 = ATC1.EEF_dict[ATC1.EEF_left].fingers_dim
        
        next_guide_beginning = get_shifted_pose(op["next_guide"]["pose_corner"],[-EEF_route[0]/2, op["next_guide"]['gap']/2, z_offset2, 0, 0, 0])
        next_guide_top = get_shifted_pose(op["next_guide"]["pose_corner"],[op["next_guide"]['width']/2, op["next_guide"]['gap']/2, op["next_guide"]['height'] + z_offset + EEF_route[2], 0, 0, 0])
        next_guide_top_beginning = get_shifted_pose(op["next_guide"]["pose_corner"],[-EEF_route[0]/2, op["next_guide"]['gap']/2, op["next_guide"]['height'] + z_offset + EEF_route[2], 0, 0, 0])
        next_guide_top_beginning_route_visualize = get_shifted_pose(op["next_guide"]["pose_corner"],[0,0,0,0,0,0])
        next_guide_top_beginning_route_offset1 = get_shifted_pose(op["next_guide"]["pose_corner"],[-max(EEF_route[0]/2 + x_offset, EEF_route[1]/2), op["next_guide"]['gap']/2, op["next_guide"]['height'] + z_offset + EEF_route[2], 0, 0, 0])        
        prev_guide_end = get_shifted_pose(op["prev_guide"]["pose_corner"],[EEF_route[0]/2, op["next_guide"]['gap']/2, 0, 0, 0, 0])
        
        cable_dir_angle = math.atan2(next_guide_top_beginning_route_offset1.position.y - prev_guide_end.position.y, next_guide_top_beginning_route_offset1.position.x - prev_guide_end.position.x)
        next_guide_top_beginning_route_offset1.orientation = get_quaternion_in_Z(cable_dir_angle).orientation
        xy_dist = math.sqrt((next_guide_top_beginning_route_offset1.position.x - prev_guide_end.position.x)**2 + (next_guide_top_beginning_route_offset1.position.y - prev_guide_end.position.y)**2)
        z_increase = (x_offset + EEF_route[0])*((next_guide_top_beginning_route_offset1.position.z - prev_guide_end.position.z)/(xy_dist))
        next_guide_top_beginning_route_offset2 = get_shifted_pose(next_guide_top_beginning_route_offset1,[x_offset + EEF_route[0], 0, z_increase, 0, 0, 0])
        
        if step2<3:
                #Apply tension and insert
                final_insert_wp = RC_insert(op, route_arm)

        if step2>=3 and step2<8:
                RC_push_inserted(arm2_side)

        if step2==8:
        #Slide
                actuate_grippers(slide_distance, gripper_speed, route_arm, grasp=False)

                waypoints3_wrist = []
                init_pose1 = arm.get_current_pose().pose
                final_insert_wp = ATC1.antiCorrectPose(init_pose1, route_arm, routing_app = True) 
                waypoints3_wrist.append(final_insert_wp)
                waypoints3_wrist.append(next_guide_top_beginning_route_offset2)
                waypoints3 = []
                for wp in waypoints3_wrist:
                        #waypoints3.append(ATC1.correctPose(wp, route_arm, rotate = True, ATC_sign = -1))
                        waypoints3.append(ATC1.correctPose(wp, route_arm, rotate = True, ATC_sign = -1, routing_app = True))
                waypoints3[0] = init_pose1
                #visualize_keypoints_simple([next_guide_top_beginning_route_offset2]+[next_guide_top_beginning_route_offset1]+[next_guide_top_beginning_route_visualize])
                plan, success = compute_cartesian_path_velocity_control([waypoints3], [speed_execution], arm_side=route_arm)
                if success:
                        execute_plan_async(route_group, plan)
                        #arm.execute(plan, wait=True)

        if step2==9:
        #Apply tension ToDo
                actuate_grippers(grasp_distance, gripper_speed, route_arm, grasp=True)

                waypoints31 = []
                init_pose1 = arm.get_current_pose().pose
                waypoints31.append(init_pose1)
                init_pose1_anticorrected = ATC1.antiCorrectPose(init_pose1, route_arm, routing_app = True)
                tension_pose = get_shifted_pose(init_pose1_anticorrected,[force_offset, 0, 0, 0, 0, 0]) #THE GUIDE WIDTH SHOULD BE DIVIDED BY 2, NOT BY 4
                waypoints31.append(ATC1.correctPose(tension_pose, route_arm, rotate = True, ATC_sign = -1, routing_app = True))
                #Move with force_control
                if force_control_active:
                        rospy.wait_for_service('/right_norbdo/tare')
                        tare_forces_srv = rospy.ServiceProxy('right_norbdo/tare', Trigger)
                        tare_res = tare_forces_srv(TriggerRequest())
                        tare_forces_srvL = rospy.ServiceProxy('left_norbdo/tare', Trigger)
                        tare_res = tare_forces_srvL(TriggerRequest())
                rospy.sleep(0.5) #REDUCE. Original 0.5
                stop_mov_force = False #reset this value before activating the force_controlled
                force_controlled = True
                plan, success = compute_cartesian_path_velocity_control([waypoints31], [speed_tension], arm_side=route_arm)
                if success:
                        success = execute_force_control(arm_side = route_arm, plan = plan)
                rospy.sleep(0.5) #REDUCE. Original 0.5
                force_controlled = False
                rospy.sleep(1.5) #REDUCE. Original 2.5

        if step2==10:
                #Bring second hand
                waypoints4 = []
                init_pose = arm2.get_current_pose().pose
                waypoints4.append(init_pose)
                init_pose_up = copy.deepcopy(init_pose)
                init_pose_up.position.z += z_offset*2 + op["prev_guide"]["height"]
                waypoints4.append(init_pose_up)
                grasp_pose_up = get_shifted_pose(next_guide_top_beginning_route_offset1, [0, 0, z_offset+ EEF_route2[2], 0, 0, 0])
                init_pose_rotated = copy.deepcopy(grasp_pose_up)
                init_pose_up_anticorrected = ATC1.antiCorrectPose(init_pose_up, arm2_side) 
                init_pose_rotated.position = init_pose_up_anticorrected.position
                waypoints4.append(ATC1.correctPose(init_pose_rotated, arm2_side, rotate = True, ATC_sign = -1))
                plan, success = compute_cartesian_path_velocity_control([waypoints4], [speed_execution], arm_side=arm2_side)
                if success:
                        execute_plan_async(group2, plan)

        if step2==11:
                actuate_grippers(open_distance, gripper_speed, arm2_side, grasp=False)

                waypoints42 = []
                init_pose = arm2.get_current_pose().pose
                waypoints42.append(init_pose)
                waypoints42.append(ATC1.correctPose(grasp_pose_up, arm2_side, rotate = True, ATC_sign = -1))
                #next_guide_top_beginning_route_offset1.position.z += 0.006
                waypoints42.append(ATC1.correctPose(next_guide_top_beginning_route_offset1, arm2_side, rotate = True, ATC_sign = -1))
                plan, success = compute_cartesian_path_velocity_control([waypoints42], [speed_execution], arm_side=arm2_side)
                if success:
                        execute_plan_async(group2, plan)
                        #arm2.execute(plan, wait=True)
                        #visualize_keypoints_simple([next_guide_top_beginning_route_offset2, grasp_pose_up])

        if step2==12:
                #Apply tension movement. ToDo
                actuate_grippers(slide_distance, gripper_speed, arm2_side, grasp=False)
                #actuate_grippers(grasp_distance, gripper_speed, route_arm, grasp=True)

                #Move arm down 
                waypoints5 = []
                init_pose = arm.get_current_pose().pose
                init_pose2 = arm2.get_current_pose().pose
                init_pose_anticorrected = ATC1.antiCorrectPose(init_pose, route_arm, routing_app = True)
                init_pose2_anticorrected = ATC1.antiCorrectPose(init_pose2, arm2_side)
                waypoints5.append(init_pose)
                pose_down = copy.deepcopy(init_pose_anticorrected)
                pose_down.position.z = init_pose2_anticorrected.position.z
                waypoints5.append(ATC1.correctPose(pose_down, route_arm, rotate = True, ATC_sign = -1, routing_app = True))
                plan, success = compute_cartesian_path_velocity_control([waypoints5], [speed_execution], arm_side=route_arm)
                if success:
                        execute_plan_async(route_group, plan)
                        #arm.execute(plan, wait=True)

        if step2==13:
                actuate_grippers(grasp_distance, gripper_speed, arm2_side, grasp=True)
                #Rotate arms
                waypoints6_circ_wrist = []
                waypoints6_center_wrist = []
                waypoints6_circ = []
                waypoints6_center = []
                init_pose = arm.get_current_pose().pose
                init_pose_guides = ATC1.antiCorrectPose(init_pose, route_arm, routing_app = True)
                init_pose2 = arm2.get_current_pose().pose
                corner_center = ATC1.antiCorrectPose(init_pose2, arm2_side)
                radius_rot = compute_distance(corner_center, init_pose_guides)
                #visualize_keypoints_simple([init_pose_guides, corner_center], "/base_link")
                
                line_center = get_shifted_pose(corner_center, [1,0,0,0,0,0])
                angle_center = math.atan2(line_center.position.y - corner_center.position.y, line_center.position.x - corner_center.position.x)
                line_guide = get_shifted_pose(next_guide_top, [1,0,0,0,0,0])
                angle_guide = math.atan2(line_guide.position.y - next_guide_top.position.y, line_guide.position.x - next_guide_top.position.x)
                rot_rad = angle_guide - angle_center 
                rot_degree = (rot_rad*180)/math.pi
                print("##############ANGLE")
                print(rot_degree)

                center_waypoints = []
                circle_waypoints = []
                success, waypoints6_center_wrist, waypoints6_circ_wrist = circular_trajectory(corner_center, init_pose_guides, rot_degree, [0,0,-1], waypoints6_center_wrist, waypoints6_circ_wrist, step = 2, rot_gripper = True)
                for wp in waypoints6_circ_wrist:
                        waypoints6_circ.append(ATC1.correctPose(wp, route_arm, rotate = True, ATC_sign = -1, routing_app = True))
                for wp in waypoints6_center_wrist:
                        waypoints6_center.append(ATC1.correctPose(wp, arm2_side, rotate = True, ATC_sign = -1))

                #visualize_keypoints_simple(waypoints6_center_wrist+waypoints6_circ_wrist)

                if route_arm == "left":
                        plan, success = dual_arm_cartesian_plan([waypoints6_circ], [speed_execution], [waypoints6_center], [speed_execution], ATC1= ATC1, sync_policy=1)
                elif route_arm == "right":
                        #visualize_keypoints_simple(waypoints6_center + waypoints6_circ, "/base_link")
                        plan, success = dual_arm_cartesian_plan([waypoints6_center], [speed_execution], [waypoints6_circ], [speed_execution], ATC1= ATC1, sync_policy=1)
                execute_plan_async("arms", plan)
                # arms.execute(plan)

                rot_center_jigs = arm.get_current_pose().pose
                rot_center = ATC1.antiCorrectPose(rot_center_jigs, arm2_side)
                rot_center_up = True

                actuate_grippers(slide_distance, gripper_speed, arm2_side, grasp=False)


def RC_insert_final(op, route_arm):
        global rot_center
        global speed_execution
        global speed_tension
        global step2
        global open_distance
        global gripper_speed
        #Apply tension (Move forward 3cm max (stopped by "force controller"))
        #Insert (Circular motion)
        #Lift both arms

        if route_arm == "left":
                arm = arm_left
                arm2 = arm_right
                arm2_side = "right"

        elif route_arm == "right":
                arm = arm_right
                arm2 = arm_left
                arm2_side = "left"

        if step2<3:
                #Apply tension and insert
                RC_insert(op, route_arm)

        if step2>=3 and step2<8:
                RC_push_inserted(arm2_side)

        if step2==8:
                #Retract
                actuate_grippers(open_distance, gripper_speed, 'both', grasp=False)

                waypoints4_1 = []
                init_pose = arm.get_current_pose().pose
                waypoints4_1.append(init_pose)
                init_pose_up = copy.deepcopy(init_pose)
                init_pose_up.position.z += z_offset*2 + op["prev_guide"]["height"]
                waypoints4_1.append(init_pose_up)

                # waypoints4_2 = []
                # init_pose2 = arm2.get_current_pose().pose
                # waypoints4_2.append(init_pose2)
                # init_pose2_up = copy.deepcopy(init_pose2)
                # init_pose2_up.position.z = init_pose_up.position.z
                # waypoints4_2.append(init_pose2_up)

                # if route_arm=="left":
                #         plan, success = dual_arm_cartesian_plan(waypoints4_1, [speed_execution], waypoints4_2, [speed_execution], ATC1= ATC1, sync_policy=2)
                # elif route_arm=="right":
                #        plan, success = dual_arm_cartesian_plan(waypoints4_2, [speed_execution], waypoints4_1, [speed_execution], ATC1= ATC1, sync_policy=2)

                init_pose2 = arm2.get_current_pose().pose
                init_pose_anticorrected = ATC1.antiCorrectPose(init_pose, route_arm, routing_app = True)
                init_pose2_anticorrected = ATC1.antiCorrectPose(init_pose2, arm2_side)
                if ((init_pose2_anticorrected.position.z) > (init_pose_anticorrected.position.z + op["prev_guide"]["height"])): #CHANGE THE WRISTS HAVE DIFFERENT SIZES
                        plan, success = compute_cartesian_path_velocity_control([waypoints4_1], [speed_execution], arm_side=route_arm)
                        if success:
                                execute_plan_async(route_group, plan)
                else:
                        plan, success = master_slave_plan(waypoints4_1, ATC1, 50.0, route_arm, type=1)
                        if success:
                                execute_plan_async("arms", plan)
                                # arms.execute(plan, wait=True)


def simplified_PC(op):
        global step1
        global step2
        global process_actionserver
        global slow_speed_execution
        global speed_execution
        global fast_speed_execution
        global speed_tension
        global force_controlled
        global force_limit
        global force_limit_connector
        global pick_grasp_offset
        global grasping_cables

        grasping_cables = False
        #print("PC simplified")
        #print("STEP1: " +str(step1)+" STEP2: " +str(step2))
        force_limit = copy.deepcopy(force_limit_connector)
        if op['spot']['name'] == "WH3":
                y_deviation = 0.0074
                z_deviation = -0.007
        else:
                y_deviation = -0.003
                z_deviation = -0.002
        if step1 == 0:
                #IMPLEMENT ALGORITHM TO FIND A FREE PATH TO MOVE
                if step2 == 0:
                        if op['spot']['name'] == "WH3":
                                waypoints_PC03 = []
                                trash_pose = arm.get_current_pose().pose
                                init_pose = arm.get_current_pose().pose
                                free_path_pose1 = copy.deepcopy(init_pose)
                                free_path_pose1.position.x -= 0.02
                                free_path_pose2 = copy.deepcopy(free_path_pose1)
                                free_path_pose2.position.y += 0.1895
                                free_path_pose3 = copy.deepcopy(free_path_pose2)
                                free_path_pose3.position.x += 0.212
                                free_path_pose3.position.y += 0.03
                                mold_up_forward = get_shifted_pose(op["spot"]["pose_corner"], [op["spot"]['width']+grasp_offset+EEF_route[0]/2, ((op["spot"]['gap']/2)+y_deviation), op["spot"]["height"] + z_offset, 0, 0, 0])
                                mold_up_forward_corrected = ATC1.correctPose(mold_up_forward, route_arm, rotate = True, ATC_sign = -1, routing_app = True, secondary_frame = True)
                                mold_up_forward = get_shifted_pose(op["spot"]["pose_corner"], [op["spot"]['width']+grasp_offset+EEF_route[0]/2, ((op["spot"]['gap']/2)+y_deviation), op["spot"]["height"], 0, 0, 0])
                                mold_up_forward_corrected_shifted = copy.deepcopy(mold_up_forward_corrected)
                                mold_up_forward_corrected_shifted.position.y = free_path_pose3.position.y
                                mold_up_forward_corrected_shifted.position.x -= 0.05
                                waypoints_PC03 = [init_pose, free_path_pose1, free_path_pose2, free_path_pose3, mold_up_forward_corrected_shifted, mold_up_forward_corrected]
                                plan, success, motion_group_plan = compute_cartesian_path_velocity_control_arms_occlusions([waypoints_PC03], [fast_speed_execution], arm_side=route_arm)
                                if success:
                                        execute_plan_async(motion_group_plan, plan)
                                        rospy.sleep(1)
                        else:
                                waypoints_PC01 = []
                                trash_pose = arm.get_current_pose().pose
                                init_pose = arm.get_current_pose().pose
                                free_path_pose1 = copy.deepcopy(init_pose)
                                free_path_pose1.position.x -= 0.25
                                mold_up_forward = get_shifted_pose(op["spot"]["pose_corner"], [op["spot"]['width']+grasp_offset+EEF_route[0]/2, ((op["spot"]['gap']/2)+y_deviation), op["spot"]["height"] + 2*z_offset, 0, 0, 0])
                                mold_up_forward_corrected = ATC1.correctPose(mold_up_forward, route_arm, rotate = True, ATC_sign = -1, routing_app = True, secondary_frame = True)
                                #free_path_pose2 = copy.deepcopy(free_path_pose1)
                                free_path_pose1.position.z = mold_up_forward_corrected.position.z
                                waypoints_PC01 = [init_pose, free_path_pose1, mold_up_forward_corrected]
                                plan, success, motion_group_plan = compute_cartesian_path_velocity_control_arms_occlusions([waypoints_PC01], [fast_speed_execution], arm_side=route_arm)
                                if success:
                                        execute_plan_async(motion_group_plan, plan)
                                        rospy.sleep(1)

                # if step2 == 0:
                #         waypoints_PC = []
                #         trash_pose = arm.get_current_pose().pose
                #         init_pose = arm.get_current_pose().pose
                #         waypoints_PC.append(init_pose)
                #         mold_up_forward = get_shifted_pose(op["spot"]["pose_corner"], [op["spot"]['width']+grasp_offset+EEF_route[0]/2, ((op["spot"]['gap']/2)-0.003), op["spot"]["height"] + 2*z_offset, 0, 0, 0])
                #         corrected_pose = ATC1.correctPose(mold_up_forward, route_arm, rotate = True, ATC_sign = -1, routing_app = True, secondary_frame = True)
                #         waypoints_PC.append(corrected_pose)
                #         print("ROUTE ARM: " + str(route_arm))
                #         """
                #         pose_other_arm = arm2.get_current_pose().pose
                #         waypoints_PC_arm2 = [] #MAKE IT MOVE JUST IF NEEDED
                #         waypoints_PC_arm2.append(pose_other_arm)
                #         pose_other_arm.position.x += move_away2_sign*0.3
                #         waypoints_PC_arm2.append(pose_other_arm)
                #         if route_arm == "left":
                #                 plan, success = dual_arm_cartesian_plan([waypoints_PC], [speed_execution], [waypoints_PC_arm2], [speed_execution], ATC1= ATC1, sync_policy=1)
                #         elif route_arm == "right":
                #                 plan, success = dual_arm_cartesian_plan([waypoints_PC_arm2], [speed_execution], [waypoints_PC], [speed_execution], ATC1= ATC1, sync_policy=1)
                #         # plan, success = master_slave_plan(waypoints_PC, ATC1, 50.0, route_arm, type=1)
                #         if success:
                #                 execute_plan_async("arms", plan)
                #                 # arms.execute(plan, wait=True)
                #         """
                #         #visualize_keypoints_simple(waypoints_PC, parent_frame="/base_link")
                #         plan, success, motion_group_plan = compute_cartesian_path_velocity_control_arms_occlusions([waypoints_PC], [fast_speed_execution], arm_side=route_arm)
                #         if success:
                #                 execute_plan_async(motion_group_plan, plan)
                #                 rospy.sleep(10)
                
                if step2 == 1:
                        waypoints_PC2 = []
                        init_pose_trash = arm.get_current_pose().pose
                        init_pose = arm.get_current_pose().pose
                        waypoints_PC2.append(init_pose)
                        mold_forward = get_shifted_pose(op["spot"]["pose_corner"], [op["spot"]['width']+grasp_offset+EEF_route[0]/2, ((op["spot"]['gap']/2)+y_deviation), z_deviation, 0, 0, 0])
                        waypoints_PC2.append(ATC1.correctPose(mold_forward, route_arm, rotate = True, ATC_sign = -1, routing_app = True, secondary_frame = True))
                        plan, success = compute_cartesian_path_velocity_control([waypoints_PC2], [slow_speed_execution], arm_side=route_arm)
                        if success:
                                execute_plan_async(route_group, plan)
                                # arm.execute(plan, wait=True)
                        rospy.sleep(1)

                if step2 == 2:
                        waypoints_PC3 = []
                        init_pose = arm.get_current_pose().pose
                        waypoints_PC3.append(init_pose)
                        #Connector insertion params
                        combs_width = 0.001
                        mold_width = 0.001
                        #extra_pull = 0.003
                        extra_pull = 0.01
                        pull_pose = get_shifted_pose(op["spot"]["pose_corner"], [op["spot"]['width']+grasp_offset+EEF_route[0]/2+pick_grasp_offset[op["spot"]["name"]]+combs_width-mold_width+extra_pull, ((op["spot"]['gap']/2)+y_deviation), z_deviation, 0, 0, 0])
                        waypoints_PC3.append(ATC1.correctPose(pull_pose, route_arm, rotate = True, ATC_sign = -1, routing_app = True, secondary_frame = True))
                        if force_control_active:
                                rospy.wait_for_service('/right_norbdo/tare')
                                tare_forces_srv = rospy.ServiceProxy('right_norbdo/tare', Trigger)
                                tare_res = tare_forces_srv(TriggerRequest())
                                tare_forces_srvL = rospy.ServiceProxy('left_norbdo/tare', Trigger)
                                tare_res = tare_forces_srvL(TriggerRequest())
                        force_controlled = True
                        plan, success = compute_cartesian_path_velocity_control([waypoints_PC3], [speed_tension], arm_side=route_arm)
                        if success:
                                success = execute_force_control(arm_side = route_arm, plan = plan)
                        rospy.sleep(0.5)
                        force_controlled = False
                        grasping_cables = True

                if step2 == 3:
                       step1 += 1
                       step2 = 0
                       process_actionserver.publish_feedback()

def EC(op):
        global motion_groups
        global pick_grasp_offset
        global z_offset_pick
        global step1
        global step2
        global process_actionserver
        global fast_speed_execution
        global slow_speed_execution
        global speed_execution
        global grasping_cables

        grasping_cables = False
        print("Pick connector")
        print("Step1: "+str(step1))
        print("Step2: "+str(step2))
        if op['spot']['side'] == "R":
                pick_arm = "right"
                fingers_size = ATC1.EEF_dict[ATC1.EEF_right].fingers_dim
        else:
                pick_arm = "left"
                fingers_size = ATC1.EEF_dict[ATC1.EEF_left].fingers_dim
        arm = motion_groups['arm_'+pick_arm]

        prepick_pose = 'arm_'+pick_arm+'_prepick'
        if op["spot"]["name"] == "WH3":
                pick_pose = get_shifted_pose(op["spot"]["pose_corner"], [op["spot"]['width']/2, - fingers_size[0]/2 - pick_grasp_offset[op["spot"]["name"]] - grasp_offset, op["spot"]["height"]/2 + 0.01, 0, 0, 0])
        else:
                pick_pose = get_shifted_pose(op["spot"]["pose_corner"], [op["spot"]['width']/2, - fingers_size[0]/2 - pick_grasp_offset[op["spot"]["name"]] - grasp_offset, op["spot"]["height"]/2, 0, 0, 0])
        pick_pose = get_shifted_pose(pick_pose, [0, 0, 0, 0, 0, -1.5708])
        pick_pose_offset = get_shifted_pose(pick_pose, [0, 0, op["spot"]["height"]/2 + z_offset_pick, 0, 0, 0])
        #visualize_keypoints_simple([pick_pose, pick_pose_offset, ATC1.correctPose(pick_pose, pick_arm, rotate = True, ATC_sign = -1, picking_app = True)], parent_frame="/base_link")

        if step1 == 0:
                if step2 == 0:
                        #Move to predefined dual-arm config (pointing down)
                        arms.set_named_target("arms_platform_5")
                        arms.go(wait=True)
                        step2=1
                        #move_group_async("arms")
                        rospy.sleep(0.5)
                
                if step2 == 1:
                        #Move torso to config
                        torso.set_named_target("torso_combs")
                        #move_group_async("torso")
                        torso.go(wait=True)
                        step2=2
                        rospy.sleep(0.5)

                if step2 == 2:
                        #Move pick_arm to predefined config (orientation for picking)
                        arm.set_named_target(prepick_pose)
                        move_group_async('arm_'+pick_arm)
                        rospy.sleep(0.5)

                if step2 == 3:
                        #Move arm to grasp cable (with approach+retract)
                        waypoints_EC1 = []
                        init_pose = arm.get_current_pose().pose
                        waypoints_EC1.append(init_pose)
                        waypoints_EC1.append(ATC1.correctPose(pick_pose_offset, pick_arm, rotate = True, ATC_sign = -1, picking_app = True))
                        plan, success = compute_cartesian_path_velocity_control([waypoints_EC1], [fast_speed_execution], arm_side=pick_arm)
                        if success:
                                execute_plan_async(route_group, plan)

                if step2 == 4:
                        actuate_grippers(open_distance, gripper_speed, pick_arm, grasp=False)
                        waypoints_EC2 = []
                        init_pose = arm.get_current_pose().pose
                        waypoints_EC2.append(init_pose)
                        waypoints_EC2.append(ATC1.correctPose(pick_pose, pick_arm, rotate = True, ATC_sign = -1, picking_app = True))
                        plan, success = compute_cartesian_path_velocity_control([waypoints_EC2], [slow_speed_execution], arm_side=pick_arm)
                        if success:
                                execute_plan_async(route_group, plan)

                if step2 == 5:
                        actuate_grippers(grasp_distance, gripper_speed, pick_arm, grasp=True)
                        rospy.sleep(1.0)
                        waypoints_EC3 = []
                        init_pose = arm.get_current_pose().pose
                        waypoints_EC3.append(init_pose)
                        waypoints_EC3.append(ATC1.correctPose(pick_pose_offset, pick_arm, rotate = True, ATC_sign = -1, picking_app = True))
                        plan, success = compute_cartesian_path_velocity_control([waypoints_EC3], [slow_speed_execution], arm_side=pick_arm)
                        if success:
                                execute_plan_async(route_group, plan)

                if step2 == 6:
                        #Move to predefined dual-arm config
                        arm.set_named_target(prepick_pose)
                        move_group_async('arm_'+pick_arm)
                        rospy.sleep(0.5)

                if step2 == 7:
                        arms.set_named_target("arms_platform_5")
                        move_group_async("arms")
                        rospy.sleep(0.5)

                if step2 == 8:
                       step1 += 1
                       step2 = 0
                       process_actionserver.publish_feedback()


def retract_arm(ret_arm, guide, special = False):
        global speed_execution
        global open_distance
        global gripper_speed
        global z_offset

        ret_group = 'arm_'+ret_arm
        arm = motion_groups[ret_group]
        if special:
                actuate_grippers(105, 50, ret_arm, grasp=False)
        else:
                actuate_grippers(open_distance, gripper_speed, ret_arm, grasp=False)
        init_pose = arm.get_current_pose().pose
        if special:
                retract_z = get_shifted_pose(guide["pose_corner"], [0, 0, guide["height"] + z_offset + 0.08, 0, 0, 0])
        else:
                retract_z = get_shifted_pose(guide["pose_corner"], [0, 0, guide["height"] + z_offset, 0, 0, 0])
        retract_z_corrected = ATC1.correctPose(retract_z, ret_arm, rotate = True, routing_app = True, ATC_sign = -1, secondary_frame=True)
        new_pose = copy.deepcopy(init_pose)
        new_pose.position.z = retract_z_corrected.position.z
        if special:
                new_pose.position.y -= 0.1
        waypoints_ret = [init_pose, new_pose]
        plan, success = compute_cartesian_path_velocity_control([waypoints_ret], [speed_execution], arm_side=ret_arm)
        if success:
                execute_plan_async(ret_group, plan)


def separate_cables(op, route_arm):
        global z_offset_photo
        global z_offset
        global x_offset
        global motion_groups
        global speed_execution
        global grasp_point_global
        global use_camera
        global confirmation_received
        global confirmation_msg
        global step1
        global step2
        global process_actionserver
        global stop_mov
        global stop_mov_force
        global force_controlled
        global holding_cables
        global force_limit_cable
        global force_limit
        global holding_comeback_pose_corrected
        global grasping_cables
        global grasp_offset

        force_limit = copy.deepcopy(force_limit_cable)

        rospy.wait_for_service('/vision/grasp_point_determination_srv')
        grasp_point_srv = rospy.ServiceProxy('/vision/grasp_point_determination_srv', cablesSeparation)
        rospy.wait_for_service('/vision/check_cable_separation_srv')
        eval_grasp_srv = rospy.ServiceProxy('/vision/check_cable_separation_srv', cablesSeparation)
        
        if route_arm == "right":
                fingers_size = ATC1.EEF_dict[ATC1.EEF_right].fingers_dim
                separation_arm = "left"
                fingers_size_sep = ATC1.EEF_dict[ATC1.EEF_left].fingers_dim
                separation_group = 'arm_left'
        else:
                fingers_size = ATC1.EEF_dict[ATC1.EEF_left].fingers_dim
                separation_arm = "right"
                fingers_size_sep = ATC1.EEF_dict[ATC1.EEF_right].fingers_dim
                separation_group = 'arm_right'

        route_group = 'arm_'+route_arm
        arm = motion_groups[route_group]
        arm_sep = motion_groups[separation_group]

        if step1 == 0:
                if step2 == 0:
                        if not grasping_cables:
                                retract_arm(route_arm, op["spot"][0])
                                step2=0
                                init_pose = arm.get_current_pose().pose
                                mold_up_forward = get_shifted_pose(op["spot"][0]["pose_corner"], [op["spot"][0]['width']+grasp_offset+fingers_size[0]/2, (op["spot"][0]['gap']/2), op["spot"][0]["height"] + 2*z_offset, 0, 0, 0])
                                mold_forward = get_shifted_pose(op["spot"][0]["pose_corner"], [op["spot"][0]['width']+grasp_offset+fingers_size[0]/2, (op["spot"][0]['gap']/2), 0, 0, 0, 0])
                                mold_up_forward_corrected = ATC1.correctPose(mold_up_forward, route_arm, rotate = True, routing_app = True, ATC_sign = -1, secondary_frame = True)
                                mold_forward_corrected = ATC1.correctPose(mold_forward, route_arm, rotate = True, routing_app = True, ATC_sign = -1, secondary_frame = True)
                                grasping_cables = True
                                waypoints0 = [init_pose, mold_up_forward_corrected, mold_forward_corrected]
                                plan, success, motion_group_plan = compute_cartesian_path_velocity_control_arms_occlusions([waypoints0], [speed_execution], arm_side=route_arm)
                                if success:
                                        execute_plan_async(motion_group_plan, plan)
                        else:
                                step2=1

                if step2 ==1:
                        actuate_grippers(slide_distance, gripper_speed, route_arm, grasp=False)
                        init_pose = arm.get_current_pose().pose
                        mold_forward_slide = get_shifted_pose(op["spot"][0]["pose_corner"], [op["spot"][0]['width']+0.09, (op["spot"][0]['gap']/2), 0.005, 0, 0, 0])
                        mold_forward_slide_corrected = ATC1.correctPose(mold_forward_slide, route_arm, rotate = True, routing_app = True, ATC_sign = -1, secondary_frame = True)
                        waypoints1 = [init_pose, mold_forward_slide_corrected]
                        plan, success = compute_cartesian_path_velocity_control([waypoints1], [speed_execution], arm_side=route_arm)
                        if success:
                                execute_plan_async(route_group, plan)                                 


                if step2 == 2:
                        os.system('cp ' + str(rospack.get_path('vision_pkg_full_demo')) + '/imgs/error_grasp_1.jpg /home/remodel/UI-REMODEL/src/assets/img/grasp_image.jpg')
                        repeat = True
                        pixel_D_param = 5
                        forward_param = False
                        while repeat:
                                if use_camera:
                                        rospy.wait_for_service('/OAK/capture_img')
                                        capture_img_srv = rospy.ServiceProxy('/OAK/capture_img', Trigger)
                                        capture_img_res = capture_img_srv(TriggerRequest())
                                        img_cables_path = capture_img_res.message
                                        if not capture_img_res.success:
                                                stop_function("Failed to take the cables image")
                                else:
                                        img_cables_path = str(rospack.get_path('vision_pkg_full_demo')) + '/imgs/Image_cables_43.jpg'

                                msg_log = String()
                                msg_log.data = "Calculating grasp point..."
                                logs_publisher.publish(msg_log)
                                grasp_point_req = cablesSeparationRequest()
                                grasp_point_req.img_path = img_cables_path
                                grasp_point_req.wh_id = op['WH']
                                grasp_point_req.separated_cable_index = op['label']
                                grasp_point_req.pixel_D = pixel_D_param
                                grasp_point_req.forward = forward_param
                                grasp_point_req.iteration = False
                                grasp_point_req.grasp_point_eval_mm = [0,0]
                                grasp_point_req.analyzed_length = 100
                                grasp_point_req.analyzed_grasp_length = 30 #Distance to the next guide to avoid collisions
                                grasp_point_req.simplified = True
                                grasp_point_req.visualize = False
                                grasp_point_res = grasp_point_srv(grasp_point_req)
                                grasp_point_global = [float(grasp_point_res.grasp_point[0])/1000, float(grasp_point_res.grasp_point[1])/1000]
                                msg_log.data = "Grasp point calculated"
                                logs_publisher.publish(msg_log)

                                repeat = False
                                img_name = img_cables_path.split('/')[-1]
                                confirm_msg = String()
                                confirm_msg.data = img_cables_path[:-len(img_name)]+'Grasp_point_'+img_name
                                confirmation_publisher.publish(confirm_msg)
                                while not confirmation_received: #Wait until the UI accepts
                                       rospy.sleep(0.1)
                                
                                os.system('cp ' + str(rospack.get_path('vision_pkg_full_demo')) + '/imgs/error_grasp_1.jpg /home/remodel/UI-REMODEL/src/assets/img/grasp_image.jpg')
                                confirmation_received = False
                                if confirmation_msg == "N":
                                        stop_function("Grasp canceled")
                                elif confirmation_msg == "R":
                                        pixel_D_param += 1
                                        forward_param = False
                                        repeat = True

                        if grasp_point_res.success:
                                actuate_grippers(open_distance, gripper_speed, separation_arm, grasp=False)
                                print(grasp_point_global)
                                init_pose = arm_sep.get_current_pose().pose

                                # test_point = get_shifted_pose(op["spot"][0]["pose_corner"], [0.03, op["spot"][0]['gap']/2, op["spot"][0]["height"], 0, 0, 0])
                                # test_point_corrected = ATC1.correctPose(test_point, separation_arm, rotate = True, routing_app = False, ATC_sign = -1, secondary_frame = True)
                                # waypoints_test = [init_pose, test_point_corrected]
                                # plan, success, motion_group_plan = compute_cartesian_path_velocity_control_arms_occlusions([waypoints_test], [speed_execution], arm_side=separation_arm)
                                # if success:
                                #         execute_plan_async(motion_group_plan, plan)
                                # exit()

                                grasp_point = get_shifted_pose(op["spot"][0]["pose_corner"], [grasp_point_global[0], op["spot"][0]['gap']/2, op["spot"][0]["height"] + grasp_point_global[1], 0, 0, 0])
                                grasp_point_offset = get_shifted_pose(grasp_point, [0, 0, (z_offset*2) - grasp_point_global[1], 0, 0, 0])
                                grasp_point_corrected = ATC1.correctPose(grasp_point, separation_arm, rotate = True, routing_app = False, ATC_sign = -1, secondary_frame = True)
                                grasp_point_offset_corrected = ATC1.correctPose(grasp_point_offset, separation_arm, rotate = True, routing_app = False, ATC_sign = -1, secondary_frame = True)                        
                                waypoints2 = [init_pose, grasp_point_offset_corrected, grasp_point_corrected]
                                print("INIT: " + str(init_pose))
                                print("GRASP: " + str(grasp_point_corrected))
                                print(grasp_point_global)
                                # plan, success, motion_group_plan = compute_cartesian_path_velocity_control_arms_occlusions([waypoints2], [speed_execution], arm_side=separation_arm)
                                # if success:
                                #         execute_plan_async(motion_group_plan, plan)
                                plan, success = compute_cartesian_path_velocity_control([waypoints2], [speed_execution], arm_side=separation_arm)
                        if success:
                                execute_plan_async(separation_group, plan)  
                        else:
                                stop_function("Grasp point determination failed")
                        #exit()
                                        
                if step2 == 3:
                        actuate_grippers(slide_distance, gripper_speed, separation_arm, grasp=False)

                        rospy.sleep(1)
                        retract_arm(route_arm, op["spot"][0], special=True)
                        grasping_cables = False
                        step2=3
                        rospy.sleep(0.5)

                        init_pose = arm_sep.get_current_pose().pose
                        grasp_point_lift = get_shifted_pose(grasp_point_offset, [x_offset+0.01, 0, z_offset/2, 0, 0, 0])
                        grasp_point_lift_corrected = ATC1.correctPose(grasp_point_lift, separation_arm, rotate = True, routing_app = False, ATC_sign = -1, secondary_frame = True)
                        waypoints3 = [init_pose, grasp_point_lift_corrected]
                        plan, success, motion_group_plan = compute_cartesian_path_velocity_control_arms_occlusions([waypoints3], [speed_execution], arm_side=separation_arm)
                        if success:
                                execute_plan_async(motion_group_plan, plan)
                        rospy.sleep(0.5)

                if step2 == 4:
                        if use_camera:
                                rospy.wait_for_service('/OAK/capture_img')
                                capture_img_srv = rospy.ServiceProxy('/OAK/capture_img', Trigger)
                                capture_img_res = capture_img_srv(TriggerRequest())
                                img_cables_path = capture_img_res.message
                                if not capture_img_res.success:
                                        stop_function("Failed to take the cables image")
                        else:
                                img_cables_path = str(rospack.get_path('vision_pkg_full_demo')) + '/imgs/Image_cables_1.jpg'

                        msg_log = String()
                        msg_log.data = "Evaluating cable separation..."
                        logs_publisher.publish(msg_log)
                        grasp_eval_req = cablesSeparationRequest()
                        grasp_eval_req.img_path = img_cables_path
                        grasp_eval_req.wh_id = op['WH']
                        grasp_eval_req.separated_cable_index = op['label']
                        grasp_eval_req.pixel_D = 5
                        grasp_eval_req.forward = False
                        grasp_eval_req.iteration = False
                        grasp_eval_req.grasp_point_eval_mm = [int((2*z_offset)*1000), int((x_offset+grasp_point_global[0])*1000)]
                        grasp_eval_req.analyzed_length = 150
                        grasp_eval_req.analyzed_grasp_length = 0
                        grasp_eval_req.simplified = True
                        grasp_eval_req.visualize = False
                        grasp_eval_res = eval_grasp_srv(grasp_eval_req) 
                        msg_log.data = grasp_eval_res.result
                        logs_publisher.publish(msg_log)
                        print(grasp_eval_res.result)

                        if grasp_eval_res.success and grasp_eval_res.separation_success:
                                #step1 += 1
                                #step2 = 0
                                print("Successful cable separation")
                                process_actionserver.publish_feedback()
                        else:
                                if grasp_eval_res.success:
                                        #stop_function("Cable separation was not succesful")
                                        print("Cable separation was not succesful")
                                else:
                                        #stop_function("Grasp evaluation failed")
                                        print("Grasp evaluation failed")
                        step2 = 5

                if step2 == 5:
                        waypoints_SC4 = []
                        init_pose = arm_sep.get_current_pose().pose
                        waypoints_SC4.append(init_pose)
                        sep_guide_forward_up = get_shifted_pose(op["spot"][1]["pose_corner"], [op["spot"][1]["width"] + fingers_size[0], op["spot"][1]["gap"]/2, op["spot"][1]["height"] + z_offset + fingers_size[2]/2, 0, 0, 0])
                        waypoints_SC4.append(ATC1.correctPose(sep_guide_forward_up, separation_arm, rotate = True, routing_app = False, ATC_sign = -1))
                        #plan, success = compute_cartesian_path_velocity_control([waypoints_SC4], [speed_execution], arm_side=separation_arm)
                        #if success:
                        #        execute_plan_async(separation_group, plan)
                        plan, success, motion_group_plan = compute_cartesian_path_velocity_control_arms_occlusions([waypoints_SC4], [speed_execution], arm_side=separation_arm)
                        if success:
                                execute_plan_async(motion_group_plan, plan)

                if step2 == 6:
                        #Apply tension
                        actuate_grippers(grasp_distance, gripper_speed, separation_arm, grasp=True)

                        waypoints_SC5 = []
                        init_pose = arm_sep.get_current_pose().pose
                        waypoints_SC5.append(init_pose)
                        sep_guide_tension = get_shifted_pose(op["spot"][1]["pose_corner"], [op["spot"][1]["width"]  + fingers_size[0] + force_offset, op["spot"][1]["gap"]/2, op["spot"][1]["height"] + z_offset + fingers_size[2]/2, 0, 0, 0])
                        waypoints_SC5.append(ATC1.correctPose(sep_guide_tension, separation_arm, rotate = True, routing_app = False, ATC_sign = -1))
                        #Move with force_control
                        if force_control_active:
                                rospy.wait_for_service('/right_norbdo/tare')
                                tare_forces_srv = rospy.ServiceProxy('right_norbdo/tare', Trigger)
                                tare_res = tare_forces_srv(TriggerRequest())
                                tare_forces_srvL = rospy.ServiceProxy('left_norbdo/tare', Trigger)
                                tare_res = tare_forces_srvL(TriggerRequest())
                        rospy.sleep(0.5) #REDUCE. Original 0.5
                        stop_mov_force = False #reset this value before activating the force_controlled
                        force_controlled = True
                        plan, success = compute_cartesian_path_velocity_control([waypoints_SC5], [speed_tension], arm_side=separation_arm)
                        if success:
                                success = execute_force_control(arm_side = separation_arm, plan = plan)
                        rospy.sleep(0.5) #REDUCE. Original 0.5
                        force_controlled = False
                        rospy.sleep(2.5) #REDUCE. Original 2.5

                if step2 == 7:
                        print("###########TEST CIRC###########")
                        #Circ motion
                        waypoints_SC6 = []
                        trash_pose = arm_sep.get_current_pose().pose
                        init_pose = arm_sep.get_current_pose().pose
                        waypoints_SC6.append(copy.deepcopy(init_pose))
                        init_pose_guides = ATC1.antiCorrectPose(init_pose, separation_arm, routing_app = False)
                        rot_center = get_shifted_pose(op["spot"][0]["pose_corner"],[op["spot"][0]['width']/2, op["spot"][0]['gap']/2, op["spot"][0]['height']/2, 0, 0, 0])
                        insert_guide_center_sep = get_shifted_pose(op["spot"][1]["pose_corner"],[op["spot"][1]['width']/2, op["spot"][1]['gap']/2, op["spot"][1]['height']/2, 0, 0, 0])
                        radius_rot = compute_distance(rot_center, init_pose_guides)
                        #final_point = in the direction rot_center --> guide a distance of radius_rot
                        final_cable_direction = get_axis(insert_guide_center_sep, rot_center)
                        final_point = copy.deepcopy(insert_guide_center_sep) #Same orientation
                        final_point.position.x = rot_center.position.x + final_cable_direction[0]*radius_rot
                        final_point.position.y = rot_center.position.y + final_cable_direction[1]*radius_rot
                        final_point.position.z = rot_center.position.z + final_cable_direction[2]*radius_rot #Maybe delete?

                        #Evaluate collisions in final_point, if there are, there slide a bit more distance and evaluate again. Check also when the distance to the next guide is already too small, so then it will just do a slide_top
                        #With the correct final_point, calculates the rotation knowing, center, initial and final arc points.

                        guide_yaw_axis = get_axis_from_RM(pose_to_frame(rot_center).M, "Y")
                        rot_axis = get_ort_axis(guide_yaw_axis, final_cable_direction)[2]
                        rot_axis = [-element for element in rot_axis]
                        dist_z = compute_distance_relative(final_point, init_pose_guides, guide_yaw_axis) #from circle to guide in z axis
                        # print(dist_z)
                        # print(radius_rot)
                        rot_degree = (math.asin(-dist_z/radius_rot))*180.0/math.pi

                        center_waypoints = []
                        circle_waypoints = []
                        success, center_waypoints, circle_waypoints = circular_trajectory(rot_center, init_pose_guides, rot_degree, rot_axis, center_waypoints, circle_waypoints, step = 2, rot_gripper = False)
                        for wp in circle_waypoints:
                                waypoints_SC6.append(ATC1.correctPose(wp, separation_arm, rotate = True, ATC_sign = -1, routing_app = False))

                        plan, success = compute_cartesian_path_velocity_control([waypoints_SC6], [speed_execution], arm_side=separation_arm)
                        if success:
                                execute_plan_async(separation_group, plan)
                        print("###########TEST CIRC END###########")

                if step2 == 8:
                        actuate_grippers(slide_distance, gripper_speed, separation_arm, grasp=False)
                        waypoints_SC7 = []
                        init_pose = arm_sep.get_current_pose().pose
                        waypoints_SC7.append(init_pose)
                        if separation_arm=="right":
                                sign_side = 1
                        else:
                                sign_side = -1
                        side_pose = get_shifted_pose(op["spot"][1]["pose_corner"], [op["spot"][1]["width"] + fingers_size[0] + x_offset, sign_side*0.1, 0, 0, 0, 0])
                        waypoints_SC7.append(ATC1.correctPose(side_pose, separation_arm, rotate = True, routing_app = False, ATC_sign = -1))
                        plan, success, motion_group_plan = compute_cartesian_path_velocity_control_arms_occlusions([waypoints_SC7], [slow_speed_execution], arm_side=separation_arm)
                        if success:
                                execute_plan_async(motion_group_plan, plan)

                if step2 == 9:
                        actuate_grippers(grasp_distance, gripper_speed, separation_arm, grasp=True)
                        waypoints_SC8 = []
                        init_pose = arm_sep.get_current_pose().pose
                        waypoints_SC8.append(init_pose)
                        if separation_arm=="right":
                                sign_side = 1
                        else:
                                sign_side = -1
                        side_pose_tension = get_shifted_pose(op["spot"][1]["pose_corner"], [op["spot"][1]["width"] + fingers_size[0] + x_offset, sign_side*(0.1 + force_offset/2), 0, 0, 0, 0])
                        waypoints_SC8.append(ATC1.correctPose(side_pose_tension, separation_arm, rotate = True, routing_app = False, ATC_sign = -1))
                        #Move with force_control
                        if force_control_active:
                                rospy.wait_for_service('/right_norbdo/tare')
                                tare_forces_srv = rospy.ServiceProxy('right_norbdo/tare', Trigger)
                                tare_res = tare_forces_srv(TriggerRequest())
                                tare_forces_srvL = rospy.ServiceProxy('left_norbdo/tare', Trigger)
                                tare_res = tare_forces_srvL(TriggerRequest())
                        rospy.sleep(0.5) #REDUCE. Original 0.5
                        stop_mov_force = False #reset this value before activating the force_controlled
                        force_controlled = True
                        plan, success = compute_cartesian_path_velocity_control([waypoints_SC8], [speed_tension], arm_side=separation_arm)
                        if success:
                                success = execute_force_control(arm_side = separation_arm, plan = plan)
                        rospy.sleep(0.5) #REDUCE. Original 0.5
                        force_controlled = False
                        rospy.sleep(2.5) #REDUCE. Original 2.5
                        holding_cables = True
                        holding_comeback_pose = get_shifted_pose(op["spot"][1]["pose_corner"],[op["spot"][1]["width"] + fingers_size[0] + x_offset + 0.05, sign_side*0.05, op["spot"][1]['height']/2, 0, 0, 1.57])
                        holding_comeback_pose_corrected = ATC1.correctPose(holding_comeback_pose, separation_arm, rotate = True, routing_app = False, ATC_sign = -1)

                if step2 == 10:
                        #retract_arm(separation_arm, op["spot"][1])
                        step1 += 1
                        step2 = 0
                        process_actionserver.publish_feedback()


def separate_cables_old(op, route_arm):
        global z_offset_photo
        global z_offset
        global x_offset
        global motion_groups
        global speed_execution
        global grasp_point_global
        global use_camera
        global confirmation_received
        global confirmation_msg
        global step1
        global step2
        global process_actionserver
        global stop_mov
        global stop_mov_force
        global force_controlled
        global holding_cables
        global force_limit_cable
        global force_limit
        global holding_comeback_pose_corrected

        force_limit = copy.deepcopy(force_limit_cable)

        rospy.wait_for_service('/vision/grasp_point_determination_srv')
        grasp_point_srv = rospy.ServiceProxy('/vision/grasp_point_determination_srv', cablesSeparation)
        rospy.wait_for_service('/vision/check_cable_separation_srv')
        eval_grasp_srv = rospy.ServiceProxy('/vision/check_cable_separation_srv', cablesSeparation)
        
        if route_arm == "right":
                fingers_size = ATC1.EEF_dict[ATC1.EEF_right].fingers_dim
                separation_arm = "left"
                fingers_size_sep = ATC1.EEF_dict[ATC1.EEF_left].fingers_dim
                separation_group = 'arm_left'
        else:
                fingers_size = ATC1.EEF_dict[ATC1.EEF_left].fingers_dim
                separation_arm = "right"
                fingers_size_sep = ATC1.EEF_dict[ATC1.EEF_right].fingers_dim
                separation_group = 'arm_right'

        route_group = 'arm_'+route_arm
        arm = motion_groups[route_group]
        arm_sep = motion_groups[separation_group]

        if step1 == 0:
                if step2 == 0:
                        actuate_grippers(open_distance, gripper_speed, route_arm, grasp=False)
                        init_pose = arm.get_current_pose().pose
                        retreat_z = get_shifted_pose(op["spot"][0]["pose_corner"], [0, 0, op["spot"][0]["height"] + z_offset_photo + fingers_size[2], 0, 0, 0])
                        retreat_z_corrected = ATC1.correctPose(retreat_z, route_arm, rotate = True, routing_app = True, ATC_sign = -1)
                        new_pose = copy.deepcopy(init_pose)
                        new_pose.position.z = retreat_z_corrected.position.z
                        waypoints1 = [init_pose, new_pose]
                        plan, success = compute_cartesian_path_velocity_control([waypoints1], [speed_execution], arm_side=route_arm)
                        if success:
                                execute_plan_async(route_group, plan) 

                if step2 == 1:
                        os.system('cp ' + str(rospack.get_path('vision_pkg_full_demo')) + '/imgs/error_grasp_1.jpg /home/remodel/UI-REMODEL/src/assets/img/grasp_image.jpg')
                        repeat = True
                        pixel_D_param = 5
                        forward_param = False
                        while repeat:
                                if use_camera:
                                        rospy.wait_for_service('/OAK/capture_img')
                                        capture_img_srv = rospy.ServiceProxy('/OAK/capture_img', Trigger)
                                        capture_img_res = capture_img_srv(TriggerRequest())
                                        img_cables_path = capture_img_res.message
                                        if not capture_img_res.success:
                                                stop_function("Failed to take the cables image")
                                else:
                                        img_cables_path = str(rospack.get_path('vision_pkg_full_demo')) + '/imgs/Image_cables_43.jpg'

                                msg_log = String()
                                msg_log.data = "Calculating grasp point..."
                                logs_publisher.publish(msg_log)
                                grasp_point_req = cablesSeparationRequest()
                                grasp_point_req.img_path = img_cables_path
                                grasp_point_req.wh_id = op['WH']
                                grasp_point_req.separated_cable_index = op['label']
                                grasp_point_req.pixel_D = pixel_D_param
                                grasp_point_req.forward = forward_param
                                grasp_point_req.iteration = False
                                grasp_point_req.grasp_point_eval_mm = [0,0]
                                grasp_point_req.analyzed_length = 100
                                grasp_point_req.analyzed_grasp_length = 30 #Distance to the next guide to avoid collisions
                                grasp_point_req.simplified = True
                                grasp_point_req.visualize = False
                                grasp_point_res = grasp_point_srv(grasp_point_req)
                                grasp_point_global = [float(grasp_point_res.grasp_point[0])/1000, float(grasp_point_res.grasp_point[1])/1000]
                                msg_log.data = "Grasp point calculated"
                                logs_publisher.publish(msg_log)

                                repeat = False
                                img_name = img_cables_path.split('/')[-1]
                                confirm_msg = String()
                                confirm_msg.data = img_cables_path[:-len(img_name)]+'Grasp_point_'+img_name
                                confirmation_publisher.publish(confirm_msg)
                                while not confirmation_received: #Wait until the UI accepts
                                       rospy.sleep(0.1)
                                
                                os.system('cp ' + str(rospack.get_path('vision_pkg_full_demo')) + '/imgs/error_grasp_1.jpg /home/remodel/UI-REMODEL/src/assets/img/grasp_image.jpg')
                                confirmation_received = False
                                if confirmation_msg == "N":
                                        stop_function("Grasp canceled")
                                elif confirmation_msg == "R":
                                        pixel_D_param += 1
                                        forward_param = False
                                        repeat = True

                        if grasp_point_res.success:
                                actuate_grippers(open_distance, gripper_speed, separation_arm, grasp=False)
                                print(grasp_point_global)
                                init_pose = arm_sep.get_current_pose().pose

                                # test_point = get_shifted_pose(op["spot"][0]["pose_corner"], [0.03, op["spot"][0]['gap']/2, op["spot"][0]["height"], 0, 0, 0])
                                # test_point_corrected = ATC1.correctPose(test_point, separation_arm, rotate = True, routing_app = False, ATC_sign = -1, secondary_frame = True)
                                # waypoints_test = [init_pose, test_point_corrected]
                                # plan, success, motion_group_plan = compute_cartesian_path_velocity_control_arms_occlusions([waypoints_test], [speed_execution], arm_side=separation_arm)
                                # if success:
                                #         execute_plan_async(motion_group_plan, plan)
                                # exit()

                                grasp_point = get_shifted_pose(op["spot"][0]["pose_corner"], [grasp_point_global[0], op["spot"][0]['gap']/2, op["spot"][0]["height"] + grasp_point_global[1], 0, 0, 0])
                                grasp_point_offset = get_shifted_pose(grasp_point, [0, 0, z_offset - grasp_point_global[1], 0, 0, 0])
                                grasp_point_corrected = ATC1.correctPose(grasp_point, separation_arm, rotate = True, routing_app = False, ATC_sign = -1, secondary_frame = True)
                                grasp_point_offset_corrected = ATC1.correctPose(grasp_point_offset, separation_arm, rotate = True, routing_app = False, ATC_sign = -1, secondary_frame = True)                        
                                waypoints2 = [init_pose, grasp_point_offset_corrected, grasp_point_corrected]
                                print("INIT: " + str(init_pose))
                                print("GRASP: " + str(grasp_point_corrected))
                                print(grasp_point_global)
                                plan, success, motion_group_plan = compute_cartesian_path_velocity_control_arms_occlusions([waypoints2], [speed_execution], arm_side=separation_arm)
                                if success:
                                        execute_plan_async(motion_group_plan, plan)
                        else:
                                stop_function("Grasp point determination failed")
                        #exit()
                                        
                if step2 == 2:
                        actuate_grippers(slide_distance, gripper_speed, separation_arm, grasp=False)

                        rospy.sleep(1)
                        init_pose = arm_sep.get_current_pose().pose
                        grasp_point_lift = get_shifted_pose(grasp_point_offset, [x_offset, 0, z_offset/2, 0, 0, 0])
                        grasp_point_lift_corrected = ATC1.correctPose(grasp_point_lift, separation_arm, rotate = True, routing_app = False, ATC_sign = -1, secondary_frame = True)
                        waypoints3 = [init_pose, grasp_point_lift_corrected]
                        plan, success = compute_cartesian_path_velocity_control([waypoints3], [speed_execution], arm_side=separation_arm)
                        if success:
                                execute_plan_async(separation_group, plan)
                        rospy.sleep(0.5)

                if step2 == 3:
                        if use_camera:
                                rospy.wait_for_service('/OAK/capture_img')
                                capture_img_srv = rospy.ServiceProxy('/OAK/capture_img', Trigger)
                                capture_img_res = capture_img_srv(TriggerRequest())
                                img_cables_path = capture_img_res.message
                                if not capture_img_res.success:
                                        stop_function("Failed to take the cables image")
                        else:
                                img_cables_path = str(rospack.get_path('vision_pkg_full_demo')) + '/imgs/Image_cables_1.jpg'

                        msg_log = String()
                        msg_log.data = "Evaluating cable separation..."
                        logs_publisher.publish(msg_log)
                        grasp_eval_req = cablesSeparationRequest()
                        grasp_eval_req.img_path = img_cables_path
                        grasp_eval_req.wh_id = op['WH']
                        grasp_eval_req.separated_cable_index = op['label']
                        grasp_eval_req.pixel_D = 5
                        grasp_eval_req.forward = False
                        grasp_eval_req.iteration = False
                        grasp_eval_req.grasp_point_eval_mm = [int((2*z_offset)*1000), int((x_offset+grasp_point_global[0])*1000)]
                        grasp_eval_req.analyzed_length = 150
                        grasp_eval_req.analyzed_grasp_length = 0
                        grasp_eval_req.simplified = True
                        grasp_eval_req.visualize = False
                        grasp_eval_res = eval_grasp_srv(grasp_eval_req) 
                        msg_log.data = grasp_eval_res.result
                        logs_publisher.publish(msg_log)
                        print(grasp_eval_res.result)

                        if grasp_eval_res.success and grasp_eval_res.separation_success:
                                #step1 += 1
                                #step2 = 0
                                print("Successful cable separation")
                                process_actionserver.publish_feedback()
                        else:
                                if grasp_eval_res.success:
                                        #stop_function("Cable separation was not succesful")
                                        print("Cable separation was not succesful")
                                else:
                                        #stop_function("Grasp evaluation failed")
                                        print("Grasp evaluation failed")
                        step2 = 4

                if step2 == 4:
                        waypoints_SC4 = []
                        init_pose = arm_sep.get_current_pose().pose
                        waypoints_SC4.append(init_pose)
                        sep_guide_forward_up = get_shifted_pose(op["spot"][1]["pose_corner"], [op["spot"][1]["width"] + fingers_size[0], op["spot"][1]["gap"]/2, op["spot"][1]["height"] + z_offset + fingers_size[2]/2, 0, 0, 0])
                        waypoints_SC4.append(ATC1.correctPose(sep_guide_forward_up, separation_arm, rotate = True, routing_app = False, ATC_sign = -1))
                        #plan, success = compute_cartesian_path_velocity_control([waypoints_SC4], [speed_execution], arm_side=separation_arm)
                        #if success:
                        #        execute_plan_async(separation_group, plan)
                        plan, success, motion_group_plan = compute_cartesian_path_velocity_control_arms_occlusions([waypoints_SC4], [speed_execution], arm_side=separation_arm)
                        if success:
                                execute_plan_async(motion_group_plan, plan)

                if step2 == 5:
                        #Apply tension
                        actuate_grippers(grasp_distance, gripper_speed, separation_arm, grasp=True)

                        waypoints_SC5 = []
                        init_pose = arm_sep.get_current_pose().pose
                        waypoints_SC5.append(init_pose)
                        sep_guide_tension = get_shifted_pose(op["spot"][1]["pose_corner"], [op["spot"][1]["width"]  + fingers_size[0] + force_offset, op["spot"][1]["gap"]/2, op["spot"][1]["height"] + z_offset + fingers_size[2]/2, 0, 0, 0])
                        waypoints_SC5.append(ATC1.correctPose(sep_guide_tension, separation_arm, rotate = True, routing_app = False, ATC_sign = -1))
                        #Move with force_control
                        if force_control_active:
                                rospy.wait_for_service('/right_norbdo/tare')
                                tare_forces_srv = rospy.ServiceProxy('right_norbdo/tare', Trigger)
                                tare_res = tare_forces_srv(TriggerRequest())
                                tare_forces_srvL = rospy.ServiceProxy('left_norbdo/tare', Trigger)
                                tare_res = tare_forces_srvL(TriggerRequest())
                        rospy.sleep(0.5) #REDUCE. Original 0.5
                        stop_mov_force = False #reset this value before activating the force_controlled
                        force_controlled = True
                        plan, success = compute_cartesian_path_velocity_control([waypoints_SC5], [speed_tension], arm_side=separation_arm)
                        if success:
                                success = execute_force_control(arm_side = separation_arm, plan = plan)
                        rospy.sleep(0.5) #REDUCE. Original 0.5
                        force_controlled = False
                        rospy.sleep(2.5) #REDUCE. Original 2.5

                if step2 == 6:
                        print("###########TEST CIRC###########")
                        #Circ motion
                        waypoints_SC6 = []
                        trash_pose = arm_sep.get_current_pose().pose
                        init_pose = arm_sep.get_current_pose().pose
                        waypoints_SC6.append(copy.deepcopy(init_pose))
                        init_pose_guides = ATC1.antiCorrectPose(init_pose, separation_arm, routing_app = False)
                        rot_center = get_shifted_pose(op["spot"][0]["pose_corner"],[op["spot"][0]['width']/2, op["spot"][0]['gap']/2, op["spot"][0]['height']/2, 0, 0, 0])
                        insert_guide_center_sep = get_shifted_pose(op["spot"][1]["pose_corner"],[op["spot"][1]['width']/2, op["spot"][1]['gap']/2, op["spot"][1]['height']/2, 0, 0, 0])
                        radius_rot = compute_distance(rot_center, init_pose_guides)
                        #final_point = in the direction rot_center --> guide a distance of radius_rot
                        final_cable_direction = get_axis(insert_guide_center_sep, rot_center)
                        final_point = copy.deepcopy(insert_guide_center_sep) #Same orientation
                        final_point.position.x = rot_center.position.x + final_cable_direction[0]*radius_rot
                        final_point.position.y = rot_center.position.y + final_cable_direction[1]*radius_rot
                        final_point.position.z = rot_center.position.z + final_cable_direction[2]*radius_rot #Maybe delete?

                        #Evaluate collisions in final_point, if there are, there slide a bit more distance and evaluate again. Check also when the distance to the next guide is already too small, so then it will just do a slide_top
                        #With the correct final_point, calculates the rotation knowing, center, initial and final arc points.

                        guide_yaw_axis = get_axis_from_RM(pose_to_frame(rot_center).M, "Y")
                        rot_axis = get_ort_axis(guide_yaw_axis, final_cable_direction)[2]
                        rot_axis = [-element for element in rot_axis]
                        dist_z = compute_distance_relative(final_point, init_pose_guides, guide_yaw_axis) #from circle to guide in z axis
                        # print(dist_z)
                        # print(radius_rot)
                        rot_degree = (math.asin(-dist_z/radius_rot))*180.0/math.pi

                        center_waypoints = []
                        circle_waypoints = []
                        success, center_waypoints, circle_waypoints = circular_trajectory(rot_center, init_pose_guides, rot_degree, rot_axis, center_waypoints, circle_waypoints, step = 2, rot_gripper = False)
                        for wp in circle_waypoints:
                                waypoints_SC6.append(ATC1.correctPose(wp, separation_arm, rotate = True, ATC_sign = -1, routing_app = False))

                        plan, success = compute_cartesian_path_velocity_control([waypoints_SC6], [speed_execution], arm_side=separation_arm)
                        if success:
                                execute_plan_async(separation_group, plan)
                        print("###########TEST CIRC END###########")

                if step2 == 7:
                        actuate_grippers(slide_distance, gripper_speed, separation_arm, grasp=False)
                        waypoints_SC7 = []
                        init_pose = arm_sep.get_current_pose().pose
                        waypoints_SC7.append(init_pose)
                        if separation_arm=="right":
                                sign_side = 1
                        else:
                                sign_side = -1
                        side_pose = get_shifted_pose(op["spot"][1]["pose_corner"], [op["spot"][1]["width"] + fingers_size[0] + x_offset, sign_side*0.1, 0, 0, 0, 0])
                        waypoints_SC7.append(ATC1.correctPose(side_pose, separation_arm, rotate = True, routing_app = False, ATC_sign = -1))
                        plan, success, motion_group_plan = compute_cartesian_path_velocity_control_arms_occlusions([waypoints_SC7], [slow_speed_execution], arm_side=separation_arm)
                        if success:
                                execute_plan_async(motion_group_plan, plan)

                if step2 == 8:
                        actuate_grippers(grasp_distance, gripper_speed, separation_arm, grasp=True)
                        waypoints_SC8 = []
                        init_pose = arm_sep.get_current_pose().pose
                        waypoints_SC8.append(init_pose)
                        if separation_arm=="right":
                                sign_side = 1
                        else:
                                sign_side = -1
                        side_pose_tension = get_shifted_pose(op["spot"][1]["pose_corner"], [op["spot"][1]["width"] + fingers_size[0] + x_offset, sign_side*(0.1 + force_offset/2), 0, 0, 0, 0])
                        waypoints_SC8.append(ATC1.correctPose(side_pose_tension, separation_arm, rotate = True, routing_app = False, ATC_sign = -1))
                        #Move with force_control
                        if force_control_active:
                                rospy.wait_for_service('/right_norbdo/tare')
                                tare_forces_srv = rospy.ServiceProxy('right_norbdo/tare', Trigger)
                                tare_res = tare_forces_srv(TriggerRequest())
                                tare_forces_srvL = rospy.ServiceProxy('left_norbdo/tare', Trigger)
                                tare_res = tare_forces_srvL(TriggerRequest())
                        rospy.sleep(0.5) #REDUCE. Original 0.5
                        stop_mov_force = False #reset this value before activating the force_controlled
                        force_controlled = True
                        plan, success = compute_cartesian_path_velocity_control([waypoints_SC8], [speed_tension], arm_side=separation_arm)
                        if success:
                                success = execute_force_control(arm_side = separation_arm, plan = plan)
                        rospy.sleep(0.5) #REDUCE. Original 0.5
                        force_controlled = False
                        rospy.sleep(2.5) #REDUCE. Original 2.5
                        holding_cables = True
                        holding_comeback_pose = get_shifted_pose(op["spot"][1]["pose_corner"],[op["spot"][1]["width"] + fingers_size[0] + x_offset + 0.05, sign_side*(0.05), op["spot"][1]['height']/2, 0, 0, 1.57])
                        holding_comeback_pose_corrected = ATC1.correctPose(holding_comeback_pose, separation_arm, rotate = True, routing_app = False, ATC_sign = -1)

                if step2 == 9:
                        #retract_arm(separation_arm, op["spot"][1])
                        step1 += 1
                        step2 = 0
                        process_actionserver.publish_feedback()


def grasp_cables(op, route_arm, separated = False):
        global step1
        global step2
        global process_actionserver
        global slow_speed_execution
        global speed_execution
        global fast_speed_execution
        global grasp_offset
        global open_distance
        global slide_distance
        global gripper_speed
        global holding_cables
        global force_limit_cable
        global force_limit
        global grasping_cables

        force_limit = copy.deepcopy(force_limit_cable)
        print("Grasp cables")
        print("Step1: "+str(step1))
        print("Step2: "+str(step2))

        #routing_app = not separated
        routing_app = True
        if separated:
                x_grasp_dist = -(grasp_offset+EEF_route[0]/2)
        else:
                x_grasp_dist = op["spot"]['width']+grasp_offset+EEF_route[0]/2

        print("###SEPARATED: " + str(separated))

        route_group = 'arm_'+route_arm
        arm = motion_groups[route_group]
        trash_pose = arm.get_current_pose().pose

        if separated:
                mold_up_forward = get_shifted_pose(op["spot"]["pose_corner"], [x_grasp_dist, (op["spot"]['gap']/2)+0.005, op["spot"]["height"] + 2*z_offset, 0, 0, 0])
                mold_forward = get_shifted_pose(op["spot"]["pose_corner"], [x_grasp_dist, (op["spot"]['gap']/2)+0.005, 0.009, 0, 0, 0])
        else:
                mold_up_forward = get_shifted_pose(op["spot"]["pose_corner"], [x_grasp_dist, op["spot"]['gap']/2, op["spot"]["height"] + 2*z_offset, 0, 0, 0])
                mold_forward = get_shifted_pose(op["spot"]["pose_corner"], [x_grasp_dist, op["spot"]['gap']/2, 0, 0, 0, 0])
        mold_up_forward_corrected = ATC1.correctPose(mold_up_forward, route_arm, rotate = True, ATC_sign = -1, routing_app = routing_app, secondary_frame = True)
        
        if separated:
                mold_forward_corrected = ATC1.correctPose(mold_forward, route_arm, rotate = True, ATC_sign = -1, routing_app = routing_app, secondary_frame = True)
        else:
                mold_forward_corrected = ATC1.correctPose(mold_forward, route_arm, rotate = True, ATC_sign = -1, routing_app = routing_app, secondary_frame = True)

        if step1 == 0:
                if step2 == 0:
                        #Move to grasp point offset
                        actuate_grippers(open_distance, gripper_speed, route_arm, grasp=False)
                        rospy.sleep(0.5)

                        waypoints_GC1 = []
                        init_pose = arm.get_current_pose().pose
                        waypoints_GC1.append(init_pose)
                        waypoints_GC1.append(mold_up_forward_corrected)
                        if separated:
                                plan, success, motion_group_plan = compute_cartesian_path_velocity_control_arms_occlusions([waypoints_GC1], [speed_execution], arm_side=route_arm)
                        else:
                                if holding_cables:
                                        plan, success = compute_cartesian_path_velocity_control([waypoints_GC1], [fast_speed_execution], arm_side=route_arm)
                                        motion_group_plan = route_group
                                        #holding_cables = False
                                else:
                                        plan, success, motion_group_plan = compute_cartesian_path_velocity_control_arms_occlusions([waypoints_GC1], [fast_speed_execution], arm_side=route_arm)
                        if success:
                                execute_plan_async(motion_group_plan, plan)
                
                if step2 == 1:
                        waypoints_GC2 = []
                        init_pose = arm.get_current_pose().pose
                        waypoints_GC2.append(init_pose)
                        waypoints_GC2.append(mold_forward_corrected)
                        plan, success = compute_cartesian_path_velocity_control([waypoints_GC2], [slow_speed_execution], arm_side=route_arm)
                        if success:
                                execute_plan_async(route_group, plan)

                if step2 == 2:
                        actuate_grippers(slide_distance, gripper_speed, route_arm, grasp=False)
                        #rospy.sleep(1) #REDUCE. Original 1
                        if separated:
                                init_pose = arm.get_current_pose().pose
                                retract_z = get_shifted_pose(op["spot"]["pose_corner"], [0, 0, op["spot"]["height"] + z_offset*2, 0, 0, 0])
                                retract_z_corrected = ATC1.correctPose(retract_z, route_arm, rotate = True, routing_app = routing_app, ATC_sign = -1, secondary_frame=True)
                                new_pose = copy.deepcopy(init_pose)
                                new_pose.position.z = retract_z_corrected.position.z
                                if route_arm=="right":
                                        sign_side = 1
                                else:
                                        sign_side = -1
                                side_pose = get_shifted_pose(new_pose, [0, sign_side*0.05, 0, 0, 0, 0])
                                waypoints_GC3 = [init_pose, new_pose, side_pose]
                                plan, success = compute_cartesian_path_velocity_control([waypoints_GC3], [speed_execution], arm_side=route_arm)
                                if success:
                                        execute_plan_async(route_group, plan)
                        else:
                                grasping_cables = True
                                step2 += 1
               
                if step2 == 3:
                        step1 += 1
                        step2 = 0
                        process_actionserver.publish_feedback()


def tape(tape_guides, grasp_guide):
        global grasp_offset
        global z_offset
        global gun_nozzle_offset
        global grasp_distance
        global step1
        global step2
        global stop_mov_force
        global force_controlled
        global force_offset
        global grasping_cables

        grasping_cables = False

        if (tape_guides[0]["pose_corner"].position.x <= grasp_guide["pose_corner"].position.x) and (tape_guides[1]["pose_corner"].position.x <= grasp_guide["pose_corner"].position.x):
                tape_arm = "left"
                grasp_arm = "right"
                fingers_size = ATC1.EEF_dict[ATC1.EEF_right].fingers_dim
                grasp_sign_arm = 1
        else:
                tape_arm = "right"
                grasp_arm = "left"
                fingers_size = ATC1.EEF_dict[ATC1.EEF_left].fingers_dim
                grasp_sign_arm = -1

        grasp_x_dir_test = get_shifted_pose(grasp_guide["pose_corner"], [1,0,0,0,0,0])
        if grasp_x_dir_test.position.x > grasp_x_dir_test.position.x:
                grasp_sign = grasp_sign_arm
        else:
                grasp_sign = -grasp_sign_arm

        tape_group_name = "arm_"+tape_arm
        grasp_group_name = "arm_"+grasp_arm
        tape_group = motion_groups[tape_group_name]
        grasp_group = motion_groups[grasp_group_name]

        guide_tape1 = get_shifted_pose(tape_guides[0]["pose_corner"], [tape_guides[0]['width']/2, tape_guides[0]['gap']/2, 0, 0, 0, 0])
        guide_tape2 = get_shifted_pose(tape_guides[1]["pose_corner"], [tape_guides[1]['width']/2, tape_guides[1]['gap']/2, 0, 0, 0, 0])
        tape_pose = get_middle_pose(guide_tape1, guide_tape2)
        tape_pose_offset = get_shifted_pose(tape_pose, [0, 0,  tape_guides[0]['height'] + z_offset + gun_nozzle_offset, 0, 0, 0])
        grasp_pose = get_shifted_pose(grasp_guide["pose_corner"], [(grasp_guide['width']/2)+grasp_sign*(grasp_guide['width']/2 + grasp_offset + fingers_size[0]/2), grasp_guide['gap']/2, 0, 0, 0, 0])
        grasp_pose_offset = get_shifted_pose(grasp_pose, [0, 0,  grasp_guide['height'] + z_offset, 0, 0, 0])

        if step1 == 0:
                if step2 == 0:
                        #Move to grasp point offset
                        retract_arm(grasp_arm, grasp_guide)
                        print("STEP0")

                if step2 == 1:
                        print("STEP1_beginning")
                        waypoints_T1 = []
                        init_pose = grasp_group.get_current_pose().pose
                        waypoints_T1.append(init_pose)
                        waypoints_T1.append(ATC1.correctPose(grasp_pose_offset, grasp_arm, rotate = True, ATC_sign = -1, routing_app = False, secondary_frame = True))
                        #visualize_keypoints_simple(waypoints_T1)
                        print("STEP1_middle")
                        plan, success, motion_group_plan = compute_cartesian_path_velocity_control_arms_occlusions([waypoints_T1], [fast_speed_execution], arm_side=grasp_arm)
                        print("STEP1_almostEnd")
                        if success:
                                execute_plan_async(motion_group_plan, plan)
                        print("STEP1")
                        rospy.sleep(0.5)

                if step2 == 2:
                        waypoints_T2 = []
                        init_pose = grasp_group.get_current_pose().pose
                        waypoints_T2.append(init_pose)
                        waypoints_T2.append(ATC1.correctPose(grasp_pose, grasp_arm, rotate = True, ATC_sign = -1, routing_app = False, secondary_frame = True))
                        plan, success = compute_cartesian_path_velocity_control([waypoints_T2], [slow_speed_execution], arm_side=grasp_arm)
                        if success:
                                execute_plan_async(grasp_group_name, plan)
                        print("STEP2")

                if step2 == 3:
                        actuate_grippers(grasp_distance, gripper_speed, grasp_arm, grasp=True)
                        #TENSION
                        # waypoints_TF1 = []
                        # init_pose = grasp_group.get_current_pose().pose
                        # waypoints_TF1.append(init_pose)
                        # grasp_pose_tension = get_shifted_pose(grasp_pose, [grasp_sign*force_offset+0.01, 0, 0, 0, 0, 0])
                        # waypoints_TF1.append(ATC1.correctPose(grasp_pose_tension, grasp_arm, rotate = True, ATC_sign = -1, routing_app = False, secondary_frame = True))
                        # if force_control_active:
                        #         rospy.wait_for_service('/right_norbdo/tare')
                        #         print("WAITING FOR TARING")
                        #         tare_forces_srv = rospy.ServiceProxy('right_norbdo/tare', Trigger)
                        #         tare_res = tare_forces_srv(TriggerRequest())
                        #         tare_forces_srvL = rospy.ServiceProxy('left_norbdo/tare', Trigger)
                        #         tare_res = tare_forces_srvL(TriggerRequest())
                        # rospy.sleep(0.5) #REDUCE. Original 0.5
                        # print("TARED CORRECTLY")
                        # stop_mov_force = False #reset this value before activating the force_controlled
                        # force_controlled = True
                        # plan, success = compute_cartesian_path_velocity_control([waypoints_TF1], [speed_tension], arm_side=route_arm)
                        # if success:
                        #         success = execute_force_control(arm_side = route_arm, plan = plan)
                        # rospy.sleep(0.5) #REDUCE. Original 0.5
                        # force_controlled = False
                        # rospy.sleep(2.5)
                        step2 = 4
                        print("STEP3")
                
                if step2 == 4:
                        waypoints_T3 = []
                        init_pose = tape_group.get_current_pose().pose
                        waypoints_T3.append(init_pose)
                        waypoints_T3.append(ATC1.correctPose(tape_pose_offset, tape_arm, rotate = True, ATC_sign = -1, routing_app = True))
                        plan, success = compute_cartesian_path_velocity_control([waypoints_T3], [speed_execution], arm_side=tape_arm)
                        if success:
                                execute_plan_async(tape_group_name, plan)
                        print("STEP4")

                if step2 == 5:
                        waypoints_T4 = []
                        init_pose = tape_group.get_current_pose().pose
                        waypoints_T4.append(init_pose)
                        waypoints_T4.append(ATC1.correctPose(tape_pose, tape_arm, rotate = True, ATC_sign = -1, routing_app = True))
                        plan, success = compute_cartesian_path_velocity_control([waypoints_T4], [slow_speed_execution], arm_side=tape_arm)
                        if success:
                                execute_plan_async(tape_group_name, plan)
                        print("STEP5")

                if step2 == 6:
                        actuate_gun()
                        rospy.sleep(1)

                if step2 == 7:
                        waypoints_T5 = []
                        init_pose = tape_group.get_current_pose().pose
                        waypoints_T5.append(init_pose)
                        waypoints_T5.append(ATC1.correctPose(tape_pose_offset, tape_arm, rotate = True, ATC_sign = -1, routing_app = True))
                        plan, success = compute_cartesian_path_velocity_control([waypoints_T5], [speed_execution], arm_side=tape_arm)
                        if success:
                                execute_plan_async(tape_group_name, plan)
               
                if step2 == 8:
                        retract_arm(grasp_arm, grasp_guide)

                #visualize_keypoints_simple([tape_pose, tape_pose_offset])