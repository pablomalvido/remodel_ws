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


#ROS init
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('elvez_process_action_server', anonymous=True)
rospack = rospkg.RosPack()
listener = tf.TransformListener()
mode_publisher = rospy.Publisher('/UI/mode', String, queue_size=1)
tool_publisher = rospy.Publisher('/UI/tool', ATC_msg, queue_size=1)
confirmation_received = False
confirmation_msg = 'N'
logs_publisher = rospy.Publisher('/UI/logs', String, queue_size=1)
confirmation_publisher = rospy.Publisher('/UI/confirm_req', String, queue_size=1)
feedback_publisher = rospy.Publisher('/UI/feedback', configProp, queue_size=1)
def callback_confirm(msg):
    global confirmation_received
    global confirmation_msg
    confirmation_msg = msg.data
    confirmation_received = True
confirmation_subscriber = rospy.Subscriber('/UI/confirm_res', String, callback_confirm) 


#Defines the movegroups. 
#IMPORTANT: These must be the name of these variables and move_groups. Otherwise, additional modifications of the code will be required.
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
arm_left = moveit_commander.MoveGroupCommander("arm_left")
arm_right = moveit_commander.MoveGroupCommander("arm_right")
arms = moveit_commander.MoveGroupCommander("arms")
torso = moveit_commander.MoveGroupCommander("torso")

motion_groups = {"arm_left": arm_left, "arm_right": arm_right, "arms": arms, "torso": torso}

arm_left.clear_pose_targets()
arm_right.clear_pose_targets()
arms.clear_pose_targets()
torso.clear_pose_targets()

arm_left.stop()
arm_right.stop()
arms.stop()
torso.stop()

# status_movement_left = 0
# status_movement_right = 0
status_movement = 0
status_movement2 = 0
stop_mov = False
force_controlled = False
larger_slide = False

ops_info_text=[]

# def callback_left(status):
#     global status_movement_left 
#     if len(status.status_list)>0:
#         status_movement_left = status.status_list[-1].status

# def callback_right(status):
#     global status_movement_right 
#     if len(status.status_list)>0:
#         status_movement_right = status.status_list[-1].status

def callback_motion_status(status):
    global status_movement 
    if len(status.status_list)>0:
        status_movement = status.status_list[-1].status

#subsL = rospy.Subscriber('/sda10f/sda10f_r1_controller/joint_trajectory_action/status', GoalStatusArray, callback_left)  
# subsR = rospy.Subscriber('/sda10f/sda10f_r2_controller/joint_trajectory_action/status', GoalStatusArray, callback_right)
subs_motion_status = rospy.Subscriber('/move_group/status', GoalStatusArray, callback_motion_status) 

def get_mode_callback(req): #Create a subs to update this varibale from other nodes like from the manual control
        global modeUI
        resp = TriggerResponse()
        resp.success=True
        resp.message=modeUI	


rospy.Service("/UI/get_mode", Trigger, get_mode_callback)

#Initialization
modeUI = 'Idle'
PC_op = ''
step1 = 0
step2 = 0
routed_guides = []
gripper_right_distance = 0
gripper_left_distance = 0
holding_cables = False
holding_comeback_pose_corrected = Pose()
grasping_cables = False

#Read config params
config_full_pkg_path = str(rospack.get_path('ROS_UI_backend')) #UI_nodes_pkg
config_file_name = config_full_pkg_path + "/files/config.csv"
config = {}
try:
        with open(config_file_name) as csvfile:
                reader = csv.DictReader(csvfile)
                for row in reader:
                        config[row['prop']] = row['value']
except:
        print("Cannot access to the configuration params")
        exit()

rospy.wait_for_service('/rosapi/nodes')
get_nodes_srv = rospy.ServiceProxy('rosapi/nodes', Nodes)
active_nodes = get_nodes_srv(NodesRequest()).nodes
real_robot_nodes = ['/io_relay','/joint_state','/joint_trajectory_action','/move_group','/motion_streaming_interface','/robot_state_publisher']
if all(x in active_nodes for x in real_robot_nodes):
        real_robot = True
else:
        real_robot = False 

print(config)
execute_grippers=False
use_camera=False

try:
        if real_robot:
                print('REAL ROBOT')
                if config['grippers_control_real'] == 'Y' or config['grippers_control_real'] == 'y':
                        execute_grippers = True #True
                else:
                        execute_grippers = False
                if config['gun_control_real'] == 'Y' or config['gun_control_real'] == 'y':
                        execute_gun = True #True
                else:
                        execute_gun = False
                if config['force_control_real'] == 'Y' or config['force_control_real'] == 'y':
                        force_control_active = True #True
                else:
                        force_control_active = False
                if config['use_camera_real'] == 'Y' or config['use_camera_real'] == 'y':
                        use_camera = True #True
                else:
                        use_camera = False
                        print(execute_grippers)
                        print(use_camera)
                speed_limit = min(float(config['speed_real_per']),0.3) #0.05
                fast_speed_execution = min(float(config['speed_fast_real_mms']),100) #40mm/s
                speed_execution = min(float(config['speed_real_mms']),80) #20mm/s
                slow_speed_execution = min(float(config['speed_slow_real_mms']),40) #10mm/s
                speed_tension = min(float(config['speed_tension_real_mms']),40) #10mm/s
        else:
                print('SIMULATED ROBOT')
                if config['grippers_control_demo'] == 'Y' or config['grippers_control_demo'] == 'y':
                        execute_grippers = True #False
                else:
                        execute_grippers = False
                if config['use_camera_demo'] == 'Y' or config['use_camera_demo'] == 'y':
                        use_camera = True #False
                else:
                        use_camera = False
                force_control_active = False
                execute_gun = False
                speed_limit = float(config['speed_demo_per']) #1
                fast_speed_execution = float(config['speed_fast_demo_mms']) #100mm/s
                speed_execution = float(config['speed_demo_mms']) #50mm/s
                slow_speed_execution = float(config['speed_slow_demo_mms']) #30mm/s
                speed_tension = float(config['speed_tension_demo_mms']) #20mm/s
except:
        print("Error defining config robot values 1")
        exit()

print(execute_grippers)
print(use_camera)        

#Speed       
arm_left.set_max_velocity_scaling_factor(speed_limit)
arm_right.set_max_velocity_scaling_factor(speed_limit)
arms.set_max_velocity_scaling_factor(speed_limit)
torso.set_max_velocity_scaling_factor(speed_limit)

try:
        #Offsets
        z_offset = float(config['offset_z'])/1000 #0.02
        x_offset = float(config['offset_x'])/1000 #0.02
        z_offset2 = float(config['offset_z'])/1000 #0.02 (What is this?)
        grasp_offset = float(config['offset_grasp'])/1000 #0.005
        force_offset = float(config['offset_force'])/1000 #0.01
        #pick_grasp_offset = float(config['offset_pick_grasp'])/1000 #0.015
        pick_grasp_offset = {}
        pick_grasp_offset['WH1'] = 0.015
        pick_grasp_offset['WH3'] = 0.0255
        z_offset_pick = float(config['offset_pick_z'])/1000 #0.05
        z_offset_photo = float(config['offset_photo_z'])/1000 #0.1
        gun_nozzle_offset = 0.1

        rot_center = Pose()
        rot_center_up = False

        #Force sensor
        force_cable = {}
        force_connector = {}
        force_cable['WH1'] = float(config['cable_tension_wh1']) #3.5N
        force_cable['WH3'] = float(config['cable_tension_wh3']) #3.5N
        force_connector['WH1'] = float(config['connector_tension_wh1']) #5N
        force_connector['WH3'] = float(config['connector_tension_wh3']) #5N

        #Gripper parameters
        open_distance = float(config['gripper_open_dist']) #105
        slide_distance = float(config['gripper_slide_dist']) #36
        grasp_distance = float(config['gripper_grasp_dist']) #30
        gripper_speed = float(config['gripper_speed']) #30
        gripper_speed_slow = float(config['gripper_speed_slow']) #20
except:
        print("Error defining config robot values 2")
        exit()

#Other
force_limit_cable = copy.deepcopy(force_cable[list(force_cable.keys())[0]])
force_limit_connector = copy.deepcopy(force_connector[list(force_connector.keys())[0]])
force_limit = copy.deepcopy(force_limit_cable)
grasp_point_global = [0,0]
gripper_right_finish = False
gripper_left_finish = False

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


def actuate_grippers(distance, speed, arm, grasp=False):
        global execute_grippers
        global gripper_left_finish
        global gripper_right_finish
        global gripper_right_distance
        global gripper_left_distance
        global stop_mov

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
                if distance > slide_distance:
                        right_gripper_msg.value = "Gripper open"
                elif distance > grasp_distance:
                        right_gripper_msg.value = "Gripper slide"
                else:
                        right_gripper_msg.value = "Gripper closed"    
                feedback_publisher.publish(right_gripper_msg)    
        if arm == "left" or arm == "both":
                gripper_left_distance = distance
                left_gripper_msg = configProp()
                left_gripper_msg.prop = "eef_left_status"
                if distance > slide_distance:
                        left_gripper_msg.value = "Gripper open"
                elif ((distance+9) > grasp_distance):
                        left_gripper_msg.value = "Gripper slide"
                else:
                        left_gripper_msg.value = "Gripper closed"    
                feedback_publisher.publish(left_gripper_msg)    


def actuate_gun():
        global step2
        global execute_gun
        global stop_mov
        if execute_gun and not stop_mov:
                rospy.wait_for_service('/gun/tape')
                tape_srv = rospy.ServiceProxy('gun/tape', Trigger)
                tare_res = tape_srv(TriggerRequest())
        step2+=1


print('EXECUTE GRIPPERS: ' + str(execute_grippers))
print('EXECUTE CAMERA: ' + str(use_camera))


class EEF(object):
    """This class defines an end effector"""
    def __init__(self, EE_end_frame=PyKDL.Frame(), EE_end_frame2=PyKDL.Frame(), x=0, y=0, z=0, ATC_frame=PyKDL.Frame(), name="", path="", fingers_dim = [0,0,0]):
        """
        - EE_end_frame: Transform from the EEF base frame (the ATC part of the tool) to its actuation frame [PyKDL.Frame]
        - x: Tool size in x dimension [float]
        - y: Tool size in y dimension [float]
        - z: Tool size in z dimension. This includes the ATC part of the tool [float]
        - ATC_frame: Frame of the EEF slot in the tool changer station [PyKDL.Frame]
        - name: Name of the tool. Name of the collision object created for the tool [string]
        - path: Path of the stl model of the tool [string]
        """
        self.EE_end_frame = EE_end_frame
        self.EE_end_frame2 = EE_end_frame2
        self.x = x
        self.y = y
        self.z = z
        self.ATC_frame = ATC_frame
        self.name = name
        self.path = path
        self.fingers_dim = fingers_dim


#Motion functions
def move_group_async(group):
        global status_movement
        global motion_groups
        global stop_mov
        global step2
        global process_actionserver
        msg_log = String()
        msg_log.data = str(group) + " moving..."
        logs_publisher.publish(msg_log)
        motion_groups[group].go(wait=False) 
        while status_movement==3 and not stop_mov:
                rospy.sleep(0.01)
        while (status_movement==0 or status_movement==1 or status_movement==2) and not stop_mov:
                rospy.sleep(0.05)
        if not stop_mov:
                step2 += 1
        if status_movement==3:
                msg_log.data = "Successful movement"
        else:
                msg_log.data = "Failure movement" 
        logs_publisher.publish(msg_log)
        process_actionserver.publish_feedback()
        

def execute_plan_async(group, plan):
        global status_movement2
        global motion_groups
        global stop_mov
        global step2
        global process_actionserver
        msg_log = String()
        msg_log.data = str(group) + " moving..."
        logs_publisher.publish(msg_log)
        motion_groups[group].execute(plan, wait=False)
        wait_step_i = 0
        while (status_movement2==3 or status_movement2==4 or status_movement2==2) and not stop_mov:
                rospy.sleep(0.01)
                wait_step_i+=1
                if wait_step_i>100:
                       break
        print(str(group) + " STATUS MOTION " + str(status_movement2))
        while (status_movement2==0 or status_movement2==1 or status_movement2==2) and not stop_mov:
                rospy.sleep(0.05)
        print(str(group) + " STATUS MOTION " + str(status_movement2))
        rospy.sleep(0.5) #REDUCE. Original 0.5
        if not stop_mov:
                step2 += 1
        if status_movement2==3:
                msg_log.data = "Successful movement"
        else:
                msg_log.data = "Failure movement"
        logs_publisher.publish(msg_log) 
        process_actionserver.publish_feedback()
       
# def move_left_arm(goal_pose, type_goal = "pose"):
#         global status_movement_left
#         global stop_mov
#         global step
#         if type_goal == "pose":
#                 arm_left.set_pose_target(goal_pose)
#         elif type_goal == "name":
#                 arm_left.set_named_target(goal_pose)
#         arm_left.go(wait=False)
#         while status_movement_left==3 and not stop_mov:
#                 rospy.sleep(0.01)
#         while (status_movement_left==0 or status_movement_left==1 or status_movement_left==2) and not stop_mov:
#                 rospy.sleep(0.05)
#         if not stop_mov:
#                 step += 1

# def move_right_arm(goal_pose, type_goal = "pose"):
#         global status_movement_right
#         global stop_mov
#         global step
#         if type_goal == "pose":
#                 arm_right.set_pose_target(goal_pose)
#         elif type_goal == "name":
#                 arm_right.set_named_target(goal_pose)
#         arm_right.go(wait=False)
#         while status_movement_right==3 and not stop_mov:
#                 rospy.sleep(0.01)
#         while (status_movement_right==0 or status_movement_right==1 or status_movement_right==2) and not stop_mov:
#                 rospy.sleep(0.05)
#         if not stop_mov:
#                 step += 1

# def move_both(goal_pose_left, goal_pose_right, type_goal = "pose"):
#         global status_movement_left
#         global status_movement_right
#         global stop_mov
#         global step
#         left_started = False
#         right_started = False
#         both_started = False
#         any_started = False
#         if type_goal == "pose":
#                 arms.set_pose_target(goal_pose_left, "arm_left_link_7_t")
#                 arms.set_pose_target(goal_pose_right, "arm_right_link_7_t")
#         elif type_goal == "name":
#                 arms.set_named_target(goal_pose_left)
#         arms.go(wait=False)
#         while not both_started and not stop_mov:
#                 if status_movement_left!=3:
#                         left_started = True
#                 if status_movement_right!=3:
#                         right_started = True
#                 if left_started and right_started:
#                         both_started = True
#                 if left_started or right_started:
#                         any_started = True
#                 rospy.sleep(0.01)
#         while (status_movement_left==0 or status_movement_left==1 or status_movement_left==2 or status_movement_right==0 or status_movement_right==1 or status_movement_right==2) and not stop_mov:
#                 rospy.sleep(0.05)
#         if not stop_mov:
#                 step += 1

# def cartesian_mov_right(plan):
#         global status_movement_right
#         global stop_mov
#         global step
#         arm_right.execute(plan, wait=False)
#         while status_movement_right==3 and not stop_mov:
#                 rospy.sleep(0.01)
#         while (status_movement_right==0 or status_movement_right==1 or status_movement_right==2) and not stop_mov:
#                 rospy.sleep(0.05)
#         rospy.sleep(0.5)
#         if not stop_mov:
#                 step += 1

# def cartesian_mov_left(plan):
#         global status_movement_left
#         global stop_mov
#         global step
#         arm_left.execute(plan, wait=False)
#         while status_movement_left==3 and not stop_mov:
#                 rospy.sleep(0.01)
#         while (status_movement_left==0 or status_movement_left==1 or status_movement_left==2) and not stop_mov:
#                 rospy.sleep(0.05)
#         rospy.sleep(0.5)
#         if not stop_mov:
#                 step += 1

# def cartesian_mov_both(plan):
#         global status_movement_left
#         global status_movement_right
#         global stop_mov
#         global step
#         arms.execute(plan, wait=False)
#         while not both_started and not stop_mov:
#                 if status_movement_left!=3:
#                         left_started = True
#                 if status_movement_right!=3:
#                         right_started = True
#                 if left_started and right_started:
#                         both_started = True
#                 if left_started or right_started:
#                         any_started = True
#                 rospy.sleep(0.01)
#         while (status_movement_left==0 or status_movement_left==1 or status_movement_left==2 or status_movement_right==0 or status_movement_right==1 or status_movement_right==2) and not stop_mov:
#                 rospy.sleep(0.05)
#         if not stop_mov:
#                 step += 1
"""
def execute_plan_async(group, plan):
        global status_movement_left
        global status_movement_right
        global motion_groups
        global stop_mov
        global step2
        if group=="arm_right":
                motion_groups[group].execute(plan, wait=False)
                while status_movement_right==3 and not stop_mov:
                        rospy.sleep(0.01)
                while (status_movement_right==0 or status_movement_right==1 or status_movement_right==2) and not stop_mov:
                        rospy.sleep(0.05)
                rospy.sleep(0.5)
        if group=="arm_left":
                motion_groups[group].execute(plan, wait=False)
                while status_movement_left==3 and not stop_mov:
                        rospy.sleep(0.01)
                while (status_movement_left==0 or status_movement_left==1 or status_movement_left==2) and not stop_mov:
                        rospy.sleep(0.05)
                rospy.sleep(0.5)        
        if group=="arms":
                left_started = False
                right_started = False
                both_started = False
                any_started = False
                motion_groups[group].execute(plan, wait=False)
                while not both_started and not stop_mov:
                        if status_movement_left!=3:
                                left_started = True
                        if status_movement_right!=3:
                                right_started = True
                        if left_started and right_started:
                                both_started = True
                        if left_started or right_started:
                                any_started = True
                        rospy.sleep(0.01)
                while (status_movement_left==0 or status_movement_left==1 or status_movement_left==2 or status_movement_right==0 or status_movement_right==1 or status_movement_right==2) and not stop_mov:                        rospy.sleep(0.05)
        if not stop_mov:
                step2 += 1
"""

class ATC(object):
    """
    This class manages everything related with the EEFs of the scene, including tool changing, goal poses corrections to the EEF action frames, etc.
    """
    global scene

    def __init__(self, frame_id = "base_link", left_tool = EEF(), eef_link_left = "", right_tool = EEF(), eef_link_right = "", ATC_tools = [], left_ATC_angle=0, right_ATC_angle=0, left_ATC_dist=0, right_ATC_dist=0):
        """
        The constructor function initializes the tools state, adds the tools to the scene, and updates the allowed collision matrix to consider EEF collisions.
        - frame_id: Name of the parent frame used to define the ATC_frames of the EEFs [String]
        - left_tool: EEF attached to the left arm of the robot [EEF]
        - eef_link_left: Name of the link attached to the EEF of the left arm [string]
        - right_tool: EEF attached to the right arm of the robot [EEF]
        - eef_link_right: Name of the link attached to the EEF of the right arm [string]
        - ATC_tools: List of EEFs located initially in the ATC station [list(EEF)]
        - left_ATC_angle: Z angle between the left arm wrist and the left arm tool changer/EEF
        - right_ATC_angle: Z angle between the left arm wrist and the right arm tool changer/EEF
        - left_ATC_dist: Length of the robot part of the left ATC
        - right_ATC_dist: Length of the robot part of the right ATC
        """
        self.left_ATC_angle = left_ATC_angle
        self.right_ATC_angle = right_ATC_angle
        self.left_ATC_dist = left_ATC_dist
        self.right_ATC_dist = right_ATC_dist
        self.eef_link_left = eef_link_left
        self.eef_link_right = eef_link_right

        #Remove other EEF collision objects from the scene
        attached_objects = scene.get_attached_objects()
        if len(attached_objects)>0:
                for object_name in attached_objects:
                    if "EEF" in object_name:
                        try:
                            scene.remove_attached_object(eef_link_left, name=object_name)
                            self.wait_update_object(object_name, EE_is_attached=False, EE_is_known=True)
                            rospy.sleep(0.5)
                        except:
                            pass
                        try:
                            scene.remove_attached_object(eef_link_right, name=object_name)
                            self.wait_update_object(object_name, EE_is_attached=False, EE_is_known=True)
                            rospy.sleep(0.5)
                        except:
                            pass

                scene_objects = scene.get_known_object_names()
                for object_name in scene_objects:   
                    if "EEF" in object_name:     
                        scene.remove_world_object(object_name)
                        self.wait_update_object(object_name, EE_is_attached=False, EE_is_known=False)
                        rospy.sleep(0.5)

        #Add collision objects for the EEFs
        self.EEF_left = ""
        self.EEF_right = ""
        #Dictionary with all the EEF
        self.EEF_dict = {}

        #Add Left EEF
        if left_tool.name != "":
            trash_pose = arm_left.get_current_pose().pose
            self.EEF_dict[left_tool.name] = left_tool
            EE_pose = geometry_msgs.msg.PoseStamped()
            EE_pose.header.frame_id = frame_id
            left_wrist_pose = arm_left.get_current_pose().pose
            EE_pose.pose = self.correctPose(left_wrist_pose, "left", ATC_sign = 1)
            self.EEF_left = left_tool.name
            try:
                scene.add_mesh(self.EEF_left, EE_pose, left_tool.path)
                self.wait_update_object(self.EEF_left, EE_is_attached=False, EE_is_known=True)
                #Attach gripper
                rospy.sleep(0.5)
                scene.attach_mesh(eef_link_left, self.EEF_left, touch_links=[eef_link_left])
                self.wait_update_object(self.EEF_left, EE_is_attached=True, EE_is_known=False)
            except:
                print("Error adding left EEF to the scene")

        #Add right EEF
        if right_tool.name != "":
            self.EEF_dict[right_tool.name] = right_tool
            EE_pose = geometry_msgs.msg.PoseStamped()
            EE_pose.header.frame_id = frame_id
            right_wrist_pose = arm_right.get_current_pose().pose
            EE_pose.pose = self.correctPose(right_wrist_pose, "right", ATC_sign = 1)
            self.EEF_right = right_tool.name
            try:
                scene.add_mesh(self.EEF_right, EE_pose, right_tool.path)
                self.wait_update_object(self.EEF_right, EE_is_attached=False, EE_is_known=True)
                #Attach gripper
                rospy.sleep(0.5)
                scene.attach_mesh(eef_link_right, self.EEF_right, touch_links=[eef_link_right])
                self.wait_update_object(self.EEF_right, EE_is_attached=True, EE_is_known=False)
            except:
                print("Error adding right EEF to the scene")

        #Add the rest of tools in the ATC platform
        for tool in ATC_tools:
            self.EEF_dict[tool.name] = tool
            EE_pose = geometry_msgs.msg.PoseStamped()
            EE_pose.header.frame_id = frame_id
            EE_pose.pose = frame_to_pose(tool.ATC_frame)
            try:
                scene.add_mesh(tool.name, EE_pose, tool.path)
                self.wait_update_object(tool.name, EE_is_attached=False, EE_is_known=True)
            except:
                print("Error adding tool to the scene")

        #Add attached EEFs to the initial ACM
        #Get initial ACM
        rospy.wait_for_service('/get_planning_scene')
        my_service = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        sceneReq = PlanningSceneComponents(components=128)
        sceneResponse = my_service(sceneReq)
        acm = sceneResponse.scene.allowed_collision_matrix

        rospy.wait_for_service('/apply_planning_scene')
        my_service = rospy.ServiceProxy('/apply_planning_scene', ApplyPlanningScene)
        sceneReq = PlanningScene()

        #Detect index of tool changer links (wrists)
        i=0
        left_attach_link_index = -1
        right_attach_link_index = -1
        for link_name in acm.entry_names:
                if link_name == eef_link_left:
                        left_attach_link_index = i
                if link_name == eef_link_right:
                        right_attach_link_index = i        
                i+=1
        if left_attach_link_index == -1 or right_attach_link_index == -1:
                print("Error")

        #Add two new columns to the existig rows
        acm.entry_names.append(self.EEF_left) #-2 index
        acm.entry_names.append(self.EEF_right) #-1 index
        i=0
        for val in range(len(acm.entry_values)):
                acm.entry_values[i].enabled.append(False)
                acm.entry_values[i].enabled.append(False)
                i+=1
        #Collisions are allowed just with the tool changers (adjacent to the tools)
        acm.entry_values[left_attach_link_index].enabled[-2] = True #Left tool changer - Left EEF
        acm.entry_values[right_attach_link_index].enabled[-1] = True #Right tool changer - Right EEF

        #Add the values of the two new rows
        entry_value_left = AllowedCollisionEntry()
        entry_value_left.enabled = []
        entry_value_right = AllowedCollisionEntry()
        entry_value_right.enabled = []
        for link_name in acm.entry_names:
                entry_value_left.enabled.append(False)
                entry_value_right.enabled.append(False)
        #Collisions are allowed just with the tool changers (adjacent to the tools)
        
        entry_value_left.enabled[left_attach_link_index] = True
        entry_value_right.enabled[right_attach_link_index] = True
        acm.entry_values.append(entry_value_left)
        acm.entry_values.append(entry_value_right)

        #Update ACM
        sceneReq.allowed_collision_matrix = acm
        sceneReq.is_diff = True
        sceneResponse = my_service(sceneReq)

        self.ACM = acm


    def wait_update_object(self, EE_name, EE_is_known=False, EE_is_attached=False):
        """
        Wait until the object has been added/removed/attached/detached in the scene. The possible combinations are:
        - Add: EE_is_known=True, EE_is_attached=False
        - Attach: EE_is_known=True, EE_is_attached=True
        - Detach: EE_is_known=True, EE_is_attached=False
        - Remove: EE_is_known=False, EE_is_attached=False

        - EE_name: Name of the EEF collision object whose state is modified [string]
        """
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < 2) and not rospy.is_shutdown():
                # Test if the EEF is in attached objects
                attached_objects = scene.get_attached_objects([EE_name])
                is_attached = len(attached_objects.keys()) > 0
                # Test if the EEF is in the scene.
                # Note that attaching the EEF will remove it from known_objects
                is_known = EE_name in scene.get_known_object_names()
                # Test if we are in the expected state
                if (EE_is_attached == is_attached) and (EE_is_known == is_known):
                        return True
                # Sleep so that we give other threads time on the processor
                rospy.sleep(0.1)
                seconds = rospy.get_time()
        return False


    def changeTool(self, new_tool, arm_side):
        """
        Manages the ATC.
        - new_tool: Name of the new tool to attach [string]
        - arm_side: Arm side in which to change the tool ["left" or "right"]
        IMPORTANT: Modify all the named target poses to match the poses defined in your SRDF file. The name of the move_groups is asumed to be: arm_left, arm_right, arms and torso.
        """
        z_offset = 0.05 #In meters
        vert_offset = 0.07 #In meters

        #Get initial ACM
        rospy.wait_for_service('/get_planning_scene')
        get_scene_service = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        getSceneReq = PlanningSceneComponents(components=128)
        sceneResponse = get_scene_service(getSceneReq)
        original_acm = sceneResponse.scene.allowed_collision_matrix
        new_acm = copy.deepcopy(original_acm)
        #Initialize servize to modify the ACM
        rospy.wait_for_service('/apply_planning_scene')
        update_scene_service = rospy.ServiceProxy('/apply_planning_scene', ApplyPlanningScene)

        # Moving to initial position
        print("Moving robot to ATC position")
        arms.set_named_target("arms_platform_5")
        arms.go(wait=True)
        rospy.sleep(0.5)
        torso.set_named_target("torso_combs")
        torso.go(wait=True)
        rospy.sleep(0.5)

        if arm_side == "right":
                arm_left.set_named_target("arm_left_down")
                arm_left.go(wait=True)
                rospy.sleep(0.5)
                arm_right.set_named_target("arm_right_ATC")
                arm_right.go(wait=True)
                #arms.go(wait=True)
                rospy.sleep(0.5)
                torso.set_named_target("torso_ATC")
                torso.go(wait=True)
                rospy.sleep(0.5)
                change_arm = arm_right
                current_tool = self.EEF_right
                eef_link_arm = self.eef_link_right
                touch_links_arm = [self.eef_link_right]
                ATC_dist = self.right_ATC_dist
                ATC_ang = self.right_ATC_angle
                self.EEF_right = "None"
        else:
                # arm_right.set_named_target("arm_right_down")
                # arm_right.go(wait=True)
                # rospy.sleep(0.5)
                # arm_left.set_named_target("arm_left_ATC")
                # arm_left.go(wait=True)
                arms.set_named_target("arms_ATC_left")
                arms.go(wait=True)
                rospy.sleep(0.5)
                torso.set_named_target("torso_ATC")
                torso.go(wait=True)
                rospy.sleep(0.5)
                change_arm = arm_left
                current_tool = self.EEF_left
                eef_link_arm = self.eef_link_left
                touch_links_arm = [self.eef_link_left]
                ATC_dist = self.left_ATC_dist
                ATC_ang = self.left_ATC_angle
                self.EEF_left = "None"

        #Modify ACM
        #Detect index of tool that is gonna be changed
        i=0
        tool_index = -1
        for link_name in new_acm.entry_names:
                if link_name == current_tool:
                        tool_index = i     
                i+=1
        if tool_index == -1:
                print("ACM update error")
        else: #Update the tool name
                new_acm.entry_names[tool_index] = new_tool

        #Calculate trajectory keypoints
        ATC_leave_pose = frame_to_pose(self.EEF_dict[current_tool].ATC_frame)
        ATC_pick_pose = frame_to_pose(self.EEF_dict[new_tool].ATC_frame)
        ATC_leave_tool_pose_offset_approach_1 = get_shifted_pose(ATC_leave_pose, [vert_offset, 0, -z_offset - self.EEF_dict[new_tool].z, 0, 0, 0])
        ATC_leave_tool_pose_offset_approach_2 = get_shifted_pose(ATC_leave_pose, [vert_offset, 0, 0, 0, 0, 0])
        ATC_leave_tool_pose_offset_retract_1 = get_shifted_pose(ATC_leave_pose, [0, 0, -z_offset, 0, 0, 0])
        ATC_pick_tool_pose_offset_approach_1 = get_shifted_pose(ATC_pick_pose, [0, 0, -z_offset - self.EEF_dict[new_tool].z, 0, 0, 0])
        ATC_pick_tool_pose_offset_approach_2 = get_shifted_pose(ATC_pick_pose, [0, 0, -z_offset, 0, 0, 0])
        ATC_pick_tool_pose_offset_retract_1 = get_shifted_pose(ATC_pick_pose, [vert_offset, 0, 0, 0, 0, 0])
        ATC_pick_tool_pose_offset_retract_2 = get_shifted_pose(ATC_pick_pose, [vert_offset, 0, -z_offset - self.EEF_dict[new_tool].z, 0, 0, 0])

        initial_pose = change_arm.get_current_pose().pose #correct it, get the pose of the EEF base
        initial_pose_corrected_dist = get_shifted_pose(initial_pose, [0, 0, ATC_dist, 0, 0, 0])
        initial_frame_corrected = pose_to_frame(initial_pose_corrected_dist)
        initial_frame_corrected.M.DoRotZ(-ATC_ang)
        initial_pose_corrected = frame_to_pose(initial_frame_corrected)

        #Create waypoints
        success, approach_leave_waypoints = interpolate_trajectory(initial_pose = initial_pose_corrected, final_pose = ATC_leave_tool_pose_offset_approach_1, step_pos_min = 0.01, step_deg_min = 2, n_points_max = 20)
        if not success:
                return
        success, approach_leave_waypoints2 = interpolate_trajectory(initial_pose = ATC_leave_tool_pose_offset_approach_1, final_pose = ATC_leave_tool_pose_offset_approach_2, step_pos_min = 0.01, step_deg_min = 2, n_points_max = 20)
        if not success:
                return
        success, leave_tool_waypoints = interpolate_trajectory(initial_pose = ATC_leave_tool_pose_offset_approach_2, final_pose = ATC_leave_pose, step_pos_min = 0.01, step_deg_min = 2, n_points_max = 20)
        if not success:
                return
        success, retract_leave_waypoints = interpolate_trajectory(initial_pose = ATC_leave_pose, final_pose = ATC_leave_tool_pose_offset_retract_1, step_pos_min = 0.01, step_deg_min = 2, n_points_max = 20)
        if not success:
                return
        success, retract_initial_waypoints = interpolate_trajectory(initial_pose = ATC_leave_tool_pose_offset_retract_1, final_pose = initial_pose_corrected, step_pos_min = 0.01, step_deg_min = 2, n_points_max = 20)
        if not success:
                return

        success, approach_pick_waypoints = interpolate_trajectory(initial_pose = initial_pose_corrected, final_pose = ATC_pick_tool_pose_offset_approach_1, step_pos_min = 0.01, step_deg_min = 2, n_points_max = 20)
        if not success:
                return
        success, approach_pick_waypoints2 = interpolate_trajectory(initial_pose = ATC_pick_tool_pose_offset_approach_1, final_pose = ATC_pick_tool_pose_offset_approach_2, step_pos_min = 0.01, step_deg_min = 2, n_points_max = 20)
        if not success:
                return
        success, pick_tool_waypoints = interpolate_trajectory(initial_pose = ATC_pick_tool_pose_offset_approach_2, final_pose = ATC_pick_pose, step_pos_min = 0.01, step_deg_min = 2, n_points_max = 20)
        if not success:
                return
        success, retract_pick_waypoints = interpolate_trajectory(initial_pose = ATC_pick_pose, final_pose = ATC_pick_tool_pose_offset_retract_1, step_pos_min = 0.01, step_deg_min = 2, n_points_max = 20)
        if not success:
                return
        success, retract_pick_waypoints2 = interpolate_trajectory(initial_pose = ATC_pick_tool_pose_offset_retract_1, final_pose = ATC_pick_tool_pose_offset_retract_2, step_pos_min = 0.01, step_deg_min = 2, n_points_max = 20)
        if not success:
                return
        success, final_waypoints = interpolate_trajectory(initial_pose = ATC_pick_tool_pose_offset_retract_2, final_pose = initial_pose_corrected, step_pos_min = 0.01, step_deg_min = 2, n_points_max = 20)
        if not success:
                return

        #Change waypoints to the wrist's pose
        approach_leave_waypoints_wrist = []
        for waypoint in approach_leave_waypoints:
                approach_leave_waypoints_wrist.append(self.correctPose(waypoint, arm_side))
        approach_leave_waypoints2_wrist = []
        for waypoint in approach_leave_waypoints2:
                approach_leave_waypoints2_wrist.append(self.correctPose(waypoint, arm_side))
        leave_tool_waypoints_wrist = []
        for waypoint in leave_tool_waypoints:
                leave_tool_waypoints_wrist.append(self.correctPose(waypoint, arm_side))
        retract_leave_waypoints_wrist = []
        for waypoint in retract_leave_waypoints:
                retract_leave_waypoints_wrist.append(self.correctPose(waypoint, arm_side))
        retract_initial_waypoints_wrist = []
        for waypoint in retract_initial_waypoints:
                retract_initial_waypoints_wrist.append(self.correctPose(waypoint, arm_side))

        approach_pick_waypoints_wrist = []
        for waypoint in approach_pick_waypoints:
                approach_pick_waypoints_wrist.append(self.correctPose(waypoint, arm_side))
        approach_pick_waypoints2_wrist = []
        for waypoint in approach_pick_waypoints2:
                approach_pick_waypoints2_wrist.append(self.correctPose(waypoint, arm_side))
        pick_tool_waypoints_wrist = []
        for waypoint in pick_tool_waypoints:
                pick_tool_waypoints_wrist.append(self.correctPose(waypoint, arm_side))
        retract_pick_waypoints_wrist = []
        for waypoint in retract_pick_waypoints:
                retract_pick_waypoints_wrist.append(self.correctPose(waypoint, arm_side))
        retract_pick_waypoints2_wrist = []
        for waypoint in retract_pick_waypoints2:
                retract_pick_waypoints2_wrist.append(self.correctPose(waypoint, arm_side))
        final_waypoints_wrist = []
        for waypoint in final_waypoints:
                final_waypoints_wrist.append(self.correctPose(waypoint, arm_side))

        #Execute
        print("Approach to ATC station")
        #plan, success = compute_cartesian_path_velocity_control([approach_leave_waypoints_wrist], [20], arm_side = arm_side, max_linear_accel = 100.0)
        plan, fraction = change_arm.compute_cartesian_path(approach_leave_waypoints_wrist, 0.01, 0.0)
        change_arm.execute(plan, wait=True)
        rospy.sleep(0.5)

        print("Approach to ATC station 2")
        plan, success = compute_cartesian_path_velocity_control([approach_leave_waypoints2_wrist], [25], arm_side = arm_side, max_linear_accel = 50.0)
        if not success:
                return current_tool, False
        #plan, fraction = change_arm.compute_cartesian_path(approach_leave_waypoints2_wrist, 0.01, 0.0)
        change_arm.execute(plan, wait=True)
        rospy.sleep(2)

        print("Insert tool")
        plan, success = compute_cartesian_path_velocity_control([leave_tool_waypoints_wrist], [10], arm_side = arm_side, max_linear_accel = 30.0)
        if not success:
                return current_tool, False
        #plan, fraction = change_arm.compute_cartesian_path(leave_tool_waypoints_wrist, 0.01, 0.0)
        change_arm.execute(plan, wait=True)
        rospy.sleep(2)

        print("Detaching gripper")
        if real_robot and not stop_mov:
                rospy.sleep(0.5)
                pneumatics_service_name = '/write_single_io'
                rospy.wait_for_service(pneumatics_service_name)
                my_service = rospy.ServiceProxy(pneumatics_service_name, WriteSingleIO)
                servReq = WriteSingleIORequest()
                servReq.address = 10026
                servReq.value = 1
                servResult = my_service(servReq)
                print(servResult.success)
        scene.remove_attached_object(eef_link_arm, name=current_tool)
        self.wait_update_object(current_tool, EE_is_attached=False, EE_is_known=True)
        rospy.sleep(0.5)
        sceneReq = PlanningScene()
        sceneReq.allowed_collision_matrix = original_acm
        sceneReq.is_diff = True
        sceneResponse = update_scene_service(sceneReq) #Still allow collision between the ATC and the tool (adjacent)
        rospy.sleep(2)

        print("Retracting arm")
        plan, success = compute_cartesian_path_velocity_control([retract_leave_waypoints_wrist], [15], arm_side = arm_side, max_linear_accel = 50.0)
        if not success:
                return current_tool, False
        #plan, fraction = change_arm.compute_cartesian_path(retract_leave_waypoints_wrist, 0.01, 0.0)
        change_arm.execute(plan, wait=True)
        rospy.sleep(0.5)

        print("Initial position")
        plan, fraction = change_arm.compute_cartesian_path(retract_initial_waypoints_wrist, 0.01, 0.0)
        change_arm.execute(plan, wait=True)
        rospy.sleep(0.5)

        print("Approach pick tool")
        plan, fraction = change_arm.compute_cartesian_path(approach_pick_waypoints_wrist, 0.01, 0.0)
        change_arm.execute(plan, wait=True)
        rospy.sleep(0.5)

        if new_tool=="EEF_taping_gun" and arm_side == "left":
                arm_left.set_named_target("arm_left_ATC_gun_offset")
                arm_left.go(wait=True)
                rospy.sleep(0.5)

                print("Pick tool")
                waypoints_ATC1 = []
                init_pose = change_arm.get_current_pose().pose
                waypoints_ATC1.append(init_pose)
                new_pose = copy.deepcopy(init_pose)
                new_pose.position.x += 0.0675
                waypoints_ATC1.append(new_pose)
                plan, success = compute_cartesian_path_velocity_control([waypoints_ATC1], [10], arm_side=arm_side)
                if success:
                        execute_plan_async(route_group, plan)
                rospy.sleep(2)

        else:
                print("Approach pick tool 2")
                plan, success = compute_cartesian_path_velocity_control([approach_pick_waypoints2_wrist], [25], arm_side = arm_side, max_linear_accel = 50.0)
                if not success:
                        return current_tool, False
                #plan, fraction = change_arm.compute_cartesian_path(approach_pick_waypoints2_wrist, 0.01, 0.0)
                change_arm.execute(plan, wait=True)
                rospy.sleep(0.5)
                sceneReq = PlanningScene()
                sceneReq.allowed_collision_matrix = new_acm
                sceneReq.is_diff = True
                sceneResponse = update_scene_service(sceneReq) #Allow collision between ATC and the new_tool before approaching (adjacent)
                rospy.sleep(0.5)

                print("Pick tool")
                plan, success = compute_cartesian_path_velocity_control([pick_tool_waypoints_wrist], [15], arm_side = arm_side, max_linear_accel = 50.0)
                if not success:
                        return current_tool, False
                #plan, fraction = change_arm.compute_cartesian_path(pick_tool_waypoints_wrist, 0.01, 0.0)
                change_arm.execute(plan, wait=True)
                rospy.sleep(2)

        print("Attaching gripper")
        if real_robot and not stop_mov:
                rospy.sleep(0.5)
                pneumatics_service_name = '/write_single_io'
                rospy.wait_for_service(pneumatics_service_name)
                my_service = rospy.ServiceProxy(pneumatics_service_name, WriteSingleIO)
                servReq = WriteSingleIORequest()
                servReq.address = 10026
                servReq.value = 0
                servResult = my_service(servReq)
                print(servResult.success)
        scene.attach_mesh(eef_link_arm, new_tool, touch_links=touch_links_arm)
        self.wait_update_object(new_tool, EE_is_attached=True, EE_is_known=False)
        sceneReq = PlanningScene()
        sceneReq.allowed_collision_matrix = new_acm
        sceneReq.is_diff = True
        sceneResponse = update_scene_service(sceneReq) #Update ACM after attaching the new tool
        self.ACM = new_acm
        rospy.sleep(3)

        if new_tool=="EEF_taping_gun" and arm_side == "left":
                print("Retracting arm")
                waypoints_ATC2 = []
                init_pose = change_arm.get_current_pose().pose
                waypoints_ATC2.append(init_pose)
                new_pose = copy.deepcopy(init_pose)
                new_pose.position.z += 0.07
                waypoints_ATC2.append(new_pose)
                plan, success = compute_cartesian_path_velocity_control([waypoints_ATC2], [10], arm_side=arm_side)
                if success:
                        execute_plan_async(route_group, plan)
                rospy.sleep(0.5)

                print("Retracting arm 2")
                waypoints_ATC3 = []
                init_pose = change_arm.get_current_pose().pose
                waypoints_ATC3.append(init_pose)
                waypoints_ATC3.append(self.correctPose(ATC_pick_tool_pose_offset_retract_2, arm_side))
                plan, success = compute_cartesian_path_velocity_control([waypoints_ATC3], [25], arm_side=arm_side)
                if success:
                        execute_plan_async(route_group, plan)
                rospy.sleep(0.5)
        else:
                print("Retracting arm")
                plan, success = compute_cartesian_path_velocity_control([retract_pick_waypoints_wrist], [15], arm_side = arm_side, max_linear_accel = 50.0)
                if not success:
                        return new_tool, False
                #plan, fraction = change_arm.compute_cartesian_path(retract_pick_waypoints_wrist, 0.01, 0.0)
                change_arm.execute(plan, wait=True)
                rospy.sleep(0.5)

                print("Retracting arm 2")
                plan, success = compute_cartesian_path_velocity_control([retract_pick_waypoints2_wrist], [25], arm_side = arm_side, max_linear_accel = 50.0)
                if not success:
                        return new_tool, False
                #plan, fraction = change_arm.compute_cartesian_path(retract_pick_waypoints2_wrist, 0.01, 0.0)
                change_arm.execute(plan, wait=True)
                rospy.sleep(0.5)

        print("Coming back to initial ATC position")
        plan, fraction = change_arm.compute_cartesian_path(final_waypoints_wrist, 0.01, 0.0)
        change_arm.execute(plan, wait=True)
        rospy.sleep(0.5)

        print("Moving robot to final position, ready to rotate torso")
        torso.set_named_target("torso_combs")
        torso.go(wait=True)
        rospy.sleep(0.5)
        arms.set_named_target("arms_platform_5")
        arms.go(wait=True)
        rospy.sleep(0.5)

        #Update the tool name
        if arm_side == "left":
                self.EEF_left = new_tool
        else:
                self.EEF_right = new_tool

        return new_tool, True


    def correctPose(self, target_pose, arm_side, rotate = False, ATC_sign = -1, routing_app = False, route_arm = True, picking_app = False, secondary_frame = False):
        """
        Corrects a target pose. Moveit plans the movement to the last link of the move_group, that is in the wrist. This function corrects the target pose so
        The action frame of the EEF is the one that moves to the desired target pose.
        - target_pose: Target pose for the EEF action frame [Pose]
        - arm_side: Arm side in which to change the tool ["left" or "right"]
        - rotate: True to rotate the pose 180 degrees in the EEF X axis. Useful to correct the Z axis direction of the EEF [bool]
        - ATC_sign: Determines the direction of the EEF base frame angle and distance difference with the arm wrist frame [1 or -1]
        - Secondary_frame: Uses the secondary EEF action frame (in the case of the grippers, the nail)
        """
        target_frame = pose_to_frame(target_pose)

        #Transform from target_pose frame to EEF action frame
        if rotate:
                target_frame.M.DoRotX(3.14) #Z axis pointing inside the tool

        if (routing_app and ((arm_side == "left" and route_arm) or (arm_side == "right" and route_arm))) or picking_app:
                target_frame.M.DoRotZ(math.pi)

        #Transform from EEF action frame to EEF base frame
        if arm_side == "right":
                ATC_dist = self.right_ATC_dist
                ATC_angle = self.right_ATC_angle
                if self.EEF_right != "" and self.EEF_right != "None":
                        if not secondary_frame:
                                frame_world_EEF_base = target_frame * get_inverse_frame(self.EEF_dict[self.EEF_right].EE_end_frame)
                        else:
                                frame_world_EEF_base = target_frame * get_inverse_frame(self.EEF_dict[self.EEF_right].EE_end_frame2)
                else:
                        frame_world_EEF_base = copy.deepcopy(target_frame)
        else:
                ATC_dist = self.left_ATC_dist
                ATC_angle = self.left_ATC_angle
                if self.EEF_left != "" and self.EEF_left != "None":
                        if not secondary_frame:
                                frame_world_EEF_base = target_frame * get_inverse_frame(self.EEF_dict[self.EEF_left].EE_end_frame)
                        else:
                                frame_world_EEF_base = target_frame * get_inverse_frame(self.EEF_dict[self.EEF_left].EE_end_frame2)                                
                else:
                        frame_world_EEF_base = copy.deepcopy(target_frame)

        # if picking_app: #and maybe depend on which arm too
        #         frame_world_EEF_base.M.DoRotZ(math.pi)

        #Transform from EEF base frame to arm wrist
        frame_base_wrist = PyKDL.Frame()
        frame_base_wrist.p = PyKDL.Vector(0, 0, ATC_sign*ATC_dist) #Adds the offset from the tool changer to the wrist
        frame_world_wrist = frame_world_EEF_base * frame_base_wrist
        if rotate:
                frame_world_wrist.M.DoRotZ(ATC_sign*ATC_angle) #Z axis difference between the tool changer and the arm wrist
        else:
                frame_world_wrist.M.DoRotZ(-ATC_sign*ATC_angle)

        pose_world_wrist = frame_to_pose(frame_world_wrist)
        if rotate:
                pose_world_wrist.position.z += 1.2

        return pose_world_wrist 


    def antiCorrectPose(self, cell_wrist_pose, arm_side, rotate = True, ATC_sign = -1, routing_app = False, route_arm = True):
        cell_wrist_frame = pose_to_frame(cell_wrist_pose)
        cell_wrist_pose_from_cell = self.correctPose(cell_wrist_pose, arm_side, rotate = rotate, ATC_sign = ATC_sign, routing_app = routing_app, route_arm = route_arm) #Trick, this frame doesn't mean anything
        cell_wrist_frame_from_cell = pose_to_frame(cell_wrist_pose_from_cell)
        guide_wrist_frame = get_inverse_frame(cell_wrist_frame) * cell_wrist_frame_from_cell
        cell_guide_frame = cell_wrist_frame * get_inverse_frame(guide_wrist_frame)
        cell_guide_pose = frame_to_pose(cell_guide_frame)
        if rotate:
                cell_guide_pose.position.z -= 2.4 #1.2*2

        return cell_guide_pose


def frame_to_pose(frame):
        """
        Convert PyKDL.Frame into a Pose
        """
        pose_result = Pose()
        pose_result.position.x = frame.p[0] 
        pose_result.position.y = frame.p[1] 
        pose_result.position.z = frame.p[2] 
        ang = frame.M.GetQuaternion() 
        pose_result.orientation.x = ang[0] 
        pose_result.orientation.y = ang[1] 
        pose_result.orientation.z = ang[2] 
        pose_result.orientation.w = ang[3]
        return pose_result


def pose_to_frame(pose):
        """
        Converts a Pose into a PyKDL.Frame
        """
        frame_result = PyKDL.Frame() 
        frame_result.p = PyKDL.Vector(pose.position.x, pose.position.y, pose.position.z) 
        frame_result.M = PyKDL.Rotation.Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        return frame_result


def listener_to_frame(trans, rot):
        frame_result = PyKDL.Frame() 
        frame_result.p = PyKDL.Vector(trans[0], trans[1], trans[2]) 
        frame_result.M = PyKDL.Rotation.Quaternion(rot[0], rot[1], rot[2], rot[3])
        return frame_result


def get_transpose_rot(rot_original):
        """
        Retrieves the transpose of a rotation matrix [PyKDL.Rotation]
        """
        rot_trans = PyKDL.Rotation(rot_original[0,0], rot_original[1,0], rot_original[2,0], rot_original[0,1], rot_original[1,1], rot_original[2,1], rot_original[0,2], rot_original[1,2], rot_original[2,2])
        return rot_trans


def get_inverse_frame(frame_original):
        """
        Retrieves the inverse of a frame [PyKDL.Frame]
        """
        frame_inv = PyKDL.Frame() 
        x = -(frame_original.p[0]*frame_original.M[0,0] + frame_original.p[1]*frame_original.M[1,0] + frame_original.p[2]*frame_original.M[2,0])
        y = -(frame_original.p[0]*frame_original.M[0,1] + frame_original.p[1]*frame_original.M[1,1] + frame_original.p[2]*frame_original.M[2,1])
        z = -(frame_original.p[0]*frame_original.M[0,2] + frame_original.p[1]*frame_original.M[1,2] + frame_original.p[2]*frame_original.M[2,2])
        frame_inv.p = PyKDL.Vector(x,y,z) 
        frame_inv.M = get_transpose_rot(frame_original.M)
        return frame_inv


def compute_distance(pose1, pose2):
    """
    Retrieves the linear distance [mm] between two poses.
    """
    dist = math.sqrt((pose1.position.x - pose2.position.x)**2 + (pose1.position.y - pose2.position.y)**2 + (pose1.position.z - pose2.position.z)**2)
    return dist
   

def compute_distance_xy(pose1, pose2):
    """
    Retrieves the linear distance [mm] between two poses in xy.
    """
    dist = math.sqrt((pose1.position.x - pose2.position.x)**2 + (pose1.position.y - pose2.position.y)**2)
    print(dist)
    return dist


def compute_angle_distance(pose1, pose2):
    """
    Retrieves the angular [rad] distance between two poses.
    """
    frame1 = pose_to_frame(pose1)
    frame2 = pose_to_frame(pose2)
    frame12 = frame1.Inverse() * frame2
    rot = abs(frame12.M.GetRotAngle()[0])
    return rot


def compute_lin_or_ang_distance(pose1, pose2, linear = True):
    """
    Retrieves the linear (linear = True) distance [mm] or angular (linear = False) [rad] distance between two poses.
    """
    if linear:
        return compute_distance(pose1, pose2)
    else:
        return compute_angle_distance(pose1, pose2)


def get_shifted_pose(origin_pose, shift):
        """
        Retrives a shifted pose
        - origin_pose: Original pose [Pose]
        - shift: List of shifts and rotations in all the angles ([x displacement, y displacement, z displacement, x rotation, y rotation, z rotation]). Dist in m and angles in rad
        """
        tf_origin = pose_to_frame(origin_pose)   
        tf_shift = PyKDL.Frame() 
        tf_shift.p = PyKDL.Vector(shift[0], shift[1], shift[2]) 
        tf_shift.M.DoRotX(shift[3]) 
        tf_shift.M.DoRotY(shift[4]) 
        tf_shift.M.DoRotZ(shift[5])
        tf_result = tf_origin * tf_shift

        pose_result = frame_to_pose(tf_result)
        return pose_result


def degree_difference(R1, R2):
        """
        Retrieves the angle difference between two PyKDL.Rotation
        """
        R_1_2 = get_transpose_rot(R1) * R2
        rad_dif = R_1_2.GetRPY()
        deg_dif = [element * (180.0/math.pi) for element in rad_dif]
        return deg_dif, rad_dif


def get_axis(pose1, pose2):
        """
        pose1: final pose
        pose2: initial pose
        """
        dist = compute_distance(pose1, pose2)
        axis = [(pose1.position.x - pose2.position.x)/dist, (pose1.position.y - pose2.position.y)/dist, (pose1.position.z - pose2.position.z)/dist]
        return axis


def get_axis_from_RM(RM, axis_name):
        axis = []
        if axis_name == "R":
                col = 0
        elif axis_name == "P":
                col = 1
        elif axis_name == "Y":
                col = 2
        axis = [RM[0, col], RM[1, col], RM[2, col]]
        return axis

def get_quaternion_in_Z(angle):
        """
        angle in rad
        """
        qw = math.cos(angle/2)
        qz = math.sin(angle/2)
        pose_quat = Pose()
        pose_quat.orientation.x = 0.0
        pose_quat.orientation.y = 0.0
        pose_quat.orientation.z = qz 
        pose_quat.orientation.w = qw
        return pose_quat


def cross_product_vectors(v1, v2):
        #print("\nv1:")
        #print(v1)
        #print("\nv2:")
        #print(v2)
        return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2]


def get_ort_axis(v1, v2):
        v3 = [0,0,0]
        v3_0 = [False, False, False]
        # print(v1)
        # print(v2)

        index = 0
        for i in v1:
                if abs(i)>1:
                        v1[index] = i/abs(i)
                index+=1
        index = 0
        for i in v2:
                if abs(i)>1:
                        v2[index] = i/abs(i)
                index+=1

        if v1[0]**2 + v2[0]**2 > 1:
                sign = v2[0]/abs(v2[0])
                v2[0] = sign * math.sqrt(1 - v1[0]**2)
                v3_0[0] = True
                if v2[0]**2 + v2[1]**2 > 1:
                        sign = v2[1]/abs(v2[1])
                        v2[1] = sign * math.sqrt(1 - v2[0]**2)
                        v2[2] = 0.0
        if v1[1]**2 + v2[1]**2 > 1:
                sign = v2[1]/abs(v2[1])
                v2[1] = sign * math.sqrt(1 - v1[1]**2)
                v3_0[1] = True
                if v2[0]**2 + v2[1]**2 > 1:
                        sign = v2[1]/abs(v2[1])
                        v2[1] = sign * math.sqrt(1 - v2[0]**2)
                        v2[2] = 0.0
        if v1[2]**2 + v2[2]**2 > 1:
                sign = v2[2]/abs(v2[2])
                v2[2] = sign * math.sqrt(1 - v1[2]**2)
                v3_0[2] = True
                if v2[0]**2 + v2[2]**2 > 1:
                        sign = v2[0]/abs(v2[0])
                        v2[0] = sign * math.sqrt(1 - v2[2]**2)
                        v2[1] = 0.0
        #print(v1)
        #print(v2)
        if not v3_0[0]:
                #v3[0] = math.sqrt(1 - v1[0]**2 - v2[0]**2)
                v3[0] = (v1[1]*v2[2] - v2[1]*v1[2])
        if not v3_0[1]:
                #v3[1] = math.sqrt(1 - v1[1]**2 - v2[1]**2)
                v3[1] = -(v1[0]*v2[2] - v2[0]*v1[2])
        if not v3_0[2]:
                #v3[2] = math.sqrt(1 - v1[2]**2 - v2[2]**2)
                v3[2] = (v1[0]*v2[1] - v2[0]*v1[1])
        return v1, v2, v3


def get_middle_pose(pose1, pose2):
        frame3 = PyKDL.Frame()
        if pose1.position.x > pose2.position.x:
                pose1_new = pose1
                pose2_new = pose2
        else:
                pose1_new = copy.deepcopy(pose2)
                pose2_new = copy.deepcopy(pose1)
        frame3.p = PyKDL.Vector((pose1_new.position.x + pose2_new.position.x)/2, (pose1_new.position.y + pose2_new.position.y)/2, (pose1_new.position.z + pose2_new.position.z)/2)
        rot_angle=math.atan2((pose1_new.position.y - pose2_new.position.y), (pose1_new.position.x - pose2_new.position.x))
        frame3.M = frame3.M.RotZ(rot_angle)        
        return frame_to_pose(frame3)


def compute_distance_relative(pose1, pose2, axis):
        dist_vector = [(pose1.position.x - pose2.position.x), (pose1.position.y - pose2.position.y), (pose1.position.z - pose2.position.z)]
        dist_in_axis = axis[0] * dist_vector[0] + axis[1] * dist_vector[1] + axis[2] * dist_vector[2]
        return dist_in_axis


def interpolate_trajectory(initial_pose, final_pose, step_pos_min, step_deg_min, n_points_max):
        """
        Creates several waypoints between two poses:
        - initial_pose: Initial pose [Pose]
        - final_pose: Final pose [Pose]
        - step_pos_min: Minimum distance between consecutive intermediate poses [m]
        - step_deg_min: Minimum angle between consecutive intermediate poses [angle]
        - n_points_max: Maximum number of waypoints of the interpolated path [int]
        """
        waypoints = []
        if initial_pose == final_pose:
                print("Cannot interpolate points, it is the same pose")
                return False, waypoints

        n_points = n_points_max
        step_pos_min = float(step_pos_min)
        step_deg_min = float(step_deg_min)
        pos_dif = compute_distance(initial_pose, final_pose)
        deg_dif, rad_dif = degree_difference(pose_to_frame(initial_pose).M, pose_to_frame(final_pose).M)
        n_points_list = []
        n_points_list.append(pos_dif/step_pos_min)
        n_points_list.append(abs(deg_dif[0])/step_deg_min)
        n_points_list.append(abs(deg_dif[1])/step_deg_min)
        n_points_list.append(abs(deg_dif[2])/step_deg_min)
        if max(n_points_list) < 20:
                n_points = int(max(n_points_list))
        
        waypoints.append(initial_pose)
        for point in range(n_points):
            if point > 0:
                x = initial_pose.position.x + ((final_pose.position.x - initial_pose.position.x)*float(point)/float(n_points))
                y = initial_pose.position.y + ((final_pose.position.y - initial_pose.position.y)*float(point)/float(n_points))
                z = initial_pose.position.z + ((final_pose.position.z - initial_pose.position.z)*float(point)/float(n_points))
                rotation = pose_to_frame(initial_pose).M
                rotation.DoRotX((rad_dif[0])*float(point)/float(n_points))
                rotation.DoRotY((rad_dif[1])*float(point)/float(n_points))
                rotation.DoRotZ((rad_dif[2])*float(point)/float(n_points))
                new_frame = PyKDL.Frame() 
                new_frame.p = PyKDL.Vector(x,y,z) 
                new_frame.M = rotation
                waypoints.append(frame_to_pose(new_frame))
        waypoints.append(final_pose)

        return True, waypoints             


def adjust_plan_speed(traj_poses, EE_speed_aux, v_change, traj_mov, max_accel, all_plans, linear = True):
    """
    Recalculate the times of an EEF trajectory to follow some target speeds.
    This function is just called internally by another function.
    - traj_poses: List of lists of EEF poses. Each sublist corresponds to the poses of a different target speed plan
    - EE_speed_aux: List of target speeds (mm/s or deg/s)
    - v_change: Dictionary with the distance needed to accelerate between succesive speed sections (mm/s or deg/s)
    - traj_mov: List of total distance/angle travelled in each speed section (mm or deg)
    - max_accel: Maximum EEF acceleration (mm/s^2 or rad/s^2)
    - all_plans: All the plans of the different speed sections without EEF speed control
    - linear: True for linear motion, False for angular motion
    """
    #print("SPEED AUX CHECK")
    #print(EE_speed_aux)
    #Variables initialization
    thres = 0.05
    if not linear:
        thres *= (0.7*(math.pi/180))
    corrected_traj = []
    zero_Jvel = []
    for i in range(len(all_plans[0].joint_trajectory.points[0].positions)):
        zero_Jvel.append(0.0)
    corrected_traj.append({'pose': traj_poses[0][0], 'state': all_plans[0].joint_trajectory.points[0].positions, 'EE_speed': 0, 'EE_accel': 0, 'time': 0, 'Jspeed': copy.deepcopy(zero_Jvel), 'Jaccel': copy.deepcopy(zero_Jvel)})

    success = True
    i=0
    s_i = 1
    v_chg_i = 0
    init_speed_change = False
    final_speed_change = False
    for plan_poses in traj_poses:
            if traj_mov[i] < thres: #No movement so the plan is skipped
                j=-1
                for pose in plan_poses:
                    j+=1
                    if pose == corrected_traj[-1]['pose']:
                            continue
                    corrected_traj.append({'pose': pose, 'state': all_plans[i].joint_trajectory.points[j].positions, 'EE_speed': 0, 'EE_accel': 0, 'time': corrected_traj[-1]['time'], 'Jspeed': copy.deepcopy(zero_Jvel), 'Jaccel': copy.deepcopy(zero_Jvel)})
                continue

            init_speed_change = EE_speed_aux[s_i] > EE_speed_aux[s_i-1] #Transition at the beginning if the previous velocity was lower
            final_speed_change = EE_speed_aux[s_i] > EE_speed_aux[s_i+1] #Transition at the end if the next velocity is higher
            j=-1
            x_plan = 0
            init_transition_indexes = []
            first_final = True
            for pose in plan_poses:
                    j+=1
                    if pose == corrected_traj[-1]['pose']: #Repeated point, skipped
                            continue
                    
                    if compute_lin_or_ang_distance(pose, corrected_traj[-1]['pose'], linear) < thres and not init_speed_change:
                            corrected_traj.append({'pose': corrected_traj[-1]['pose'], 'state': all_plans[i].joint_trajectory.points[j].positions, 'EE_speed': 0, 'EE_accel': 0, 'time': corrected_traj[-1]['time'], 'Jspeed': copy.deepcopy(zero_Jvel), 'Jaccel': copy.deepcopy(zero_Jvel)})
                            continue
                    
                    #Acceleration/decelarion at the beginning of the current speed section
                    if init_speed_change:
                            if final_speed_change:
                                if (v_change[v_chg_i]['x_min_req'] + v_change[v_chg_i+1]['x_min_req']) > traj_mov[i]: #It is not possible to reach the target speed and decelerate on time
                                    t2= (-2*EE_speed_aux[s_i+1] + math.sqrt(2*(EE_speed_aux[s_i+1]**2 + EE_speed_aux[s_i-1]**2 + 2*max_accel*traj_mov[i])))/(2*max_accel)
                                    v_change[v_chg_i+1]['x_min_req'] = (EE_speed_aux[s_i+1] + max_accel*t2)*t2 - (0.5 * max_accel * t2**2)
                                    v_change[v_chg_i]['x_min_req'] = traj_mov[i] - v_change[v_chg_i+1]['x_min_req']
                            x_plan += compute_lin_or_ang_distance(pose, corrected_traj[-1]['pose'], linear) #Distance travelled until the moment
                            #Adds the new configuration with empty speed, accel and time, which will be calculated later
                            corrected_traj.append({'pose': pose, 'state': all_plans[i].joint_trajectory.points[j].positions, 'EE_speed': 0, 'EE_accel': 0, 'time': 0, 'Jspeed': copy.deepcopy(zero_Jvel), 'Jaccel': copy.deepcopy(zero_Jvel)})
                            init_transition_indexes.append(len(corrected_traj)-1)

                            #When the distance travelled until the moment is higher than the required distance for the initial acceleration, the speed, acceleration and time for all these points is calculated
                            if (x_plan + compute_lin_or_ang_distance(pose, plan_poses[min(j+1,len(plan_poses)-1)], linear)) > v_change[v_chg_i]['x_min_req'] or (traj_mov[i] < v_change[v_chg_i]['x_min_req'] and j >= (len(plan_poses)-1)): #Determine the real discrete point where the speed change is completed
                                    speed_diff = EE_speed_aux[s_i] - EE_speed_aux[s_i-1] #Speed difference between consecutive target speed sections
                                    trans_accel = min((2*speed_diff*EE_speed_aux[s_i-1] + speed_diff**2)/(2*x_plan),max_accel) #Acceleration is constant as we are considering an u.a.r.m.
                                    for index in init_transition_indexes:
                                            #Apply the equations of a trapezoidal speed profile to calculate the times during the acceleration period
                                            corrected_traj[index]['EE_accel'] = trans_accel
                                            #Calculate the time difference between consecutive points
                                            t_A = trans_accel/2
                                            t_B = corrected_traj[index-1]['EE_speed']
                                            t_C = -compute_lin_or_ang_distance(corrected_traj[index]['pose'], corrected_traj[index-1]['pose'], linear)
                                            new_time_1 = (-t_B + math.sqrt(t_B**2 - 4*t_A*t_C))/(2*t_A)
                                            new_time_2 = (-t_B - math.sqrt(t_B**2 - 4*t_A*t_C))/(2*t_A)
                                            if new_time_1 < 0 and new_time_2 < 0:
                                                new_time = 0
                                                success = False
                                            elif new_time_1 < 0:
                                                new_time = new_time_2
                                            elif new_time_2 < 0:
                                                new_time = new_time_1
                                            elif new_time_1 < new_time_2:
                                                new_time = new_time_1
                                            else:
                                                new_time = new_time_2
                                            #print(speed_diff)
                                            #print(trans_accel)
                                            #print(new_time)
                                            #print("---")
                                            corrected_traj[index]['time'] = corrected_traj[index-1]['time'] + new_time
                                            corrected_traj[index]['EE_speed'] = corrected_traj[index-1]['EE_speed'] + trans_accel*new_time #Calculate the EEF speed reached in the next point
                                    corrected_traj[-1]['EE_accel'] = 0 #Finish the movement with accel 0, velocity will be constant after the transition
                                    corrected_traj[init_transition_indexes[0]-1]['EE_accel'] = trans_accel #Start the movement accelerating
                                    v_chg_i += 1
                                    init_speed_change = False

                    #Acceleration/decelarion at the end of the current speed section
                    elif final_speed_change:
                            x_plan += compute_lin_or_ang_distance(pose, corrected_traj[-1]['pose'], linear)
                            x_left = traj_mov[i] - x_plan #Remaining distance to travel before the end of the current speed section
                            #When the remaining distance is lower than the minimum distance required for the final acceleration/deceleration, it must start from the prev point
                            if x_left < v_change[v_chg_i]['x_min_req']: 
                                    if first_final:
                                            x_trans = x_left + compute_lin_or_ang_distance(pose, corrected_traj[-1]['pose'], linear) #The prev x
                                            speed_diff = EE_speed_aux[s_i+1] - corrected_traj[-1]['EE_speed']
                                            trans_accel_1 = (2*speed_diff*EE_speed_aux[s_i] + speed_diff**2)/(2*x_trans)
                                            trans_accel_2 = copy.deepcopy(-max_accel)
                                            if abs(trans_accel_1)>abs(trans_accel_2):
                                                trans_accel = trans_accel_2
                                            else:
                                                trans_accel = trans_accel_1
                                            first_final = False
                                            corrected_traj[-1]['EE_accel'] = trans_accel

                                    if speed_diff == 0:
                                        corrected_traj.append({'pose': pose, 'state': all_plans[i].joint_trajectory.points[j].positions, 'EE_speed': 0, 'EE_accel': 0, 'time': corrected_traj[-1]['time'], 'Jspeed': copy.deepcopy(zero_Jvel), 'Jaccel': copy.deepcopy(zero_Jvel)})
                                        continue

                                    #Apply the equations of a trapezoidal speed profile to calculate the times during the acceleration/deceleration period
                                    t_A = trans_accel/2
                                    t_B = corrected_traj[-1]['EE_speed']
                                    t_C = -compute_lin_or_ang_distance(pose, corrected_traj[-1]['pose'], linear)
                                    t_shorter = False
                                    
                                    if (t_B**2 - 4*t_A*t_C) <= 0 or t_shorter:
                                        new_time = (-t_B)/(2*t_A)
                                    else:
                                        new_time_1 = (-t_B + math.sqrt(t_B**2 - 4*t_A*t_C))/(2*t_A)
                                        new_time_2 = (-t_B - math.sqrt(t_B**2 - 4*t_A*t_C))/(2*t_A)
                                        if new_time_1 < 0 and new_time_2 < 0:
                                            new_time = 0
                                            success = False
                                        elif new_time_1 < 0:
                                            new_time = new_time_2
                                        elif new_time_2 < 0:
                                            new_time = new_time_1
                                        elif new_time_1 < new_time_2:
                                            new_time = new_time_1
                                        else:
                                            new_time = new_time_2
                                        if new_time > corrected_traj[-1]['time'] - corrected_traj[-2]['time']:
                                            pass
                                        else:
                                            t_shorter = False

                                    new_total_time = corrected_traj[-1]['time'] + new_time
                                    new_speed = corrected_traj[-1]['EE_speed'] + trans_accel*new_time

                                    corrected_traj.append({'pose': pose, 'state': all_plans[i].joint_trajectory.points[j].positions, 'EE_speed': new_speed, 'EE_accel': trans_accel, 'time': new_total_time, 'Jspeed': copy.deepcopy(zero_Jvel), 'Jaccel': copy.deepcopy(zero_Jvel)})
                                    if (linear and x_left < 0.1) or (not linear and x_left < 0.07*(math.pi/180)): #Last point
                                            corrected_traj[-1]['EE_accel'] = 0    
                                            v_chg_i += 1
                                            final_speed_change = False

                            #When there is still time before the final acceleration/deceleration the speed is kept constant at the target speed and the acceleration is 0                
                            else:
                                    corrected_traj.append({'pose': pose, 'state': all_plans[i].joint_trajectory.points[j].positions, 'EE_speed': EE_speed_aux[s_i], 'EE_accel': 0, 'time': corrected_traj[-1]['time'] + compute_lin_or_ang_distance(pose, corrected_traj[-1]['pose'],linear)/EE_speed_aux[s_i], 'Jspeed': copy.deepcopy(zero_Jvel), 'Jaccel': copy.deepcopy(zero_Jvel)}) 

                    #If there are no target speed changes between sections, or the next target speed is higher than the current, the speed is kept constant until the end of the plan
                    else:
                            corrected_traj.append({'pose': pose, 'state': all_plans[i].joint_trajectory.points[j].positions, 'EE_speed': EE_speed_aux[s_i], 'EE_accel': 0, 'time': corrected_traj[-1]['time'] + compute_lin_or_ang_distance(pose, corrected_traj[-1]['pose'],linear)/EE_speed_aux[s_i], 'Jspeed': copy.deepcopy(zero_Jvel), 'Jaccel': copy.deepcopy(zero_Jvel)}) 
                            #print(corrected_traj[-1]['time'])  
                            #print(EE_speed_aux[s_i])
                            #print(compute_lin_or_ang_distance(pose, corrected_traj[-1]['pose'],linear))
                            #print("----")

            i+=1
            s_i += 1
    return corrected_traj, success


def compute_cartesian_path_velocity_control(waypoints_list, EE_speed, EE_ang_speed = [], arm_side = "left", max_linear_accel = 200.0, max_ang_accel = 140.0, extra_info = False, step = 0.002):
        """
        Function that generates a single-arm motion plan with control on the EEF speed, ready to be executed.
        - waypoints_list: List of lists of waypoints [Poses]. Every sublist has a different EEF target speed associated
        - EE_speed: The list of EEF target linear speeds for each speed section of the trajectory [mm/s]
        - EE_ang_speed: The List of EEF target angular speeds for each speed section of the trajectory [deg/s]
        - max_linear_accel: Maximum linear acceleration [mm/s^2]
        - max_angular_accel: Maximum linear acceleration [deg/s^2]
        - extra_info: Used for the sync policy 3 dual-arm function. Leave it as False otherwise
        - Step: Step used for the compute_cartesian_path function [mm]
        """
        
        success = True
        #Selects the arm to be planned
        if arm_side == "left":
                arm = arm_left
                fkln = ['arm_left_link_7_t'] #Modify with the name of the final link of the left arm
        else:
                arm = arm_right
                fkln = ['arm_right_link_7_t'] #Modify with the name of the final link of the right arm

        EE_ang_speed = []
        if len(EE_ang_speed)==0: #If the angular speed limit is not specified, it considers a limit of 1 mm/s --> 0.7 deg/s
                for s in EE_speed:
                    #print("ENTER IN THE LOOP")
                    EE_ang_speed.append(s*0.7)
        # print("LIN SPEED")
        # print(EE_speed)
        # print("ANG SPEED")
        # print(EE_ang_speed)

        #Convert to rads
        for i in range(len(EE_speed)):
            EE_ang_speed[i]*=(math.pi/180)

        # print("ANG SPEED RADS")
        # print(EE_ang_speed)

        max_ang_accel *= (math.pi/180)

        #Define the speed profile accelerations
        EE_speed_aux = copy.deepcopy(EE_speed)
        EE_speed_aux.insert(0,0)
        EE_speed_aux.append(0)
        # print("SPEED_AUX")
        # print(EE_speed_aux)
        EE_ang_speed_aux = copy.deepcopy(EE_ang_speed)
        EE_ang_speed_aux.insert(0,0)
        EE_ang_speed_aux.append(0)
        # print("ANG SPEED RADS AUX")
        # print(EE_ang_speed_aux)
        v_change = []
        v_change_ang = []

        #Calculates the minimum distance required to accelerate and/or decelerate the EEF between the different speed sections
        for i in range(len(EE_speed_aux)-1):
                max_linear_accel_sign = copy.deepcopy(max_linear_accel)
                if EE_speed_aux[i]>EE_speed_aux[i+1]:
                    max_linear_accel_sign *= -1
                max_ang_accel_sign = copy.deepcopy(max_ang_accel)
                if EE_ang_speed_aux[i]>EE_ang_speed_aux[i+1]:
                    max_ang_accel_sign *= -1
                
                t_req_lin = (EE_speed_aux[i+1]-EE_speed_aux[i])/max_linear_accel_sign
                t_req_ang = (EE_ang_speed_aux[i+1]-EE_ang_speed_aux[i])/max_ang_accel_sign
                change = {'t_req': t_req_lin, 'x_min_req': EE_speed_aux[i]*t_req_lin + (max_linear_accel_sign*(t_req_lin**2))/2}
                change_ang = {'t_req': t_req_ang, 'x_min_req': EE_ang_speed_aux[i]*t_req_ang + (max_ang_accel_sign*(t_req_ang**2))/2}

                v_change.append(change)
                v_change_ang.append(change_ang)

        #Plan the trajectory of every speed section using the compute_cartesian_path function or a custom IK trajectory solver.
        #This plan don't have any control over the speed of the end effector
        all_plans = []
        for traj in waypoints_list:
                (plan, fraction) = arm.compute_cartesian_path(traj, step, 0.0)
                all_plans.append(plan)
                rs = RobotState()
                for j_name in plan.joint_trajectory.joint_names:
                        rs.joint_state.name.append(j_name)
                for state in plan.joint_trajectory.points[-1].positions:
                        rs.joint_state.position.append(state)        
                arm.set_start_state(rs)
        arm.set_start_state_to_current_state()

        #Get all the EEF poses of the generated plan using a forward kinematics solver
        traj_poses = []
        traj_mov_position = []
        traj_mov_angle = []
        rospy.wait_for_service('compute_fk')
        fk_srv = rospy.ServiceProxy('compute_fk', GetPositionFK)
        rs = RobotState()
        for j_name in plan.joint_trajectory.joint_names:
                rs.joint_state.name.append(j_name)
        for plan in all_plans:
                plan_poses = []
                traj_mov_i_position = 0
                traj_mov_i_angle = 0
                for joint_state in plan.joint_trajectory.points:
                        rs.joint_state.position = []
                        for joint in joint_state.positions:
                                rs.joint_state.position.append(joint)
                        header = Header(0,rospy.Time.now(),"torso_base_link") #Modify with the frame_id of the robot
                        header.frame_id = plan.joint_trajectory.header.frame_id
                        plan_pose_meters = fk_srv(header, fkln, rs).pose_stamped[0].pose
                        plan_pose_mm = copy.deepcopy(plan_pose_meters)
                        plan_pose_mm.position.x *= 1000
                        plan_pose_mm.position.y *= 1000
                        plan_pose_mm.position.z *= 1000
                        plan_poses.append(plan_pose_mm)
                        if len(plan_poses) > 1:
                                traj_mov_i_position += compute_distance(plan_poses[-2], plan_poses[-1])
                                traj_mov_i_angle += compute_angle_distance(plan_poses[-2], plan_poses[-1])
                traj_poses.append(plan_poses)
                traj_mov_position.append(traj_mov_i_position) #Total travelled distance in each speed section
                traj_mov_angle.append(traj_mov_i_angle) #Total travelled angle in each speed section

        #Get max joint velocities from the URDF robot description
        robot_desc = rospy.get_param('robot_description')
        root = ET.fromstring(robot_desc)
                
        vel_limit = {}
        for child in root: 
                if child.tag == "joint":
                        j_name = child.get("name")
                        if child.get("type") == "revolute":
                                for joint_attrib in child:
                                        if joint_attrib.tag == "limit":
                                                vel_limit[j_name] = float(joint_attrib.get("velocity"))*0.9 #The limit is set at a 90% of the URDF limit

        #Recalculate all the times of the trajectory according to the specified target speed profiles. 
        #The calculation is done both for the linear and the angular movements
        corrected_traj, success_lin = adjust_plan_speed(traj_poses, EE_speed_aux, v_change, traj_mov_position, max_linear_accel, all_plans, linear = True)
        # print("EEF ANG AUX")
        # print(EE_ang_speed_aux)
        corrected_traj_ang, success_ang = adjust_plan_speed(traj_poses, EE_ang_speed_aux, v_change_ang, traj_mov_angle, max_ang_accel, all_plans, linear = False)
        if not success_lin or not success_ang:
            success = False

        #Merges the linear and angular constraint plans by selecting the larger times, to not exceed any of the limits.
        first_accel = True
        full_corrected_traj = copy.deepcopy(corrected_traj)
        for i in range(len(corrected_traj)-1):
            if (corrected_traj_ang[i+1]['time'] - corrected_traj_ang[i]['time']) > (corrected_traj[i+1]['time'] - corrected_traj[i]['time']):
                full_corrected_traj[i+1]['time'] = full_corrected_traj[i]['time'] + (corrected_traj_ang[i+1]['time'] - corrected_traj_ang[i]['time'])
            else:
                full_corrected_traj[i+1]['time'] = full_corrected_traj[i]['time'] + (corrected_traj[i+1]['time'] - corrected_traj[i]['time'])
            if extra_info:
                if corrected_traj[i+1]['EE_accel'] == 0 and corrected_traj_ang[i+1]['EE_accel'] == 0 and first_accel:
                        first_accel = False
                        t_accel = full_corrected_traj[i+1]['time']
                if corrected_traj[i+1]['EE_accel'] == 0 and corrected_traj_ang[i+1]['EE_accel'] == 0 and ((i+1)<(len(corrected_traj)-1)):
                        t_dec = full_corrected_traj[i+1]['time']

        zero_Jvel = []
        for i in range(len(all_plans[0].joint_trajectory.points[0].positions)):
                zero_Jvel.append(0.0)

        #Recalculate the times of the trajectory when any of the joint velocity limits are exceeded (> 90% of limit)
        update_time = False
        full_corrected_traj_with_limits = copy.deepcopy(full_corrected_traj)
        # for i in range(len(full_corrected_traj)-1):
        #         time_diff = full_corrected_traj[i+1]['time']-full_corrected_traj[i]['time']
        #         updated_new_times = []
        #         new_Jspeed = 0
        #         angle_diff = 0
        #         new_Jaccel=0
        #         new_time=0
        #         for j in range(len(full_corrected_traj[i]['state'])):
        #                 if time_diff != 0:
        #                     angle_diff = full_corrected_traj[i+1]['state'][j]-full_corrected_traj[i]['state'][j]
        #                     new_Jaccel = (angle_diff-(full_corrected_traj_with_limits[i]['Jspeed'][j]*time_diff))*(2/(time_diff**2))
        #                     new_Jspeed = full_corrected_traj_with_limits[i]['Jspeed'][j] + (new_Jaccel*time_diff)
        #                     new_time = full_corrected_traj_with_limits[i]['time'] + time_diff

        #                 joint_name = rs.joint_state.name[j]
        #                 if vel_limit[joint_name] < new_Jspeed or time_diff == 0:
        #                     new_Jspeed = vel_limit[joint_name]
        #                     new_Jaccel = ((2*full_corrected_traj_with_limits[i]['Jspeed'][j]*(new_Jspeed - full_corrected_traj_with_limits[i]['Jspeed'][j])) + (new_Jspeed - full_corrected_traj_with_limits[i]['Jspeed'][j])**2)/(2*angle_diff)
        #                     if abs(new_Jaccel) < 0.0001:
        #                         new_time_step = (angle_diff)/new_Jspeed
        #                     else:
        #                         new_time_step = (new_Jspeed - full_corrected_traj_with_limits[i]['Jspeed'][j])/new_Jaccel
        #                     updated_new_times.append(new_time_step)
        #                     update_time = True

        #                 full_corrected_traj_with_limits[i+1]['time'] = new_time
        #                 full_corrected_traj_with_limits[i+1]['Jaccel'][j] = new_Jaccel
        #                 full_corrected_traj_with_limits[i+1]['Jspeed'][j] = new_Jspeed

        #         if update_time: #If any of the limits is exceeded, recalculate times, joint velocities and accelerations
        #             print("Joint limits speed exceeded")
        #             update_time = False
        #             new_time_max = max(updated_new_times)
        #             time_diff = new_time_max
        #             full_corrected_traj_with_limits[i+1]['time'] = full_corrected_traj_with_limits[i]['time'] + time_diff
        #             for j in range(len(full_corrected_traj[i]['state'])):
        #                 angle_diff = full_corrected_traj[i+1]['state'][j]-full_corrected_traj[i]['state'][j]
        #                 new_Jaccel = (angle_diff-(full_corrected_traj_with_limits[i]['Jspeed'][j]*time_diff))*(2/(time_diff**2))
        #                 new_Jspeed = full_corrected_traj_with_limits[i]['Jspeed'][j] + (new_Jaccel*time_diff)
        #                 full_corrected_traj_with_limits[i+1]['Jaccel'][j] = new_Jaccel
        #                 full_corrected_traj_with_limits[i+1]['Jspeed'][j] = new_Jspeed

        #         if extra_info and t_accel == full_corrected_traj[i+1]['time']:
        #             t_accel = full_corrected_traj_with_limits[i+1]['time']
        #         if extra_info and t_dec == full_corrected_traj[i+1]['time']:
        #             t_dec = full_corrected_traj_with_limits[i+1]['time']

        # full_corrected_traj_with_limits[-1]['Jspeed'] = copy.deepcopy(zero_Jvel)

        #Uncomment if we want to visualize all the information of the generated plan
        """
        i=0
        for traj_point in full_corrected_traj_with_limits:
                print(i)
                print("Position: " + str(traj_point['pose'].position.x) + ", " + str(traj_point['pose'].position.y) + ", " + str(traj_point['pose'].position.z))
                print("Velocity: " + str(traj_point['EE_speed']))
                print("Acceleration: " + str (traj_point['EE_accel']))
                print("Time: " + str (traj_point['time']))
                print("Joint angles: " + str (traj_point['state']))
                print("Joint velocities: " + str (traj_point['Jspeed']))
                print("Joint accelerations: " + str (traj_point['Jaccel']))
                print("----------------------")
                i+=1
        """

        #Adjust the generated plan to the RobotTrajectory() msg structure
        new_plan = RobotTrajectory()
        new_plan.joint_trajectory.header.frame_id = "base_link"
        new_plan.joint_trajectory.joint_names = copy.deepcopy(rs.joint_state.name)
        for state in full_corrected_traj_with_limits:
                point = JointTrajectoryPoint()
                point.positions = copy.deepcopy(state['state'])
                point.velocities = copy.deepcopy(state['Jspeed'])
                point.accelerations = copy.deepcopy(state['Jaccel'])
                point.effort = []
                point.time_from_start.secs = int(copy.deepcopy(state['time']))
                point.time_from_start.nsecs = int((copy.deepcopy(state['time']) - int(copy.deepcopy(state['time'])))*1000000000)
                new_plan.joint_trajectory.points.append(point)

        if extra_info:
                return new_plan, success, t_accel, t_dec
        else:
                return new_plan, success         


def merge_plans(planL, planR):
        """
        Merge a left arm plan (planL) and a right arm plan (planR) into a single dual-arm plan, modifying their speed so 
        both arms finish their motion at the same time
        """
        
        #speed_limit = 1
        length_L = len(planL.joint_trajectory.points)
        duration_L = float(planL.joint_trajectory.points[-1].time_from_start.secs) + 0.000000001 * float(planL.joint_trajectory.points[-1].time_from_start.nsecs)
        length_R = len(planR.joint_trajectory.points)
        duration_R = float(planR.joint_trajectory.points[-1].time_from_start.secs) + 0.000000001 * float(planR.joint_trajectory.points[-1].time_from_start.nsecs)
        
        #Intialize variables
        joints_long = []
        vel_long = []        
        accel_long = []
        times_long = []
        joints_short = []
        vel_short = []        
        accel_short = []
        times_short = []
        joints_L = []
        vel_L = []        
        accel_L = []
        joints_R = []
        vel_R = []        
        accel_R = []
        times = []

        add_points = False
        if length_L > length_R:
            plan_long = planL
            plan_short = planR
            length_long = length_L
            length_short = length_R
            duration_long = duration_L
            duration_short = duration_R
            long_plan_name = "L"
            add_points = True
        elif length_L < length_R:
            plan_long = planR
            plan_short = planL
            length_long = length_R
            length_short = length_L
            duration_long = duration_R
            duration_short = duration_L
            long_plan_name = "R"
            add_points = True
        else: #Both plans have the same number of points, so it is no necessary to add new points (add_points = False)
            plan_long = planR
            plan_short = planL
            duration_long = duration_R
            duration_short = duration_L
            long_plan_name = "R"                

        #Extract the values of each plan
        for point in plan_long.joint_trajectory.points:
            joints_long.append(point.positions)
            vel_long.append(point.velocities)
            accel_long.append(point.accelerations)
            times_long.append(float(point.time_from_start.secs) + 0.000000001 * float(point.time_from_start.nsecs))

        for point in plan_short.joint_trajectory.points:
            joints_short.append(point.positions)
            vel_short.append(point.velocities)
            accel_short.append(point.accelerations)
            times_short.append(float(point.time_from_start.secs) + 0.000000001 * float(point.time_from_start.nsecs))        

        time_interpolation = False
        if duration_long >= duration_short:
            times = times_long[:]
        else:
            times = times_short[:]
            time_interpolation = True
        
        n_low_sep = 0
        n_high_sep = 0
        lower_sep = 0
        higher_sep = 0
        lower_amo = 0
        higher_amo = 0
        n_high_amo = 0
        n_low_amo = 0
        indexes = []

        #If one plan is larger than the other, add new points equally distributed to the shorter plan by linear interpolation.
        #The times used for merging the plans are the ones of the slowest plan.
        if add_points:

            if length_short >= ((length_long/2) - 1):
                separation = length_short/(length_long - length_short + 1)
                amount = 1
                lower_sep = int(separation)
                higher_sep = lower_sep + 1
                n_low_sep = int(((length_long - length_short + 1) * (separation - float(higher_sep)))/(float(lower_sep) - float(higher_sep))-1)
                n_high_sep = int(length_long - length_short - n_low_sep)

                indexes = []
                for i in range(n_high_sep):
                    indexes.append(higher_sep + (i * (higher_sep + 1)))
                if len(indexes) != 0:
                    start_index = indexes[-1]
                else:
                    start_index = -1
                for i in range(n_low_sep):
                    indexes.append(start_index + ((i + 1) * (lower_sep + 1)))                

                for index in indexes:
                    new_pos_zip = zip(joints_short[index-1],joints_short[index])
                    new_pos = [(x + y)/2 for (x, y) in new_pos_zip]
                    joints_short.insert(index, new_pos)
                    new_vel_zip = zip(vel_short[index-1],vel_short[index])
                    new_vel = [(x + y)/2 for (x, y) in new_vel_zip]
                    vel_short.insert(index, new_vel)
                    new_accel_zip = zip(accel_short[index-1],accel_short[index])
                    new_accel = [(x + y)/2 for (x, y) in new_accel_zip]
                    accel_short.insert(index, new_accel)
                    if time_interpolation:
                        new_time = (times[index-1] + times[index])/2
                        times.insert(index, new_time)
                                
            else:
                separation = 1
                amount = (length_long - length_short)/(length_short - 1)
                lower_amo = int(amount)
                higher_amo = lower_amo + 1
                n_high_amo = int((length_long + lower_amo - length_short * (1 + lower_amo))/(higher_amo - lower_amo))
                n_low_amo = (length_short - 1 - n_high_amo)

                indexes = []
                new_pos = []
                new_vel = []
                new_accel = []
                new_time = []
                for i in range(n_high_amo):
                    new_pos_zip = zip(joints_short[i],joints_short[i+1])
                    new_vel_zip = zip(vel_short[i],vel_short[i+1])
                    new_accel_zip = zip(accel_short[i],accel_short[i+1])
                    for j in range(higher_amo):
                        indexes.append(((higher_amo + 1) * i) + j + 1)
                        new_pos.append([((y - x)*(j+1)/(higher_amo+1))+x for (x, y) in new_pos_zip])
                        new_vel.append([((y - x)*(j+1)/(higher_amo+1))+x for (x, y) in new_vel_zip])
                        new_accel.append([((y - x)*(j+1)/(higher_amo+1))+x for (x, y) in new_accel_zip])
                        if time_interpolation:
                            new_time.append(((times_short[i+1] - times_short[i])*(j+1)/(higher_amo+1))+times_short[i])

                try:
                    start_index = indexes[-1]
                except:
                    start_index = -1
                for i in range(n_low_amo):
                    new_pos_zip = zip(joints_short[i+n_high_amo],joints_short[i+1+n_high_amo])
                    new_vel_zip = zip(vel_short[i+n_high_amo],vel_short[i+1+n_high_amo])
                    new_accel_zip = zip(accel_short[i+n_high_amo],accel_short[i+1+n_high_amo])
                    for j in range(lower_amo):
                        indexes.append((start_index + 1) + ((lower_amo + 1) * i) + j + 1)
                        new_pos.append([((y - x)*(j+1)/(lower_amo+1))+x for (x, y) in new_pos_zip])
                        new_vel.append([((y - x)*(j+1)/(lower_amo+1))+x for (x, y) in new_vel_zip])
                        new_accel.append([((y - x)*(j+1)/(lower_amo+1))+x for (x, y) in new_accel_zip])
                        if time_interpolation:
                            new_time.append(((times_short[i+1+n_high_amo] - times_short[i+n_high_amo])*(j+1)/(lower_amo+1))+times_short[i+n_high_amo])

                i = 0
                for index in indexes:
                    joints_short.insert(index, new_pos[i])
                    vel_short.insert(index, new_vel[i])
                    accel_short.insert(index, new_accel[i])
                    if time_interpolation:
                        times.insert(index, new_time[i])
                    i += 1

        if long_plan_name == "L":
            joints_L = joints_long
            vel_L = vel_long        
            accel_L = accel_long
            joints_R = joints_short
            vel_R = vel_short        
            accel_R = accel_short
        else:
            joints_R = joints_long
            vel_R = vel_long        
            accel_R = accel_long
            joints_L = joints_short
            vel_L = vel_short        
            accel_L = accel_short

        #After adding all the points, both plans have the same length and they are ready to be merged in a single plan for the joints of both arms
        merged_plan = RobotTrajectory()
        merged_plan.joint_trajectory.header.frame_id = copy.copy(planL.joint_trajectory.header.frame_id)
        merged_plan.joint_trajectory.joint_names = copy.deepcopy(planL.joint_trajectory.joint_names) + copy.deepcopy(planR.joint_trajectory.joint_names)

        #times = [element/speed_limit for element in times] #*4

        for index in range(len(times)):
            point = JointTrajectoryPoint()
            point.positions = tuple(joints_L[index]) + tuple(joints_R[index])
            point.velocities = tuple(vel_L[index]) + tuple(vel_R[index])
            point.accelerations = tuple(accel_L[index]) + tuple(accel_R[index])
            point.effort = []
            point.time_from_start.secs = int(times[index])
            point.time_from_start.nsecs = int((times[index] - int(times[index]))*1000000000)
            merged_plan.joint_trajectory.points.append(point)

        return merged_plan



def fill_times_arm(values_1, times_1, times_2):
        """
        values_1 are the plan values of an arm at times_1. This function fills the plan with the values at times_2 too.
        This function is just called internally by the merge_plans_keep_speed() function
        """
        values_1_original = copy.deepcopy(values_1)
        for t in times_2:  
                if t not in times_1:
                        if t < max(times_1):
                                time_index = [n for n,i in enumerate(times_1) if i>t ][0]
                                prev_time = times_1[time_index-1]
                                next_time = times_1[time_index]
                                new_J = []
                                new_V = []
                                new_A = []
                                #Uses the trapezoidal profile equations to calculate the values at times_2, from the values in the previous and next times of times_1
                                for i in range(len(values_1_original[prev_time]['J'])):
                                        new_V_i = values_1_original[prev_time]['V'][i] + values_1_original[prev_time]['A'][i] * (t-prev_time)
                                        new_J_i = values_1_original[prev_time]['J'][i] + values_1_original[prev_time]['V'][i] * (t-prev_time) + 0.5*values_1_original[prev_time]['A'][i] * ((t-prev_time)**2)
                                        new_A_i = (2*(values_1_original[next_time]['J'][i] - new_J_i - new_V_i*(next_time-t)))/((next_time-t)**2)
                                        new_J.append(new_J_i)
                                        new_V.append(new_V_i)
                                        new_A.append(new_A_i)
                                values_1[t] = {'J': new_J, 'V': new_V, 'A': new_A}
                        else:
                                values_1[t] = copy.deepcopy(values_1_original[max(times_1)])
                                values_1[t]['V'] = [0]*7
                                values_1[t]['A'] = [0]*7
                sorted_keys = sorted(values_1.keys())
                values_1_sorted = {key:values_1[key] for key in sorted_keys}
        return values_1_sorted


def merge_plans_keep_speed(planL, planR):
        """
        Merge a left arm plan (planL) and a right arm plan (planR) into a single dual-arm plan keeping the speed of each individual arm
        """
        times_L = []
        times_R = []
        values_L = {}
        values_R = {}

        #Extract information from plans
        for point in planL.joint_trajectory.points:
                new_time_L = float(point.time_from_start.secs) + 0.000000001 * float(point.time_from_start.nsecs)
                times_L.append(new_time_L)
                values_L[new_time_L] = {'J': point.positions, 'V': point.velocities, 'A': point.accelerations}
        
        for point in planR.joint_trajectory.points:
                new_time_R = float(point.time_from_start.secs) + 0.000000001 * float(point.time_from_start.nsecs)
                times_R.append(new_time_R)
                values_R[new_time_R] = {'J': point.positions, 'V': point.velocities, 'A': point.accelerations}

        times_both = times_L + times_R
        times_both = list(dict.fromkeys(times_both)) #Remove repeated times
        times_both.sort()

        #For each arm plan, it creates new points for the times of the other arm plan, so it is possible to merge them later
        values_L = fill_times_arm(values_L, times_L, times_R)
        values_R = fill_times_arm(values_R, times_R, times_L)

        #After adding all the points, both plans have the same length and they are ready to be merged in a single plan for the joints of both arms
        merged_plan = RobotTrajectory()
        merged_plan.joint_trajectory.header.frame_id = copy.copy(planL.joint_trajectory.header.frame_id)
        merged_plan.joint_trajectory.joint_names = copy.deepcopy(planL.joint_trajectory.joint_names) + copy.deepcopy(planR.joint_trajectory.joint_names)
        
        for t in times_both:
                point = JointTrajectoryPoint()
                point.positions = tuple(values_L[t]['J']) + tuple(values_R[t]['J'])
                point.velocities = tuple(values_L[t]['V']) + tuple(values_R[t]['V'])
                point.accelerations = tuple(values_L[t]['A']) + tuple(values_R[t]['A'])
                point.effort = []
                point.time_from_start.secs = int(t)
                point.time_from_start.nsecs = int((t - int(t))*1000000000)
                merged_plan.joint_trajectory.points.append(point)

        return merged_plan



def dual_arm_cartesian_plan(waypoints_left, speed_left, waypoints_right, speed_right, ATC1, sync_policy = 1, extra_info=False, rs=RobotState(), rs_l=RobotState(), rs_r=RobotState()):
        """
        This function generates a dual-arm motion plan ready to be executed. Three different synchronization policies can be used:
        1. Both arms finish their motion at the same time. The maximum EEF speed of each arm can be set.
        2. Each arm moves at its own speed.
        3. Both arms reach a sequence of intermediate dual-arm waypoints at the same time. A maximum EEF speed can be set.

        - waypoints_left: List of lists of waypoints [Poses] for the left EEF. Each sublist corresponds to a different target speed. In policy 3 there must be just one sublist.
        - speed_left: List of EEF speeds for each section of the left arm trajectory [mm/s]
        - waypoints_right: List of lists of waypoints [Poses] for the right EEF. Each sublist corresponds to a different target speed. In policy 3 there must be just one sublist.
        - speed_right: List of EEF speeds for each section of the right arm trajectory [mm/s]
        - sync_policy: Synchronization policy used [1, 2 or 3, explained above]
        - extra_info: Leave it as empty. Used for recursive calls of this function
        - rs: robot state. Leave it as empty. Used for recursive calls of this function
        - rs_l: left arm state. Leave it as empty. Used for recursive calls of this function
        - rs_r: right arm state. Leave it as empty. Used for recursive calls of this function
        - ATC1: ATC object of the simulation
        """
        
        #Clear previous targets
        arm_left.clear_pose_targets()
        arm_right.clear_pose_targets()
        arms.clear_pose_targets()

        #Managed by another function
        if sync_policy == 3:
                if (len(waypoints_left[0]) != len(waypoints_right[0])) or (len(speed_left) != len(speed_right)):
                        plan = RobotTrajectory()
                        return plan, False
                else:
                        speed_3 = min([speed_left[0], speed_right[0]])
                        waypoints_3 = []
                        for i in range(len(waypoints_left[0])):
                                wp_i = [waypoints_left[0][i],waypoints_right[0][i]]
                                waypoints_3.append(wp_i)
                        plan, success = dual_arm_intermediate_waypoints_plan(waypoints_3, speed_3, ATC1=ATC1)
                        return plan, success

        #Get the allowed collision matrix (ACM)
        rospy.wait_for_service('/get_planning_scene')
        my_service = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        sceneReq = PlanningSceneComponents(components=128)
        sceneResponse = my_service(sceneReq)
        acm = sceneResponse.scene.allowed_collision_matrix
        acm_correct = copy.deepcopy(acm)

        #Modify ACM to not consider collisions between arms
        rospy.wait_for_service('/apply_planning_scene')
        my_service = rospy.ServiceProxy('/apply_planning_scene', ApplyPlanningScene)
        sceneReq = PlanningScene()
        arms_joint_list = robot.get_joint_names("arms")
        for i in range(len(acm.entry_values)):
                for j in range(len(acm.entry_values)):
                        if i!=j and ((i not in arms_joint_list or "EEF" in i) and (j not in arms_joint_list or "EEF" in j)):
                                acm.entry_values[i].enabled[j] = True #No collisions
                        else:
                                acm.entry_values[i].enabled[j] = False
        sceneReq.allowed_collision_matrix = acm
        sceneReq.is_diff = True
        sceneResponse = my_service(sceneReq)

        if extra_info:
                planR, successR, t_acc_r, t_dec_r = compute_cartesian_path_velocity_control(waypoints_right, speed_right, EE_ang_speed = [], arm_side = "right", extra_info = extra_info)
                planL, successL, t_acc_l, t_dec_l = compute_cartesian_path_velocity_control(waypoints_left, speed_left, EE_ang_speed = [], extra_info = extra_info)
        else:
                planR, successR = compute_cartesian_path_velocity_control(waypoints_right, speed_right, EE_ang_speed = [], arm_side = "right")
                planL, successL = compute_cartesian_path_velocity_control(waypoints_left, speed_left, EE_ang_speed = [])
        if not successR or not successL:
                return [], False

        #Finish moving both arms at the same time
        if sync_policy == 1:
                if len(rs.joint_state.position) > 0: #In case this function is called by sync policy 3, it updates the start state of the arm
                        arm_left.set_start_state(rs_l) 
                        arm_right.set_start_state(rs_r)      
                        arms.set_start_state(rs)
                #Adjusts the step of the shorter plan and recalculates it, so both have approximately the same number of points
                if len(planR.joint_trajectory.points) > len(planL.joint_trajectory.points):
                        planL, success = compute_cartesian_path_velocity_control(waypoints_left, [speed_left[-1]], EE_ang_speed = [], arm_side = "left", step = 0.002*(float(len(planL.joint_trajectory.points))/float(len(planR.joint_trajectory.points))))
                else:
                        planR, success = compute_cartesian_path_velocity_control(waypoints_right, [speed_right[-1]], EE_ang_speed = [], arm_side = "right", step = 0.002*(float(len(planR.joint_trajectory.points))/float(len(planL.joint_trajectory.points))))
                plan_both = merge_plans(planL, planR)

        #Each arm moves at its own speed
        elif sync_policy == 2:
                plan_both = merge_plans_keep_speed(planL, planR)
        else:
                return [], False

        #In case both arms need to finish at the same time (sync policy 1), the real speed of one of the arms changes, therefore it is recalculated
        total_t_R = float(planR.joint_trajectory.points[-1].time_from_start.secs) + 0.000000001 * float(planR.joint_trajectory.points[-1].time_from_start.nsecs)
        total_t_L = float(planL.joint_trajectory.points[-1].time_from_start.secs) + 0.000000001 * float(planL.joint_trajectory.points[-1].time_from_start.nsecs)
        if extra_info:
                if total_t_R > total_t_L:
                        v_real = [speed_left[-1] * (total_t_L/total_t_R), speed_right[-1]]
                        t_acc = t_acc_r
                        t_dec = t_dec_r
                else:
                        v_real = [speed_left[-1], speed_right[-1] * (total_t_R/total_t_L)]
                        t_acc = t_acc_l
                        t_dec = t_dec_l

        #After calculating the plans, the correct ACM is restablished and then, the plan validity is checked for every configuration            
        sceneReq = PlanningScene()
        sceneReq.allowed_collision_matrix = acm_correct
        sceneReq.is_diff = True
        sceneResponse = my_service(sceneReq)

        plan_both_success = True
        rospy.wait_for_service('/check_state_validity')
        my_service = rospy.ServiceProxy('/check_state_validity', GetStateValidity)
        check_stateReq = GetStateValidityRequest()
        check_robot_state = JointState()

        check_robot_state.name = copy.deepcopy(plan_both.joint_trajectory.joint_names)
        state_robot = robot.get_current_state()
        add_joints_index = []
        i = 0
        for joint_name in state_robot.joint_state.name:
                if joint_name not in check_robot_state.name:
                     add_joints_index.append(i)  
                i+=1 
        for i in add_joints_index: #Adds the joints that are not in the generated plan, for instance, the joints of the torso
                check_robot_state.name.append(state_robot.joint_state.name[i])

        #Add both end effectors to check the collisions
        attached_gripper_left = scene.get_attached_objects([ATC1.EEF_left])[ATC1.EEF_left]
        attached_gripper_right = scene.get_attached_objects([ATC1.EEF_right])[ATC1.EEF_right]
        check_stateReq.robot_state.attached_collision_objects.append(attached_gripper_left)
        check_stateReq.robot_state.attached_collision_objects.append(attached_gripper_right)

        #Check the validity of all the states of the trajectory    
        for joint_state in plan_both.joint_trajectory.points:
                check_robot_state.position = []
                for joint in joint_state.positions:
                        check_robot_state.position.append(joint)
                for i in add_joints_index:
                        check_robot_state.position.append(state_robot.joint_state.position[i])
                check_stateReq.robot_state.joint_state = check_robot_state
                stateValidityResp = my_service(check_stateReq)
                if not stateValidityResp.valid:
                        print(check_robot_state)
                        plan_both_success = False
                        break

        if plan_both_success:
                if extra_info: #Required when sync policy 3 is used
                        return plan_both, True, t_acc, t_dec, v_real
                else:
                        return plan_both, True
        else:
                print("Error generating dual-arm plan")
                plan = RobotTrajectory()
                if extra_info: #Required when sync policy 3 is used
                        return plan, False, 0, 0, 0
                else:
                        return plan, False


def dual_arm_intermediate_waypoints_plan(waypoints, speed, ATC1):
        """
        This function generates a dual-arm motion plan ready to be executed, where both arms reach a 
        sequence of intermediate dual-arm waypoints at the same time. A maximum EEF speed can be set.
        - waypoints: List of dual-arm waypoints. Each dual-arm waypoint is a list with 2 poses, one for the left arm and the other for the right.
        - speed: Maximum linear speed of any of the arms [mm/s].
        - ATC1: ATC object of the simulation
        """

        all_plans = []
        all_extra_info = []
        first = True
        #Plan the movement to every dual-arm waypoint with sync policy 1
        for i in range(1, len(waypoints)):
                if first:
                        plan, success, t_acc, t_dec, v_real = dual_arm_cartesian_plan([[waypoints[i-1][0], waypoints[i][0]]], [speed], [[waypoints[i-1][1], waypoints[i][1]]], [speed], ATC1=ATC1, sync_policy=1, extra_info=True)
                        first = False
                else:
                        plan, success, t_acc, t_dec, v_real = dual_arm_cartesian_plan([[waypoints[i-1][0], waypoints[i][0]]], [speed], [[waypoints[i-1][1], waypoints[i][1]]], [speed], ATC1=ATC1, sync_policy=1, extra_info=True, rs = rs, rs_l = rs_l, rs_r = rs_r)   
                if not success:
                        return [], False
                all_plans.append(plan)
                all_extra_info.append({'t_acc': t_acc, 't_dec': t_dec, 'v_real': v_real}) #Times required to accelerate and deccelerate and real speed of the arms
                rs = RobotState()
                for j_name in plan.joint_trajectory.joint_names:
                        rs.joint_state.name.append(j_name)
                for state in plan.joint_trajectory.points[-1].positions:
                        rs.joint_state.position.append(state) 

                rs_l = RobotState()
                index_left = []
                for j_name in plan.joint_trajectory.joint_names:
                        if "left" in j_name:
                                rs_l.joint_state.name.append(j_name)
                                index_left.append(True)
                        else:
                                index_left.append(False)
                i = 0
                for state in plan.joint_trajectory.points[-1].positions:
                        if index_left[i]:
                                rs_l.joint_state.position.append(state) 
                        i+=1

                rs_r = RobotState()
                index_right = []
                for j_name in plan.joint_trajectory.joint_names:
                        if "right" in j_name:
                                rs_r.joint_state.name.append(j_name)
                                index_right.append(True)
                        else:
                                index_right.append(False)
                i = 0
                for state in plan.joint_trajectory.points[-1].positions:
                        if index_right[i]:
                                rs_r.joint_state.position.append(state) 
                        i+=1

                #Update the start state for the next dual-arm waypoint path calculation
                arm_left.set_start_state(rs_l) 
                arm_right.set_start_state(rs_r)      
                arms.set_start_state(rs)

        #After calculating all the individual trajectories, restore the real state
        arm_left.set_start_state_to_current_state() 
        arm_right.set_start_state_to_current_state() 
        arms.set_start_state_to_current_state() 

        all_extra_info_real = copy.deepcopy(all_extra_info)
        all_plans_t_dict = []
        all_plans_dict = []

        #Forward kinematics of all the configurations of the plan, in order to get all the EEF poses for each arm
        rospy.wait_for_service('compute_fk')
        fk_srv = rospy.ServiceProxy('compute_fk', GetPositionFK)
        rs = RobotState()
        fkln = ['arm_right_link_7_t', 'arm_left_link_7_t'] #Modify with the names of the final links of the arms
        for j_name in all_plans[0].joint_trajectory.joint_names:
                rs.joint_state.name.append(j_name)

        for i in range(len(all_plans)):
                all_plans_t_dict.append({})
                all_plans_dict.append([])
                for point in all_plans[i].joint_trajectory.points:
                        time_i = float(point.time_from_start.secs) + 0.000000001 * float(point.time_from_start.nsecs)
                        rs.joint_state.position = []
                        for joint in point.positions:
                                rs.joint_state.position.append(joint)
                        header = Header(0,rospy.Time.now(),"torso_base_link") #Modify with the reference frame name
                        header.frame_id = all_plans[0].joint_trajectory.header.frame_id
                        plan_pose_meters_r = fk_srv(header, fkln, rs).pose_stamped[0].pose
                        plan_pose_mm_r = copy.deepcopy(plan_pose_meters_r)
                        plan_pose_mm_r.position.x *= 1000
                        plan_pose_mm_r.position.y *= 1000
                        plan_pose_mm_r.position.z *= 1000
                        plan_pose_meters_l = fk_srv(header, fkln, rs).pose_stamped[1].pose
                        plan_pose_mm_l = copy.deepcopy(plan_pose_meters_l)
                        plan_pose_mm_l.position.x *= 1000
                        plan_pose_mm_l.position.y *= 1000
                        plan_pose_mm_l.position.z *= 1000
                        all_plans_t_dict[-1][time_i] = {'J': point.positions, 'V': point.velocities, 'A': point.accelerations, 'pose_r': plan_pose_mm_r, 'pose_l': plan_pose_mm_l}
                        all_plans_dict[-1].append({'t': time_i, 'J': list(point.positions), 'V': list(point.velocities), 'A': list(point.accelerations), 'pose_r': plan_pose_mm_r, 'pose_l': plan_pose_mm_l, 'mod': False})
                        #Get the real times required to accelerate and decelerate, necessary to merge the consecutive plans
                        if time_i <= all_extra_info[i]['t_acc']:
                                all_extra_info_real[i]['t_acc'] = time_i
                        if time_i <= all_extra_info[i]['t_dec']:
                                all_extra_info_real[i]['t_dec'] = time_i

        all_plans_dict_updated = copy.deepcopy(all_plans_dict)
        for i in range(len(all_plans)-1):
                #Distance required to change speed from plan i to i+1
                x_diff_l_1 = compute_distance(all_plans_t_dict[i][all_extra_info_real[i]['t_dec']]['pose_l'], all_plans_dict[i][-1]['pose_l']) #More accurate if it is done point by point incrementally
                x_diff_l_2 = compute_distance(all_plans_dict[i][0]['pose_l'], all_plans_t_dict[i+1][all_extra_info_real[i+1]['t_acc']]['pose_l']) ### MODIFY TO i+1
                x_diff_l = x_diff_l_1 + x_diff_l_2
                x_diff_r_1 = compute_distance(all_plans_t_dict[i][all_extra_info_real[i]['t_dec']]['pose_r'], all_plans_dict[i][-1]['pose_r'])
                x_diff_r_2 = compute_distance(all_plans_dict[i][0]['pose_r'], all_plans_t_dict[i+1][all_extra_info_real[i+1]['t_acc']]['pose_r'])
                x_diff_r = x_diff_r_1 + x_diff_r_2
                v_diff_l = all_extra_info_real[i+1]['v_real'][0] - all_extra_info_real[i]['v_real'][0] #Left arm speed change between consecutive plans
                v_diff_r = all_extra_info_real[i+1]['v_real'][1] - all_extra_info_real[i]['v_real'][1] #Right arm speed change between consecutive plans
                t_diff_l = (2*x_diff_l)/((2*all_extra_info_real[i]['v_real'][0])+v_diff_l) #Times required for the speed changes
                t_diff_r = (2*x_diff_r)/((2*all_extra_info_real[i]['v_real'][1])+v_diff_r)
                if t_diff_l >= t_diff_r:
                        t_diff = t_diff_l
                        accel = v_diff_l/t_diff
                        last_v = all_extra_info_real[i]['v_real'][0]
                else:
                        t_diff = t_diff_r
                        accel = v_diff_r/t_diff
                        last_v = all_extra_info_real[i]['v_real'][1]

                index_dec = next((index for (index, d) in enumerate(all_plans_dict[i]) if d["t"] == all_extra_info_real[i]['t_dec']), None)
                index_acc = next((index for (index, d) in enumerate(all_plans_dict[i]) if d["t"] == all_extra_info_real[i]['t_acc']), None)
                index_acc_next = next((index for (index, d) in enumerate(all_plans_dict[i+1]) if d["t"] == all_extra_info_real[i+1]['t_acc']), None)

                #Recalculates the times of the plans during the speed transitions (accelerations or decelerations) to merge them. A constant acceleration is considered
                for p in range(1,len(all_plans_dict[i])): #p is the index of the points of each plan
                        #Constant speed section
                        if p > index_acc and p < index_dec:
                                all_plans_dict_updated[i][p]['t'] = (all_plans_dict[i][p]['t'] - all_plans_dict[i][p-1]['t']) + all_plans_dict_updated[i][p-1]['t']
                        #Deceleration section
                        elif p > index_dec:
                                t_A = accel/2
                                t_B = last_v
                                if t_diff_l >= t_diff_r: #The slower arm is used to determine the times
                                        t_C = -compute_lin_or_ang_distance(all_plans_dict[i][p]['pose_l'], all_plans_dict[i][p-1]['pose_l'], linear=True)
                                else:
                                        t_C = -compute_lin_or_ang_distance(all_plans_dict[i][p]['pose_r'], all_plans_dict[i][p-1]['pose_r'], linear=True)  

                                new_time_1 = (-t_B + math.sqrt(t_B**2 - 4*t_A*t_C))/(2*t_A)
                                new_time_2 = (-t_B - math.sqrt(t_B**2 - 4*t_A*t_C))/(2*t_A)
                                if new_time_1 < 0 and new_time_2 < 0:
                                        new_time = 0
                                        success = False
                                elif new_time_1 < 0:
                                        new_time = new_time_2
                                elif new_time_2 < 0:
                                        new_time = new_time_1
                                elif new_time_1 < new_time_2:
                                        new_time = new_time_1
                                else:
                                        new_time = new_time_2
                                all_plans_dict_updated[i][p]['t'] = all_plans_dict_updated[i][p-1]['t'] + new_time
                                all_plans_dict_updated[i][p]['mod'] = True
                                last_v += accel*new_time

                all_plans_dict_updated[i+1][0]['t'] = all_plans_dict_updated[i][-1]['t']

                for p in range(1,len(all_plans_dict[i+1])): #p is the index of the points of each plan
                        #Acceleration section
                        if p <= index_acc_next:
                                t_A = accel/2
                                t_B = last_v
                                if t_diff_l >= t_diff_r: #The slower arm is used to determine the times
                                        t_C = -compute_lin_or_ang_distance(all_plans_dict[i+1][p]['pose_l'], all_plans_dict[i+1][p-1]['pose_l'], linear=True)
                                else:
                                        t_C = -compute_lin_or_ang_distance(all_plans_dict[i+1][p]['pose_r'], all_plans_dict[i+1][p-1]['pose_r'], linear=True)  

                                new_time_1 = (-t_B + math.sqrt(t_B**2 - 4*t_A*t_C))/(2*t_A)
                                new_time_2 = (-t_B - math.sqrt(t_B**2 - 4*t_A*t_C))/(2*t_A)
                                if new_time_1 < 0 and new_time_2 < 0:
                                        new_time = 0
                                        success = False
                                elif new_time_1 < 0:
                                        new_time = new_time_2
                                elif new_time_2 < 0:
                                        new_time = new_time_1
                                elif new_time_1 < new_time_2:
                                        new_time = new_time_1
                                else:
                                        new_time = new_time_2
                                all_plans_dict_updated[i+1][p]['t'] = all_plans_dict_updated[i+1][p-1]['t'] + new_time
                                last_v += accel*new_time
                                all_plans_dict_updated[i+1][p]['mod'] = True
                        else:
                                all_plans_dict_updated[i+1][p]['t'] = (all_plans_dict[i+1][p]['t'] - all_plans_dict[i+1][p-1]['t']) + all_plans_dict_updated[i+1][p-1]['t']

        
        #Once the times have been recalculated for merging the plans, they are added into a single plan
        all_plans_together = []
        for plan in all_plans_dict_updated:
                all_plans_together += plan

        #The configurations with no transition time between them are removed
        final_plan = []
        for i in range(len(all_plans_together)-1):
                time_diff = all_plans_together[i+1]['t']-all_plans_together[i]['t']
                if time_diff != 0:
                        final_plan.append(all_plans_together[i])
        final_plan.append(all_plans_together[-1])

        #The joint velocities and accelerations of every configuration are updated with the new times
        for i in range(len(final_plan)-1):
                time_diff = final_plan[i+1]['t']-final_plan[i]['t']
                for j in range(len(final_plan[i]['J'])):
                        if final_plan[i]['mod']:
                                angle_diff = final_plan[i+1]['J'][j]-final_plan[i]['J'][j]
                                new_Jaccel = (angle_diff-(final_plan[i]['V'][j]*time_diff))*(2/(time_diff**2))
                                new_Jspeed = final_plan[i]['V'][j] + (new_Jaccel*time_diff)
                                final_plan[i+1]['V'][j] = new_Jspeed
                                final_plan[i+1]['A'][j] = new_Jaccel

        #Uncomment to visualize the full dual-arm trajectory in cartesian space
        """
        for point in final_plan:
                print(str(point['t']))
                print(str(point['pose_l'].position.x) + ", " + str(point['pose_l'].position.y))
                print(str(point['pose_r'].position.x) + ", " + str(point['pose_r'].position.y))
                print("---------")
        """

        #Adjust the plan to the RobotTrajectory() message structure
        merged_plan = RobotTrajectory()
        merged_plan.joint_trajectory.header.frame_id = copy.copy(all_plans[0].joint_trajectory.header.frame_id)
        merged_plan.joint_trajectory.joint_names = copy.deepcopy(all_plans[0].joint_trajectory.joint_names)
        
        for p in all_plans_together:
                point = JointTrajectoryPoint()
                point.positions = p['J']
                point.velocities = p['V']
                point.accelerations = p['A']
                point.effort = []
                point.time_from_start.secs = int(p['t'])
                point.time_from_start.nsecs = int((p['t'] - int(p['t']))*1000000000)
                merged_plan.joint_trajectory.points.append(point)
        
        return merged_plan, True

        
def master_slave_constant_distance(waypoints_gripper, ATC1, speed = 30, arm = "left"):
        """
        Funtion to generate dual-arm master-slave plans where the slave arm motion are calculated to keep the distance between the EEFs constant
        - waypoints_gripper: List of waypoints of the master arm [poses]
        - speed: EEF speed of the master arm [mm/s]
        - arm: Master arm side ["left" or "right"]
        """
        left_wrist_pose = arm_left.get_current_pose().pose
        right_wrist_pose = arm_right.get_current_pose().pose
        left_wrist_frame = pose_to_frame(left_wrist_pose)
        right_wrist_frame = pose_to_frame(right_wrist_pose)

        #Transform initial poses from the wrist to the EEF frame
        aux_left_wrist2_pose = ATC1.correctPose(left_wrist_pose, "left")
        aux_left_wrist2_frame = pose_to_frame(aux_left_wrist2_pose)
        transform_left_wrist_EEF = get_inverse_frame(aux_left_wrist2_frame) * left_wrist_frame
        left_EEF_frame = left_wrist_frame * transform_left_wrist_EEF

        aux_right_wrist2_pose = ATC1.correctPose(right_wrist_pose, "right")
        aux_right_wrist2_frame = pose_to_frame(aux_right_wrist2_pose)
        transform_right_wrist_EEF = get_inverse_frame(aux_right_wrist2_frame) * right_wrist_frame
        right_EEF_frame = right_wrist_frame * transform_right_wrist_EEF

        #Get the transformation matrix between EEFs, that will be kept constant
        arms_diff_frame = PyKDL.Frame()
        arms_diff_frame = get_inverse_frame(left_EEF_frame) * right_EEF_frame #Left to Right frame
        if arm == "right":
                arms_diff_frame = get_inverse_frame(arms_diff_frame) #Right to Left frame

        #Calculate the waypoints of the slave arm keeping the initial transformation matrix between the EEFs constant
        sec_arm_wp_gripper = []
        for wp_g in waypoints_gripper:
                if arm == "left":
                        wp_frame = pose_to_frame(wp_g) * transform_left_wrist_EEF
                        sec_arm = "right"
                elif arm == "right":
                        wp_frame = pose_to_frame(wp_g) * transform_right_wrist_EEF
                        sec_arm = "left"
                else:
                        return [], False
                sec_wp = frame_to_pose(wp_frame * arms_diff_frame)
                sec_arm_wp_gripper.append(ATC1.correctPose(sec_wp, sec_arm))

        #Generate the dual-arm plan      
        if arm == "left":
                plan, success = dual_arm_cartesian_plan([waypoints_gripper], [speed], [sec_arm_wp_gripper], [speed], ATC1= ATC1, sync_policy=1)
        elif arm == "right":
                plan, success = dual_arm_cartesian_plan([sec_arm_wp_gripper], [speed], [waypoints_gripper], [speed], ATC1= ATC1, sync_policy=1)
        else:
                plan = []
                success = False

        return plan, success



def master_slave_identical_motion(waypoints_gripper, ATC1, speed = 30, arm = "left"):
        """
        Funtion to generate dual-arm master-slave plans where the slave arm motion is calculated to be exactly the same as the master motion
        - waypoints_gripper: List of waypoints of the master arm [poses]
        - speed: EEF speed of the master arm [mm/s]
        - arm: Master arm side ["left" or "right"]
        """
        left_wrist_pose = arm_left.get_current_pose().pose
        right_wrist_pose = arm_right.get_current_pose().pose
        left_wrist_frame = pose_to_frame(left_wrist_pose)
        right_wrist_frame = pose_to_frame(right_wrist_pose)

        #Saves the first previous frame of the master arm to start the calculation
        if arm == "left":
                sec_arm_wp_gripper = [right_wrist_pose]
                sec_arm_frame_prev = right_wrist_frame
                prev_frame = left_wrist_frame
                sec_arm = "right"
                #Get the transformation matrix from the wrist frame to the EEF frame
                aux_left_wrist2_pose = ATC1.correctPose(left_wrist_pose, "left")
                aux_left_wrist2_frame = pose_to_frame(aux_left_wrist2_pose)
                transform_wrist_EEF = get_inverse_frame(aux_left_wrist2_frame) * left_wrist_frame
        else:
                sec_arm_wp_gripper = [left_wrist_pose]
                sec_arm_frame_prev = left_wrist_frame
                prev_frame = right_wrist_frame
                sec_arm = "left"
                #Get the transformation matrix from the wrist frame to the EEF frame
                aux_right_wrist2_pose = ATC1.correctPose(right_wrist_pose, "right")
                aux_right_wrist2_frame = pose_to_frame(aux_right_wrist2_pose)
                transform_wrist_EEF = get_inverse_frame(aux_right_wrist2_frame) * right_wrist_frame

        #Slave arm waypoints are calculated getting the master transform from its previous point to the current, and applying the same transform to the slave arm previous point
        for wp_g in waypoints_gripper:
                wp_frame = pose_to_frame(wp_g) * transform_wrist_EEF
                prev_current_diff_frame = get_inverse_frame(prev_frame)*wp_frame
                prev_frame = wp_frame #Update
                        
                sec_wp = frame_to_pose(sec_arm_frame_prev * prev_current_diff_frame)
                sec_arm_wp_gripper.append(ATC1.correctPose(sec_wp, sec_arm))
                sec_arm_frame_prev = pose_to_frame(sec_wp) #Update
                
        #Generate the dual-arm plan      
        if arm == "left":
                plan, success = dual_arm_cartesian_plan([waypoints_gripper], [speed], [sec_arm_wp_gripper], [speed], ATC1= ATC1, sync_policy=1)
        elif arm == "right":
                plan, success = dual_arm_cartesian_plan([sec_arm_wp_gripper], [speed], [waypoints_gripper], [speed], ATC1= ATC1, sync_policy=1)
        else:
                plan = []
                success = False

        return plan, success


def master_slave_plan(waypoints, ATC1, speed=30, arm="left", type=1):
    """
    Funtion to generate dual-arm master-slave plans
        - waypoints_gripper: List of waypoints of the master arm [poses]
        - speed: EEF speed of the master arm [mm/s]
        - arm: Master arm side ["left" or "right"]
        - type: It can be 1 or 2.
                type 1: The distance between the EEFs is constant during the motion
                type 2: The motion of the slave arm is identical to the motion of the master arm
    """
    if type==1:
        plan, success = master_slave_constant_distance(waypoints, ATC1, speed, arm)
        return plan, success
    elif type==2:
        plan, success = master_slave_identical_motion(waypoints, ATC1, speed, arm)
        return plan, success
    else:
        return [], False


def compute_cartesian_path_velocity_control_arms_occlusions(waypoints_list, EE_speed, EE_ang_speed = [], arm_side = "left", max_linear_accel = 200.0, max_ang_accel = 140.0, extra_info = False, step = 0.002):
        global motion_groups
        global max_arms_dist

        if arm_side == "left":
                arm1 = motion_groups['arm_left']
                arm2 = motion_groups['arm_right']
        elif arm_side == "right":
                arm1 = motion_groups['arm_right']
                arm2 = motion_groups['arm_left']

        move_both = False
        trash_pose = arm1.get_current_pose().pose
        arm1_pose = arm1.get_current_pose().pose
        arm2_pose = arm2.get_current_pose().pose

        all_wp=[]
        for wp_list in waypoints_list:
                for wp_i in wp_list:
                        all_wp.append(wp_i)

        # print("##############POSES###############")
        # print(arm1_pose)
        # print(arm2_pose)
        # print(all_wp[0])
        # print(all_wp[-1])
        # print("##################################")

        initial_distance = compute_distance_xy(arm1_pose,arm2_pose)
        for i in range(len(all_wp)):
                if compute_distance_xy(all_wp[i],arm2_pose) < min(max_arms_dist, initial_distance):
                                move_both = True
                                break
                if i<(len(all_wp)-1):                        
                        if compute_distance_xy(all_wp[i],all_wp[i+1]) > 0.01:
                                print("REQUIRED INTERPOLATION")
                                success, interpolated_traj = interpolate_trajectory(initial_pose = all_wp[i], final_pose = all_wp[i+1], step_pos_min = 0.01, step_deg_min = 5, n_points_max = 100)
                                if len(interpolated_traj) > 2:
                                        for int_wp in interpolated_traj[1:-1]: #First and last are not checked here
                                                if compute_distance_xy(int_wp,arm2_pose) < min(max_arms_dist, initial_distance):
                                                        move_both = True
                                                        break

        if move_both:
                motion_group_plan = 'arms' 
                #if compute_distance_xy(arm1_pose,arm2_pose) < max_arms_dist+0.1: #If they are very close is better to just move master-slave
                if False:
                        print("Master slave motion to avoid collision")
                        plan, success = master_slave_plan(all_wp, ATC1, 50.0, arm_side, type=1)
                else: #Otherwise just the final point of the second arm is updated to avoid collisions  
                        print("Dual arm motion to avoid collision")                      
                        listener_success = False 
                        while not listener_success:
                                try:
                                        (T_BT, R_BT) = listener.lookupTransform('/base_link', '/torso_link_b1', rospy.Time(0))
                                        frame_BT = listener_to_frame(T_BT, R_BT)
                                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                                        rospy.sleep(0.1)
                                        print("Listener error")
                                        continue
                                listener_success = True
                        final_pose1_TG = frame_to_pose(get_inverse_frame(frame_BT) * pose_to_frame(all_wp[-1]))
                        initial_pose2_TG = frame_to_pose(get_inverse_frame(frame_BT) * pose_to_frame(arm2_pose))
                        final_pose2_TG = copy.deepcopy(initial_pose2_TG)
                        if arm_side == 'left':
                                final_pose2_TG.position.y = final_pose1_TG.position.y - (max_arms_dist + 0.1)
                        elif arm_side == 'right':
                                final_pose2_TG.position.y = final_pose1_TG.position.y + (max_arms_dist + 0.1)
                        final_frame2_TG = pose_to_frame(final_pose2_TG)
                        final_pose2_BG = frame_to_pose(frame_BT * final_frame2_TG)
                        if arm_side == "left":
                                wp_left = copy.deepcopy(waypoints_list)
                                if initial_distance < (max_arms_dist + 0.02):
                                        speed_left = copy.deepcopy([min(EE_speed)*0.75])
                                        speed_right = [max(EE_speed)*1.25]
                                else:
                                        speed_left = copy.deepcopy(EE_speed)
                                        speed_right = [max(EE_speed)]
                                success, wp_right_interpolated = interpolate_trajectory(initial_pose = arm2_pose, final_pose = final_pose2_BG, step_pos_min = 0.02, step_deg_min = 5, n_points_max = 20)
                                wp_right = [wp_right_interpolated]                               
                        elif arm_side == 'right':
                                wp_right = copy.deepcopy(waypoints_list)
                                if initial_distance < (max_arms_dist + 0.02):
                                        speed_right = copy.deepcopy([min(EE_speed)*0.75])
                                        speed_left = [max(EE_speed)*1.25]
                                else:
                                        speed_right = copy.deepcopy(EE_speed)
                                        speed_left = [max(EE_speed)]
                                success, wp_left_interpolated = interpolate_trajectory(initial_pose = arm2_pose, final_pose = final_pose2_BG, step_pos_min = 0.02, step_deg_min = 5, n_points_max = 20)
                                wp_left = [wp_left_interpolated]
                                #wp_left = [[arm2_pose, final_pose2_BG]]
                                #visualize_keypoints_simple(wp_right[0] + wp_left[0])
                        plan, success = dual_arm_cartesian_plan(wp_left, speed_left, wp_right, speed_right, ATC1= ATC1, sync_policy=2)
                
        else:
                print("No motion collision between arms")
                motion_group_plan = 'arm_'+arm_side
                plan, success = compute_cartesian_path_velocity_control(waypoints_list, EE_speed, EE_ang_speed, arm_side, max_linear_accel, max_ang_accel, extra_info, step)

        return plan, success, motion_group_plan
######################################################################################################################################

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


def interpolate_trajectory(initial_pose, final_pose, step_pos_min, step_deg_min, n_points_max):
        """
        Creates several waypoints between two poses
        """
        waypoints = []
        if initial_pose == final_pose:
                print("Cannot interpolate points, it is the same pose")
                return False, waypoints

        n_points = n_points_max
        step_pos_min = float(step_pos_min)
        step_deg_min = float(step_deg_min)
        pos_dif = compute_distance(initial_pose, final_pose)
        deg_dif, rad_dif = degree_difference(pose_to_frame(initial_pose).M, pose_to_frame(final_pose).M)
        n_points_list = []
        n_points_list.append(pos_dif/step_pos_min)
        n_points_list.append(abs(deg_dif[0])/step_deg_min)
        n_points_list.append(abs(deg_dif[1])/step_deg_min)
        n_points_list.append(abs(deg_dif[2])/step_deg_min)
        if max(n_points_list) < 20:
                n_points = int(max(n_points_list))
        
        waypoints.append(initial_pose)
        for point in range(n_points):
            if point > 0:
                x = initial_pose.position.x + ((final_pose.position.x - initial_pose.position.x)*float(point)/float(n_points))
                y = initial_pose.position.y + ((final_pose.position.y - initial_pose.position.y)*float(point)/float(n_points))
                z = initial_pose.position.z + ((final_pose.position.z - initial_pose.position.z)*float(point)/float(n_points))
                rotation = pose_to_frame(initial_pose).M
                rotation.DoRotX((rad_dif[0])*float(point)/float(n_points))
                rotation.DoRotY((rad_dif[1])*float(point)/float(n_points))
                rotation.DoRotZ((rad_dif[2])*float(point)/float(n_points))
                new_frame = PyKDL.Frame() 
                new_frame.p = PyKDL.Vector(x,y,z) 
                new_frame.M = rotation
                waypoints.append(frame_to_pose(new_frame))
        waypoints.append(final_pose)

        return True, waypoints


def circular_trajectory(center, initial_pose, degree, rot_axis, center_waypoints, circle_waypoints, step = 2, rot_gripper = False): #R axis is x and Rot axis is z in its own rotation frame
        R_axis = get_axis(initial_pose, center)
        threshold_ort = 0.05 #0.001
        if abs(cross_product_vectors(R_axis, rot_axis)) > threshold_ort:
                print("Error. axis are not orthogonal")
                return False, center_waypoints, circle_waypoints
        R_axis, rot_axis, y_axis = get_ort_axis(R_axis, rot_axis)
        RM_world_own = PyKDL.Frame() 
        RM_world_own.p = PyKDL.Vector(center.position.x, center.position.y, center.position.z)
        RM_world_own.M = PyKDL.Rotation(R_axis[0], y_axis[0], rot_axis[0], R_axis[1], y_axis[1], rot_axis[1], R_axis[2], y_axis[2], rot_axis[2])

        #Get initial gripper orientation from own circle frame
        Rot_own_world = get_transpose_rot(RM_world_own.M)
        Rot_world_gripper_cicle = PyKDL.Rotation.Quaternion(initial_pose.orientation.x, initial_pose.orientation.y, initial_pose.orientation.z, initial_pose.orientation.w) 
        initial_gripper_circle_ori_own = Rot_own_world * Rot_world_gripper_cicle
        Rot_world_gripper_center = PyKDL.Rotation.Quaternion(center.orientation.x, center.orientation.y, center.orientation.z, center.orientation.w)
        initial_gripper_center_ori_own = Rot_own_world * Rot_world_gripper_center

        R = compute_distance(center, initial_pose)
        
        #Initialize variables
        circle_waypoints_own = []
        center_waypoints_own = []
        new_waypoint_circle_own = PyKDL.Frame()
        new_waypoint_circle_own.p = PyKDL.Vector(R,0,0)
        new_waypoint_circle_own.M = initial_gripper_circle_ori_own
        circle_waypoints_own.append(new_waypoint_circle_own)
        new_waypoint_center_own = PyKDL.Frame()
        new_waypoint_center_own.p = PyKDL.Vector(0,0,0)
        new_waypoint_center_own.M = initial_gripper_center_ori_own
        #center_waypoint_init_own = new_waypoint_center_own
        center_waypoints_own.append(new_waypoint_center_own)
        circle_waypoints.append(initial_pose) #pose
        center_waypoints.append(center) #pose

        #Create several waypoints for the circular trajectory in its own frame
        for deg in range(step, int(degree)+step, step):
                if abs(deg) >= abs(degree): #To have inclusive range
                        deg = degree
                deg = float(deg)*(math.pi/180.0)
                new_waypoint_circle_own = PyKDL.Frame()
                new_waypoint_circle_own.p = PyKDL.Vector(R * math.cos(deg), R * math.sin(deg), 0) #In its own frame the rotation is around z axis
                #print(new_waypoint_circle_own.p)
                #new_waypoint_circle_own.M = circle_waypoints_own[-1].M
                new_waypoint_circle_own.M = circle_waypoints_own[0].M
                new_waypoint_center_own = PyKDL.Frame()
                #new_waypoint_center_own = center_waypoints_own[-1]
                new_waypoint_center_own = copy.deepcopy(center_waypoints_own[0])
                #print(new_waypoint_center_own)
                if rot_gripper:
                        new_waypoint_circle_own.M.DoRotZ(deg)
                        new_waypoint_center_own.M.DoRotZ(deg)
                        #print(new_waypoint_center_own)
                circle_waypoints_own.append(new_waypoint_circle_own) #frame
                center_waypoints_own.append(new_waypoint_center_own) #frame
                #Transform this circular curve that is in its own frame to the real rotation frame
                circle_waypoint_pose_new = frame_to_pose(RM_world_own * new_waypoint_circle_own) #pose
                center_waypoints_pose_new = frame_to_pose(RM_world_own * new_waypoint_center_own) #pose
                if not rot_gripper:
                       circle_waypoint_pose_new.orientation = initial_pose.orientation

                circle_waypoints.append(circle_waypoint_pose_new) #pose
                center_waypoints.append(center_waypoints_pose_new) #pose

        return True, center_waypoints, circle_waypoints

######################################################################################################################################

# def route_cables_2(op_info, last_guide, connector_hand, con_pose, cable_pose):
#         if connector_hand == "left":
#                 arm_con = arm_left
#                 arm_cable = arm_right
#                 cable_hand = "right"
#         else:
#                 arm_con = arm_right
#                 arm_cable = arm_left
#                 cable_hand = "left"

#         #We asume that the cables are already grasped. This will be more complex when we consider the cable separation

#         #Analyzes the sequence of jigs to know if there are corners (turns) and how many jigs group we must guide together
#         route_plan = []
#         first_guide = True
#         if abs(op_info['spot'][0]['pose_corner'].position.x - last_guide['pose_corner'].position.x) >= 0.001:
#                 direction = (op_info['spot'][0]['pose_corner'].position.y - last_guide['pose_corner'].position.y)/(op_info['spot'][0]['pose_corner'].position.x - last_guide['pose_corner'].position.x)
#                 direction_angle = math.atan(direction)*180/math.pi
#         else:
#                 direction_angle = 90
#         #direction_angle)
#         direction_x = get_axis(op_info['spot'][0]['pose_corner'], last_guide['pose_corner'])
#         end_guide_prev = get_shifted_pose(last_guide['pose_corner'], [last_guide['width'], last_guide['gap']/2, 0, 0, 0, 0]) 
#         start_first_guide = get_shifted_pose(op_info['spot'][0]['pose_corner'], [0, op_info['spot'][0]['gap']/2, 0, 0, 0, 0])

#         if axis_angle2D(direction_x[0:2], get_axis_from_RM(pose_to_frame(last_guide['pose_corner']).M, "R")[0:2]) < 90:
#                         route_plan.append(str(last_guide['jig']) + ":ok")
#         else:                
#                         route_plan.append(str(last_guide['jig']) + ":rot")

#         if compute_distance(end_guide_prev, start_first_guide) > 0.06:
#                 route_plan.append('slide')
#         else:
#                 route_plan.append('lift')

#         for guide in op_info['spot']:
#                 #print(guide)
#                 start_guide_current = get_shifted_pose(guide['pose_corner'], [0, guide['gap']/2, 0, 0, 0, 0])
#                 guide_current = get_shifted_pose(guide['pose_corner'], [guide['width']/2, guide['gap']/2, 0, 0, 0, 0])
#                 end_guide_current = get_shifted_pose(guide['pose_corner'], [guide['width'], guide['gap']/2, 0, 0, 0, 0]) 

#                 if not first_guide:
#                         dist_guides = compute_distance(end_guide_prev, start_guide_current)
#                         if abs(guide_current.position.x - end_guide_prev.position.x) >= 0.001:
#                                 direction = (guide_current.position.y - end_guide_prev.position.y)/(guide_current.position.x - end_guide_prev.position.x)
#                                 direction_angle = math.atan(direction)*180/math.pi
#                         else:
#                                 direction = 1000
#                                 direction_angle = 90
#                         direction_x = get_axis(guide_current, guide_prev)
#                         #direction_z = get_axis_from_RM(pose_to_frame(guide_current).M, "Y")
#                         #direction_y = get_ort_axis(direction_z, direction_x)[2]
#                         print(direction_x)
#                         print(get_axis_from_RM(pose_to_frame(guide_current).M, "R"))
#                         if ((abs(direction_x[0])-abs(get_axis_from_RM(pose_to_frame(guide_current).M, "R")[0]))*180/math.pi)>10:
#                                 dir_change = True
#                         else:
#                                 dir_change = False
#                         print(direction_angle)
#                         #Take decisions (threshold for the small variations)
#                         if dist_guides > 0.06:
#                                 route_plan.append('insert')
#                         deg_dif, rad_dif = degree_difference(pose_to_frame(guide_current).M, pose_to_frame(guide_prev).M)
#                         #print(deg_dif)
#                         if (abs(direction_angle - direction_angle_prev) > 5 and not ((deg_dif[2]<5 and deg_dif[2]>-5) or (deg_dif[2]>175 or deg_dif[2]<-175))) or abs(direction_angle - direction_angle_prev) > 20:
#                                 if dir_change_prev:
#                                         route_plan.append('corner before insert')
#                                 else:
#                                         route_plan.append('corner after insert')

#                 else:
#                         first_guide = False
#                         print(direction_x)
#                         print(get_axis_from_RM(pose_to_frame(guide_current).M, "R"))
#                         if ((abs(direction_x[0])-abs(get_axis_from_RM(pose_to_frame(guide_current).M, "R")[0]))*180/math.pi)>10:
#                                 dir_change = True
#                         else:
#                                 dir_change = False

#                 if axis_angle2D(direction_x[0:2], get_axis_from_RM(pose_to_frame(guide_current).M, "R")[0:2]) > 90:
#                         route_plan.append(str(guide['jig']) + ":ok")
#                 else:
#                         route_plan.append(str(guide['jig']) + ":rot")

#                 guide_prev = guide_current
#                 end_guide_prev = end_guide_current
#                 direction_angle_prev = direction_angle
#                 dir_change_prev = dir_change

#         route_plan.append('insert')
        
#         print(route_plan)
        
#         #The cable hand reorients and slides the cable towards the next guide if there is distance enough
#         i = 1
#         route_plan_2 = []
#         for operation in route_plan[1:]:
#                 if operation == "lift":
#                         route_plan_2.append({"action": "lift", "keypoint": [route_plan[i+1]]})
#                 elif operation == "slide":
#                         route_plan_2.append({"action": "slide", "keypoint": [route_plan[i-1], route_plan[i+1]]})
#                 elif operation == "insert":
#                         if (i+1) < len(route_plan):
#                                 route_plan_2.append({"action": "insert", "keypoint": [route_plan[i-1], route_plan[i+1]]}) #In case of collisions we will need to reevaluate the distance to the next guide after moving forward #Also if route_plan[i+1] is not a corner
#                                 route_plan_2.append({"action": "slide", "keypoint": [route_plan[i-1], route_plan[i+1]]})
#                         else:
#                                 route_plan_2.append({"action": "insert_last", "keypoint": [route_plan[i-1]]})
#                                 route_plan_2.append({"action": "open", "keypoint": [route_plan[i-1]]})
#                 elif operation == "corner after insert":
#                         if route_plan[i-1] == "insert":
#                                 route_plan_2.append({"action": "corner_bottom", "keypoint": [route_plan[i-2], route_plan[i+1]]})
#                         else:
#                                 route_plan_2.append({"action": "corner_top", "keypoint": [route_plan[i-1], route_plan[i+1]]})
#                 elif operation == "corner before insert":
#                         if route_plan_2[-1]["action"] == "slide":
#                                 route_plan_2[-1] = {"action": "slide_corner", "keypoint": [route_plan_2[-1]["keypoint"][0], route_plan_2[-1]["keypoint"][1], route_plan[i+1]]}
#                         elif route_plan_2[-1]["action"] == "slide_top":
#                                 route_plan_2[-1] = {"action": "slide_top_corner", "keypoint": [route_plan_2[-1]["keypoint"][0], route_plan[i+1]]}
#                 else:
#                         if route_plan[i-1] != "lift" and route_plan[i-1] != "slide" and route_plan[i-1] != "insert":
#                                 route_plan_2.append({"action": "slide_top", "keypoint": [route_plan[i]]})
                
#                 i+=1
#         #print(route_plan_2)

#         for action in route_plan_2:
#                 print(action)

#         op_info['spot'].append(last_guide) #To find its information there when it searches
#         con_hand_state = 'idle'
#         circ_insert = True
#         for operation in route_plan_2:
#                 op_keypoints = []
#                 guides = []
#                 rot_gripper = False
#                 for key in operation['keypoint']:
#                         #op_keypoints.append(key.split(':')) #[jig, turn]
#                         print(key)
#                         guide = copy.deepcopy(list(filter(lambda x: (x["jig"] == key.split(':')[0]), op_info['spot']))[0])
#                         #print(guide)
#                         #print(key.split(':')[1])
#                         if key.split(':')[1] == "rot":
#                                 guide["pose_corner"] = get_shifted_pose(guide["pose_corner"], [guide['width'], guide['gap'], 0, 0, 0, math.pi])
#                                 guide["collisions"] = [-(guide["collisions"][1] - guide['width']), -guide['collisions'][0] + guide['width'], -(guide["collisions"][3] - guide['gap']), -guide["collisions"][2] + guide['gap']]
#                                 rot_gripper = True
#                         guides.append(guide)
#                         #print(guide)
                
#                 if operation['action'] == "lift":
#                         waypoints_lift, cable_pose = route_lift(guides[0], cable_pose, cable_hand, arm_cable, rot_gripper)
#                 if operation['action'] == "slide":
#                         waypoints_slide_con, waypoints_slide_cable, con_pose, cable_pose = route_slide(guides[0], guides[1], con_pose, cable_pose, cable_hand, arm_cable, con_hand_state, rot_gripper) #Check collisions
#                         #con_hand_state = 'grasping'
#                 if operation['action'] == "insert":
#                         if con_hand_state == 'idle':
#                                 rot_point = get_shifted_pose(last_guide['pose_corner'], [last_guide['width']/2, last_guide['gap']/2, last_guide['height']/2, 0, 0, 0]) 
#                         else:
#                                 rot_point = copy.deepcopy(con_pose)
#                         waypoints_insert, cable_pose = route_insert(guides[0], cable_pose, cable_hand, arm_cable, rot_point, guides[1], circ_insert, rot_gripper) #Check collisions, maybe it is not possible to insert right there and has to go down a bit forward
#                         circ_insert = True
#                 if operation['action'] == "insert_last":
#                         if con_hand_state == 'idle':
#                                 rot_point = get_shifted_pose(last_guide['pose_corner'], [last_guide['width']/2, last_guide['gap']/2, last_guide['height']/2, 0, 0, 0]) 
#                         else:
#                                 rot_point = copy.deepcopy(con_pose)
#                         waypoints_insert, cable_pose = route_insert(guides[0], cable_pose, cable_hand, arm_cable, rot_point, {}, circ_insert, rot_gripper) #Check collisions, maybe it is not possible to insert right there and has to go down a bit forward
#                         circ_insert = True
#                 if operation['action'] == "corner_top":
#                         waypoints_slide_cable, cable_pose = route_corner_top(guides[0], guides[1], cable_pose, cable_hand, arm_cable, rot_gripper)
#                         circ_insert = False
#                 if operation['action'] == "corner_bottom":
#                         pass
#                 if operation['action'] == "slide_corner":
#                         waypoints_slide_cable, cable_pose = route_corner_slide(guides[0], guides[1], guides[2], cable_pose, cable_hand, arm_cable, rot_gripper)
#                         circ_insert = False
#                 if operation['action'] == "slide_top_corner":
#                         pass
#                 if operation['action'] == "slide_top":
#                         waypoints_slide_top, cable_pose = route_slide_top(guides[0], cable_pose, cable_hand, arm_cable, rot_gripper)
#                 if operation['action'] == "open":
#                         waypoints_open_con, waypoints_open_cable, con_pose, cable_pose = route_open(con_pose, cable_pose, cable_hand, arm_cable, con_hand_state, rot_gripper)

#         return con_pose, cable_pose

#################################################################################################################

#Subscribers
def callback_forceR(force):
        global force_limit
        global force_controlled
        global stop_mov_force
        if force_controlled:

                Fxy = math.sqrt(force.Fx**2 + force.Fy**2)
                if force_limit < Fxy and not stop_mov_force:
                        print("Force is too high, stop the motion")
                        print(force)
                        stop_mov_force = True
                        arm_left.stop()
                        arm_right.stop()

subsForce = rospy.Subscriber('/right_norbdo/forces', forces, callback_forceR)  

def callback_forceL(force):
        global force_limit
        global force_controlled
        global stop_mov_force
        if force_controlled:

                Fxy = math.sqrt(force.Fx**2 + force.Fy**2)
                if force_limit < Fxy and not stop_mov_force:
                        print("Force is too high, stop the motion")
                        print(force)
                        stop_mov_force = True
                        arm_left.stop()
                        arm_right.stop()

subsForce = rospy.Subscriber('/left_norbdo/forces', forces, callback_forceL)  

def callback_status_exec_traj(status):
    global status_movement2
    global status_movement_left 
    global status_movement_right 
    if len(status.status_list)>0:
        status_movement2 = status.status_list[-1].status
        status_movement_left = status.status_list[-1].status 
        status_movement_right = status.status_list[-1].status 

def callback_right(status):
    global status_movement_right 
    if len(status.status_list)>0:
        status_movement_right = status.status_list[-1].status        
 
#subsL = rospy.Subscriber('/sda10f/sda10f_r1_controller/joint_trajectory_action/status', GoalStatusArray, callback_left)  
subsMotionStatus2 = rospy.Subscriber('/execute_trajectory/status', GoalStatusArray, callback_status_exec_traj)  
#subsR = rospy.Subscriber('/execute_trajectory/status', GoalStatusArray, callback_right) 
status_movement_left = 0
status_movement_right = 0
stop_mov_force = False

def execute_force_control(arm_side, plan):
        global status_movement_left
        global status_movement_right
        global stop_mov
        global stop_mov_force
        global step2
        global process_actionserver

        print("ENTERING FORCE CONTROL FUNCTION")
        msg_log = String()
        msg_log.data = str(arm_side) + " arm moving with force control..."
        logs_publisher.publish(msg_log)

        if arm_side == "left":
                arm_left.execute(plan, wait=False)
                while (status_movement_left==3 or status_movement_left==2 or status_movement_left==4) and not stop_mov and not stop_mov_force: #Before the motion initializes (3: SUCCEEDED, 2: PREEMTED)
                        rospy.sleep(0.01)
                print("STATUS MOVEMENT FORCE CONTROL: " + str(status_movement_left))
                print("STOP MOV FORCE: " + str(stop_mov_force))
                while (status_movement_left==0 or status_movement_left==1) and not stop_mov and not stop_mov_force: #Stop until the motion is completed
                        rospy.sleep(0.05)
                print("STATUS MOVEMENT FORCE CONTROL: " + str(status_movement_left))
                if not stop_mov:
                        step2+=1
                process_actionserver.publish_feedback()
                stop_mov_force = False
                if status_movement_left==3: #SUCCEEDED
                        msg_log.data = "Successful motion, not force limit detected"
                        logs_publisher.publish(msg_log)
                        return True
                elif status_movement_left==5: #REJECTED
                        msg_log.data = "Failure motion"
                        logs_publisher.publish(msg_log)
                        return False
                else:
                        msg_log.data = "Successful motion, force limit detected"
                        logs_publisher.publish(msg_log)
                        print("Motion stopped because the force limit has been reached")
                        return True

        elif arm_side == "right":
                arm_right.execute(plan, wait=False)
                while (status_movement_right==3 or status_movement_right==2 or status_movement_right==4) and not stop_mov and not stop_mov_force: #Before the motion initializes (3: SUCCEEDED, 2: PREEMTED)
                        rospy.sleep(0.01)
                print("STATUS MOVEMENT FORCE CONTROL: " + str(status_movement_right))
                print("STOP MOV FORCE: " + str(stop_mov_force))
                while (status_movement_right==0 or status_movement_right==1) and not stop_mov and not stop_mov_force: #Stop until the motion is completed
                        rospy.sleep(0.05)
                print("STATUS MOVEMENT FORCE CONTROL: " + str(status_movement_right))
                print(status_movement_right)
                if not stop_mov:
                        step2+=1
                process_actionserver.publish_feedback()
                stop_mov_force = False
                if status_movement_right==3: #SUCCEEDED
                        msg_log.data = "Successful motion, not force limit detected"
                        logs_publisher.publish(msg_log)
                        return True
                elif status_movement_right==5: #REJECTED
                        msg_log.data = "Failure motion"
                        logs_publisher.publish(msg_log)
                        return False
                else:
                        msg_log.data = "Successful motion, force limit detected"
                        logs_publisher.publish(msg_log)
                        print("Motion stopped because the force limit has been reached")
                        return True

        else:
                return False

##################################################################################################################

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


def stop_function(message):
        global stop_mov
        stop_mov = True
        msg_log = String()
        msg_log.data = message
        logs_publisher.publish(msg_log)
        exit()


def move_home():
        print("Moving home")
        torso.set_named_target("torso_combs")
        move_group_async("torso")
        # torso.go(wait=True)
        rospy.sleep(0.5)
        #arms.set_named_target("arms_platform_3_corrected")
        arms.set_named_target("arms_platform_5")
        move_group_async("arms")
        # arms.go(wait=True)
        rospy.sleep(0.5)


def get_last_connector_info(op):
        global ops_info
        current_index = ops_info.index(op)
        op_index = 0
        last_PC = {}
        for op_i in ops_info:
                if op_index >= current_index:
                        break
                if op_i['type'] == "PC":
                        last_PC = op_i
                op_index+=1
        return last_PC


def update_route_arm_info(op):
        global route_arm
        global arm
        global arm2
        global EEF_route
        global move_away2_sign
        global route_group
        global group2
        global force_cable 
        global force_connector
        global force_limit_cable
        global force_limit_connector

        force_limit_cable = force_cable[op['spot']['name']]
        force_limit_connector = force_connector[op['spot']['name']]
        if op['spot']['side'] == "R":
                route_arm = "right"
                route_group = "arm_right"
                group2 = "arm_left"
                arm = arm_right
                arm2 = arm_left
                EEF_route = ATC1.EEF_dict[ATC1.EEF_right].fingers_dim
                move_away2_sign = -1
        else:
                route_arm = "left"
                route_group = "arm_left"
                group2 = "arm_right"
                arm = arm_left
                arm2 = arm_right
                EEF_route = ATC1.EEF_dict[ATC1.EEF_left].fingers_dim
                move_away2_sign = 1
               

def check_tools(op):
        global step2
        if op['type'] != 'T':
                if ATC1.EEF_right=="EEF_taping_gun":
                        #MOVE TO A CONFIGURATION WITH THE GUN HORIZONTAL IN SEVERAL STEPS TO AVOID COLLISION (MOVE CONFIG)
                        changed_tool, success = ATC1.changeTool("EEF_gripper_right", "right")
                        step2 = 0
                elif ATC1.EEF_left=="EEF_taping_gun":
                        #MOVE TO A CONFIGURATION WITH THE GUN HORIZONTAL IN SEVERAL STEPS TO AVOID COLLISION (MOVE CONFIG)
                        #arms.set_named_target("arms_left_tape")
                        #arms.go(wait=True)
                        #rospy.sleep(0.5)
                        changed_tool, success = ATC1.changeTool("EEF_gripper_left", "left")
                        step2 = 0
        else:
                print("TAPE")
                if (op["spot"][0]["pose_corner"].position.x <= op["spot"][2]["pose_corner"].position.x) and (op["spot"][1]["pose_corner"].position.x <= op["spot"][2]["pose_corner"].position.x):   #Tape with left
                        print("LEFT TAPE")
                        if ATC1.EEF_right=="EEF_taping_gun":
                                print("HAVE TO CHANGE RIGHT TOOL")
                                changed_tool, success = ATC1.changeTool("EEF_gripper_right", "right")
                                step2 = 0
                        if ATC1.EEF_left=="EEF_gripper_left":
                                print("HAVE TO CHANGE LEFT TOOL")
                                actuate_grippers(grasp_distance, gripper_speed, "left", grasp=False)
                                changed_tool, success = ATC1.changeTool("EEF_taping_gun", "left")
                                step2 = 0
                                #arms.set_named_target("arms_left_tape")
                                #arms.go(wait=True)
                                #rospy.sleep(0.5)
                else: #Tape with right
                        if ATC1.EEF_left=="EEF_taping_gun":
                                changed_tool, success = ATC1.changeTool("EEF_gripper_left", "left")
                                step2 = 0
                        if ATC1.EEF_right=="EEF_gripper_right":
                                changed_tool, success = ATC1.changeTool("EEF_taping_gun", "right")
                                step2 = 0
                                #MOVE TO A CONFIGURATION WITH THE GUN VERTICAL IN SEVERAL STEPS TO AVOID COLLISION (MOVE CONFIG)
                
       

def execute_operation(op):
        global PC_op
        global route_arm
        global arm
        global arm2
        global EEF_route
        global move_away2_sign
        global route_group
        global group2  

        check_tools(op)

        if op['type'] == "EC":
                EC(op)              
            
        elif op['type'] == "PC":
                update_route_arm_info(op)
                simplified_PC(op)
                
        elif op['type'] == "RC":
                PC_op = get_last_connector_info(op)
                update_route_arm_info(PC_op)
                route_cables(op, PC_op, route_arm)

        elif op['type'] == "SC":
                PC_op = get_last_connector_info(op)
                update_route_arm_info(PC_op)
                separate_cables(op, route_arm)

        elif op['type'] == "GC" or op['type'] == "GCS":
                PC_op = get_last_connector_info(op)
                update_route_arm_info(PC_op)
                if op['type'] == "GC":
                        grasp_cables(op, route_arm)
                else:
                        grasp_cables(op, route_arm, separated=True)

        elif op['type'] == "T":
                PC_op = get_last_connector_info(op)
                tape(op["spot"][:2],op["spot"][2])


##################################################################################################################
class ElvezProcessClass(object):
        _feedback=process_UIFeedback() #Class attributes
        _result=process_UIResult()
        _index = 0
        _stop_action = False

        def __init__(self):
                self._as=actionlib.SimpleActionServer("ElvezProcess_as", process_UIAction, self.goal_callback, False) #Defines the action server: Name of the server, type and callback function
                self._as.register_preempt_callback(self.preempt_callback)
                self._as.start() #Starts the action server, so it can be called from the client

        def preempt_callback(self):
                global stop_mov
                global modeUI
                print("Preempted in: " + str(self._index))
                self._stop_action = True
                self._as.set_preempted()                
                arm_left.stop()
                arm_right.stop()
                arms.stop()
                torso.stop()
                #pub_simulation_state.publish("Stopped")
                stop_mov = True
                msg = String()
                msg.data = "Idle"
                modeUI="Idle"
                mode_publisher.publish(msg)

        def publish_feedback(self, message=""):
                global step1
                global step2
                self._feedback.index = self._index
                self._feedback.subindex = step1
                self._feedback.subindex2 = step2
                self._feedback.message = message
                self._as.publish_feedback(self._feedback)

        def goal_callback(self, goal):
                #This is the function called when the server receives a call from the client, and it receives the info of the goal topic, sent from the client
                global stop_mov
                global step1
                global step2
                global modeUI
                self._stop_action = False
                stop_mov = False
                r=rospy.Rate(1) #1Hz
                success=True
                self._index = goal.index
                step1 = goal.subindex
                step2 = goal.subindex2
                msg = String()
                msg.data = "Running"
                modeUI = "Running"
                mode_publisher.publish(msg)
                print("STEP1: " +str(step1)+" STEP2: " +str(step2))
                if goal.index == 0 and step1==0 and step2==0:
                        move_home()
                        step2 = 0
                if goal.auto:
                        while self._index < len(ops_info):
                                if self._stop_action:
                                        success = False
                                        break
                                else:
                                        execute_operation(ops_info[self._index])
                                        if not stop_mov:
                                                msg_log = String()
                                                msg_log.data = "Operation " + str(self._index) + " finished"
                                                logs_publisher.publish(msg_log)
                                                print(str(self._index) + " executed")
                                                self._index+=1
                                                step1 = 0
                                        else:
                                              print("Paused in: " + str(self._index) + ", step1: " + str(step1) + ", step2: " + str(step2)) 
                                        self.publish_feedback()
                                        rospy.sleep(1)
                else:
                        execute_operation(ops_info[self._index])
                        if not stop_mov:
                                msg_log = String()
                                msg_log.data = "Operation " + str(self._index) + " finished"
                                logs_publisher.publish(msg_log)
                                print(str(self._index) + " executed")
                                self._index+=1
                                step1 = 0
                        else:
                                print("Paused in: " + str(self._index) + ", step1: " + str(step1) + ", step2: " + str(step2)) 
                        self.publish_feedback() 

                msg.data = "Idle"
                modeUI = "Idle"
                mode_publisher.publish(msg)
                self.publish_feedback("done") 

                if success:
                        if not stop_mov and goal.auto:
                                msg_log = String()
                                msg_log.data = "Process execution finished successfully"
                                logs_publisher.publish(msg_log)
                        self._result.success = success
                        rospy.loginfo('Successful test')
                        self._as.set_succeeded(self._result) #Set the action as succeded and sent the result to the client


def all_operations_callback(req): 
        """
        Service that returns operations to perform
        """
        global ops_info_text
        resp = StringArrayResponse()
        resp.success = False
    
        for op in ops_info_text:
                resp.msg.append(op)

        resp.success = True
        return resp

rospy.Service('/process/all_operations', StringArray, all_operations_callback)


def ATC_callback(req):
        global modeUI
        global step2
        new_tool_name = 'EEF_'+req.new_tool
        if ATC1.EEF_right != new_tool_name:
                msg = String()
                msg.data = "Running"
                modeUI="Running"
                mode_publisher.publish(msg)
                changed_tool, success = ATC1.changeTool(new_tool_name, req.arm_side)
                step2 = 0
                if success:
                        msg = ATC_msg()
                        msg.new_tool = changed_tool
                        msg.arm_side = req.arm_side
                        tool_publisher.publish(msg)
        msg = String()
        msg.data = "Idle"
        modeUI="Idle"
        mode_publisher.publish(msg)

rospy.Subscriber('/UI/ATC', ATC_msg, ATC_callback) 

##################################################################################################################
if __name__ == '__main__':
        #IMPORTANT: Modify all the named target poses to match the poses defined in your SRDF file
        #Define parameters of the tools for the ATC objects

        process_actionserver = ElvezProcessClass()

        eef_link_left = "left_tool_exchanger"
        touch_links_left = ["left_tool_exchanger"]
        eef_link_right = "right_tool_exchanger"
        touch_links_right = ["right_tool_exchanger"]

        gripper_left_ATC_frame = PyKDL.Frame()
        gripper_left_ATC_frame.p = PyKDL.Vector(0.750469, 0.1866, 1.47921) #From the cell
        gripper_left_ATC_frame.M.DoRotY(1.57)
        gripper_left_ATC_frame.M.DoRotZ(3.14)
        gripper_left_ATC_pose = frame_to_pose(gripper_left_ATC_frame)

        gripper_right_ATC_frame = PyKDL.Frame()
        gripper_right_ATC_frame.p = PyKDL.Vector(0.8, -0.3, 1.5)
        gripper_right_ATC_frame.M.DoRotY(1.57)
        gripper_right_ATC_frame.M.DoRotZ(3.14)
        gripper_right_ATC_pose = frame_to_pose(gripper_right_ATC_frame)

        gripper_end_frame_L = PyKDL.Frame() 
        gripper_end_frame_L.p = PyKDL.Vector(0.094, 0, 0.2617)#(0.0998, 0, 0.2356) #Center pad with ATC Z+0.068-0,013 (z= 252.7 + 0.009=261.7)
        gripper_end_frame_L2 = PyKDL.Frame()
        gripper_end_frame_L2.p = PyKDL.Vector(0.094, 0, 0.275)#(0.0998, 0, 0.25) #Nail with ATC Z+0.068-0,013 (z+ 0.009=0.275)
        # gripper_end_frame_L = PyKDL.Frame() 
        # gripper_end_frame_L.p = PyKDL.Vector(0.094, 0, 0.1977)
        # gripper_end_frame_L2 = PyKDL.Frame()
        # gripper_end_frame_L2.p = PyKDL.Vector(0.094, 0, 0.21)
        
        gripper_end_frame_R = PyKDL.Frame() 
        gripper_end_frame_R.p = PyKDL.Vector(0.094, 0, 0.1977) #Center pad
        gripper_end_frame_R2 = PyKDL.Frame() 
        gripper_end_frame_R2.p = PyKDL.Vector(0.094, 0, 0.21) #Nail

        fingers_thickness = 0.013 #xleft_ATC_dist
        fingers_width = 0.034 #y when fingers closed
        fingers_tip_z = 0.0153 #z distance from the fingers center to the tip
        fingers_dim = [fingers_thickness, fingers_width, fingers_tip_z]

        # gun_ATC_frame = PyKDL.Frame()
        # gun_ATC_frame.p = PyKDL.Vector(0.782896, -0.0554083, 1.52741)
        # gun_ATC_frame.M.DoRotY(1.57)
        # gun_ATC_frame.M.DoRotZ(3.14)
        # gun_ATC_pose = frame_to_pose(gun_ATC_frame)
        # gun_end_frame = PyKDL.Frame()
        # gun_end_frame.p = PyKDL.Vector(0.105, -0.055, 0.11) #Deepness, Distance to wall, Distance to guide
        # gun_end_frame.M.DoRotY(1.57)
        gun_ATC_frame = PyKDL.Frame()
        gun_ATC_frame.p = PyKDL.Vector(0.6827, -0.0295, 1.52)
        gun_ATC_frame.M.DoRotY(1.57)
        gun_ATC_frame.M.DoRotZ(3.14)
        gun_ATC_pose = frame_to_pose(gun_ATC_frame)
        gun_end_frame = PyKDL.Frame()
        gun_end_frame.p = PyKDL.Vector(-0.01, -0.071, 0.23463) #Deepness, Distance to wall, Distance to guide
        gun_end_frame.M.DoRotZ(3.141592)
        
        #EE_file_path_gripper_L = str(rospack.get_path('motoman_sda10f_support')) + '/meshes/EE/gripper_side_simplified_left_noATC_v4.stl'
        EE_file_path_gripper_L = str(rospack.get_path('motoman_sda10f_support')) + '/meshes/EE/gripper_side_simplified_left_ATC_v4_ROS.stl'
        EE_file_path_gripper_R = str(rospack.get_path('motoman_sda10f_support')) + '/meshes/EE/gripper_side_simplified_right_noATC_v4_ROS.stl'
        #EE_file_path_gun = str(rospack.get_path('motoman_sda10f_support')) + '/meshes/EE/gun_simplified_ros.stl'
        EE_file_path_gun = str(rospack.get_path('motoman_sda10f_support')) + '/meshes/EE/gun_vert_v3_ROS.stl'

        gripper_left = EEF(EE_end_frame = gripper_end_frame_L, EE_end_frame2=gripper_end_frame_L2, x = 0.073, y = 0.025, z = 0.24, fingers_dim = fingers_dim, ATC_frame = gripper_left_ATC_frame, name = "EEF_gripper_left", path = EE_file_path_gripper_L)
        gripper_right = EEF(EE_end_frame = gripper_end_frame_R, EE_end_frame2=gripper_end_frame_R2, x = 0.073, y = 0.025, z = 0.24, fingers_dim = fingers_dim, ATC_frame = gripper_right_ATC_frame, name = "EEF_gripper_right", path = EE_file_path_gripper_R)
        gun = EEF(EE_end_frame = gun_end_frame, x = 0.35, y = 0.3, z = 0.1, ATC_frame = gun_ATC_frame, name = "EEF_taping_gun", path = EE_file_path_gun)

        # Moving to initial position
        if not real_robot:
                move_home()

        #Define the ATC instance
        max_arms_dist = frame_to_pose(gripper_end_frame_L).position.x + frame_to_pose(gripper_end_frame_R).position.x + fingers_thickness + 0.05
        ATC1 = ATC(left_tool=gripper_left, right_tool=gripper_right, eef_link_left=eef_link_left, eef_link_right=eef_link_right, ATC_tools=[gun], left_ATC_angle=0.7854, right_ATC_angle=-0.7854, left_ATC_dist=0.083, right_ATC_dist=0.054)
        #ATC1 = ATC(left_tool=gripper_left, right_tool=gripper_right, eef_link_left=eef_link_left, eef_link_right=eef_link_right, ATC_tools=[], left_ATC_angle=0.7854, right_ATC_angle=-0.7854, left_ATC_dist=0.054, right_ATC_dist=0.054)
        #ATC1 = ATC(left_tool=gun, right_tool=gripper_right, eef_link_left=eef_link_left, eef_link_right=eef_link_right, ATC_tools=[gripper_left], left_ATC_angle=0.7854, right_ATC_angle=-0.7854, left_ATC_dist=0.083, right_ATC_dist=0.054)

        ##### Define target points
        #Get all operations: Pick cable from combs, Place cable in guide, Route cable through guide 1 and 2
        rospy.wait_for_service('ELVEZ_platform_handler/all_operations')
        my_service = rospy.ServiceProxy('ELVEZ_platform_handler/all_operations', all_operations)
        opsReq = all_operationsRequest()
        opsResult = my_service(opsReq)

        #Gets info about the components (jigs and combs) and cables involved
        ops_info = []
        ops_info_text = []
        last_PC_spot = {}
        for ops in opsResult.data:
                ops_temp = {}
                ops_info_text_temp = ""
                ops_temp['type'] = ops.type
                if ops.type == "EC":
                        rospy.wait_for_service('/ELVEZ_platform_handler/guide_info')
                        my_service = rospy.ServiceProxy('/ELVEZ_platform_handler/guide_info', guide_info)
                        guideReq = guide_infoRequest()
                        guideReq.jig = ops.spot[0].jig
                        guideReq.guide = ops.spot[0].id
                        guideResult = my_service(guideReq)
                        guide = guideResult.data
                        ops_temp['spot'] = {'name': str(ops.label[0]), 'jig': ops.spot[0].jig, 'id': ops.spot[0].id, 'side': ops.spot[0].side, 'pose_corner': guide.key_corner_frame, 'gap': guide.key_gap, 'width': guide.key_length, 'height': guide.key_height, 'dimensions': guide.dimensions}
                        ops_info_text_temp = 'Pick ' + str(ops.label[0]) + " from " + str(ops.spot[0].jig)

                elif ops.type == "PC":
                        rospy.wait_for_service('/ELVEZ_platform_handler/guide_info')
                        my_service = rospy.ServiceProxy('/ELVEZ_platform_handler/guide_info', guide_info)
                        guideReq = guide_infoRequest()
                        guideReq.jig = ops.spot[0].jig
                        guideReq.guide = ops.spot[0].id
                        guideResult = my_service(guideReq)
                        guide = guideResult.data
                        ops_temp['spot'] = {'name': str(ops.label[0]), 'jig': ops.spot[0].jig, 'id': ops.spot[0].id, 'side': ops.spot[0].side, 'pose_corner': guide.key_corner_frame, 'gap': guide.key_gap, 'width': guide.key_length, 'height': guide.key_height, 'height_corner': guide.key_height_corner, 'collisions': guide.collisions, 'dimensions': guide.dimensions}
                        last_PC_spot = copy.deepcopy(ops_temp['spot'])
                        ops_info_text_temp = 'Place connector ' + str(ops.label[0]) + " in " + str(ops.spot[0].jig)

                elif ops.type == "RC":
                        ###Spot
                        rospy.wait_for_service('/ELVEZ_platform_handler/guide_info')
                        my_service = rospy.ServiceProxy('/ELVEZ_platform_handler/guide_info', guide_info)
                        ops_temp['spot'] = []
                        spot_text_temp = ''
                        for spot_i in ops.spot:
                                guideReq = guide_infoRequest()
                                guideReq.jig = spot_i.jig
                                guideReq.guide = spot_i.id
                                guideResult = my_service(guideReq)
                                guide = guideResult.data
                                ops_temp['spot'].append({'jig': spot_i.jig, 'id': spot_i.id, 'pose_corner': guide.key_corner_frame, 'gap': guide.key_gap, 'width': guide.key_length, 'height': guide.key_height, 'height_corner': guide.key_height_corner, 'collisions': guide.collisions, 'dimensions': guide.dimensions})
                                spot_text_temp += str(spot_i.jig) + "-"
                        
                        ops_info_text_temp = 'Route cables of ' + str(ops.label[0]) + " along " + str(spot_text_temp[:-1])
                                
                        ###Label
                        ops_temp['label'] = ops.label[0]
                        rospy.wait_for_service('ELVEZ_platform_handler/connector_info')
                        my_service_con = rospy.ServiceProxy('ELVEZ_platform_handler/connector_info', connector_info)
                        rospy.wait_for_service('ELVEZ_platform_handler/cable_info')
                        my_service_cab = rospy.ServiceProxy('ELVEZ_platform_handler/cable_info', cable_info)
                        
                        if ops.label[0][:2]=='WH':
                                #route cable group
                                ops_temp['label'] = ops.label[0]
                        else:
                                conReq = connector_infoRequest()
                                conReq.label = ops.label[0]
                                conResult = my_service_con(conReq)
                                cablesGroup = []
                                for cable in conResult.cables:
                                        cablesGroup.append(cable.label)
                                conReq.label = 'WH'+str(conResult.WH)
                                conResult = my_service_con(conReq)
                                cablesWH = {}
                                cablesGroupIndex = []
                                for cable in conResult.cables:
                                        if cable.label in cablesGroup:
                                                cablesGroupIndex.append(int(cable.pins[0]))
                                        cablesWH[cable.label]=int(cable.pins[0])
                                if max(cablesGroupIndex) < (len(cablesWH)-1):
                                        #separate upper cables and place them in another guide
                                        sub_op_temp = {}
                                        sub_op_temp['type'] = 'SC'
                                        sub_op_temp['WH'] = str(conResult.WH)
                                        sub_op_temp['label'] = list(range(max(cablesGroupIndex)+1, len(cablesWH)))
                                        sub_op_temp['spot'] = []
                                        sub_op_temp['spot'].append(last_PC_spot)
                                        guideReq = guide_infoRequest()
                                        guideReq.jig = 'J'+str(conResult.WH)+'S' #DEFINE THIS ELEMENT
                                        guideReq.guide = '1'
                                        guideResult = my_service(guideReq)
                                        guide = guideResult.data
                                        sub_op_temp['spot'].append({'pose_corner': guide.key_corner_frame, 'gap': guide.key_gap, 'width': guide.key_length, 'height': guide.key_height, 'height_corner': guide.key_height_corner, 'collisions': guide.collisions, 'dimensions': guide.dimensions})
                                        ops_info.append(sub_op_temp)
                                        ops_info_text.append('Separate cables: ' + str(sub_op_temp['label']) + " in J" + str(conResult.WH) + "S")
                                        #grasp cable group from the first route position
                                        sub_op_temp = {}
                                        sub_op_temp['type'] = 'GC'
                                        sub_op_temp['label'] = ops.label[0]
                                        #sub_op_temp['spot'] = ops_temp['spot'][0]
                                        sub_op_temp['spot'] = last_PC_spot
                                        ops_info.append(sub_op_temp)
                                        ops_info_text.append('Grasp cable group ' + str(ops.label[0]) + " from " + str(sub_op_temp['spot']['jig']))
                                        #route cable group
                                        ops_temp['label'] = ops.label[0]
                                else:
                                        #grasp cable group from the separation guide
                                        sub_op_temp = {}
                                        sub_op_temp['type'] = 'GCS'
                                        sub_op_temp['label'] = ops.label[0]
                                        guideReq = guide_infoRequest()
                                        guideReq.jig = 'J'+str(conResult.WH)+'S'
                                        guideReq.guide = '1'
                                        guideResult = my_service(guideReq)
                                        guide = guideResult.data
                                        sub_op_temp['spot']={'pose_corner': guide.key_corner_frame, 'gap': guide.key_gap, 'width': guide.key_length, 'height': guide.key_height, 'height_corner': guide.key_height_corner, 'collisions': guide.collisions, 'dimensions': guide.dimensions}
                                        ops_info.append(sub_op_temp)
                                        ops_info_text.append('Grasp cable group ' + str(ops.label[0]) + " from J" + str(conResult.WH) + "S")
                                        #route cable group
                                        ops_temp['label'] = ops.label[0]
                elif ops.type == "T":
                        rospy.wait_for_service('/ELVEZ_platform_handler/guide_info')
                        my_service = rospy.ServiceProxy('/ELVEZ_platform_handler/guide_info', guide_info)
                        guideReq = guide_infoRequest()
                        ops_temp['spot'] = []
                        for spot_i in ops.spot:
                                guideReq.jig = spot_i.jig
                                guideReq.guide = spot_i.id
                                guideResult = my_service(guideReq)
                                guide = guideResult.data
                                ops_temp['spot'].append({'jig': ops.spot[0].jig, 'id': ops.spot[0].id, 'side': ops.spot[0].side, 'pose_corner': guide.key_corner_frame, 'gap': guide.key_gap, 'width': guide.key_length, 'height': guide.key_height, 'height_corner': guide.key_height_corner, 'collisions': guide.collisions, 'dimensions': guide.dimensions})
                        text_spot_T = str(ops.spot[0].jig) + ' and ' + str(ops.spot[1].jig)
                        ops_info_text_temp = 'Tape cables between ' + str(text_spot_T)
                        
                # con_temp = []
                # cab_temp = {}
                # for label in ops.label:
                #         if label[:2] != 'CA':
                #                 con_temp.append(label) 
                #                 conReq = connector_infoRequest()
                #                 conReq.label = label
                #                 conResult = my_service_con(conReq)
                #                 for cable in conResult.cables:
                #                         cabReq = cable_infoRequest()
                #                         cabReq.label = cable.label
                #                         cabResult = my_service_cab(cabReq)
                #                         cab_temp[cable.label] = cabResult.diameter
                #         else:
                #                 if not 'Free' in con_temp:
                #                         con_temp.append('Free')
                #                 cabReq = cable_infoRequest()
                #                 cabReq.label = label
                #                 cabResult = my_service_cab(cabReq)
                #                 cab_temp[label] = cabResult.diameter                
                # #connector cables grasped in the second end
                # ops_temp['label'] = {'con': con_temp, 'cab': cab_temp}
                ops_info.append(ops_temp)
                ops_info_text.append(ops_info_text_temp)

        #actuate_grippers(open_distance, gripper_speed, 'both', grasp=False)

        #print(ops_info)

        #For this specific application
        route_arm = "right"
        route_group = "arm_right"
        group2 = "arm_left"
        arm = arm_right
        arm2 = arm_left
        EEF_route = ATC1.EEF_dict[ATC1.EEF_right].fingers_dim
        move_away2_sign = -1
        count = 0

        rospy.spin()