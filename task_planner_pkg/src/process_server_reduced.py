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

print('EXECUTE GRIPPERS: ' + str(execute_grippers))
print('EXECUTE CAMERA: ' + str(use_camera))

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


######################################################################################################################################



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