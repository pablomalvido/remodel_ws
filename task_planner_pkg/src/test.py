#! /usr/bin/env python

import copy
import rospy
import PyKDL 
import csv
import rospkg
from moveit_msgs.msg import *
from moveit_msgs.srv import *
import geometry_msgs.msg
from utils import *
from elvez_pkg.msg import *
from elvez_pkg.srv import *
from task_planner_pkg.msg import *
from task_planner_pkg.srv import *
import actionlib
from task_planner_pkg.msg import EEFActAction, EEFActResult, EEFActFeedback, EEFActGoal
from task_planner_pkg.msg import ExecutePlanAction, ExecutePlanGoal, ExecutePlanFeedback

rospack = rospkg.RosPack()
rospy.init_node('test_node', anonymous=True)

#################### CONFIGURATION #########################
config_full_pkg_path = str(rospack.get_path('ROS_UI_backend'))
config_file_name = config_full_pkg_path + "/files/config.csv"
config1 = {}
config2 = {}
try:
        with open(config_file_name) as csvfile:
                reader = csv.DictReader(csvfile)
                for row in reader:
                        config1[row['prop']] = row['value']
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

print(config1)
execute_grippers=False
use_camera=False

try:
        if real_robot:
                print('REAL ROBOT')
                if config1['grippers_control_real'] == 'Y' or config1['grippers_control_real'] == 'y':
                        config2['execute_grippers'] = True #True
                else:
                        config2['execute_grippers'] = False
                if config1['gun_control_real'] == 'Y' or config1['gun_control_real'] == 'y':
                        config2['execute_gun'] = True #True
                else:
                        config2['execute_gun'] = False
                if config1['force_control_real'] == 'Y' or config1['force_control_real'] == 'y':
                        config2['force_control_active'] = True #True
                else:
                        config2['force_control_active'] = False
                if config1['use_camera_real'] == 'Y' or config1['use_camera_real'] == 'y':
                        config2['use_camera'] = True #True
                else:
                        config2['use_camera'] = False
                        print(execute_grippers)
                        print(use_camera)
                config2['speed_limit'] = min(float(config1['speed_real_per']),0.3) #0.05
                config2['fast_speed_execution'] = min(float(config1['speed_fast_real_mms']),100) #40mm/s
                config2['speed_execution'] = min(float(config1['speed_real_mms']),80) #20mm/s
                config2['slow_speed_execution'] = min(float(config1['speed_slow_real_mms']),40) #10mm/s
                config2['speed_tension'] = min(float(config1['speed_tension_real_mms']),40) #10mm/s
        else:
                print('SIMULATED ROBOT')
                if config1['grippers_control_demo'] == 'Y' or config1['grippers_control_demo'] == 'y':
                        config2['execute_grippers'] = True #False
                else:
                        config2['execute_grippers'] = False
                if config1['use_camera_demo'] == 'Y' or config1['use_camera_demo'] == 'y':
                        config2['use_camera'] = True #False
                else:
                        config2['use_camera'] = False
                config2['force_control_active'] = False
                config2['execute_gun'] = False
                config2['speed_limit'] = float(config1['speed_demo_per']) #1
                config2['fast_speed_execution'] = float(config1['speed_fast_demo_mms']) #100mm/s
                config2['speed_execution'] = float(config1['speed_demo_mms']) #50mm/s
                config2['slow_speed_execution'] = float(config1['speed_slow_demo_mms']) #30mm/s
                config2['speed_tension'] = float(config1['speed_tension_demo_mms']) #20mm/s
        
        #Offsets
        config2['z_offset'] = float(config1['offset_z'])/1000 #0.02
        config2['x_offset'] = float(config1['offset_x'])/1000 #0.02
        config2['grasp_offset'] = float(config1['offset_grasp'])/1000 #0.005
        config2['force_offset'] = float(config1['offset_force'])/1000 #0.01
        #pick_grasp_offset = float(config['offset_pick_grasp'])/1000 #0.015
        pick_grasp_offset = {}
        pick_grasp_offset['WH1'] = 0.015
        pick_grasp_offset['WH3'] = 0.0255
        config2['pick_grasp_offset'] = pick_grasp_offset
        config2['z_offset_pick'] = float(config1['offset_pick_z'])/1000 #0.05
        config2['z_offset_photo'] = float(config1['offset_photo_z'])/1000 #0.1
        config2['gun_nozzle_offset'] = 0.1

        rot_center = Pose()
        rot_center_up = False

        #Force sensor
        force_cable = {}
        force_connector = {}
        force_cable['WH1'] = float(config1['cable_tension_wh1']) #3.5N
        force_cable['WH3'] = float(config1['cable_tension_wh3']) #3.5N
        force_connector['WH1'] = float(config1['connector_tension_wh1']) #5N
        force_connector['WH3'] = float(config1['connector_tension_wh3']) #5N
        config2['force_cable'] = force_cable
        config2['force_connector'] = force_connector

        #Gripper parameters
        config2['open_distance'] = float(config1['gripper_open_dist']) #105
        config2['slide_distance'] = float(config1['gripper_slide_dist']) #36
        config2['grasp_distance'] = float(config1['gripper_grasp_dist']) #30
        config2['gripper_speed'] = float(config1['gripper_speed']) #30
        config2['gripper_speed_slow'] = float(config1['gripper_speed_slow']) #20
except:
        print("Error defining config robot values")
        exit()

######################## INITIALIZATION ##############################

rospy.wait_for_service('/adv_manip/define_motion_groups')
def_motion_groups_srv = rospy.ServiceProxy('/adv_manip/define_motion_groups', DefMotionGroups)
rospy.wait_for_service('/adv_manip/max_speed_groups')
def_max_speed_srv = rospy.ServiceProxy('/adv_manip/max_speed_groups', MaxSpeedGroups)
rospy.wait_for_service('/adv_manip/def_EEF')
def_EEF_srv = rospy.ServiceProxy('/adv_manip/def_EEF', DefEEF)
rospy.wait_for_service('/adv_manip/def_ATC')
def_ATC_srv = rospy.ServiceProxy('/adv_manip/def_ATC', DefATC)

motion_groups = ["arm_left", "arm_right", "arms", "torso"]
req = DefMotionGroupsRequest()
req.left = "arm_left"; req.right = "arm_right"; req.arms = "arms"; req.torso = "torso"
def_motion_groups_srv(req)

req = MaxSpeedGroupsRequest()
req.speed = 0.2
for group in motion_groups:
    req.group = group
    def_max_speed_srv(req)

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
gripper_end_frame_R = PyKDL.Frame() 
gripper_end_frame_R.p = PyKDL.Vector(0.094, 0, 0.1977) #Center pad
gripper_end_frame_R2 = PyKDL.Frame() 
gripper_end_frame_R2.p = PyKDL.Vector(0.094, 0, 0.21) #Nail

fingers_thickness = 0.013 #xleft_ATC_dist
fingers_width = 0.034 #y when fingers closed
fingers_tip_z = 0.0153 #z distance from the fingers center to the tip
fingers_dim = [fingers_thickness, fingers_width, fingers_tip_z]

gun_ATC_frame = PyKDL.Frame()
gun_ATC_frame.p = PyKDL.Vector(0.6827, -0.0295, 1.52)
gun_ATC_frame.M.DoRotY(1.57)
gun_ATC_frame.M.DoRotZ(3.14)
gun_ATC_pose = frame_to_pose(gun_ATC_frame)
gun_end_frame = PyKDL.Frame()
gun_end_frame.p = PyKDL.Vector(-0.01, -0.071, 0.23463) #Deepness, Distance to wall, Distance to guide
gun_end_frame.M.DoRotZ(3.141592)

eef_link_left = "left_tool_exchanger"
touch_links_left = ["left_tool_exchanger"]
eef_link_right = "right_tool_exchanger"
touch_links_right = ["right_tool_exchanger"]

EE_file_path_gripper_L = str(rospack.get_path('motoman_sda10f_support')) + '/meshes/EE/gripper_side_simplified_left_ATC_v4_ROS.stl'
EE_file_path_gripper_R = str(rospack.get_path('motoman_sda10f_support')) + '/meshes/EE/gripper_side_simplified_right_noATC_v4_ROS.stl'
EE_file_path_gun = str(rospack.get_path('motoman_sda10f_support')) + '/meshes/EE/gun_vert_v3_ROS.stl'

req = DefEEFRequest()
req.EE_end_frame = frame_to_pose(gripper_end_frame_L); req.EE_end_frame2 = frame_to_pose(gripper_end_frame_L2); req.x = 0.073; req.y = 0.025; req.z = 0.24; req.fingers_dim = fingers_dim; req.ATC_frame = frame_to_pose(gripper_left_ATC_frame); req.name = "EEF_gripper_left"; req.path = EE_file_path_gripper_L
def_EEF_srv(req)
req.EE_end_frame = frame_to_pose(gripper_end_frame_R); req.EE_end_frame2 = frame_to_pose(gripper_end_frame_R2); req.x = 0.073; req.y = 0.025; req.z = 0.24; req.fingers_dim = fingers_dim; req.ATC_frame = frame_to_pose(gripper_right_ATC_frame); req.name = "EEF_gripper_right"; req.path = EE_file_path_gripper_R
def_EEF_srv(req)
req.EE_end_frame = frame_to_pose(gun_end_frame); req.EE_end_frame2 = Pose(); req.x = 0.35; req.y = 0.3; req.z = 0.1; req.fingers_dim = []; req.ATC_frame = frame_to_pose(gun_ATC_frame); req.name = "EEF_taping_gun"; req.path = EE_file_path_gun
def_EEF_srv(req)

req = DefATCRequest()
req.left_tool="EEF_gripper_left"; req.right_tool="EEF_gripper_right"; req.eef_link_left=eef_link_left; req.eef_link_right=eef_link_right; req.ATC_tools=["EEF_taping_gun"]; req.left_ATC_angle=0.7854; req.right_ATC_angle=-0.7854; req.left_ATC_dist=0.083; req.right_ATC_dist=0.054
def_ATC_srv(req)

rospy.wait_for_service('/EEF/init')
init_eef_srv = rospy.ServiceProxy('/EEF/init', EEFInit)
req = EEFInitRequest()
req.grasp_distance = config2['grasp_distance']; req.slide_distance = config2['slide_distance']; req.exe = config2['execute_grippers']
init_eef_srv(req)

#################################### ROBOT OPERATIONS ####################################

def EC(op, step2=0, config=[], route_group=""):

        global grasping_cables

        step1_increase = 0
        grasping_cables = False
        print("Pick connector")
        print("Step2: "+str(step2))
        if op['spot']['side'] == "R":
                pick_arm = "right"
        else:
                pick_arm = "left"
        fingers_size = get_fingers_size(pick_arm)
        main_arm = 'arm_'+pick_arm

        prepick_pose = 'arm_'+pick_arm+'_prepick'
        if op["spot"]["name"] == "WH3":
                pick_pose = get_shifted_pose(op["spot"]["pose_corner"], [op["spot"]['width']/2, - fingers_size[0]/2 - config['pick_grasp_offset'][op["spot"]["name"]] - config['grasp_offset'], op["spot"]["height"]/2 + 0.01, 0, 0, 0])
        else:
                pick_pose = get_shifted_pose(op["spot"]["pose_corner"], [op["spot"]['width']/2, - fingers_size[0]/2 - config['pick_grasp_offset'][op["spot"]["name"]] - config['grasp_offset'], op["spot"]["height"]/2, 0, 0, 0])
        pick_pose = get_shifted_pose(pick_pose, [0, 0, 0, 0, 0, -1.5708])
        pick_pose_offset = get_shifted_pose(pick_pose, [0, 0, op["spot"]["height"]/2 + config['z_offset_pick'], 0, 0, 0])

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
                plan, success = compute_cartesian_path_velocity_control([waypoints_EC1], [config['fast_speed_execution']], arm_side=pick_arm)
                if success:
                        step2+=execute_plan_async(route_group, plan)

        if step2 == 4:
                actuate_grippers(config['open_distance'], config['gripper_speed'], pick_arm, config, grasp=False)
                waypoints_EC2 = []
                init_pose = get_current_pose(main_arm).pose
                waypoints_EC2.append(init_pose)
                waypoints_EC2.append(correctPose(pick_pose, pick_arm, rotate = True, ATC_sign = -1, picking_app = True))
                plan, success = compute_cartesian_path_velocity_control([waypoints_EC2], [config['slow_speed_execution']], arm_side=pick_arm)
                if success:
                        step2+=execute_plan_async(route_group, plan)

        if step2 == 5:
                actuate_grippers(config['grasp_distance'], config['gripper_speed'], pick_arm, config, grasp=True)
                rospy.sleep(1.0)
                waypoints_EC3 = []
                init_pose = get_current_pose(main_arm).pose
                waypoints_EC3.append(init_pose)
                waypoints_EC3.append(correctPose(pick_pose_offset, pick_arm, rotate = True, ATC_sign = -1, picking_app = True))
                plan, success = compute_cartesian_path_velocity_control([waypoints_EC3], [config['slow_speed_execution']], arm_side=pick_arm)
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


########################## ADVANCED MANIPULATION FUNCTIONS #######################################

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

def correctPose(target_pose, arm_side, rotate = False, ATC_sign = -1, routing_app = False, route_arm = True, picking_app = False, secondary_frame = False):
        rospy.wait_for_service('/adv_manip/correct_pose')
        correct_srv = rospy.ServiceProxy('/adv_manip/correct_pose', GetCorrectPose)
        req = GetCorrectPoseRequest()
        req.target_pose = target_pose; req.arm_side = arm_side; req.rotate = rotate; req.ATC_sign = ATC_sign; req.routing_app = routing_app; req.route_arm = route_arm; req.picking_app = picking_app; req.secondary_frame = secondary_frame
        return correct_srv(req).pose

def get_current_pose(group):
        rospy.wait_for_service('/adv_manip/get_current_pose')
        get_pose_srv = rospy.ServiceProxy('/adv_manip/get_current_pose', GetGroupPose)
        req = GetGroupPoseRequest()
        req.group = group
        return get_pose_srv(req).poseSt

def compute_cartesian_path_velocity_control(waypoints_list, EE_speed, EE_ang_speed = [], arm_side = "left", max_linear_accel = 200.0, max_ang_accel = 140.0, step = 0.002):
        rospy.wait_for_service('/adv_manip/compute_cartesian_path_velocity_control')
        compute_path_srv = rospy.ServiceProxy('/adv_manip/compute_cartesian_path_velocity_control', ComputePath)
        req = ComputePathRequest()
        for list_wp in waypoints_list:
                msg = float_list()
                for wp in list_wp:
                        msg.data.append(wp)
                #msg.data = list_wp
                req.waypoints_list.append(msg), 
        req.EE_speed = EE_speed; req.EE_ang_speed = EE_ang_speed; req.arm_side = arm_side; req.max_linear_accel = max_linear_accel; req.max_ang_accel = max_ang_accel; req.step = step
        resp = compute_path_srv(req)
        return resp.plan, resp.success

"""
def move_group_async(group):
        rospy.wait_for_service('/adv_manip/move_group_async')
        move_async_srv = rospy.ServiceProxy('/adv_manip/move_group_async', ExecuteMovement)
        req = ExecuteMovementRequest()
        req.group = group
        return move_async_srv(req).step_inc

def execute_plan_async(group, plan):
        rospy.wait_for_service('/adv_manip/execute_plan_async')
        move_async_srv = rospy.ServiceProxy('/adv_manip/execute_plan_async', ExecuteMovement)
        req = ExecuteMovementRequest()
        req.group = group
        req.plan = plan
        return move_async_srv(req).step_inc
"""

step_inc_fb = 0
async_move_done=False
def async_move_feedback_callback(fb):
        global async_move_done
        global step_inc_fb
        if fb.done:
                async_move_done = True
                step_inc_fb = fb.step_inc

def move_group_async(group):
        global async_move_done
        global step_inc_fb
        client=actionlib.SimpleActionClient('/adv_manip/move_group_async', ExecutePlanAction)
        client.wait_for_server()
        goal = ExecutePlanGoal()
        goal.group = group
        client.send_goal(goal, feedback_cb=async_move_feedback_callback)
        while not async_move_done:
                rospy.sleep(0.05)
        async_move_done = False
        return step_inc_fb

async_plan_done=False
def async_plan_feedback_callback(fb):
        global async_plan_done
        global step_inc_fb
        if fb.done:
                async_plan_done = True
                step_inc_fb = fb.step_inc

def execute_plan_async(group, plan):
        global async_plan_done
        global step_inc_fb
        client=actionlib.SimpleActionClient('/adv_manip/execute_plan_async', ExecutePlanAction)
        client.wait_for_server()
        goal = ExecutePlanGoal()
        goal.group = group; goal.plan = plan
        client.send_goal(goal, feedback_cb=async_plan_feedback_callback)
        while not async_plan_done:
                rospy.sleep(0.05)
        async_plan_done = False
        return step_inc_fb

def get_fingers_size(side):
        rospy.wait_for_service('/adv_manip/get_fingers_size')
        fingers_srv = rospy.ServiceProxy('/adv_manip/get_fingers_size', FingersDim)
        req = FingersDimRequest()
        req.side = side
        return fingers_srv(req.side).data

eef_done=False
def eef_feedback_callback(fb):
        global eef_done
        if fb.done:
                eef_done = True

def actuate_eef(type, distance=0, speed=0, arm="left", grasp=False):
        global eef_done
        client=actionlib.SimpleActionClient('/EEF/actuate', EEFActAction) #Stablishes the connection with the server
        client.wait_for_server()
        goal = EEFActGoal()
        goal.type = type; goal.distance = distance; goal.speed = speed; goal.arm = arm; goal.grasp = grasp
        client.send_goal(goal, feedback_cb=eef_feedback_callback)
        while not eef_done:
                rospy.sleep(0.05)
        eef_done = False

def actuate_grippers(distance, speed, arm, config, grasp=False):
        actuate_eef(0, distance, speed, arm, grasp)

def actuate_gun():
        actuate_eef(1)

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

##################### TASK PLANNER ######################
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


def get_next_connector_info(op):
        global ops_info
        current_index = ops_info.index(op)
        op_index = 0
        next_PC = {}
        for op_i in ops_info:
                if op_index <= current_index:
                        print("NOT PC")
                        print(op_i)
                        op_index+=1
                        continue
                if op_i['type'] == "PC":
                        print("PC")
                        print(op_i)
                        next_PC = op_i
                op_index+=1
        return next_PC


def update_route_arm_info(op):
        global route_arm
        global move_away2_sign
        global route_group
        global group2

        if op['spot']['side'] == "R":
                route_arm = "right"
                route_group = "arm_right"
                group2 = "arm_left"
                move_away2_sign = -1
        else:
                route_arm = "left"
                route_group = "arm_left"
                group2 = "arm_right"
                move_away2_sign = 1


def execute_operation(op):
        global PC_op
        global route_arm
        global move_away2_sign
        global route_group
        global group2 
        global step1
        global step2 
        global config2

        if op['type'] == "EC":
                PC_op = get_next_connector_info(op)
                update_route_arm_info(PC_op)
                step2, step1_inc = EC(op, step2, config2, route_group)              

        """    
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
        """
        step1 += step1_inc

#CAD Platform info
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
        ops_info.append(ops_temp)
        ops_info_text.append(ops_info_text_temp)

#print(ops_info)

step1 = 0
step2 = 0
execute_operation(ops_info[step1])