#! /usr/bin/env python

import copy
import rospy
import PyKDL 
import csv
import os
import rospkg
from std_msgs.msg import *
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
from path_planning_pkg.srv import *
from vision_pkg_full_demo.srv import *

rospack = rospkg.RosPack()
rospy.init_node('test_node', anonymous=True)
logs_publisher = rospy.Publisher('/UI/logs', String, queue_size=1)
confirmation_publisher = rospy.Publisher('/UI/confirm_req', String, queue_size=1)
feedback_publisher = rospy.Publisher('/UI/feedback', configProp, queue_size=1)
stop_publisher = rospy.Publisher('/task_planner/operation_stop', Bool, queue_size=1)

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
        #pick_grasp_offset = float(config1['offset_pick_grasp'])/1000 #0.015
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
        force_cable['WH2'] = float(config1['cable_tension_wh1']) #3.5N
        force_cable['WH3'] = float(config1['cable_tension_wh3']) #3.5N
        force_connector['WH1'] = float(config1['connector_tension_wh1']) #5N
        force_connector['WH2'] = float(config1['connector_tension_wh1']) #5N
        force_connector['WH3'] = float(config1['connector_tension_wh3']) #5N
        config2['force_cable'] = force_cable
        config2['force_connector'] = force_connector
        force_limit_connector = 0
        force_limit_cable = 0

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
#Global variables
holding_cables = False
holding_comeback_pose_corrected = Pose()
grasping_cables = False
separated_global = False #True when a cable group has been separated for routing
separation_arm = ""

#Motion groups
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

def EC(op, step2=0, config=[], route_group="", route_arm=""):

        global grasping_cables

        grasping_cables = False
        print("Pick connector")
        print("Step2: "+str(step2))
        pick_arm = copy.deepcopy(route_arm)
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
                actuate_grippers(config['open_distance'], config['gripper_speed'], pick_arm, grasp=False)
                waypoints_EC2 = []
                init_pose = get_current_pose(main_arm).pose
                waypoints_EC2.append(init_pose)
                waypoints_EC2.append(correctPose(pick_pose, pick_arm, rotate = True, ATC_sign = -1, picking_app = True))
                plan, success = compute_cartesian_path_velocity_control([waypoints_EC2], [config['slow_speed_execution']], arm_side=pick_arm)
                if success:
                        step2+=execute_plan_async(route_group, plan)

        if step2 == 5:
                actuate_grippers(config['grasp_distance'], config['gripper_speed'], pick_arm, grasp=True)
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
                step2+=move_group_async("arms")
                rospy.sleep(0.5)
        
        if step2 == 8:
                step2 = 0
                #PUBLISH FEEDBACK AFTER FINISHING EACH OPERATION
        
        return step2


def PC(op, step2=0, config=[], route_group="", route_arm=""):
        #global process_actionserver
        global force_limit_cable
        global force_limit_connector
        global grasping_cables

        print("PC")
        force_limit_connector = config["force_connector"][op['label']]
        force_limit_cable = config["force_connector"][op['label']]

        grasping_cables = False
        fingers_size = get_fingers_size(route_arm)

        #IMPLEMENT ALGORITHM TO FIND A FREE PATH TO MOVE
        if step2 == 0:
                waypoints_PC1 = [get_current_pose(route_group).pose]
                free_path, success = find_path(arm_side=route_arm, end_guide=op['spot']['jig'], wh=op['label'][-1], offset_z=(3*config['z_offset']))
                for p in free_path:
                        if compute_distance_xy(p, op['spot']['center_pose']) < 0.1:
                                print("CLOSE")    
                                print(compute_distance_xy(p, op['spot']['center_pose']))
                                break
                        print(compute_distance_xy(p, op['spot']['center_pose']))
                        waypoints_PC1.append(correctPose(p, route_arm, rotate = True, ATC_sign = -1, routing_app = True, secondary_frame = True))    
                mold_up_forward = get_shifted_pose(op["spot"]["pose_corner"], [op["spot"]['width']+config['grasp_offset']+fingers_size[0]/2, ((op["spot"]['gap']/2)), op["spot"]["height"] + 2*config['z_offset'], 0, 0, 0])
                waypoints_PC1.append(correctPose(mold_up_forward, route_arm, rotate = True, ATC_sign = -1, routing_app = True, secondary_frame = True))
                #visualize_keypoints_simple(waypoints_PC1, '/base_link')
                plan, success, motion_group_plan = compute_cartesian_path_velocity_control_arms_occlusions([waypoints_PC1], [config['fast_speed_execution']], arm_side=route_arm)
                if success:
                        step2+=execute_plan_async(motion_group_plan, plan)
                        rospy.sleep(0.5)
        
        if step2 == 1:
                waypoints_PC2 = []
                init_pose = get_current_pose(route_group).pose
                waypoints_PC2.append(init_pose)
                mold_forward = get_shifted_pose(op["spot"]["pose_corner"], [op["spot"]['width']+config['grasp_offset']+fingers_size[0]/2, ((op["spot"]['gap']/2)), 0, 0, 0, 0])
                waypoints_PC2.append(correctPose(mold_forward, route_arm, rotate = True, ATC_sign = -1, routing_app = True, secondary_frame = True))
                plan, success = compute_cartesian_path_velocity_control([waypoints_PC2], [config['slow_speed_execution']], arm_side=route_arm)
                if success:
                        step2+=execute_plan_async(route_group, plan)
                rospy.sleep(1)

        if step2 == 2:
                waypoints_PC3 = []
                init_pose = get_current_pose(route_group).pose
                waypoints_PC3.append(init_pose)
                #Connector insertion params
                combs_width = 0.001
                mold_width = 0.001
                extra_pull = 0.01
                pull_pose = get_shifted_pose(op["spot"]["pose_corner"], [op["spot"]['width']+config['grasp_offset']+fingers_size[0]/2+pick_grasp_offset[op["spot"]["name"]]+combs_width-mold_width+extra_pull, ((op["spot"]['gap']/2)), 0, 0, 0, 0])
                waypoints_PC3.append(correctPose(pull_pose, route_arm, rotate = True, ATC_sign = -1, routing_app = True, secondary_frame = True))
                plan, success = compute_cartesian_path_velocity_control([waypoints_PC3], [config['speed_tension']], arm_side=route_arm)
                if success:
                        step2+=execute_force_control(group = route_group, plan = plan, limit = force_limit_connector, force_active=config['force_control_active'])
                rospy.sleep(0.5)
                grasping_cables = True

        if step2 == 3:
                step2 = 0
                #process_actionserver.publish_feedback()
        
        return step2


def retract_arm(ret_arm, guide, config =[], special = False):
        step_inc = 0
        ret_group = 'arm_'+ret_arm
        if special:
                actuate_grippers(105, 50, ret_arm, grasp=False)
        else:
                actuate_grippers(config['open_distance'], config['gripper_speed'], ret_arm, grasp=False)
        init_pose = get_current_pose(ret_group).pose
        if special:
                retract_z = get_shifted_pose(guide["pose_corner"], [0, 0, guide["height"] + config['z_offset'] + 0.08, 0, 0, 0])
        else:
                retract_z = get_shifted_pose(guide["pose_corner"], [0, 0, guide["height"] + config['z_offset'], 0, 0, 0])
        retract_z_corrected = correctPose(retract_z, ret_arm, rotate = True, routing_app = True, ATC_sign = -1, secondary_frame=True)
        new_pose = copy.deepcopy(init_pose)
        new_pose.position.z = retract_z_corrected.position.z
        if special:
                new_pose.position.y -= 0.1
        waypoints_ret = [init_pose, new_pose]
        plan, success = compute_cartesian_path_velocity_control([waypoints_ret], [config['speed_execution']], arm_side=ret_arm)
        if success:
                step_inc = execute_plan_async(ret_group, plan)
        return step_inc


def separate_cables(op, step2=0, config=[], route_group="", route_arm=""):
        global grasp_point_global
        global grasp_angle_global
        global confirmation_received
        global confirmation_msg
        #global process_actionserver
        global holding_cables
        global force_limit_cable
        global holding_comeback_pose_corrected
        global grasping_cables
        global separated_global
        global separation_arm
        
        print("SEPARATE CABLES")
        if not op["sep_guide"]:
                separated_global = True #When the separated cable group will be routed next. Do not apply when the separated group is gonna be placed in a separation guide
        
        #Check gripper orientation
        rotate_gripper_z = False
        switch_arms = False
        next_guide_fw = get_shifted_pose(op["spot"][1]["pose_corner"], [0.01, 0, 0, 0, 0, 0])
        con_guide_fw = get_shifted_pose(op["spot"][0]["pose_corner"], [0.01, 0, 0, 0, 0, 0])
        if compute_distance_xy(op["spot"][0]["pose_corner"], next_guide_fw) < compute_distance_xy(op["spot"][0]["pose_corner"], op["spot"][1]["pose_corner"]): #Direction of routing is not forward
                #print("BACKWARD INITIALLY")
                next_guide_corner = get_shifted_pose(op["spot"][1]["pose_corner"], [op["spot"][1]["width"], op["spot"][1]["gap"], 0, 0, 0, math.pi])  
        else:
                #print("FORWARD INITIALLY")
                next_guide_corner = copy.deepcopy(op["spot"][1]["pose_corner"])      
        next_guide_fw = get_shifted_pose(next_guide_corner, [1, 0, 0, 0, 0, 0])
        angle_next_guide = math.atan2(next_guide_fw.position.y - next_guide_corner.position.y, next_guide_fw.position.x - next_guide_corner.position.x)*180/math.pi
        angle_con_guide = math.atan2(con_guide_fw.position.y - op["spot"][0]["pose_corner"].position.y, con_guide_fw.position.x - op["spot"][0]["pose_corner"].position.x)*180/math.pi
        if ((abs(angle_next_guide) > 100) or (abs(angle_con_guide) > 100)):
                if op["sep_guide"]:
                        print("ROTATE GRIPPER")
                        rotate_gripper_z = True #Rotates the gripper 180deg
                else:
                        switch_arms = True #Separate cables with arm 2
        else:
                print("DON'T ROTATE GRIPPER")

        if (route_arm == "right" and not switch_arms) or (route_arm == "left" and switch_arms):
                separation_arm = "left"
                separation_group = 'arm_left'
        else:
                separation_arm = "right"
                separation_group = 'arm_right'

        fingers_size = get_fingers_size(route_arm)
        fingers_size_sep = get_fingers_size(separation_arm)
        grasping_cables = False

        if step2 == 0:
                actuate_grippers(config['open_distance'], config['gripper_speed'], route_arm, grasp=False)
                init_pose = get_current_pose(route_group).pose
                retreat_z = get_shifted_pose(op["spot"][0]["pose_corner"], [0, 0, op["spot"][0]["height"] + config['z_offset_photo'] + fingers_size[2], 0, 0, 0])
                retreat_z_corrected = correctPose(retreat_z, route_arm, rotate = True, routing_app = True, ATC_sign = -1)
                new_pose = copy.deepcopy(init_pose)
                new_pose.position.z = retreat_z_corrected.position.z
                waypoints1 = [init_pose, new_pose]
                plan, success = compute_cartesian_path_velocity_control([waypoints1], [config['speed_execution']], arm_side=route_arm)
                print(success)
                if success:
                        step2 += execute_plan_async(route_group, plan)
                        print(step2)                                

        if step2 == 1:
                os.system('cp ' + str(rospack.get_path('vision_pkg_full_demo')) + '/imgs/error_grasp_1.jpg /home/remodel/UI-REMODEL/src/assets/img/grasp_image.jpg')
                repeat = True
                pixel_D_param = 5
                while repeat:
                        if config['use_camera']:
                                img_cables_path = capture_img()
                                if not capture_success:
                                        stop_function("Failed to take the cables image")
                        else:
                                img_cables_path = str(rospack.get_path('vision_pkg_full_demo')) + '/imgs/Image_cables_wh1_386.jpg' #Image_cables_43
                                separation_point = [19.498687694359937, -12.956617868950271, -1.0002328393548083]
                                separation_point_success = True

                        msg_log = String()
                        msg_log.data = "Calculating grasp point..."
                        logs_publisher.publish(msg_log)
                        #separation_point, separation_point_success = compute_separation_pose(img_cables_path, op['WH'], op['label'], pixel_D_param, dist_next=50, forward=False)
                        print("SEPARATION POINT: " + str(separation_point))
                        grasp_point_global = [float(separation_point[0])/1000, float(separation_point[1])/1000]
                        grasp_angle_temp = float(separation_point[2])
                        if grasp_angle_temp > 0:
                                grasp_angle_global = min(grasp_angle_temp, float(15*math.pi/180))
                        else:
                                grasp_angle_global = max(grasp_angle_temp, float(-15*math.pi/180))
                        grasp_angle_temp_display = grasp_angle_temp*180/math.pi
                        grasp_angle_global_display = grasp_angle_global*180/math.pi
                        msg_log.data = "Grasp point calculated. Calc angle: " + str(grasp_angle_temp_display) + ", used angle: " + str(grasp_angle_global_display)   
                        logs_publisher.publish(msg_log)

                        repeat = False
                        img_name = img_cables_path.split('/')[-1]
                        confirm_msg = String()
                        confirm_msg.data = img_cables_path[:-len(img_name)]+'Grasp_point_'+img_name
                        #confirmation_publisher.publish(confirm_msg)
                        #while not confirmation_received: #Wait until the UI accepts
                        #        rospy.sleep(0.1)
                        
                        os.system('cp ' + str(rospack.get_path('vision_pkg_full_demo')) + '/imgs/error_grasp_1.jpg /home/remodel/UI-REMODEL/src/assets/img/grasp_image.jpg')
                        confirmation_received = False
                        # if confirmation_msg == "N":
                        #         stop_function("Grasp canceled")
                        # elif confirmation_msg == "R":
                        #         pixel_D_param += 1
                        #         repeat = True

                if separation_point_success:
                        print("A")
                        actuate_grippers(config['open_distance'], config['gripper_speed'], separation_arm, grasp=False)
                        print(grasp_point_global)
                        init_pose = get_current_pose(separation_group).pose

                        # test_point = get_shifted_pose(op["spot"][0]["pose_corner"], [0.03, op["spot"][0]['gap']/2, op["spot"][0]["height"], 0, 0, 0])
                        # test_point_corrected = ATC1.correctPose(test_point, separation_arm, rotate = True, routing_app = False, ATC_sign = -1, secondary_frame = True)
                        # waypoints_test = [init_pose, test_point_corrected]
                        # plan, success, motion_group_plan = compute_cartesian_path_velocity_control_arms_occlusions([waypoints_test], [speed_execution], arm_side=separation_arm)
                        # if success:
                        #         execute_plan_async(motion_group_plan, plan)
                        # exit()

                        grasp_point = get_shifted_pose(op["spot"][0]["pose_corner"], [grasp_point_global[0], op["spot"][0]['gap']/2, op["spot"][0]["height"] + grasp_point_global[1], 0, 0, 0])
                        grasp_point_offset = get_shifted_pose(grasp_point, [0, 0, (config['z_offset']*2) - grasp_point_global[1], 0, 0, 0])
                        grasp_point_corrected = correctPose(grasp_point, separation_arm, rotate = True, routing_app = not rotate_gripper_z, ATC_sign = -1, secondary_frame = True)
                        grasp_point_offset_corrected = correctPose(grasp_point_offset, separation_arm, rotate = True, routing_app = not rotate_gripper_z, ATC_sign = -1, secondary_frame = True)                        
                        waypoints2 = [init_pose, grasp_point_offset_corrected, grasp_point_corrected]
                        plan, success = compute_cartesian_path_velocity_control([waypoints2], [config['speed_execution']], arm_side=separation_arm)
                if success:
                        step2 += execute_plan_async(separation_group, plan) 
                        print(step2) 
                else:
                        stop_function("Grasp point determination failed")
                #exit()
                                
        if step2 == 2:
                print("AA")
                actuate_grippers(config['slide_distance'], config['gripper_speed'], separation_arm, grasp=False)
                rospy.sleep(0.5)
                print("BB")
                init_pose = get_current_pose(separation_group).pose
                grasp_point_lift = get_shifted_pose(grasp_point_offset, [config['x_offset'], 0, config['z_offset']/2, 0, 0, 0])
                grasp_point_lift_corrected = correctPose(grasp_point_lift, separation_arm, rotate = True, routing_app = not rotate_gripper_z, ATC_sign = -1, secondary_frame = True)
                waypoints3 = [init_pose, grasp_point_lift_corrected]
                print("CC")
                plan, success, motion_group_plan = compute_cartesian_path_velocity_control_arms_occlusions([waypoints3], [config['speed_execution']], arm_side=separation_arm)
                print(success)
                print(motion_group_plan)
                if success:
                        step2 += execute_plan_async(motion_group_plan, plan)
                rospy.sleep(0.5)
                
        if step2 == 3:
                if config['use_camera']:
                        img_cables_path, capture_success = capture_img()
                        if not capture_success:
                                stop_function("Failed to take the cables image")
                else:
                        img_cables_path = str(rospack.get_path('vision_pkg_full_demo')) + '/imgs/Image_cables_wh1_387.jpg' #Image_cables_1

                msg_log = String()
                msg_log.data = "Evaluating cable separation..."
                logs_publisher.publish(msg_log)
                grasp_point_eval_mm = [int((2*config['z_offset'])*1000), int((config['x_offset']+grasp_point_global[0])*1000)]
                result_msg, separation_success, eval_success = check_cable_separation(img_path=img_cables_path, wh=op['WH'], sep_index=op['label'], pixel_D=8, grasp_point_eval_mm=grasp_point_eval_mm)
                msg_log.data = result_msg
                logs_publisher.publish(msg_log)
                print(result_msg)

                if eval_success and separation_success:
                        #step1 += 1
                        #step2 = 0
                        print("Successful cable separation")
                        #process_actionserver.publish_feedback()
                else:
                        if eval_success:
                                #stop_function("Cable separation was not succesful")
                                print("Cable separation was not succesful")
                        else:
                                #stop_function("Grasp evaluation failed")
                                print("Grasp evaluation failed")
                step2 = 4

        if step2 == 4 and op["sep_guide"]:
                waypoints_SC4 = []
                init_pose = get_current_pose(separation_group).pose
                waypoints_SC4.append(init_pose)
                sep_guide_forward_up = get_shifted_pose(next_guide_corner, [op["spot"][1]["width"] + fingers_size[0] + 0.02, op["spot"][1]["gap"]/2, op["spot"][1]["height"] + config['z_offset'] + fingers_size[2]/2, 0, 0, 0])
                waypoints_SC4.append(correctPose(sep_guide_forward_up, separation_arm, rotate = True, routing_app = not rotate_gripper_z, ATC_sign = -1))
                plan, success, motion_group_plan = compute_cartesian_path_velocity_control_arms_occlusions([waypoints_SC4], [config['speed_execution']], arm_side=separation_arm)
                if success:
                        step2 += execute_plan_async(motion_group_plan, plan)
                rospy.sleep(0.5)

        if step2 == 5 and op["sep_guide"]:
                #Apply tension
                actuate_grippers(config['grasp_distance'], config['gripper_speed'], separation_arm, grasp=True)

                waypoints_SC5 = []
                init_pose = get_current_pose(separation_group).pose
                waypoints_SC5.append(init_pose)
                sep_guide_tension = get_shifted_pose(next_guide_corner, [op["spot"][1]["width"]  + fingers_size[0] + config['force_offset'] +0.02, op["spot"][1]["gap"]/2, op["spot"][1]["height"] + config['z_offset'] + fingers_size[2]/2, 0, 0, 0])
                waypoints_SC5.append(correctPose(sep_guide_tension, separation_arm, rotate = True, routing_app = not rotate_gripper_z, ATC_sign = -1))
                #Move with force_control
                plan, success = compute_cartesian_path_velocity_control([waypoints_SC5], [config['speed_tension']], arm_side=separation_arm)
                if success:
                        step2 += execute_force_control(group = separation_group, plan = plan, limit = force_limit_cable, force_active=config['force_control_active'])
                rospy.sleep(0.5)

        if step2 == 6 and op["sep_guide"]:
                print("###########TEST CIRC###########")
                #Circ motion
                waypoints_SC6 = []
                init_pose = get_current_pose(separation_group).pose
                waypoints_SC6.append(copy.deepcopy(init_pose))
                init_pose_guides = antiCorrectPose(init_pose, separation_arm, routing_app = not rotate_gripper_z)
                rot_center = get_shifted_pose(op["spot"][0]["pose_corner"],[op["spot"][0]['width']/2, op["spot"][0]['gap']/2, op["spot"][0]['height']/2, 0, 0, 0])
                insert_guide_center_sep = get_shifted_pose(next_guide_corner,[op["spot"][1]['width']/2, op["spot"][1]['gap']/2, op["spot"][1]['height']/2, 0, 0, 0])
                radius_rot = compute_distance(rot_center, init_pose_guides)
                #final_point = in the direction rot_center --> guide a distance of radius_rot
                final_cable_direction = get_axis(insert_guide_center_sep, rot_center)
                final_point = copy.deepcopy(insert_guide_center_sep) #Same orientation
                final_point.position.x = rot_center.position.x + final_cable_direction[0]*radius_rot
                final_point.position.y = rot_center.position.y + final_cable_direction[1]*radius_rot
                final_point.position.z = rot_center.position.z #+ final_cable_direction[2]*radius_rot #Maybe delete?
                #Evaluate collisions in final_point, if there are, there slide a bit more distance and evaluate again. Check also when the distance to the next guide is already too small, so then it will just do a slide_top
                #With the correct final_point, calculates the rotation knowing, center, initial and final arc points.

                guide_yaw_axis = get_axis_from_RM(pose_to_frame(rot_center).M, "Y")
                print(guide_yaw_axis)
                rot_axis = get_ort_axis(guide_yaw_axis, final_cable_direction)[2]
                print(rot_axis)
                rot_axis = [-element for element in rot_axis]
                dist_z = compute_distance_relative(final_point, init_pose_guides, guide_yaw_axis) #from circle to guide in z axis
                print(dist_z)
                print(radius_rot)
                print(final_cable_direction)
                print(rot_axis)
                #visualize_keypoints_simple([rot_center, init_pose, init_pose_guides, insert_guide_center_sep, final_point], '/torso_base_link')
                rot_degree = (math.asin(-dist_z/radius_rot))*180.0/math.pi
                print(rot_degree)
                center_waypoints = []
                circle_waypoints = []
                success, center_waypoints, circle_waypoints = circular_trajectory(rot_center, init_pose_guides, rot_degree, rot_axis, center_waypoints, circle_waypoints, step = 2, rot_gripper = False)
                #visualize_keypoints_simple(circle_waypoints, '/torso_base_link')
                for wp in circle_waypoints:
                        waypoints_SC6.append(correctPose(wp, separation_arm, rotate = True, ATC_sign = -1, routing_app = not rotate_gripper_z))
                #visualize_keypoints_simple(waypoints_SC6, '/base_link')
                plan, success = compute_cartesian_path_velocity_control([waypoints_SC6], [config['speed_execution']], arm_side=separation_arm)
                if success:
                        step2 += execute_plan_async(separation_group, plan)
                print("###########TEST CIRC END###########")

        if step2 == 7 and op["sep_guide"]:
                actuate_grippers(config['slide_distance'], config['gripper_speed'], separation_arm, grasp=False)
                waypoints_SC7 = []
                init_pose = get_current_pose(separation_group).pose
                waypoints_SC7.append(init_pose)
                if separation_arm=="right":
                        sign_side = 1
                else:
                        sign_side = -1
                side_pose = get_shifted_pose(next_guide_corner, [op["spot"][1]["width"] + fingers_size[0] + config['x_offset'], sign_side*0.1, 0, 0, 0, 0])
                waypoints_SC7.append(correctPose(side_pose, separation_arm, rotate = True, routing_app = not rotate_gripper_z, ATC_sign = -1))
                plan, success, motion_group_plan = compute_cartesian_path_velocity_control_arms_occlusions([waypoints_SC7], [config['slow_speed_execution']], arm_side=separation_arm)
                if success:
                        step2 += execute_plan_async(motion_group_plan, plan)

        if step2 == 8 and op["sep_guide"]:
                actuate_grippers(config['grasp_distance'], config['gripper_speed'], separation_arm, grasp=True)
                waypoints_SC8 = []
                init_pose = get_current_pose(separation_group).pose
                waypoints_SC8.append(init_pose)
                if separation_arm=="right":
                        sign_side = 1
                else:
                        sign_side = -1
                side_pose_tension = get_shifted_pose(next_guide_corner, [op["spot"][1]["width"] + fingers_size[0] + config['x_offset'], sign_side*(0.1 + config['force_offset']/2), 0, 0, 0, 0])
                waypoints_SC8.append(correctPose(side_pose_tension, separation_arm, rotate = True, routing_app = not rotate_gripper_z, ATC_sign = -1))
                #Move with force_control
                plan, success = compute_cartesian_path_velocity_control([waypoints_SC8], [config['speed_tension']], arm_side=separation_arm)
                if success:
                        step2+=execute_force_control(group = separation_group, plan = plan, limit = force_limit_cable, force_active=config['force_control_active'])
                rospy.sleep(0.5)
                holding_cables = True
                holding_comeback_pose = get_shifted_pose(op["spot"][1]["pose_corner"],[op["spot"][1]["width"] + fingers_size[0] + config['x_offset'] + 0.05, sign_side*0.05, op["spot"][1]['height']/2, 0, 0, 1.57])
                holding_comeback_pose_corrected = correctPose(holding_comeback_pose, separation_arm, rotate = True, routing_app = not rotate_gripper_z, ATC_sign = -1)

        if (step2 == 9 and op["sep_guide"]) or (step2==4 and not op["sep_guide"]):
                #retract_arm(separation_arm, op["spot"][1], config)
                step2 = 0
                #process_actionserver.publish_feedback()

        return step2

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

def antiCorrectPose(target_pose, arm_side, rotate = False, ATC_sign = -1, routing_app = False, route_arm = True):
        rospy.wait_for_service('/adv_manip/anti_correct_pose')
        anticorrect_srv = rospy.ServiceProxy('/adv_manip/anti_correct_pose', GetCorrectPose)
        req = GetCorrectPoseRequest()
        req.target_pose = target_pose; req.arm_side = arm_side; req.rotate = rotate; req.ATC_sign = ATC_sign; req.routing_app = routing_app; req.route_arm = route_arm
        return anticorrect_srv(req).pose

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
                req.waypoints_list.append(msg) 
        req.EE_speed = EE_speed; req.EE_ang_speed = EE_ang_speed; req.arm_side = arm_side; req.max_linear_accel = max_linear_accel; req.max_ang_accel = max_ang_accel; req.step = step
        resp = compute_path_srv(req)
        return resp.plan, resp.success

def compute_cartesian_path_velocity_control_arms_occlusions(waypoints_list, EE_speed, EE_ang_speed = [], arm_side = "left", max_linear_accel = 200.0, max_ang_accel = 140.0, step = 0.002):
        rospy.wait_for_service('/adv_manip/compute_cartesian_path_velocity_control_arms_occlusions')
        compute_path_occ_srv = rospy.ServiceProxy('/adv_manip/compute_cartesian_path_velocity_control_arms_occlusions', ComputePath)
        req = ComputePathRequest()
        for list_wp in waypoints_list:
                msg = float_list()
                for wp in list_wp:
                        msg.data.append(wp)
                req.waypoints_list.append(msg) 
        req.EE_speed = EE_speed; req.EE_ang_speed = EE_ang_speed; req.arm_side = arm_side; req.max_linear_accel = max_linear_accel; req.max_ang_accel = max_ang_accel; req.step = step
        resp = compute_path_occ_srv(req)
        return resp.plan, resp.success, resp.group

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
        print("MOVEMENT DONE")
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
                state = client.get_state()
                if state==2 or state == 3 or state == 4:
                        async_move_done = True
        async_move_done = False
        return step_inc_fb

async_plan_done=False
def async_plan_feedback_callback(fb):
        global async_plan_done
        global step_inc_fb
        print("MOVEMENT DONE")
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
                state = client.get_state()
                if state==2 or state == 3 or state == 4:
                        async_plan_done = True
        async_plan_done = False
        return step_inc_fb

force_plan_done=False
def execute_force_control_callback(fb):
        global force_plan_done
        global step_inc_fb
        print("MOVEMENT DONE")
        if fb.done:
                force_plan_done = True
                step_inc_fb = fb.step_inc

def execute_force_control(group, plan, limit, force_active = True):
        global force_plan_done
        global step_inc_fb
        client=actionlib.SimpleActionClient('/adv_manip/execute_force_control', ExecutePlanAction)
        client.wait_for_server()
        goal = ExecutePlanGoal()
        goal.group = group; goal.plan = plan; goal.force_active = force_active; goal.force_limit = limit
        client.send_goal(goal, feedback_cb=execute_force_control_callback)
        while not force_plan_done:
                rospy.sleep(0.05)
                state = client.get_state()
                if state==2 or state == 3 or state == 4:
                        force_plan_done = True
        force_plan_done = False
        return step_inc_fb

def stop_function(msg):
        stop_publisher.publish(Bool())
        msg_log = String()
        msg_log.data = msg
        logs_publisher.publish(msg_log)

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
                state = client.get_state()
                if state==2 or state == 3 or state == 4:
                        eef_done = True
        eef_done = False

def actuate_grippers(distance, speed, arm, grasp=False):
        actuate_eef(0, distance, speed, arm, grasp)

def actuate_gun():
        actuate_eef(1)

def find_path(arm_side, end_guide, wh, offset_z, interpolate_z = False):
        print("finding")
        rospy.wait_for_service('/adv_manip/find_free_path_rrt')
        find_path_srv = rospy.ServiceProxy('/adv_manip/find_free_path_rrt', pathFinder)
        req = pathFinderRequest()
        req.arm = arm_side; req.end_guide = end_guide; req.WH = wh; req.offset_z = offset_z; req.interpolate_z = interpolate_z; req.show = False; req.target_provided = False
        resp = find_path_srv(req)
        return resp.path, resp.success

def capture_img():
        rospy.wait_for_service('/OAK/capture_img')
        capture_img_srv = rospy.ServiceProxy('/OAK/capture_img', Trigger)
        capture_img_res = capture_img_srv(TriggerRequest())
        return capture_img_res.message, capture_img_res.success

def compute_separation_pose(img_path, wh, sep_index, pixel_D, dist_next, forward=False):
        rospy.wait_for_service('/vision/grasp_point_determination_srv')
        grasp_point_srv = rospy.ServiceProxy('/vision/grasp_point_determination_srv', cablesSeparation)
        req = cablesSeparationRequest()
        req.img_path = img_path; req.wh_id = wh; req.separated_cable_index = sep_index; req.pixel_D = pixel_D; req.forward = forward; req.iteration = False; 
        req.analyzed_length = 100; req.analyzed_grasp_length = min(dist_next, 50); req.simplified = True; req.visualize = False
        res = grasp_point_srv(req)
        return res.grasp_point, res.success

def check_cable_separation(img_path, wh, sep_index, pixel_D, grasp_point_eval_mm=[]):
        rospy.wait_for_service('/vision/check_cable_separation_srv')
        eval_grasp_srv = rospy.ServiceProxy('/vision/check_cable_separation_srv', cablesSeparation)
        req = cablesSeparationRequest()
        req.img_path = img_path; req.wh_id = wh; req.separated_cable_index = sep_index; req.pixel_D = pixel_D; req.forward = False; req.iteration = False; 
        req.grasp_point_eval_mm = grasp_point_eval_mm; req.analyzed_length = 150; req.simplified = True; req.visualize = False
        res = eval_grasp_srv(req)
        return res.result, res.separation_success, res.success

################## Other functions ################
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

def visualize_keypoints_simple(poses, parent_frame = "/base_link"):
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

confirmation_received = False
confirmation_msg = 'N'
def callback_confirm(msg):
    global confirmation_received
    global confirmation_msg
    confirmation_msg = msg.data
    confirmation_received = True
confirmation_subscriber = rospy.Subscriber('/UI/confirm_res', String, callback_confirm) 

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
        global separated_global
        global separation_arm

        if (op['spot']['side'] == "R" and not separated_global) or (separation_arm == "right" and separated_global):
                route_arm = "right"
                route_group = "arm_right"
                group2 = "arm_left"
                move_away2_sign = -1
        else:
                route_arm = "left"
                route_group = "arm_left"
                group2 = "arm_right"
                move_away2_sign = 1
        separated_global = False


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
                step2 = EC(op, step2, config2, route_group, route_arm)              
    
        elif op['type'] == "PC":
                update_route_arm_info(op)
                step2 = PC(op, step2, config2, route_group, route_arm)
                        
        elif op['type'] == "SC":
                PC_op = get_last_connector_info(op)
                update_route_arm_info(PC_op)
                step2 = separate_cables(op, step2, config2, route_group, route_arm)
        """
        elif op['type'] == "RC":
                PC_op = get_last_connector_info(op)
                update_route_arm_info(PC_op)
                route_cables(op, PC_op, route_arm)

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


#CAD Platform info
rospy.wait_for_service('ELVEZ_platform_handler/all_operations')
my_service = rospy.ServiceProxy('ELVEZ_platform_handler/all_operations', all_operations)
opsReq = all_operationsRequest()
opsResult = my_service(opsReq)

#Gets info about the components (jigs and combs) and cables involved
ops_info = []
ops_info_text = []
last_PC_spot = {}
routed_cables = []
WH_sep = []
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
                ops_temp['label'] = str(ops.label[0])
                ops_info_text_temp = 'Pick ' + str(ops.label[0]) + " from " + str(ops.spot[0].jig)

        elif ops.type == "PC":
                rospy.wait_for_service('/ELVEZ_platform_handler/guide_info')
                my_service = rospy.ServiceProxy('/ELVEZ_platform_handler/guide_info', guide_info)
                guideReq = guide_infoRequest()
                guideReq.jig = ops.spot[0].jig
                guideReq.guide = ops.spot[0].id
                guideResult = my_service(guideReq)
                guide = guideResult.data
                ops_temp['spot'] = {'name': str(ops.label[0]), 'jig': ops.spot[0].jig, 'id': ops.spot[0].id, 'side': ops.spot[0].side, 'pose_corner': guide.key_corner_frame, 'center_pose': guide.key_center_frame, 'gap': guide.key_gap, 'width': guide.key_length, 'height': guide.key_height, 'height_corner': guide.key_height_corner, 'collisions': guide.collisions, 'dimensions': guide.dimensions}
                ops_temp['label'] = str(ops.label[0])
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
                
                if not (ops.label[0][:2]=='WH'): #Previous operations before routing the required cable group
                        conReq = connector_infoRequest()
                        conReq.label = ops.label[0]
                        groupResult = my_service_con(conReq)
                        cablesGroup = {}
                        conReq.label = 'WH'+str(groupResult.WH)
                        ops_temp['WH'] = str(groupResult.WH)
                        whResult = my_service_con(conReq)
                        cablesWH_remaining = {}
                        for cable in whResult.cables:
                                if not (cable.label in routed_cables):
                                        cablesWH_remaining[cable.label]=int(cable.pins[0]) #Cables of the WH that haven't been routed yet
                        for cable in groupResult.cables:
                                cablesGroup[cable.label]=int(cable.pins[0])
                                routed_cables.append(cable.label)
                        max_index_group = max(zip(cablesGroup.values(), cablesGroup.keys()))[0]
                        min_index_group = min(zip(cablesGroup.values(), cablesGroup.keys()))[0]
                        max_index_WH_remaining = max(zip(cablesWH_remaining.values(), cablesWH_remaining.keys()))[0]

                        if len(cablesGroup) < (len(cablesWH_remaining)):
                                if groupResult.WH in WH_sep: #ERROR
                                        print("ERROR. Cannot handle more than one separation per wiring harness")
                                else:
                                        sub_op_temp = {}
                                        sub_op_temp['type'] = 'SC'
                                        sub_op_temp['WH'] = str(groupResult.WH)
                                        sub_op_temp['spot'] = []
                                        sub_op_temp['spot'].append(last_PC_spot)

                                        if max_index_group < max_index_WH_remaining: #Route cables in the bottom. separate upper cables and place them in another guide
                                                sub_op_temp['label'] = [max_index_group+1]
                                                guideReq = guide_infoRequest()
                                                guideReq.jig = 'J'+str(groupResult.WH)+'S' #DEFINE THIS ELEMENT
                                                guideReq.guide = '1'
                                                guideResult = my_service(guideReq)
                                                guide = guideResult.data #Separation guide
                                                sub_op_temp['spot'].append({'pose_corner': guide.key_corner_frame, 'gap': guide.key_gap, 'width': guide.key_length, 'height': guide.key_height, 'height_corner': guide.key_height_corner, 'collisions': guide.collisions, 'dimensions': guide.dimensions})
                                                sub_op_temp['sep_guide'] = True
                                                ops_info.append(sub_op_temp)
                                                ops_info_text.append('Separate the ' + str(int(max_index_WH_remaining-max_index_group)) + ' top cables of WH' + str(groupResult.WH) + " in J" + str(groupResult.WH) + "S")
                                                #Grasp cable group from the first route position
                                                sub_op_temp = {}
                                                sub_op_temp['type'] = 'GC'
                                                sub_op_temp['label'] = ops.label[0]
                                                sub_op_temp['spot'] = []
                                                sub_op_temp['spot'].append(last_PC_spot)
                                                ops_info.append(sub_op_temp)
                                                ops_info_text.append('Grasp cable group ' + str(ops.label[0]) + " from " + str(sub_op_temp['spot'][0]['jig']))
                                                WH_sep.append(groupResult.WH) #Add the WH to the list of already separated WHs                                                        

                                        else: #Route cables in the top
                                                sub_op_temp['spot'].append(ops_temp['spot'][0]) #First routing guide after separation
                                                sub_op_temp['label'] = [min_index_group]
                                                #print("MIN: " + str(min_index_group))
                                                sub_op_temp['sep_guide'] = False
                                                ops_info.append(sub_op_temp)
                                                ops_info_text.append('Separate the ' + str(int(len(cablesGroup))) + ' top cables of WH' + str(groupResult.WH))

                        else: #Route remaining cables
                                if groupResult.WH in WH_sep: #grasp cable group from the separation guide
                                        sub_op_temp = {}
                                        sub_op_temp['type'] = 'GCS'
                                        sub_op_temp['label'] = ops.label[0]
                                        guideReq = guide_infoRequest()
                                        guideReq.jig = 'J'+str(groupResult.WH)+'S'
                                        guideReq.guide = '1'
                                        guideResult = my_service(guideReq)
                                        guide = guideResult.data
                                        sub_op_temp['spot']=[]
                                        sub_op_temp['spot'].append({'pose_corner': guide.key_corner_frame, 'gap': guide.key_gap, 'width': guide.key_length, 'height': guide.key_height, 'height_corner': guide.key_height_corner, 'collisions': guide.collisions, 'dimensions': guide.dimensions})
                                        sub_op_temp['spot'].append(last_PC_spot)
                                        ops_info.append(sub_op_temp)
                                        ops_info_text.append('Grasp cable group ' + str(ops.label[0]) + " from J" + str(groupResult.WH) + "S")
                                else:
                                        sub_op_temp = {}
                                        sub_op_temp['type'] = 'GC'
                                        sub_op_temp['label'] = ops.label[0]
                                        sub_op_temp['spot'] = [last_PC_spot]
                                        ops_info.append(sub_op_temp)
                                        ops_info_text.append('Grasp cable group ' + str(ops.label[0]) + " from " + str(sub_op_temp['spot'][0]['jig']))
                else:
                        ops_temp['WH'] = str(ops.label[0][2])
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

step1 = 2
step2 = 0
set_named_target('arms', "arms_platform_5")
move_group_async("arms")
rospy.sleep(0.5)
print("READY")
execute_operation(ops_info[step1])