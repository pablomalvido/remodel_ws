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
from task_planner_pkg.msg import ATCAction, ATCGoal
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
routed_guides = []

#Motion groups
rospy.wait_for_service('/adv_manip/define_motion_groups')
def_motion_groups_srv = rospy.ServiceProxy('/adv_manip/define_motion_groups', DefMotionGroups)
rospy.wait_for_service('/adv_manip/max_speed_groups')
def_max_speed_srv = rospy.ServiceProxy('/adv_manip/max_speed_groups', MaxSpeedGroups)
rospy.wait_for_service('/adv_manip/def_EEF')
def_EEF_srv = rospy.ServiceProxy('/adv_manip/def_EEF', DefEEF)
rospy.wait_for_service('/adv_manip/def_ATC')
def_ATC_srv = rospy.ServiceProxy('/adv_manip/def_ATC', DefATC)
rospy.wait_for_service('ELVEZ_platform_handler/tool_info')
get_all_tools_srv = rospy.ServiceProxy('ELVEZ_platform_handler/get_tools_service', get_list)
rospy.wait_for_service('ELVEZ_platform_handler/tool_info')
get_tool_info_srv = rospy.ServiceProxy('ELVEZ_platform_handler/tool_info', tool_info)

motion_groups = ["arm_left", "arm_right", "arms", "torso"]
req = DefMotionGroupsRequest()
req.left = "arm_left"; req.right = "arm_right"; req.arms = "arms"; req.torso = "torso"
def_motion_groups_srv(req)

req = MaxSpeedGroupsRequest()
req.speed = 0.2
for group in motion_groups:
    req.group = group
    def_max_speed_srv(req)

#init_tools = {'left':'EEF_gripper_left','right':'EEF_gripper_right'}
init_tools = {'left':'EEF_taping_gun','right':'EEF_gripper_right'}
ATC_tools = []
ATC_extra_z=0.065
torso_height = 1.2
tools = get_all_tools_srv().data
for tool in tools:
        req = DefEEFRequest()
        info = get_tool_info_srv(tool)
        req.EE_end_frame = info.pose_end
        req.ATC_frame = info.pose_base
        req.ATC_frame.position.z += torso_height
        req.x = info.dim_tool[0];req.y = info.dim_tool[1];req.z = info.dim_tool[2]
        req.name = tool
        req.path = str(rospack.get_path('motoman_sda10f_support')) + '/meshes/EE/' + str(tool) + ".stl"
        if info.type == 'gripper':
                req.EE_end_frame2 = info.pose_nail
                req.fingers_dim = info.dim_fingers
                if tool == 'EEF_gripper_left':
                        req.EE_end_frame.position.z+=ATC_extra_z
                        req.EE_end_frame2.position.z+=ATC_extra_z
        if tool != init_tools['left'] and tool != init_tools['right']:
                ATC_tools.append(tool)
        def_EEF_srv(req)

eef_link_left = "left_tool_exchanger"
touch_links_left = ["left_tool_exchanger"]
eef_link_right = "right_tool_exchanger"
touch_links_right = ["right_tool_exchanger"]
req = DefATCRequest()
req.left_tool=init_tools['left']; req.right_tool=init_tools['right']; req.eef_link_left=eef_link_left; req.eef_link_right=eef_link_right; req.ATC_tools=ATC_tools; req.left_ATC_angle=0.7854; req.right_ATC_angle=-0.7854; req.left_ATC_dist=0.083; req.right_ATC_dist=0.054
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
                        # test_point_corrected = correctPose(test_point, separation_arm, rotate = True, routing_app = False, ATC_sign = -1, secondary_frame = True)
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


def route_cables(op, step1=0, step2=0, config={}, route_group="", route_arm="", connector_op={}):
        global rot_center
        #global process_actionserver
        global larger_slide
        global grasping_cables

        grasping_cables = False
        min_dist_guides = 0.05
        max_angle_diff = 10.0*(math.pi/180.0)

        index = 0
        corrected_guides = copy.deepcopy(op)
        routing_operations = []

        deep = False
        if len(op["spot"]) == 1 or op['WH'] == '1':
                deep = True
        
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
                                print("#####ANGLES#######")
                                print(cable_dir_angle)
                                print(next_guide_dir)
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
                        print("#####ANGLES#######")
                        print(prev_guide_dir)
                        print(cable_dir_angle)
                        print(next_guide_dir)
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
                        step1_inc = 0
                        print(op['op'])
                        if op['op'] == "first_lift":
                                step2, step1_inc = RC_first_lift(op, step2, config, route_group, route_arm)
                                step1+=step1_inc
                        elif op['op'] == "first_corner":
                                step2, step1_inc = RC_first_corner(op, step2, config, route_group, route_arm)
                                step1+=step1_inc
                        elif op['op'] == "route_top":
                                step2, step1_inc = RC_route_top(op, step2, config, route_group, route_arm)
                                step1+=step1_inc
                        elif op['op'] == "insert_lift":
                                step2, step1_inc = RC_insert_lift(op, step2, config, route_group, route_arm, deep)
                                step1+=step1_inc
                        elif op['op'] == "insert_corner":
                                step2, step1_inc = RC_insert_corner(op, step2, config, route_group, route_arm, deep)
                                step1+=step1_inc
                        elif op['op'] == "insert_grasp_corner":
                                step2, step1_inc = RC_insert_grasp_corner(op, step2, config, route_group, route_arm, deep)
                                step1+=step1_inc
                        elif op['op'] == "insert_final":
                                step2, step1_inc = RC_insert_final(op, step2, config, route_group, route_arm, deep)
                                step1+=step1_inc
                        #process_actionserver.publish_feedback()
                        if step1_inc==0:
                               break
                i+=1
        return step1, step2


def RC_insert(op, step2, config, route_group, route_arm, deep = True):
        global rot_center
        global rot_center_up
        global larger_slide
        global force_limit_cable

        #arm = motion_group[route_group], arm2 = motion_group[aux_group], aux_arm = aux_arm, fingers_size = fingers_size, EEF2 = fingers_size_aux
        waypoints1 = []
        if route_arm == "left":
                aux_group = "arm_right"
                aux_arm = "right"
        elif route_arm == "right":
                aux_group = "arm_left"
                aux_arm = "left"
        fingers_size = get_fingers_size(route_arm)

        #insert_guide_center = get_shifted_pose(op["prev_guide"]["pose_corner"],[op["prev_guide"]['width']/2, op["prev_guide"]['gap']/2, fingers_size[2], 0, 0, 0])
        if deep:
                Z_insert_increment = 0
        else:
                Z_insert_increment = 0.005
        insert_guide_center = get_shifted_pose(op["prev_guide"]["pose_corner"],[op["prev_guide"]['width']/2, op["prev_guide"]['gap']/2, Z_insert_increment, 0, 0, 0]) #Originally z increment = 0 but it was going too deep

        if step2==0:
                #Apply tension
                init_pose = get_current_pose(route_group).pose
                if larger_slide:
                        top_guide_end = get_shifted_pose(op["prev_guide"]["pose_corner"],[op["prev_guide"]['width'] + fingers_size[0]/2 + 0.003, op["prev_guide"]['gap']/2, op["prev_guide"]['height'] + config['z_offset'] + fingers_size[2], 0, 0, 0]) #THE GUIDE WIDTH SHOULD BE DIVIDED BY 2, NOT BY 4
                else:
                        top_guide_end = get_shifted_pose(op["prev_guide"]["pose_corner"],[op["prev_guide"]['width']/4 + fingers_size[0]/2, op["prev_guide"]['gap']/2, op["prev_guide"]['height'] + config['z_offset'] + fingers_size[2], 0, 0, 0]) #THE GUIDE WIDTH SHOULD BE DIVIDED BY 2, NOT BY 4
                waypoints1.append(init_pose)
                waypoints1.append(correctPose(top_guide_end, route_arm, rotate = True, ATC_sign = -1, routing_app = True))
                plan, success = compute_cartesian_path_velocity_control([waypoints1], [config['speed_execution']], arm_side=route_arm)
                if success:
                        step2+=execute_plan_async(route_group, plan)
                        #arm.execute(plan, wait=True)
        
        if step2==1:
                actuate_grippers(config['grasp_distance'], config['gripper_speed'], route_arm, grasp=True)

                waypoints12 = []
                if larger_slide:
                        top_guide_end = get_shifted_pose(op["prev_guide"]["pose_corner"],[op["prev_guide"]['width'] + fingers_size[0]/2 + 0.003, op["prev_guide"]['gap']/2, op["prev_guide"]['height'] + config['z_offset'] + fingers_size[2], 0, 0, 0]) #THE GUIDE WIDTH SHOULD BE DIVIDED BY 2, NOT BY 4
                else:
                        top_guide_end = get_shifted_pose(op["prev_guide"]["pose_corner"],[op["prev_guide"]['width']/4 + fingers_size[0]/2, op["prev_guide"]['gap']/2, op["prev_guide"]['height'] + config['z_offset'] + fingers_size[2], 0, 0, 0]) #THE GUIDE WIDTH SHOULD BE DIVIDED BY 2, NOT BY 4
                waypoints12.append(correctPose(top_guide_end, route_arm, rotate = True, ATC_sign = -1, routing_app = True))
                top_guide_forward = get_shifted_pose(top_guide_end,[config['force_offset'], 0, 0, 0, 0, 0])
                waypoints12.append(correctPose(top_guide_forward, route_arm, rotate = True, ATC_sign = -1, routing_app = True))
                #Move with force_control
                plan, success = compute_cartesian_path_velocity_control([waypoints12], [config['speed_tension']], arm_side=route_arm)
                if success:
                        step2+=execute_force_control(group = route_group, plan = plan, limit = force_limit_cable, force_active=config['force_control_active'])
                rospy.sleep(0.5)

        if step2==2:
                if rot_center_up:
                        waypoints2_1 = []
                        waypoints2_2 = []
                        init_pose1 = get_current_pose(route_group).pose
                        waypoints2_1.append(init_pose1)
                        init_pose_guides = antiCorrectPose(init_pose1, route_arm, routing_app = True) 
                        init_pose2 = get_current_pose(aux_group).pose
                        waypoints2_2.append(init_pose2)
                        init_pose_guides2 = antiCorrectPose(init_pose2, aux_arm, routing_app = True, route_arm = False) 
                        insert_pose_guides_1 = init_pose_guides
                        insert_pose_guides_1.position.z = insert_guide_center.position.z
                        waypoints2_1.append(correctPose(insert_pose_guides_1, route_arm, rotate = True, ATC_sign = -1, routing_app = True))
                        insert_pose_guides_2 = init_pose_guides2
                        insert_pose_guides_2.position.z = insert_guide_center.position.z
                        waypoints2_2.append(correctPose(insert_pose_guides_2, aux_arm, rotate = True, ATC_sign = -1, routing_app = True, route_arm = False))
                        #visualize_keypoints_simple([waypoints2_1[-1], waypoints2_2[-1]], parent_frame="/base_link")
                        if route_arm == "left":
                                plan, success = dual_arm_cartesian_plan([waypoints2_1], [config['speed_execution']], [waypoints2_2], [config['speed_execution']], sync_policy=1)
                        elif route_arm == "right":
                                plan, success = dual_arm_cartesian_plan([waypoints2_2], [config['speed_execution']], [waypoints2_1], [config['speed_execution']], sync_policy=1)
                        if success:
                                step2+=execute_plan_async("arms", plan)
                                #arms.execute(plan, wait=True)
                        rot_center = insert_pose_guides_2

                        if step2 == 3:
                               rot_center_up = False
                        return step2, insert_pose_guides_1

                else:
                        print("###########TEST CIRC###########")
                        rot_center_up = False
                        #Circ motion
                        waypoints2 = []
                        init_pose = get_current_pose(route_group).pose
                        print("INIT POSE: " + str(init_pose))
                        waypoints2.append(copy.deepcopy(init_pose))
                        init_pose_guides = antiCorrectPose(init_pose, route_arm, routing_app = True)
                        #init_pose_check = correctPose(init_pose_guides, route_arm, rotate = True, ATC_sign = -1, routing_app = True)
                        #visualize_keypoints_simple([init_pose_guides, init_pose], parent_frame = "/torso_base_link")

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
                        #visualize_keypoints_simple(circle_waypoints, parent_frame = "/torso_base_link")
                        for wp in circle_waypoints:
                                waypoints2.append(correctPose(wp, route_arm, rotate = True, ATC_sign = -1, routing_app = True))
                                
                        #plan, fraction = arm.compute_cartesian_path(waypoints2, 0.01, 0.0)
                        #arm.execute(plan, wait=True)
                        plan, success = compute_cartesian_path_velocity_control([waypoints2], [config['speed_execution']], arm_side=route_arm)
                        if success:
                                step2+=execute_plan_async(route_group, plan)
                                #arm.execute(plan, wait=True)
                        print("###########TEST CIRC END###########")
                        return step2, circle_waypoints[-1]
        return step2, []


def RC_push_inserted(step2, push_group, push_arm, config):
        global routed_guides
        global holding_cables
        global holding_comeback_pose_corrected

        rospy.wait_for_service('compute_ik')
        ik_srv = rospy.ServiceProxy('compute_ik', GetPositionIK)
        fingers_size = get_fingers_size(route_arm)

        routed_guides_copy = copy.deepcopy(routed_guides)
        for guide in routed_guides_copy:
                top_guide_beginning_backward = get_shifted_pose(guide["pose_corner"],[-(guide['width']/2 + fingers_size[0]/2 + config['grasp_offset']), guide['gap']/2, guide['height'] + config['z_offset'] + fingers_size[2], 0, 0, 0])
                bottom_guide_beginning_backward = get_shifted_pose(guide["pose_corner"],[-(guide['width']/2 + fingers_size[0]/2 + config['grasp_offset']), guide['gap']/2, guide['height']/4, 0, 0, 0])
                ik_rq = PositionIKRequest()
                ik_rq.robot_state = RobotState()
                ik_rq.group_name = push_group
                pose_ik = correctPose(top_guide_beginning_backward, push_arm, rotate = True, ATC_sign = -1, routing_app = True, route_arm = False)
                ik_rq.pose_stamped = get_stampedpose(pose_ik, push_group)
                ik_rq.constraints = Constraints()
                ik_rq.constraints.name = ''
                print("calculating IK")
                ik_res = ik_srv(ik_rq)
                if ik_res.error_code.val!=1:
                        print("IK ERROR")
                        print(ik_res.error_code)
                        routed_guides = []
                        break
                
                if step2==3:
                        if holding_cables:
                                actuate_grippers(config['open_distance'], config['gripper_speed'], push_arm, grasp=False)
                                init_pose = get_current_pose(push_group).pose
                                waypoints_RCI0 = [init_pose, holding_comeback_pose_corrected]
                                plan, success = compute_cartesian_path_velocity_control([waypoints_RCI0], [config['speed_execution']], arm_side=push_arm)
                                if success:
                                        step2+=execute_plan_async(push_group, plan)
                                holding_cables = False
                                rospy.sleep(1)
                        else:
                                step2+=1

                if step2==4:
                        #Push down with second arm
                        actuate_grippers(config['open_distance'], config['gripper_speed'], push_arm, grasp=False)
                        waypoints_RCI1 = []
                        init_pose = get_current_pose(push_group).pose
                        waypoints_RCI1.append(init_pose)
                        up_pose = copy.deepcopy(init_pose)
                        top_guide_beginning_backward_corrected = correctPose(top_guide_beginning_backward, push_arm, rotate = True, ATC_sign = -1, routing_app = True, route_arm = False)
                        up_pose.position.z = top_guide_beginning_backward_corrected.position.z
                        # success, waypoints_RCI1_int = interpolate_trajectory(initial_pose = init_pose, final_pose = up_pose, step_pos_min = 0.02, step_deg_min = 5, n_points_max = 20)
                        # if not success:
                        #         return
                        waypoints_RCI1.append(up_pose)
                        # waypoints_RCI1_int[0] = init_pose
                        plan, success = compute_cartesian_path_velocity_control([waypoints_RCI1], [config['speed_execution']], arm_side=push_arm)
                        if success:
                                step2+=execute_plan_async(push_group, plan)
                        print("STEP 4")
                        rospy.sleep(1)

                if step2==5:
                        waypoints_RCI2 = []
                        init_pose = get_current_pose(push_group).pose
                        waypoints_RCI2.append(init_pose)
                        waypoints_RCI2.append(correctPose(top_guide_beginning_backward, push_arm, rotate = True, ATC_sign = -1, routing_app = True, route_arm = False))
                        #success, waypoints_RCI2_int = interpolate_trajectory(initial_pose = init_pose, final_pose = waypoints_RCI2[-1], step_pos_min = 0.02, step_deg_min = 5, n_points_max = 20)
                        #if not success:
                        #        return
                        #waypoints_RCI2_int[0] = init_pose
                        plan, success = compute_cartesian_path_velocity_control([waypoints_RCI2], [config['fast_speed_execution']], arm_side=push_arm)
                        if success:
                                step2+=execute_plan_async(push_group, plan)
                        print("STEP 5")
                        rospy.sleep(1)

                if step2==6:
                        actuate_grippers(config['grasp_distance'], config['gripper_speed'], push_arm, grasp=False)
                        waypoints_RCI3 = []
                        init_pose = get_current_pose(push_group).pose
                        waypoints_RCI3.append(init_pose)
                        waypoints_RCI3.append(correctPose(bottom_guide_beginning_backward, push_arm, rotate = True, ATC_sign = -1, routing_app = True, route_arm = False, secondary_frame=True))
                        plan, success = compute_cartesian_path_velocity_control([waypoints_RCI3], [config['slow_speed_execution']], arm_side=push_arm)
                        if success:
                                step2+=execute_plan_async(push_group, plan)

                if step2==7:
                        waypoints_RCI4 = []
                        init_pose = get_current_pose(push_group).pose
                        waypoints_RCI4.append(init_pose)
                        waypoints_RCI4.append(correctPose(top_guide_beginning_backward, push_arm, rotate = True, ATC_sign = -1, routing_app = True, route_arm = False))
                        plan, success = compute_cartesian_path_velocity_control([waypoints_RCI4], [config['speed_execution']], arm_side=push_arm)
                        if success:
                                step2+=execute_plan_async(push_group, plan)
                        routed_guides.pop(0) #remove the guide that has been pushed from the list
                        step2 = 3

        if len(routed_guides)==0:
                step2 = 8 

        return step2                               
       

def RC_first_lift(op, step2, config, route_group, route_arm):
        global routed_guides

        fingers_size = get_fingers_size(route_arm)
        waypoints = []
        step1_inc = 0

        if step2==0:
                actuate_grippers(config['slide_distance'], config['gripper_speed'], route_arm, grasp=False)
                rospy.sleep(0.5)
                init_pose = get_current_pose(route_group).pose
                waypoints.append(init_pose)
                next_guide_top_beginning = get_shifted_pose(op["next_guide"]["pose_corner"],[- fingers_size[0]/2, op["next_guide"]['gap']/2, op["next_guide"]['height'] + config['z_offset'] + fingers_size[2], 0, 0, 0])
                next_guide_top = get_shifted_pose(op["next_guide"]["pose_corner"],[op["next_guide"]['width']/2, op["next_guide"]['gap']/2, op["next_guide"]['height'] + config['z_offset'] + fingers_size[2], 0, 0, 0])
                waypoints.append(correctPose(next_guide_top_beginning, route_arm, rotate = True, ATC_sign = -1, routing_app = True))
                #plan, success = compute_cartesian_path_velocity_control([waypoints], [speed_execution], arm_side=route_arm)
                plan, success, motion_group_plan = compute_cartesian_path_velocity_control_arms_occlusions([waypoints], [config['speed_execution']], arm_side=route_arm)
                if success:
                        step2+=execute_plan_async(motion_group_plan, plan)
                        #arm.execute(plan, wait=True)
        if step2==1:
                waypoints2 = []
                init_pose = get_current_pose(route_group).pose
                waypoints2.append(init_pose)
                #waypoints2.append(waypoints[-1])
                waypoints.append(correctPose(next_guide_top, route_arm, rotate = True, ATC_sign = -1, routing_app = True))
                plan, success = compute_cartesian_path_velocity_control([waypoints2], [config['speed_execution']], arm_side=route_arm)
                if success:
                        step2+=execute_plan_async(route_group, plan)
                        #arm.execute(plan, wait=True)

        if step2==2:
                routed_guides.append(op['next_guide'])
                step1_inc = 1
                step2 = 0
                
        return step2, step1_inc


def RC_first_corner(op, step2, config, route_group, route_arm):
        global rot_center
        global rot_center_up
        #Apply tension (Move forward 3cm max (stopped by "force controller"))
        #Insert (Circular motion)
        #Slide and lift (till x_offset)
        #Apply tension to cable
        #Bring second hand (lift from wherever it is, move to offset, go to the corner cable position to grasp)
        #Move first arm to the end of the top of the guide
        #Save in a global variable that in the next insert, it must be done with both arms and linear motion (Instead of circular)
        
        step1_inc = 0
        fingers_size = get_fingers_size(route_arm)
        
        #Slide
        waypoints3 = []
        init_pose = get_current_pose(route_group).pose
        waypoints3.append(init_pose)
        next_guide_top_beginning_route_offset1 = get_shifted_pose(op["next_guide"]["pose_corner"],[-max(fingers_size[0]/2 + config['x_offset'], fingers_size[1]/2), op["next_guide"]['gap']/2, op["next_guide"]['height'] + config['z_offset'] + fingers_size[2], 0, 0, 0])        
        prev_guide_end = get_shifted_pose(op["prev_guide"]["pose_corner"],[fingers_size[0]/2, op["next_guide"]['gap']/2, 0, 0, 0, 0])
        
        cable_dir_angle = math.atan2(next_guide_top_beginning_route_offset1.position.y - prev_guide_end.position.y, next_guide_top_beginning_route_offset1.position.x - prev_guide_end.position.x)
        next_guide_top_beginning_route_offset1.orientation = get_quaternion_in_Z(cable_dir_angle).orientation
        xy_dist = math.sqrt((next_guide_top_beginning_route_offset1.position.x - prev_guide_end.position.x)**2 + (next_guide_top_beginning_route_offset1.position.y - prev_guide_end.position.y)**2)
        z_increase = (config['x_offset'] + fingers_size[0])*((next_guide_top_beginning_route_offset1.position.z - prev_guide_end.position.z)/(xy_dist))
        next_guide_top_beginning_route_offset2 = get_shifted_pose(next_guide_top_beginning_route_offset1,[config['x_offset'] + fingers_size[0], 0, z_increase, 0, 0, 0])
        waypoints3.append(correctPose(next_guide_top_beginning_route_offset2, route_arm, rotate = True, ATC_sign = -1, routing_app = True))

        if step2==0:        
                actuate_grippers(config['slide_distance'], config['gripper_speed'], route_arm, grasp=False)
                plan, success = compute_cartesian_path_velocity_control([waypoints3], [config['speed_execution']], arm_side=route_arm)
                if success:
                        step2+=execute_plan_async(route_group, plan)
                        #arm.execute(plan, wait=True)

        if step2 >= 1:
                step2 = RC_pivoting_rotation(op, step2, config, route_group, route_arm, init_step=1)
        
        if step2 == 9:
                step2 = 0
                step1_inc = 1
        
        return step2, step1_inc


def RC_route_top(op, step2, config, route_group, route_arm):
        global routed_guides

        step1_inc = 0
        fingers_size = get_fingers_size(route_arm)
        waypoints = []

        if step2==0:
                waypoints.append(get_current_pose(route_group).pose)
                for guide in op["guides"][1:]:
                        top_guide = get_shifted_pose(guide["pose_corner"],[guide['width']/2, guide['gap']/2, guide['height'] + config['z_offset'] + fingers_size[2], 0, 0, 0])
                        waypoints.append(correctPose(top_guide, route_arm, rotate = True, ATC_sign = -1, routing_app = True))

                plan, success = compute_cartesian_path_velocity_control([waypoints], [config['speed_execution']], arm_side=route_arm)
                if success:
                        step2+=execute_plan_async(route_group, plan)
                        #arm.execute(plan, wait=True)

        routed_guides.pop(-1)
        if step2==1:
                for guide in op["guides"]:
                        routed_guides.append(guide)
                step2=0
                step1_inc=1

        return step2, step1_inc


def RC_insert_lift(op, step2, config, route_group, route_arm, deep):
        global rot_center
        global routed_guides
        global holding_cables
        global holding_comeback_pose_corrected
        #Apply tension (Move forward 3cm max (stopped by "force controller"))
        #Insert (Circular motion)
        #Slide (till x_offset)
        #Bring second hand (lift from wherever it is, move to offset, go down and grasp)
        #Lift (beginning and forward)

        step1_inc = 0

        if route_arm == "left":
                aux_group = "arm_right"
                aux_arm = "right"
        elif route_arm == "right":
                aux_group = "arm_left"
                aux_arm = "left"

        fingers_size = get_fingers_size(route_arm)
        fingers_size_aux = get_fingers_size(aux_arm)
        
        if step2<3:
                #Apply tension and insert
                step2, final_insert_wp = RC_insert(op, step2, config, route_group, route_arm, deep)
        
        if step2>=3 and step2<8:
                step2 = RC_push_inserted(step2, aux_group, aux_arm, config)

        if step2>=8 and step2<=12:
                #Slide
                actuate_grippers(config['slide_distance'], config['gripper_speed'], route_arm, grasp=False)

                waypoints3_wrist = []
                init_pose1 = get_current_pose(route_group).pose
                final_insert_wp = antiCorrectPose(init_pose1, route_arm, routing_app = True) 
                waypoints3_wrist.append(final_insert_wp)
                prev_guide_end = get_shifted_pose(op["prev_guide"]["pose_corner"],[op["prev_guide"]['width']/2 + fingers_size[0]/2, op["prev_guide"]['gap']/2, config['z_offset'], 0, 0, 0])
                next_guide_beginning = get_shifted_pose(op["next_guide"]["pose_corner"],[-fingers_size[0]/2, op["next_guide"]['gap']/2, config['z_offset'], 0, 0, 0])
                next_guide_top_beginning = get_shifted_pose(op["next_guide"]["pose_corner"],[-fingers_size[0], op["next_guide"]['gap']/2, op["next_guide"]['height'] + config['z_offset'] + fingers_size[2], 0, 0, 0])
                next_guide_top = get_shifted_pose(op["next_guide"]["pose_corner"],[op["next_guide"]['width']/2, op["next_guide"]['gap']/2, op["next_guide"]['height'] + config['z_offset'] + fingers_size[2], 0, 0, 0])
                if compute_distance(prev_guide_end, next_guide_beginning) > config['x_offset']:
                        if step2==8:
                                next_guide_offset = get_shifted_pose(op["next_guide"]["pose_corner"],[-config['x_offset'] - fingers_size[0]/2, op["next_guide"]['gap']/2, config['z_offset'], 0, 0, 0])
                                waypoints3_wrist.append(next_guide_offset)
                                waypoints3_fingers = []
                                for wp in waypoints3_wrist:
                                        waypoints3_fingers.append(correctPose(wp, route_arm, rotate = True, ATC_sign = -1, routing_app = True))
                                #print(waypoints3_fingers)
                                waypoints3_fingers[0] = init_pose1
                                plan, success = compute_cartesian_path_velocity_control([waypoints3_fingers], [config['speed_execution']], arm_side=route_arm, step = 0.001)
                                if success:
                                        step2+=execute_plan_async(route_group, plan)
                                        #arm.execute(plan, wait=True)

                        if step2==9:
                                #Bring second hand
                                if holding_cables:
                                        actuate_grippers(config['open_distance'], config['gripper_speed'], aux_arm, grasp=False)
                                        init_pose = get_current_pose(aux_group).pose
                                        waypoints40 = [init_pose, holding_comeback_pose_corrected]
                                        plan, success = compute_cartesian_path_velocity_control([waypoints40], [config['speed_execution']], arm_side=aux_arm)
                                        if success:
                                                step2+=execute_plan_async(aux_group, plan)
                                else:
                                        step2+=1
                        
                        if step2==10:
                                actuate_grippers(config['open_distance'], config['gripper_speed'], aux_arm, grasp=False)
                                waypoints4 = []
                                prev_guide_forward = get_shifted_pose(op["prev_guide"]["pose_corner"],[op["prev_guide"]['width'] + config['grasp_offset'] + fingers_size_aux[0]/2, op["prev_guide"]['gap']/2, config['z_offset'], 0, 0, 0])
                                prev_guide_forward_up = get_shifted_pose(prev_guide_forward, [0, 0, op["prev_guide"]['height'] + config['z_offset']*2 + fingers_size_aux[2], 0, 0, 0])
                                prev_guide_forward_up_corrected = correctPose(prev_guide_forward_up, aux_arm, rotate = True, ATC_sign = -1, routing_app = True, route_arm = False)
                                init_pose = get_current_pose(aux_group).pose
                                waypoints4.append(init_pose)
                                init_pose_up = copy.deepcopy(init_pose)
                                init_pose_up.position.z = prev_guide_forward_up_corrected.position.z
                                waypoints4.append(init_pose_up)
                                waypoints4.append(correctPose(prev_guide_forward_up, aux_arm, rotate = True, ATC_sign = -1, routing_app = True, route_arm = False))
                                plan, success = compute_cartesian_path_velocity_control([waypoints4], [config['fast_speed_execution']], arm_side=aux_arm)
                                if success:
                                        step2+=execute_plan_async(aux_group, plan)
                                        #arm2.execute(plan, wait=True)
                                #Apply tension movement. ToDo

                        if step2==11:
                                waypoints5 = []
                                init_pose = get_current_pose(aux_group).pose
                                waypoints5.append(init_pose)
                                prev_guide_forward = get_shifted_pose(op["prev_guide"]["pose_corner"],[op["prev_guide"]['width'] + config['grasp_offset'] + fingers_size_aux[0]/2, op["prev_guide"]['gap']/2, config['z_offset'], 0, 0, 0])
                                waypoints5.append(correctPose(prev_guide_forward, aux_arm, rotate = True, ATC_sign = -1, routing_app = True, route_arm = False))
                                plan, success = compute_cartesian_path_velocity_control([waypoints5], [config['speed_execution']], arm_side=aux_arm)
                                if success:
                                        step2+=execute_plan_async(aux_group, plan)
                                rot_center = copy.deepcopy(prev_guide_forward)

                        if step2==12:
                                actuate_grippers(config['grasp_distance'], config['gripper_speed'], aux_arm, grasp=True)
                                step2=13

                else:
                        prev_guide_center = get_shifted_pose(op["prev_guide"]["pose_corner"],[op["prev_guide"]['width']/2, op["prev_guide"]['gap']/2, op["prev_guide"]['height']/2, 0, 0, 0])
                        rot_center = copy.deepcopy(prev_guide_center)
                        step2=13

        if step2==13:
                #Slide to the top
                waypoints5 = []
                init_pose = get_current_pose(route_group).pose
                waypoints5.append(init_pose)
                waypoints5.append(correctPose(next_guide_top_beginning, route_arm, rotate = True, ATC_sign = -1, routing_app = True))
                plan, success = compute_cartesian_path_velocity_control([waypoints5], [config['speed_execution']], arm_side=route_arm)
                if success:
                        step2+=execute_plan_async(route_group, plan)
                        #arm.execute(plan, wait=True)

        if step2==14:
                waypoints52 = []
                waypoints52.append(correctPose(next_guide_top_beginning, route_arm, rotate = True, ATC_sign = -1, routing_app = True))
                waypoints52.append(correctPose(next_guide_top, route_arm, rotate = True, ATC_sign = -1, routing_app = True))
                plan, success = compute_cartesian_path_velocity_control([waypoints52], [config['speed_execution']], arm_side=route_arm)
                if success:
                        step2+=execute_plan_async(route_group, plan)
                        #arm.execute(plan, wait=True)

        if step2==15:
                routed_guides.append(op['next_guide'])
                step2=0
                step1_inc=1

        return step2, step1_inc


def RC_insert_corner(op, step2, config, route_group, route_arm, deep):
        global rot_center

        step1_inc = 0

        if route_arm == "left":
                aux_group = "arm_right"
                aux_arm = "right"
        elif route_arm == "right":
                aux_group = "arm_left"
                aux_arm = "left"

        fingers_size = get_fingers_size(route_arm)
        fingers_size_aux = get_fingers_size(aux_arm)
        
        if step2<3:
                #Apply tension and insert
                step2, final_insert_wp = RC_insert(op, step2, config, route_group, route_arm, deep)  

        if step2>=3 and step2<8:
                step2 = RC_push_inserted(step2, aux_group, aux_arm, config)

        if step2==8:
                #Slide
                actuate_grippers(config['slide_distance'], config['gripper_speed'], route_arm, grasp=False)

                #Move up
                init_pose1 = get_current_pose(route_group).pose
                final_insert_wp = antiCorrectPose(init_pose1, route_arm, routing_app = True) 
                waypoints2 = []
                waypoints2.append(init_pose1)
                next_guide = get_shifted_pose(op["next_guide"]["pose_corner"],[-config['x_offset'] - fingers_size[0], op["next_guide"]['gap']/2, op["next_guide"]['height']/2, 0, 0, 0])
                waypoints2.append(correctPose(next_guide, route_arm, rotate = True, ATC_sign = -1, routing_app = True))
                plan, success = compute_cartesian_path_velocity_control([waypoints2], [config['speed_execution']], arm_side=route_arm)
                if success:
                        step2+=execute_plan_async(route_group, plan)
                        #arm.execute(plan, wait=True)

        if step2==9:
                #Bring second hand
                waypoints3 = []
                prev_guide_backward = get_shifted_pose(op["prev_guide"]["pose_corner"],[- config['grasp_offset'] - fingers_size_aux[0]/2, op["prev_guide"]['gap']/2, config['z_offset'], 0, 0, 0])
                prev_guide_backward_up = get_shifted_pose(prev_guide_backward, [0, 0, op["prev_guide"]['height'] + config['z_offset']*2 + fingers_size_aux[2], 0, 0, 0])
                prev_guide_backward_up_corrected = correctPose(prev_guide_backward_up, aux_arm, rotate = True, ATC_sign = -1, routing_app = True, route_arm = False)
                init_pose = get_current_pose(aux_group).pose
                waypoints3.append(init_pose)
                init_pose_up = copy.deepcopy(init_pose)
                init_pose_up.position.z = prev_guide_backward_up_corrected.position.z
                waypoints3.append(init_pose_up)
                waypoints3.append(correctPose(prev_guide_backward_up, aux_arm, rotate = True, ATC_sign = -1, routing_app = True, route_arm = False))
                plan, success = compute_cartesian_path_velocity_control([waypoints3], [config['fast_speed_execution']], arm_side=aux_arm)
                if success:
                        step2+=execute_plan_async(aux_group, plan)
                        #arm2.execute(plan, wait=True)
                #Apply tension movement. ToDo

        if step2==10:
                actuate_grippers(config['open_distance'], config['gripper_speed'], aux_arm, grasp=False)
                waypoints4 = []
                init_pose = get_current_pose(aux_group).pose
                waypoints4.append(init_pose)
                prev_guide_backward = get_shifted_pose(op["prev_guide"]["pose_corner"],[- config['grasp_offset'] - fingers_size_aux[0]/2, op["prev_guide"]['gap']/2, config['z_offset'], 0, 0, 0])
                waypoints4.append(correctPose(prev_guide_backward, aux_arm, rotate = True, ATC_sign = -1, routing_app = True, route_arm = False))
                plan, success = compute_cartesian_path_velocity_control([waypoints4], [config['speed_execution']], arm_side=aux_arm)
                if success:
                        step2+=execute_plan_async(aux_group, plan)
                rot_center = copy.deepcopy(prev_guide_backward)

        if step2==11:
                actuate_grippers(config['grasp_distance'], config['gripper_speed'], aux_arm, grasp=True)
                step2=12

        if step2==12:
                #Slide to the top
                waypoints5 = []
                init_pose = get_current_pose(route_group).pose
                waypoints5.append(init_pose)
                next_guide_top_beginning = get_shifted_pose(op["next_guide"]["pose_corner"],[-fingers_size[0], op["next_guide"]['gap']/2, op["next_guide"]['height'] + config['z_offset'] + fingers_size[2], 0, 0, 0])
                waypoints5.append(correctPose(next_guide_top_beginning, route_arm, rotate = True, ATC_sign = -1, routing_app = True))
                plan, success = compute_cartesian_path_velocity_control([waypoints5], [config['speed_execution']], arm_side=route_arm)
                if success:
                        step2+=execute_plan_async(route_group, plan)
                        #arm.execute(plan, wait=True)

        if step2==13:
                waypoints6 = []
                next_guide_top_beginning = get_shifted_pose(op["next_guide"]["pose_corner"],[-fingers_size[0], op["next_guide"]['gap']/2, op["next_guide"]['height'] + config['z_offset'] + fingers_size[2], 0, 0, 0])
                next_guide_top = get_shifted_pose(op["next_guide"]["pose_corner"],[op["next_guide"]['width']/2, op["next_guide"]['gap']/2, op["next_guide"]['height'] + config['z_offset'] + fingers_size[2], 0, 0, 0])
                waypoints6.append(correctPose(next_guide_top_beginning, route_arm, rotate = True, ATC_sign = -1, routing_app = True))
                waypoints6.append(correctPose(next_guide_top, route_arm, rotate = True, ATC_sign = -1, routing_app = True))
                plan, success = compute_cartesian_path_velocity_control([waypoints6], [config['speed_execution']], arm_side=route_arm)
                if success:
                        step2+=execute_plan_async(route_group, plan)
                        #arm.execute(plan, wait=True)

        if step2==14:
                routed_guides.append(op['next_guide'])
                step2=0
                step1_inc=1

        return step2, step1_inc
        


def RC_pivoting_rotation(op, step2, config, route_group, route_arm, init_step):
        global rot_center
        global rot_center_up

        if route_arm == "left":
                aux_group = "arm_right"
                aux_arm = "right"
        elif route_arm == "right":
                aux_group = "arm_left"
                aux_arm = "left"

        fingers_size = get_fingers_size(route_arm)
        fingers_size_aux = get_fingers_size(aux_arm)

        next_guide_top = get_shifted_pose(op["next_guide"]["pose_corner"],[op["next_guide"]['width']/2, op["next_guide"]['gap']/2, op["next_guide"]['height'] + config['z_offset'] + fingers_size[2], 0, 0, 0])
        next_guide_top_beginning_route_offset1 = get_shifted_pose(op["next_guide"]["pose_corner"],[-max(fingers_size[0]/2 + config['x_offset'], fingers_size[1]/2), op["next_guide"]['gap']/2, op["next_guide"]['height'] + config['z_offset'] + fingers_size[2], 0, 0, 0])
        prev_guide_end = get_shifted_pose(op["prev_guide"]["pose_corner"],[fingers_size[0]/2, op["next_guide"]['gap']/2, 0, 0, 0, 0])
        cable_dir_angle = math.atan2(next_guide_top_beginning_route_offset1.position.y - prev_guide_end.position.y, next_guide_top_beginning_route_offset1.position.x - prev_guide_end.position.x)
        print("CABLE DIR ANGLE")
        print(cable_dir_angle)
        next_guide_top_beginning_route_offset1.orientation = get_quaternion_in_Z(cable_dir_angle).orientation

        if step2==init_step:
                #Bring second hand. Raise and rotate (No XY translation)
                waypoints4 = []
                init_pose = get_current_pose(aux_group).pose
                waypoints4.append(init_pose)
                init_pose_up = copy.deepcopy(init_pose)
                init_pose_up.position.z += config['z_offset']*2 + op["prev_guide"]["height"]
                waypoints4.append(init_pose_up)
                grasp_pose_up = get_shifted_pose(next_guide_top_beginning_route_offset1, [0, 0, config['z_offset']+ fingers_size_aux[2], 0, 0, 0])
                init_pose_rotated = copy.deepcopy(grasp_pose_up)
                init_pose_up_anticorrected = antiCorrectPose(init_pose_up, aux_arm) 
                init_pose_rotated.position = init_pose_up_anticorrected.position
                init_pose_rotated_corrected = correctPose(init_pose_rotated, aux_arm, rotate = True, ATC_sign = -1)
                init_pose_rotated_corrected.position = init_pose_up.position #Comment this line to rotate around the grip point
                waypoints4.append(init_pose_rotated_corrected)
                plan, success = compute_cartesian_path_velocity_control([waypoints4], [config['speed_execution']], arm_side=aux_arm)
                if success:
                        step2+=execute_plan_async(aux_group, plan)
                #exit()

        if step2==init_step+1:
                actuate_grippers(config['open_distance'], config['gripper_speed'], aux_arm, grasp=False)

                waypoints42 = []
                init_pose = get_current_pose(aux_group).pose
                waypoints42.append(init_pose)
                waypoints42.append(correctPose(grasp_pose_up, aux_arm, rotate = True, ATC_sign = -1))
                #next_guide_top_beginning_route_offset1.position.z += 0.006
                waypoints42.append(correctPose(next_guide_top_beginning_route_offset1, aux_arm, rotate = True, ATC_sign = -1))
                plan, success = compute_cartesian_path_velocity_control([waypoints42], [config['speed_execution']], arm_side=aux_arm)
                if success:
                        step2+=execute_plan_async(aux_group, plan)
                        #arm2.execute(plan, wait=True)
                        #visualize_keypoints_simple([next_guide_top_beginning_route_offset2, grasp_pose_up])

        if step2==init_step+2:
                #Apply tension movement. ToDo
                actuate_grippers(config['slide_distance'], config['gripper_speed'], aux_arm, grasp=False)
                #actuate_grippers(grasp_distance, gripper_speed, route_arm, grasp=True)

                #Move arm down 
                waypoints5 = []
                init_pose = get_current_pose(route_group).pose
                init_pose2 = get_current_pose(aux_group).pose
                init_pose_anticorrected = antiCorrectPose(init_pose, route_arm, routing_app = True)
                init_pose2_anticorrected = antiCorrectPose(init_pose2, aux_arm)
                waypoints5.append(init_pose)
                pose_down = copy.deepcopy(init_pose_anticorrected)
                pose_down.position.z = init_pose2_anticorrected.position.z
                waypoints5.append(correctPose(pose_down, route_arm, rotate = True, ATC_sign = -1, routing_app = True))
                plan, success = compute_cartesian_path_velocity_control([waypoints5], [config['speed_execution']], arm_side=route_arm)
                if success:
                        step2+=execute_plan_async(route_group, plan)
                        #arm.execute(plan, wait=True)

        if step2==init_step+3:
                actuate_grippers(config['grasp_distance'], config['gripper_speed'], aux_arm, grasp=True)
                #Rotate arms
                waypoints6_circ_wrist = []
                waypoints6_center_wrist = []
                waypoints6_circ = []
                waypoints6_center = []
                init_pose = get_current_pose(route_group).pose
                init_pose_guides = antiCorrectPose(init_pose, route_arm, routing_app = True)
                init_pose2 = get_current_pose(aux_group).pose
                corner_center = antiCorrectPose(init_pose2, aux_arm)
                radius_rot = compute_distance(corner_center, init_pose_guides)
                #visualize_keypoints_simple([init_pose_guides, corner_center], "/base_link")
                
                line_center = get_shifted_pose(corner_center, [1,0,0,0,0,0])
                angle_center = math.atan2(line_center.position.y - corner_center.position.y, line_center.position.x - corner_center.position.x)
                line_guide = get_shifted_pose(next_guide_top, [1,0,0,0,0,0])
                angle_guide = math.atan2(line_guide.position.y - next_guide_top.position.y, line_guide.position.x - next_guide_top.position.x)
                rot_rad = angle_guide - angle_center 
                rot_degree = (rot_rad*180)/math.pi
                if aux_arm == "right":
                        rot_degree = -rot_degree
                        invert_y_axis = False
                else:
                        invert_y_axis = True

                if rot_degree > 180:
                        rot_degree -= 360
                if rot_degree < -180:
                        rot_degree += 360
               
                print("##############ANGLE")
                print(rot_degree)

                success, waypoints6_center_wrist, waypoints6_circ_wrist = circular_trajectory(corner_center, init_pose_guides, rot_degree, [0,0,-1], waypoints6_center_wrist, waypoints6_circ_wrist, step = 2, rot_gripper = True, invert_y_axis=invert_y_axis)
                for wp in waypoints6_circ_wrist:
                        waypoints6_circ.append(correctPose(wp, route_arm, rotate = True, ATC_sign = -1, routing_app = True))
                for wp in waypoints6_center_wrist:
                        waypoints6_center.append(correctPose(wp, aux_arm, rotate = True, ATC_sign = -1))

                #visualize_keypoints_simple(waypoints6_center_wrist+waypoints6_circ_wrist)

                if route_arm == "left":
                        plan, success = dual_arm_cartesian_plan([waypoints6_circ], [config['speed_execution']], [waypoints6_center], [config['speed_execution']], ATC1= ATC1, sync_policy=1)
                elif route_arm == "right":
                        #visualize_keypoints_simple(waypoints6_center + waypoints6_circ, "/base_link")
                        plan, success = dual_arm_cartesian_plan([waypoints6_center], [config['speed_execution']], [waypoints6_circ], [config['speed_execution']], ATC1= ATC1, sync_policy=1)
                step2+=execute_plan_async("arms", plan)
                # arms.execute(plan)

                rot_center_jigs = get_current_pose(route_group).pose
                rot_center = antiCorrectPose(rot_center_jigs, aux_arm)
                rot_center_up = True

                actuate_grippers(config['slide_distance'], config['gripper_speed'], aux_arm, grasp=False)

        return step2


def RC_insert_grasp_corner(op, step2, config, route_group, route_arm, deep):
        global rot_center
        global rot_center_up
        global force_limit_cable
        #Apply tension (Move forward 3cm max (stopped by "force controller"))
        #Insert (Circular motion)
        #Slide and lift (till x_offset)
        #Apply tension to cable
        #Bring second hand (lift from wherever it is, move to offset, go to the corner cable position to grasp)
        #Move first arm to the end of the top of the guide
        #Save in a global variable that in the next insert, it must be done with both arms and linear motion (Instead of circular)

        step1_inc = 0

        if route_arm == "left":
                aux_group = "arm_right"
                aux_arm = "right"
        elif route_arm == "right":
                aux_group = "arm_left"
                aux_arm = "left"

        fingers_size = get_fingers_size(route_arm)
        fingers_size_aux = get_fingers_size(aux_arm)
        
        next_guide_top_beginning_route_offset1 = get_shifted_pose(op["next_guide"]["pose_corner"],[-max(fingers_size[0]/2 + config['x_offset'], fingers_size[1]/2), op["next_guide"]['gap']/2, op["next_guide"]['height'] + config['z_offset'] + fingers_size[2], 0, 0, 0])        
        prev_guide_end = get_shifted_pose(op["prev_guide"]["pose_corner"],[fingers_size[0]/2, op["next_guide"]['gap']/2, 0, 0, 0, 0])
        
        cable_dir_angle = math.atan2(next_guide_top_beginning_route_offset1.position.y - prev_guide_end.position.y, next_guide_top_beginning_route_offset1.position.x - prev_guide_end.position.x)
        next_guide_top_beginning_route_offset1.orientation = get_quaternion_in_Z(cable_dir_angle).orientation
        xy_dist = math.sqrt((next_guide_top_beginning_route_offset1.position.x - prev_guide_end.position.x)**2 + (next_guide_top_beginning_route_offset1.position.y - prev_guide_end.position.y)**2)
        z_increase = (config['x_offset'] + fingers_size[0])*((next_guide_top_beginning_route_offset1.position.z - prev_guide_end.position.z)/(xy_dist))
        next_guide_top_beginning_route_offset2 = get_shifted_pose(next_guide_top_beginning_route_offset1,[config['x_offset'] + fingers_size[0], 0, z_increase, 0, 0, 0])
        
        if step2<3:
                #Apply tension and insert
                step2, final_insert_wp = RC_insert(op, step2, config, route_group, route_arm, deep)

        if step2>=3 and step2<8:
                step2 = RC_push_inserted(step2, aux_group, aux_arm, config)

        if step2==8:
        #Slide
                actuate_grippers(config['slide_distance'], config['gripper_speed'], route_arm, grasp=False)

                waypoints3_wrist = []
                init_pose1 = get_current_pose(route_group).pose
                final_insert_wp = antiCorrectPose(init_pose1, route_arm, routing_app = True) 
                waypoints3_wrist.append(final_insert_wp)
                waypoints3_wrist.append(next_guide_top_beginning_route_offset2)
                waypoints3 = []
                for wp in waypoints3_wrist:
                        #waypoints3.append(correctPose(wp, route_arm, rotate = True, ATC_sign = -1))
                        waypoints3.append(correctPose(wp, route_arm, rotate = True, ATC_sign = -1, routing_app = True))
                waypoints3[0] = init_pose1
                #visualize_keypoints_simple([next_guide_top_beginning_route_offset2]+[next_guide_top_beginning_route_offset1]+[next_guide_top_beginning_route_visualize])
                plan, success = compute_cartesian_path_velocity_control([waypoints3], [config['speed_execution']], arm_side=route_arm)
                if success:
                        step2+=execute_plan_async(route_group, plan)
                        #arm.execute(plan, wait=True)

        if step2==9:
        #Apply tension ToDo
                actuate_grippers(config['grasp_distance'], config['gripper_speed'], route_arm, grasp=True)

                waypoints31 = []
                init_pose1 = get_current_pose(route_group).pose
                waypoints31.append(init_pose1)
                init_pose1_anticorrected = antiCorrectPose(init_pose1, route_arm, routing_app = True)
                tension_pose = get_shifted_pose(init_pose1_anticorrected,[config['force_offset'], 0, 0, 0, 0, 0]) #THE GUIDE WIDTH SHOULD BE DIVIDED BY 2, NOT BY 4
                waypoints31.append(correctPose(tension_pose, route_arm, rotate = True, ATC_sign = -1, routing_app = True))
                #Move with force_control
                plan, success = compute_cartesian_path_velocity_control([waypoints31], [config['speed_tension']], arm_side=route_arm)
                if success:
                        step2+=execute_force_control(group = route_group, plan = plan, limit = force_limit_cable, force_active=config['force_control_active'])
                rospy.sleep(0.5)

        if step2>=10:
                step2 = RC_pivoting_rotation(op, step2, config, route_group, route_arm, init_step=10)

        if step2==11:
                step2=0
                step1_inc=1

        return step2, step1_inc
                

def RC_insert_final(op, step2, config, route_group, route_arm, deep):
        global rot_center
        #Apply tension (Move forward 3cm max (stopped by "force controller"))
        #Insert (Circular motion)
        #Lift both arms

        step1_inc = 0

        if route_arm == "left":
                aux_group = "arm_right"
                aux_arm = "right"
        elif route_arm == "right":
                aux_group = "arm_left"
                aux_arm = "left"

        fingers_size = get_fingers_size(route_arm)
        fingers_size_aux = get_fingers_size(aux_arm)

        if step2<3:
                #Apply tension and insert
                step2, wp = RC_insert(op, step2, config, route_group, route_arm, deep)

        if step2>=3 and step2<8:
                step2 = RC_push_inserted(step2, aux_group, aux_arm, config)

        if step2==8:
                #Retract
                actuate_grippers(config['open_distance'], config['gripper_speed'], 'both', grasp=False)

                waypoints4_1 = []
                init_pose = get_current_pose(route_group).pose
                waypoints4_1.append(init_pose)
                init_pose_up = copy.deepcopy(init_pose)
                init_pose_up.position.z += config['z_offset']*2 + op["prev_guide"]["height"]
                waypoints4_1.append(init_pose_up)

                init_pose2 = get_current_pose(aux_group).pose
                init_pose_anticorrected = antiCorrectPose(init_pose, route_arm, routing_app = True)
                init_pose2_anticorrected = antiCorrectPose(init_pose2, aux_arm)
                if ((init_pose2_anticorrected.position.z) > (init_pose_anticorrected.position.z + op["prev_guide"]["height"])): #CHANGE THE WRISTS HAVE DIFFERENT SIZES
                        plan, success = compute_cartesian_path_velocity_control([waypoints4_1], [config['speed_execution']], arm_side=route_arm)
                        if success:
                                step2+=execute_plan_async(route_group, plan)
                else:
                        plan, success = master_slave_plan(waypoints4_1, 50.0, route_arm, type=1)
                        if success:
                                step2+=execute_plan_async("arms", plan)
                                # arms.execute(plan, wait=True)

        if step2==9:
                step2=0
                step1_inc=1

        return step2, step1_inc


def grasp_cables(op, step2, config, route_group, route_arm, separated = False):
        #global process_actionserver
        global holding_cables
        global grasping_cables

        print("Grasp cables")
        print("Step2: "+str(step2))

        #Check gripper orientation
        rotate_gripper_z = False
        if separated:
                con_pose = copy.deepcopy(op["spot"][1]["pose_corner"])
                grasp_guide_fw = get_shifted_pose(op["spot"][0]["pose_corner"], [0.01, 0, 0, 0, 0, 0])
                if compute_distance_xy(con_pose, grasp_guide_fw) < compute_distance_xy(con_pose, op["spot"][0]["pose_corner"]): #Direction of routing is not forward
                        print("GRASP BACKWARD INITIALLY")
                        grasp_guide_corner = get_shifted_pose(op["spot"][0]["pose_corner"], [op["spot"][0]["width"], op["spot"][0]["gap"], 0, 0, 0, math.pi])  
                else:
                        print("GRASP FORWARD INITIALLY")
                        grasp_guide_corner = copy.deepcopy(op["spot"][0]["pose_corner"])
        else:
                if holding_cables:
                        grasp_guide_corner = copy.deepcopy(op["spot"][0]["pose_corner"])
                else:
                        grasp_guide_corner = get_shifted_pose(op["spot"][0]["pose_corner"], [0.015, 0, -0.007, 0, 0, 0])


        #routing_app = not separated
        routing_app = True
        fingers_size = get_fingers_size(route_arm)
        if separated:
                x_grasp_dist = -(config['grasp_offset']+fingers_size[0]/2)
        else:
                x_grasp_dist = op["spot"][0]['width']+config['grasp_offset']+fingers_size[0]/2

        if separated:
                mold_up_forward = get_shifted_pose(grasp_guide_corner, [x_grasp_dist, (op["spot"][0]['gap']/2)+0.005, op["spot"][0]["height"] + 2*config['z_offset'], 0, 0, 0])
                mold_forward = get_shifted_pose(grasp_guide_corner, [x_grasp_dist, (op["spot"][0]['gap']/2)+0.005, 0.009, 0, 0, 0])
        else:
                mold_up_forward = get_shifted_pose(grasp_guide_corner, [x_grasp_dist, op["spot"][0]['gap']/2, op["spot"][0]["height"] + 2*config['z_offset'], 0, 0, 0])
                mold_forward = get_shifted_pose(grasp_guide_corner, [x_grasp_dist, op["spot"][0]['gap']/2, 0, 0, 0, 0])
        mold_up_forward_corrected = correctPose(mold_up_forward, route_arm, rotate = True, ATC_sign = -1, routing_app = routing_app, secondary_frame = True)
        
        if separated:
                mold_forward_corrected = correctPose(mold_forward, route_arm, rotate = True, ATC_sign = -1, routing_app = routing_app, secondary_frame = True)
        else:
                mold_forward_corrected = correctPose(mold_forward, route_arm, rotate = True, ATC_sign = -1, routing_app = routing_app, secondary_frame = True)

        if step2 == 0:
                #Move to grasp point offset
                actuate_grippers(config['open_distance'], config['gripper_speed'], route_arm, grasp=False)
                rospy.sleep(0.5)

                waypoints_GC1 = []
                init_pose = get_current_pose(route_group).pose
                waypoints_GC1.append(init_pose)
                waypoints_GC1.append(mold_up_forward_corrected)
                if separated:
                        plan, success, motion_group_plan = compute_cartesian_path_velocity_control_arms_occlusions([waypoints_GC1], [config['speed_execution']], arm_side=route_arm)
                else:
                        if holding_cables:
                                plan, success = compute_cartesian_path_velocity_control([waypoints_GC1], [config['fast_speed_execution']], arm_side=route_arm)
                                motion_group_plan = route_group
                        else:
                                plan, success, motion_group_plan = compute_cartesian_path_velocity_control_arms_occlusions([waypoints_GC1], [config['fast_speed_execution']], arm_side=route_arm)
                if success:
                        step2 += execute_plan_async(motion_group_plan, plan)
        
        if step2 == 1:
                waypoints_GC2 = []
                init_pose = get_current_pose(route_group).pose
                waypoints_GC2.append(init_pose)
                waypoints_GC2.append(mold_forward_corrected)
                plan, success = compute_cartesian_path_velocity_control([waypoints_GC2], [config['slow_speed_execution']], arm_side=route_arm)
                if success:
                        step2 += execute_plan_async(route_group, plan)

        if step2 == 2:
                actuate_grippers(config['slide_distance'], config['gripper_speed'], route_arm, grasp=False)
                #rospy.sleep(1) #REDUCE. Original 1
                if separated:
                        init_pose = get_current_pose(route_group).pose
                        retract_z = get_shifted_pose(grasp_guide_corner, [0, 0, op["spot"][0]["height"] + config['z_offset']*2, 0, 0, 0])
                        retract_z_corrected = correctPose(retract_z, route_arm, rotate = True, routing_app = routing_app, ATC_sign = -1, secondary_frame=True)
                        new_pose = copy.deepcopy(init_pose)
                        new_pose.position.z = retract_z_corrected.position.z
                        if route_arm=="right":
                                sign_side = 1
                        else:
                                sign_side = -1
                        side_pose = get_shifted_pose(new_pose, [0, sign_side*0.05, 0, 0, 0, 0])
                        waypoints_GC3 = [init_pose, new_pose, side_pose]
                        plan, success = compute_cartesian_path_velocity_control([waypoints_GC3], [config['speed_execution']], arm_side=route_arm)
                        if success:
                                step2 += execute_plan_async(route_group, plan)
                else:
                        grasping_cables = True
                        step2 += 1
        
        if step2 == 3:
                step2 = 0
                #process_actionserver.publish_feedback()        
        return step2


def tape(tape_guides, grasp_guide, step2, config):
        global grasping_cables

        grasping_cables = False

        if (tape_guides[0]["pose_corner"].position.x <= grasp_guide["pose_corner"].position.x) and (tape_guides[1]["pose_corner"].position.x <= grasp_guide["pose_corner"].position.x):
                tape_arm = "left"
                grasp_arm = "right"
                tape_group = "arm_left"
                grasp_group = "arm_right"
                grasp_sign_arm = 1
        else:
                tape_arm = "right"
                grasp_arm = "left"
                tape_group = "arm_right"
                grasp_group = "arm_left"
                grasp_sign_arm = -1

        fingers_size = get_fingers_size(grasp_arm)

        grasp_x_dir_test = get_shifted_pose(grasp_guide["pose_corner"], [1,0,0,0,0,0])
        if grasp_x_dir_test.position.x > grasp_x_dir_test.position.x:
                grasp_sign = grasp_sign_arm
        else:
                grasp_sign = -grasp_sign_arm

        guide_tape1 = get_shifted_pose(tape_guides[0]["pose_corner"], [tape_guides[0]['width']/2, tape_guides[0]['gap']/2, 0, 0, 0, 0])
        guide_tape2 = get_shifted_pose(tape_guides[1]["pose_corner"], [tape_guides[1]['width']/2, tape_guides[1]['gap']/2, 0, 0, 0, 0])
        tape_pose = get_middle_pose(guide_tape1, guide_tape2)
        tape_pose_offset = get_shifted_pose(tape_pose, [0, 0,  tape_guides[0]['height'] + config['z_offset'] + config['gun_nozzle_offset'], 0, 0, 0])
        grasp_pose = get_shifted_pose(grasp_guide["pose_corner"], [(grasp_guide['width']/2)+grasp_sign*(grasp_guide['width']/2 + config['grasp_offset'] + fingers_size[0]/2), grasp_guide['gap']/2, 0, 0, 0, 0])
        grasp_pose_offset = get_shifted_pose(grasp_pose, [0, 0,  grasp_guide['height'] + config['z_offset'], 0, 0, 0])

        if step2 == 0:
                #Move to grasp point offset
                step2+=retract_arm(grasp_arm, grasp_guide, config)

        if step2 == 1:
                waypoints_T1 = []
                init_pose = get_current_pose(grasp_group).pose
                waypoints_T1.append(init_pose)
                waypoints_T1.append(correctPose(grasp_pose_offset, grasp_arm, rotate = True, ATC_sign = -1, routing_app = False, secondary_frame = True))
                #visualize_keypoints_simple(waypoints_T1)
                plan, success, motion_group_plan = compute_cartesian_path_velocity_control_arms_occlusions([waypoints_T1], [config['fast_speed_execution']], arm_side=grasp_arm)
                if success:
                        step2+=execute_plan_async(motion_group_plan, plan)
                rospy.sleep(0.5)

        if step2 == 2:
                waypoints_T2 = []
                init_pose = get_current_pose(grasp_group).pose
                waypoints_T2.append(init_pose)
                waypoints_T2.append(correctPose(grasp_pose, grasp_arm, rotate = True, ATC_sign = -1, routing_app = False, secondary_frame = True))
                plan, success = compute_cartesian_path_velocity_control([waypoints_T2], [config['slow_speed_execution']], arm_side=grasp_arm)
                if success:
                        step2+=execute_plan_async(grasp_group, plan)
        
        if step2 == 3:
                actuate_grippers(config['grasp_distance'], config['gripper_speed'], grasp_arm, grasp=True)
                waypoints_T3 = []
                init_pose = get_current_pose(tape_group).pose
                waypoints_T3.append(init_pose)
                waypoints_T3.append(correctPose(tape_pose_offset, tape_arm, rotate = True, ATC_sign = -1, routing_app = True))
                plan, success = compute_cartesian_path_velocity_control([waypoints_T3], [config['speed_execution']], arm_side=tape_arm)
                if success:
                        step2+=execute_plan_async(tape_group, plan)

        if step2 == 4:
                waypoints_T4 = []
                init_pose = get_current_pose(tape_group).pose
                waypoints_T4.append(init_pose)
                waypoints_T4.append(correctPose(tape_pose, tape_arm, rotate = True, ATC_sign = -1, routing_app = True))
                plan, success = compute_cartesian_path_velocity_control([waypoints_T4], [config['slow_speed_execution']], arm_side=tape_arm)
                if success:
                        step2+=execute_plan_async(tape_group, plan)

        if step2 == 5:
                step2+=actuate_gun()
                rospy.sleep(1)

        if step2 == 6:
                waypoints_T5 = []
                init_pose = get_current_pose(tape_group).pose
                waypoints_T5.append(init_pose)
                waypoints_T5.append(correctPose(tape_pose_offset, tape_arm, rotate = True, ATC_sign = -1, routing_app = True))
                plan, success = compute_cartesian_path_velocity_control([waypoints_T5], [config['speed_execution']], arm_side=tape_arm)
                if success:
                        step2+=execute_plan_async(tape_group, plan)
        
        if step2 == 7:
                step2+=retract_arm(grasp_arm, grasp_guide, config)
                
        if step2 == 8:
                step2=0
        return step2

                #visualize_keypoints_simple([tape_pose, tape_pose_offset])
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

def get_stampedpose(pose, group):
        rospy.wait_for_service('/adv_manip/get_stampedpose')
        get_stamped_srv = rospy.ServiceProxy('/adv_manip/get_stampedpose', GetStampedPose)
        req = GetStampedPoseRequest()
        req.pose = pose; req.group = group
        return get_stamped_srv(req).poseSt

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

def dual_arm_cartesian_plan(waypoints_left=[], EE_speed_L=0, waypoints_right=[], EE_speed_R=0, sync_policy=1):
        rospy.wait_for_service('/adv_manip/dual_arm_cartesian_plan')
        compute_path_dual_srv = rospy.ServiceProxy('/adv_manip/dual_arm_cartesian_plan', ComputePathDual)
        req = ComputePathDualRequest()
        for list_wp_L in waypoints_left:
                msg = float_list()
                for wp in list_wp_L:
                        msg.data.append(wp)
                req.waypoints_list_L.append(msg) 
        for list_wp_R in waypoints_right:
                msg = float_list()
                for wp in list_wp_R:
                        msg.data.append(wp)
                req.waypoints_list_R.append(msg) 
        req.EE_speed_L = EE_speed_L; req.EE_speed_R = EE_speed_R; req.sync_policy = sync_policy
        resp = compute_path_dual_srv(req)
        return resp.plan, resp.success

def master_slave_plan(list_wp, EE_speed, arm_side, type=1):
        rospy.wait_for_service('/adv_manip/master_slave_plan')
        master_slave_plan_srv = rospy.ServiceProxy('/adv_manip/master_slave_plan', ComputeMasterSlavePath)
        req = ComputeMasterSlavePathRequest()
        req.waypoints_list = list_wp; req.EE_speed = EE_speed; req.arm_side = arm_side; req.sync_policy = type
        resp = master_slave_plan_srv(req)
        return resp.plan, resp.success

#ROS actions
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

def get_tool(side):
        rospy.wait_for_service('/adv_manip/get_tool')
        tool_srv = rospy.ServiceProxy('/adv_manip/get_tool', StringSrv)
        req = StringSrvRequest()
        req.data = side
        return tool_srv(req.data).data

def change_tool(new_tool, side):
        print("CHANGE TOOL")
        client=actionlib.SimpleActionClient('/adv_manip/ATC', ATCAction) #Stablishes the connection with the server
        client.wait_for_server()
        goal = ATCGoal()
        goal.tool = new_tool; goal.side = side
        client.send_goal(goal)
        success = False
        atc_done = False
        while not atc_done:
                rospy.sleep(0.05)
                state = client.get_state()
                if state==2 or state == 3 or state == 4:
                        atc_done = True
                if state == 3:
                        success = True
        return success

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
        success = False
        while not eef_done:
                rospy.sleep(0.05)
                state = client.get_state()
                if state==2 or state == 3 or state == 4:
                        eef_done = True
                if state == 3:
                        success = True
        eef_done = False
        return success

def actuate_grippers(distance, speed, arm, grasp=False):
        success = actuate_eef(0, distance, speed, arm, grasp)

def actuate_gun():
        success = actuate_eef(1)
        if success:
                return 1
        else:
                return 0

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
                print(op_i['type'])
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


def check_tools(op, config, step2):
        success = True
        if op['type'] != 'T':
                if get_tool("right") == "EEF_taping_gun":
                        #MOVE TO A CONFIGURATION WITH THE GUN HORIZONTAL IN SEVERAL STEPS TO AVOID COLLISION (MOVE CONFIG)
                        success = change_tool("EEF_gripper_right", "right")
                        step2 = 0
                elif get_tool("left") =="EEF_taping_gun":
                        #MOVE TO A CONFIGURATION WITH THE GUN HORIZONTAL IN SEVERAL STEPS TO AVOID COLLISION (MOVE CONFIG)
                        success = change_tool("EEF_gripper_left", "left")
                        step2 = 0
        else:
                print("TAPE")
                if (op["spot"][0]["pose_corner"].position.x <= op["spot"][2]["pose_corner"].position.x) and (op["spot"][1]["pose_corner"].position.x <= op["spot"][2]["pose_corner"].position.x):   #Tape with left
                        #LEFT ARM TAPES
                        if get_tool("right")=="EEF_taping_gun":
                                print("HAVE TO CHANGE RIGHT TOOL")
                                success = change_tool("EEF_gripper_right", "right")
                                step2 = 0
                        if get_tool("left")=="EEF_gripper_left":
                                print("HAVE TO CHANGE LEFT TOOL")
                                actuate_grippers(config['grasp_distance'], config['gripper_speed'], "left", grasp=False)
                                success = change_tool("EEF_taping_gun", "left")
                                step2 = 0
                else: #Tape with right
                        if get_tool("left")=="EEF_taping_gun":
                                success = change_tool("EEF_gripper_left", "left")
                                step2 = 0
                        if get_tool("right")=="EEF_gripper_right":
                                success = change_tool("EEF_taping_gun", "right")
                                step2 = 0
                                #MOVE TO A CONFIGURATION WITH THE GUN VERTICAL IN SEVERAL STEPS TO AVOID COLLISION (MOVE CONFIG)
        if not success:
                stop_function("ATC ERROR")
        print(step2)
        print(success)
        return step2, success

def execute_operation(op):
        global PC_op
        global route_arm
        global move_away2_sign
        global route_group
        global group2 
        global step1
        global step2 
        global config2

        step2, success = check_tools(op, config2, step2)
        print(step2)
        print(success)

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
        
        elif op['type'] == "RC":
                PC_op = get_last_connector_info(op)
                update_route_arm_info(PC_op)
                step1, step2 = route_cables(op, step1, step2, config2, route_group, route_arm, PC_op)

        elif op['type'] == "GC" or op['type'] == "GCS":
                PC_op = get_last_connector_info(op)
                update_route_arm_info(PC_op)
                if op['type'] == "GC":
                        step2 = grasp_cables(op, step2, config2, route_group, route_arm)
                else:
                        step2 = grasp_cables(op, step2, config2, route_group, route_arm, separated=True)

        elif op['type'] == "T":
                PC_op = get_last_connector_info(op)
                update_route_arm_info(PC_op)
                step2 = tape(op["spot"][:2],op["spot"][2], step2, config2)


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

step1 = 0
step2 = 0
op = 3
set_named_target('arms', "arms_platform_5")
move_group_async("arms")
rospy.sleep(0.5)
print("READY")
execute_operation(ops_info[op])