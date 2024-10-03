#! /usr/bin/env python

import copy
import rospy
import PyKDL 
import moveit_commander
import rospkg
from moveit_msgs.msg import *
from moveit_msgs.srv import *
import geometry_msgs.msg
from utils import *
from operations_test import *
from elvez_pkg.msg import *
from elvez_pkg.srv import *
from task_planner_pkg.srv import *

rospack = rospkg.RosPack()
rospy.init_node('test_node', anonymous=True)

rospy.wait_for_service('/adv_manip/define_motion_groups')
def_motion_groups_srv = rospy.ServiceProxy('/adv_manip/define_motion_groups', DefMotionGroups)
rospy.wait_for_service('/adv_manip/max_speed_groups')
def_max_speed_srv = rospy.ServiceProxy('/adv_manip/max_speed_groups', MaxSpeedGroups)
rospy.wait_for_service('/adv_manip/def_EEF')
def_EEF_srv = rospy.ServiceProxy('/adv_manip/def_EEF', DefEEF)
rospy.wait_for_service('/adv_manip/def_ATC')
def_ATC_srv = rospy.ServiceProxy('/adv_manip/def_ATC', DefATC)
rospy.wait_for_service('/adv_manip/set_named_target')
set_named_target_srv = rospy.ServiceProxy('/adv_manip/set_named_target', MotionGroupsCommand)
rospy.wait_for_service('/adv_manip/go')
go_srv = rospy.ServiceProxy('/adv_manip/go', MotionGroupsCommand)

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
req.EE_end_frame = gripper_end_frame_L; req.EE_end_frame2 = gripper_end_frame_L2; req.x = 0.073; req.y = 0.025; req.z = 0.24; req.fingers_dim = fingers_dim; req.ATC_frame = gripper_left_ATC_frame; req.name = "EEF_gripper_left"; req.path = EE_file_path_gripper_L
def_EEF_srv(req)
req.EE_end_frame = gripper_end_frame_R; req.EE_end_frame2 = gripper_end_frame_R2; req.x = 0.073; req.y = 0.025; req.z = 0.24; req.fingers_dim = fingers_dim; req.ATC_frame = gripper_right_ATC_frame; req.name = "EEF_gripper_right"; req.path = EE_file_path_gripper_R
def_EEF_srv(req)
req.EE_end_frame = gun_end_frame; req.EE_end_frame2 = Pose(); req.x = 0.35; req.y = 0.3; req.z = 0.1; req.fingers_dim = []; req.ATC_frame = gun_ATC_frame; req.name = "EEF_taping_gun"; req.path = EE_file_path_gun
def_EEF_srv(req)

req = DefATCRequest()
req.left_tool="EEF_gripper_left"; req.right_tool="EEF_gripper_right"; req.eef_link_left=eef_link_left; req.eef_link_right=eef_link_right; req.ATC_tools=["EEF_taping_gun"]; req.left_ATC_angle=0.7854; req.right_ATC_angle=-0.7854; req.left_ATC_dist=0.083; req.right_ATC_dist=0.054
def_ATC_srv(req)

#####################
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
        global config

        if op['type'] == "EC":
            step2, step1_inc = EC(op, step2, config, route_group)              

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

#####################################
print("CCC")

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

print(ops_info)

config={'grasp_offset':0.05, 'gripper_speed':30, 'open_distance':90, 'slide_distance':40, 'grasp_distance':30}

init_grippers(False)
step1 = 0
step2 = 0
execute_operation(ops_info[step1])