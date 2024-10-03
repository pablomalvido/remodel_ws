#! /usr/bin/env python

import copy
import rospy
import PyKDL 
import rospkg
import geometry_msgs.msg
from utils import *
from elvez_pkg.msg import *
from elvez_pkg.srv import *
from task_planner_pkg.srv import *

rospack = rospkg.RosPack()
rospy.init_node('test_node', anonymous=True)

print("START")
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
print(def_motion_groups_srv(req))

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
print(def_EEF_srv(req))
req.EE_end_frame = frame_to_pose(gripper_end_frame_R); req.EE_end_frame2 = frame_to_pose(gripper_end_frame_R2); req.x = 0.073; req.y = 0.025; req.z = 0.24; req.fingers_dim = fingers_dim; req.ATC_frame = frame_to_pose(gripper_right_ATC_frame); req.name = "EEF_gripper_right"; req.path = EE_file_path_gripper_R
def_EEF_srv(req)
req.EE_end_frame = frame_to_pose(gun_end_frame); req.EE_end_frame2 = Pose(); req.x = 0.35; req.y = 0.3; req.z = 0.1; req.fingers_dim = []; req.ATC_frame = frame_to_pose(gun_ATC_frame); req.name = "EEF_taping_gun"; req.path = EE_file_path_gun
def_EEF_srv(req)

req = DefATCRequest()
req.left_tool="EEF_gripper_left"; req.right_tool="EEF_gripper_right"; req.eef_link_left=eef_link_left; req.eef_link_right=eef_link_right; req.ATC_tools=["EEF_taping_gun"]; req.left_ATC_angle=0.7854; req.right_ATC_angle=-0.7854; req.left_ATC_dist=0.083; req.right_ATC_dist=0.054
print(def_ATC_srv(req))

req = MotionGroupsCommandRequest()
req.group = 'arms'; req.target = "arms_platform_4"; req.wait = True
print(set_named_target_srv(req))
go_srv(req)

req.target = "arms_platform_5"
set_named_target_srv(req)
go_srv(req)
