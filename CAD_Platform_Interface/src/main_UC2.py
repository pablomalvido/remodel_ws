#!/usr/bin/env python2

import rospy
import PyKDL 
import numpy as np
import math 
import tf 
import time
import os
import copy
from elvez_platform import Scene, Transform, Platform, ItemID
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray 
from collectData import InputFilesDataCollector
import visualization as visualization
from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger, TriggerResponse
from elvez_pkg.msg import *
from elvez_pkg.srv import *

#Create a publisher for the markers of the components of the ELVEZ platform
topic = 'visualization_marker_array' 
publisher = rospy.Publisher(topic, MarkerArray, queue_size=1)

#Initialize ROS node 
rospy.init_node('UC2_handler') 
rate = rospy.Rate(10)

#Create broacaster and listener 
br = tf.TransformBroadcaster() 
listener = tf.TransformListener() 

#Obtain input parameters from the launcher
base_frame = rospy.get_param('~base_frame', "") 
files_path_rel = rospy.get_param('~files_path_rel', "") 
files_path = os.path.join(os.path.dirname(__file__), "../" + files_path_rel)
cad_name = rospy.get_param('~cad_name', "")
cad_name_combs = rospy.get_param('~cad_name_combs', "")
cad_name_ATC = rospy.get_param('~cad_name_ATC', "")
ids_file = os.path.join(files_path, cad_name + "_ids.wri")
if cad_name_combs != "":
    ids_combs_file = os.path.join(files_path, cad_name_combs + "_ids.wri")
else:
    ids_combs_file = ""
if cad_name_ATC != "":
    ids_ATC_file = os.path.join(files_path, cad_name_ATC + "_ids.wri")
else:
    ids_ATC_file = ""
jigs_file = rospy.get_param('~jigs_file', "")
components_file = rospy.get_param('~components_file', "")
WH_file = rospy.get_param('~WH_file', "")
sequence_file = rospy.get_param('~seq_file', "")
markerArray = MarkerArray() 
package_path = 'file://' + files_path + 'stl/'  

#Extract data from input files
platform = Platform(cad_name, files_path)
transforms = platform.useful_transforms
if cad_name_combs != "":
    combs = Platform(cad_name_combs, files_path)
    transforms_combs = combs.useful_transforms
else:
    combs = []
    transforms_combs = []
if cad_name_ATC != "":
    ATC = Platform(cad_name_ATC, files_path)
    transforms_ATC = ATC.useful_transforms
else:
    ATC = []
    transforms_ATC = []
dict_elvez = InputFilesDataCollector(files_path, ids_file, ids_combs_file, ids_ATC_file, jigs_file, components_file, WH_file, sequence_file)
dict_elvez.showInfo()  #Print the extracted information from the input files
jigs_complete_dict = {}  #Variable filled with the info returned by the create_jigs_struct() function

#print(dict_elvez.dict_jigs)

#Function for broadcasting transforms
def broadcastTransform(br, frame, frame_id, parent_frame, time=rospy.get_rostime()): 
    br.sendTransform((frame.p.x(), frame.p.y(), frame.p.z()), 
        frame.M.GetQuaternion(), 
        time, 
        frame_id, 
        parent_frame) 


#Function for extracting the frame info of a transform
def extract_frame(trans = PyKDL.Frame()):
    frame = PyKDL.Frame()
    frame.p = trans.p
    frame.M = trans.M
    return frame


#Function for transforming the tf returned by a listener to a KDL frame
def tfToKDL(tf): 
    frame = PyKDL.Frame() 
    frame.p.x(tf[0][0]) 
    frame.p.y(tf[0][1]) 
    frame.p.z(tf[0][2]) 
    frame.M = PyKDL.Rotation.Quaternion( 
        tf[1][0], 
        tf[1][1], 
        tf[1][2], 
        tf[1][3] 
    ) 
    return frame 

#Function for transforming a kdl frame in a Pose (for sending it in msgs and srvs)
def fromKdlToPose(kdl_frame): 
    pose = Pose() 
    pose.position.x = kdl_frame.p[0] 
    pose.position.y = kdl_frame.p[1] 
    pose.position.z = kdl_frame.p[2] 
    ang = kdl_frame.M.GetQuaternion() 
    pose.orientation.x = ang[0] 
    pose.orientation.y = ang[1] 
    pose.orientation.z = ang[2] 
    pose.orientation.w = ang[3] 
    return pose

#Generation of a complete dictionary for the jigs referring all their frames to the base_link
def create_jigs_struct():
    """
    This function returns a dictionary with all the info for all the jigs, i.e. the frame of its down left corner seen from the base_link, its dimensions and all the information about their guides and taping spots with all their frames referred to the base_link
    """
    jig_full_dict = {}

#It tries to listen until it gets a value
    get_listener = False
    while not get_listener:
        try:
            tf_platform = listener.lookupTransform(base_frame, '/platform_rf', rospy.Time(0)) 
            get_listener = True
        except:
            get_listener = False
            time.sleep(0.05)
            print("Not yet. Platform")

    if cad_name_combs != "":
        get_listener2 = False
        while not get_listener2:
            try:
                tf_combs = listener.lookupTransform(base_frame, '/combs_rf', rospy.Time(0)) 
                get_listener2 = True
            except:
                get_listener2 = False
                time.sleep(0.05)
                print("Not yet. Combs") 

    if cad_name_ATC != "":
        get_listener3 = False
        while not get_listener3:
            try:
                tf_ATC = listener.lookupTransform(base_frame, '/ATC_rf', rospy.Time(0)) 
                get_listener3 = True
            except:
                get_listener3 = False
                time.sleep(0.05)
                print("Not yet. ATC") 

    for trans in transforms: #Useful transforms
        if trans.getID().getType() == 1:  #If it is a jig
            if(trans.getCommercial() in dict_elvez.dict_jigs):
                jig_temp_dict = {}
                
                #It goes through all the chain of transformations until the base_link 
                jig_frame_frombase = extract_frame(trans)
                parent_trans = trans.parent
                while parent_trans.isUseful():
                    parent_frame = extract_frame(parent_trans)
                    jig_frame_frombase = parent_frame * jig_frame_frombase
                    parent_trans = parent_trans.parent
                #jig_frame_frombase = tfToKDL(tf_platform) * jig_frame_frombase
                jig_frame_frombase = tfToKDL(tf_platform) * jig_frame_frombase

                label = trans.getID().getLabel()
                commercial = trans.getID().getCommercialID()
                dimensions = [dict_elvez.dict_jigs[commercial]['xdim'], dict_elvez.dict_jigs[commercial]['ydim'], dict_elvez.dict_jigs[commercial]['zdim']]
                collisions = [dict_elvez.dict_jigs[commercial]['xcol1'], dict_elvez.dict_jigs[commercial]['xcol2'], dict_elvez.dict_jigs[commercial]['ycol1'], dict_elvez.dict_jigs[commercial]['ycol2']]
                
                if 'guides' in dict_elvez.dict_jigs[trans.getCommercial()]:
                    guides_dic = {}
                    for guide in dict_elvez.dict_jigs[trans.getCommercial()]['guides']:
                        guides_dic[guide] = copy.deepcopy(dict_elvez.dict_jigs[trans.getCommercial()]['guides'][guide])
                        guides_dic[guide]['key']['frame'] = jig_frame_frombase * guides_dic[guide]['key']['frame']
                        guides_dic[guide]['key']['center_pose'] = jig_frame_frombase * guides_dic[guide]['key']['center_pose']
                        guides_dic[guide]['collision']['frame'] = jig_frame_frombase * guides_dic[guide]['collision']['frame']
                    jig_temp_dict['guides'] = guides_dic
                
                if 'tape_spots' in dict_elvez.dict_jigs[trans.getCommercial()]:
                    tape_dic = {}
                    for tape in dict_elvez.dict_jigs[trans.getCommercial()]['tape_spots']:
                        tape_dic[tape] = copy.deepcopy(dict_elvez.dict_jigs[trans.getCommercial()]['tape_spots'][tape])
                        tape_dic[tape]['frame'] = jig_frame_frombase * tape_dic[tape]['frame']
                        tape_dic[tape]['center_pose'] = jig_frame_frombase * tape_dic[tape]['center_pose']
                    jig_temp_dict['tape_spots'] = tape_dic
                
                jig_temp_dict['dimensions'] = dimensions
                jig_temp_dict['collisions'] = collisions
                jig_temp_dict['jig_frame'] = jig_frame_frombase
                jig_temp_dict['commercial'] = commercial
                jig_temp_dict['Type'] = "J"
                
                jig_full_dict[label] = jig_temp_dict

        if trans.getID().getType() == 2:  #If it is a box
            if(trans.getCommercial() in dict_elvez.dict_jigs):
                box_temp_dict = {}
                
                #It goes through all the chain of transformations until the base_link 
                box_frame_frombase = extract_frame(trans)
                parent_trans = trans.parent
                while parent_trans.isUseful():
                    parent_frame = extract_frame(parent_trans)
                    box_frame_frombase = parent_frame * box_frame_frombase
                    parent_trans = parent_trans.parent
                box_frame_frombase = tfToKDL(tf_platform) * box_frame_frombase

                label = trans.getID().getLabel()
                commercial = trans.getID().getCommercialID()
                dimensions = [dict_elvez.dict_jigs[commercial]['xdim'], dict_elvez.dict_jigs[commercial]['ydim'], dict_elvez.dict_jigs[commercial]['zdim']]
                
                if 'trays' in dict_elvez.dict_jigs[trans.getCommercial()]:
                    trays_dic = {}
                    for tray in dict_elvez.dict_jigs[trans.getCommercial()]['trays']:
                        trays_dic[tray] = copy.deepcopy(dict_elvez.dict_jigs[trans.getCommercial()]['trays'][tray])
                        trays_dic[tray]['frame'] = box_frame_frombase * trays_dic[tray]['frame']
                        trays_dic[tray]['center_pose'] = box_frame_frombase * trays_dic[tray]['center_pose']
                    box_temp_dict['trays'] = trays_dic
                
                box_temp_dict['dimensions'] = dimensions
                box_temp_dict['box_frame'] = box_frame_frombase
                box_temp_dict['commercial'] = commercial
                box_temp_dict['Type'] = "B"

                jig_full_dict[label] = box_temp_dict

    for trans in transforms_combs: #Useful transforms combs
        if trans.getID().getType() == 3:  #If it is a comb
            if(trans.getCommercial() in dict_elvez.dict_jigs):
                comb_temp_dict = {}
                
                #It goes through all the chain of transformations until the base_link 
                comb_frame_frombase = extract_frame(trans)
                parent_trans = trans.parent
                while parent_trans.isUseful():
                    parent_frame = extract_frame(parent_trans)
                    comb_frame_frombase = parent_frame * comb_frame_frombase
                    parent_trans = parent_trans.parent
                #jig_frame_frombase = tfToKDL(tf_platform) * jig_frame_frombase
                comb_frame_frombase = tfToKDL(tf_combs) * comb_frame_frombase

                label = trans.getID().getLabel()
                commercial = trans.getID().getCommercialID()
                dimensions = [dict_elvez.dict_jigs[commercial]['xdim'], dict_elvez.dict_jigs[commercial]['ydim'], dict_elvez.dict_jigs[commercial]['zdim']]
                collisions = [dict_elvez.dict_jigs[commercial]['xcol1'], dict_elvez.dict_jigs[commercial]['xcol2'], dict_elvez.dict_jigs[commercial]['ycol1'], dict_elvez.dict_jigs[commercial]['ycol2']]
                
                if 'guides' in dict_elvez.dict_jigs[trans.getCommercial()]:
                    guides_dic = {}
                    for guide in dict_elvez.dict_jigs[trans.getCommercial()]['guides']:
                        guides_dic[guide] = copy.deepcopy(dict_elvez.dict_jigs[trans.getCommercial()]['guides'][guide])
                        guides_dic[guide]['key']['frame'] = comb_frame_frombase * guides_dic[guide]['key']['frame'] #From local to global
                        guides_dic[guide]['key']['center_pose'] = comb_frame_frombase * guides_dic[guide]['key']['center_pose']
                        guides_dic[guide]['collision']['frame'] = comb_frame_frombase * guides_dic[guide]['collision']['frame']
                    comb_temp_dict['guides'] = guides_dic
                
                
                comb_temp_dict['dimensions'] = dimensions
                comb_temp_dict['collisions'] = collisions
                comb_temp_dict['jig_frame'] = comb_frame_frombase #comb_frame?
                comb_temp_dict['commercial'] = commercial
                comb_temp_dict['Type'] = "C"
                
                jig_full_dict[label] = comb_temp_dict

    for trans in transforms_ATC: #Useful transforms ATC
        if trans.getID().getType() == 4:  #If it is a ATC station
            if(trans.getCommercial() in dict_elvez.dict_jigs):
                ATC_temp_dict = {}
                
                #It goes through all the chain of transformations until the base_link 
                ATC_frame_frombase = extract_frame(trans)
                parent_trans = trans.parent
                while parent_trans.isUseful():
                    parent_frame = extract_frame(parent_trans)
                    ATC_frame_frombase = parent_frame * ATC_frame_frombase
                    parent_trans = parent_trans.parent
                ATC_frame_frombase = tfToKDL(tf_ATC) * ATC_frame_frombase

                label = trans.getID().getLabel() #To remove the A at the beginning and have the name of the tool
                commercial = trans.getID().getCommercialID()
                ATC_temp_dict['dimensions'] = [dict_elvez.dict_jigs[commercial]['xdim'], dict_elvez.dict_jigs[commercial]['ydim'], dict_elvez.dict_jigs[commercial]['zdim']]
                ATC_temp_dict['dimensions_tool'] = [dict_elvez.dict_jigs[commercial]['x_tool'], dict_elvez.dict_jigs[commercial]['y_tool'], dict_elvez.dict_jigs[commercial]['z_tool']]

                ATC_temp_dict['tool_type'] = dict_elvez.dict_jigs[commercial]['tool']

                if ATC_temp_dict['tool_type'] == 'gripper':
                    ATC_temp_dict['frame_nail'] = dict_elvez.dict_jigs[trans.getCommercial()]['frame_nail']
                    ATC_temp_dict['dimensions_fingers'] = [dict_elvez.dict_jigs[commercial]['finger_length'], dict_elvez.dict_jigs[commercial]['finger_width'], dict_elvez.dict_jigs[commercial]['finger_height']]
                        
                ATC_temp_dict['frame_base'] = ATC_frame_frombase * dict_elvez.dict_jigs[trans.getCommercial()]['frame_base'] #local to global. THE MOST IMPORTANT FOR THE ATC
                ATC_temp_dict['frame_end'] = dict_elvez.dict_jigs[trans.getCommercial()]['frame_end'] #This stays local, seen from the base
                ATC_temp_dict['commercial'] = commercial
                ATC_temp_dict['tool_name'] = trans.getID().getLabel()[1:] #To remove the A at the beginning and have the name of the tool
                ATC_temp_dict['Type'] = "A"
                    
                jig_full_dict[label] = ATC_temp_dict
                

    return jig_full_dict

jigs_complete_dict = create_jigs_struct()
#print(jigs_complete_dict["C1"])
#print("AAA")
print(jigs_complete_dict)


#RVIZ visualization
for trans in transforms: 
    parent = trans.parent 
    path = package_path + "platform/" + trans.getID().getCommercialID() + ".STL" 
    color = visualization.Color(0.5, 0.5, 0.5, 1)
    scale=trans.scale
    #scale=np.array([0.001, 0.001, 0.001])
    #scale=np.array([1, 1, 1]) 
    if parent.isUseful():
        parent_name = parent.getName()
    else:
        parent_name = "platform_rf"
    marker = visualization.createMesh(parent_name, mesh_path=path, transform=trans, color=color, scale=scale) 
    marker.id = len(markerArray.markers) 
    marker.text = trans.item_id.id_list[0] 
    markerArray.markers.append(marker)

#Keypoints in RVIZ
    if(trans.getCommercial() in dict_elvez.dict_jigs):
    #print(trans.getCommercial())
    #print(dict_elvez.dict_jigs[trans.getCommercial()]['guides'])
        if(trans.getID().getType() == 1):
            if 'guides' in dict_elvez.dict_jigs[trans.getCommercial()]:
                for guide in dict_elvez.dict_jigs[trans.getCommercial()]['guides']:
                    #print(guide)
                    frame = PyKDL.Frame()
                    #frame = dict_elvez.dict_jigs[trans.getCommercial()]['guides'][guide]['key']['frame']
                    frame = dict_elvez.dict_jigs[trans.getCommercial()]['guides'][guide]['key']['center_pose']
                    #print(frame)
                    color = visualization.Color(1, 0, 0, 1)
                    #scaleKP=np.array([0.001, 0.001, 0.001])
                    scaleKP=np.array([1, 1, 1])
                    keypoint = visualization.createKeypoint(frame_id=trans.getName(), transform=frame, scale=scaleKP, color=color)
                    keypoint.id = len(markerArray.markers)
                    keypoint.lifetime = rospy.Duration(0)
                    keypoint.text = "Keypoint guide"
                    markerArray.markers.append(keypoint)
            if 'tape_spots' in dict_elvez.dict_jigs[trans.getCommercial()]:
                for tape in dict_elvez.dict_jigs[trans.getCommercial()]['tape_spots']:
                    #print(guide)
                    frame = PyKDL.Frame()
                    #frame = dict_elvez.dict_jigs[trans.getCommercial()]['tape_spots'][tape]['frame']
                    frame = dict_elvez.dict_jigs[trans.getCommercial()]['tape_spots'][tape]['center_pose']
                    #print(frame)
                    color = visualization.Color(0, 0, 1, 1)
                    #scaleKP=np.array([0.001, 0.001, 0.001])
                    scaleKP=np.array([1, 1, 1])
                    keypoint = visualization.createKeypoint(frame_id=trans.getName(), transform=frame, scale=scaleKP, color=color)
                    keypoint.id = len(markerArray.markers)
                    keypoint.lifetime = rospy.Duration(0)
                    keypoint.text = "Keypoint tape"
                    markerArray.markers.append(keypoint)
        elif(trans.getID().getType() == 2):
            if 'trays' in dict_elvez.dict_jigs[trans.getCommercial()]:
                for tray in dict_elvez.dict_jigs[trans.getCommercial()]['trays']:
                    frame = PyKDL.Frame()
                    frame = dict_elvez.dict_jigs[trans.getCommercial()]['trays'][tray]['center_pose']
                    color = visualization.Color(0, 1, 0, 1)
                    scaleKP=np.array([1, 1, 1])
                    keypoint = visualization.createKeypoint(frame_id=trans.getName(), transform=frame, scale=scaleKP, color=color)
                    keypoint.id = len(markerArray.markers)
                    keypoint.lifetime = rospy.Duration(0)
                    keypoint.text = "Keypoint tray"
                    markerArray.markers.append(keypoint)
    """
    if(trans.getName()=="ID000004"):
    print("ID000004 detected")
        guide = PyKDL.Frame()
        guide.p = PyKDL.Vector(34, 5.94, 165)
        guide.M = PyKDL.Rotation.Quaternion(0, 0, 0, 1)
        scaleKP=np.array([0.001, 0.001, 0.001])
    #scaleKP=np.array([1, 1, 1])
        keypoint = visualization.createKeypoint(frame_id=trans.getName(), transform=guide, scale=scaleKP)
        keypoint.id = len(markerArray.markers)
    keypoint.lifetime = rospy.Duration(0)
    keypoint.text = "Keypoint"
        markerArray.markers.append(keypoint)
    """
if cad_name_combs != "":
  for trans in transforms_combs: 
    parent = trans.parent 
    path = package_path + "combs/" + trans.getID().getCadID() + ".STL" 
    color = visualization.Color(0.5, 0.5, 0.5, 1)
    scale=trans.scale
    #scale=np.array([0.001, 0.001, 0.001])
    #scale=np.array([1, 1, 1]) 
    if parent.isUseful():
        parent_name = parent.getName()
    else:
        parent_name = "combs_rf"
    marker = visualization.createMesh(parent_name, mesh_path=path, transform=trans, color=color, scale=scale) 
    marker.id = len(markerArray.markers) 
    marker.text = trans.item_id.id_list[0] 
    markerArray.markers.append(marker)

#Keypoints in RVIZ
    if(trans.getCommercial() in dict_elvez.dict_jigs):
    #print(trans.getCommercial())
    #print(dict_elvez.dict_jigs[trans.getCommercial()]['guides'])
        if(trans.getID().getType() == 3):
            if 'guides' in dict_elvez.dict_jigs[trans.getCommercial()]:
                for guide in dict_elvez.dict_jigs[trans.getCommercial()]['guides']:
                    #print(guide)
                    frame = PyKDL.Frame()
                    #frame = dict_elvez.dict_jigs[trans.getCommercial()]['guides'][guide]['key']['frame']
                    frame = dict_elvez.dict_jigs[trans.getCommercial()]['guides'][guide]['key']['center_pose']
                    #print(frame)
                    color = visualization.Color(1, 0, 0, 1)
                    #scaleKP=np.array([0.001, 0.001, 0.001])
                    scaleKP=np.array([1, 1, 1])
                    keypoint = visualization.createKeypoint(frame_id=trans.getName(), transform=frame, scale=scaleKP, color=color)
                    keypoint.id = len(markerArray.markers)
                    keypoint.lifetime = rospy.Duration(0)
                    keypoint.text = "Keypoint guide"
                    markerArray.markers.append(keypoint)

if cad_name_ATC != "":
  for trans in transforms_ATC: 
    parent = trans.parent 
    path = package_path + "ATC/" + trans.getID().getCadID() + ".STL" 
    color = visualization.Color(0.5, 0.5, 0.5, 1)
    scale=trans.scale
    #scale=np.array([0.001, 0.001, 0.001])
    #scale=np.array([1, 1, 1]) 
    if parent.isUseful():
        parent_name = parent.getName()
    else:
        parent_name = "ATC_rf"
    marker = visualization.createMesh(parent_name, mesh_path=path, transform=trans, color=color, scale=scale) 
    marker.id = len(markerArray.markers) 
    marker.text = trans.item_id.id_list[0] 
    markerArray.markers.append(marker)

#Keypoints in RVIZ
    if(trans.getCommercial() in dict_elvez.dict_jigs):
        if(trans.getID().getType() == 4):
            frame = PyKDL.Frame()
            frame = dict_elvez.dict_jigs[trans.getCommercial()]['frame_base']
            color = visualization.Color(1, 0, 1, 1)
            #scaleKP=np.array([0.001, 0.001, 0.001])
            scaleKP=np.array([1, 1, 1])
            keypoint = visualization.createKeypoint(frame_id=trans.getName(), transform=frame, scale=scaleKP, color=color)
            keypoint.id = len(markerArray.markers)
            keypoint.lifetime = rospy.Duration(0)
            keypoint.text = "Keypoint tool base"
            markerArray.markers.append(keypoint)
    """
#Check the transforms generated by the create_jigs_structure() function
for jig in jigs_complete_dict:
    frame_check = jigs_complete_dict[jig]['jig_frame']
    color = visualization.Color(0, 1, 0, 1)
    scaleKP=np.array([1, 1, 1])
    marker_check = visualization.createKeypoint(frame_id='/platform_rf', transform=frame_check, scale=scaleKP, color=color)
    marker_check.id = len(markerArray.markers)
    marker_check.lifetime = rospy.Duration(0)
    marker_check.text = "Check"
    markerArray.markers.append(marker_check)

    frame_check = jigs_complete_dict[jig]['guides']['1']['key']['frame']
    marker_check = visualization.createKeypoint(frame_id='/platform_rf', transform=frame_check, scale=scaleKP, color=color)
    marker_check.id = len(markerArray.markers)
    marker_check.lifetime = rospy.Duration(0)
    marker_check.text = "Check"
    markerArray.markers.append(marker_check)
    """


#SERVICES
#Global variables
operation_index = 0
#Services for identifying WH components with the vision system
connector_info_service = 'ELVEZ_platform_handler/connector_info'
cable_info_service = 'ELVEZ_platform_handler/cable_info'
#Services for providing info about the keypoints of the ELVEZ platform
tray_info_service = 'ELVEZ_platform_handler/tray_info'
jig_info_service = 'ELVEZ_platform_handler/jig_info'
guide_info_service = 'ELVEZ_platform_handler/guide_info'
taping_spot_info_service = 'ELVEZ_platform_handler/taping_spot_info'
all_guides_service = 'ELVEZ_platform_handler/all_guides'
#Services for going through the operation sequence list
next_operation_service = 'ELVEZ_platform_handler/next_operation'
reset_sequence_list_service = 'ELVEZ_platform_handler/reset_sequence_list'
reset_sequence_list_to_service = 'ELVEZ_platform_handler/reset_sequence_list_to'
index_operation_service = 'ELVEZ_platform_handler/index_operation'
all_operations_service = 'ELVEZ_platform_handler/all_operations'
all_cad_components_service = 'ELVEZ_platform_handler/all_cad_components'
tool_info_service = 'ELVEZ_platform_handler/tool_info'


#Define callback services
def connector_info_callback(req): 
    """
    Service that returns information about the required connector or devices for its identification with the vision system
    """
    resp = connector_infoResponse()
    if req.label in dict_elvez.dict_components['connector']:
        resp.component = 'CON'
        resp.color = dict_elvez.dict_components['connector'][req.label]['color']
        resp.reference = dict_elvez.dict_components['connector'][req.label]['reference']
        resp.model = dict_elvez.dict_components['connector'][req.label]['model']
        resp.dimensions = [dict_elvez.dict_components['connector'][req.label]['xdim'], dict_elvez.dict_components['connector'][req.label]['ydim'], dict_elvez.dict_components['connector'][req.label]['zdim']]
        #resp.dimensions = [10.0, 20.0, 5.0]
        print([dict_elvez.dict_components['connector'][req.label]['xdim']+1, dict_elvez.dict_components['connector'][req.label]['ydim'], dict_elvez.dict_components['connector'][req.label]['zdim']])
    elif req.label in dict_elvez.dict_components['device']:
        resp.component = 'DEV'
        resp.color = dict_elvez.dict_components['device'][req.label]['color']
        resp.model = dict_elvez.dict_components['device'][req.label]['model']
        resp.type = dict_elvez.dict_components['device'][req.label]['type']
    
    for WH in dict_elvez.dict_WH:
        if req.label == dict_elvez.dict_WH[WH]['first_con']:
            resp.WH = WH
            resp.box = dict_elvez.dict_WH[WH]['box']
            resp.tray = dict_elvez.dict_WH[WH]['tray']
            for branch in dict_elvez.dict_WH[WH]['end_con']:
                for cable in dict_elvez.dict_WH[WH]['end_con'][branch]:
                    data = pins_data()
                    data.label = cable
                    data.pins = dict_elvez.dict_WH[WH]['end_con'][branch][cable]['first_pins']
                    resp.cables.append(data)
        elif req.label in dict_elvez.dict_WH[WH]['end_con']:
            resp.WH = WH
            resp.box = dict_elvez.dict_WH[WH]['box']
            resp.tray = dict_elvez.dict_WH[WH]['tray']
            for cable in dict_elvez.dict_WH[WH]['end_con'][req.label]:
                data = pins_data()
                data.label = cable
                #data.pins = dict_elvez.dict_WH[WH]['end_con'][req.label][cable]['end_pins']
                data.pins = dict_elvez.dict_WH[WH]['end_con'][req.label][cable]['first_pins']
                resp.cables.append(data)
    
    resp.success = True
    return resp

rospy.Service(connector_info_service, connector_info, connector_info_callback)


def cable_info_callback(req): 
    """
    Service that returns information about the required cable for its identification with the vision system
    """
    resp = cable_infoResponse()
    if req.label in dict_elvez.dict_components['cable']:
        resp.component = 'CAB'
        resp.color = dict_elvez.dict_components['cable'][req.label]['color']
        resp.length = dict_elvez.dict_components['cable'][req.label]['length']
        resp.diameter = dict_elvez.dict_components['cable'][req.label]['diameter']

        for WH in dict_elvez.dict_WH:
            for branch in dict_elvez.dict_WH[WH]['end_con']:
                if req.label in dict_elvez.dict_WH[WH]['end_con'][branch]:
                    resp.WH = WH
                    resp.box = dict_elvez.dict_WH[WH]['box']
                    resp.tray = dict_elvez.dict_WH[WH]['tray']
                    data1 = pins_data()
                    data2 = pins_data()
                    data1.label = dict_elvez.dict_WH[WH]['first_con']
                    data1.pins = dict_elvez.dict_WH[WH]['end_con'][branch][req.label]['first_pins']
                    data2.label = branch
                    data2.pins = dict_elvez.dict_WH[WH]['end_con'][branch][req.label]['end_pins']
                    resp.connectors=[data1, data2]

        resp.success = True
        
    else:
        resp.success = False

    return resp

rospy.Service(cable_info_service, cable_info, cable_info_callback)


def tray_info_callback(req): #corner frame, center frame, dim, WH
    """
    Service that returns information about the required tray of a box
    """
    resp = tray_infoResponse()
    print(req.box)
    print(req.tray)
    resp.success = False
    if req.box in jigs_complete_dict:
        if 'trays' in jigs_complete_dict[req.box]:
            if req.tray in jigs_complete_dict[req.box]['trays']:
                resp.key_corner_frame = fromKdlToPose(jigs_complete_dict[req.box]['trays'][req.tray]['frame'])
                resp.key_center_frame = fromKdlToPose(jigs_complete_dict[req.box]['trays'][req.tray]['center_pose'])
                resp.dimensions = [jigs_complete_dict[req.box]['trays'][req.tray]['xdim'], jigs_complete_dict[req.box]['trays'][req.tray]['ydim'], jigs_complete_dict[req.box]['trays'][req.tray]['zdim']]
                for WH in dict_elvez.dict_WH:
                    if req.box == dict_elvez.dict_WH[WH]['box'] and req.tray == dict_elvez.dict_WH[WH]['tray']:                
                        resp.WH = WH
                resp.success = True
    return resp

rospy.Service(tray_info_service, tray_info, tray_info_callback)


def guide_info_callback(req): 
    """
    Service that returns information about the required guide of a jig
    """
    resp = guide_infoResponse()
    print(req.jig)
    print(req.guide)
    data = jig_guide_data()
    resp.success = False
    if req.jig in jigs_complete_dict:
        if 'guides' in jigs_complete_dict[req.jig]:
            if req.guide in jigs_complete_dict[req.jig]['guides']:
                data.id = req.guide
                data.key_length = jigs_complete_dict[req.jig]['guides'][req.guide]['key']['length']
                data.key_gap = jigs_complete_dict[req.jig]['guides'][req.guide]['key']['gap']
                data.key_height = jigs_complete_dict[req.jig]['guides'][req.guide]['key']['height']
                data.key_height_corner = jigs_complete_dict[req.jig]['guides'][req.guide]['key']['height_corner']
                data.collisions = jigs_complete_dict[req.jig]['collisions']
                data.key_corner_frame = fromKdlToPose(jigs_complete_dict[req.jig]['guides'][req.guide]['key']['frame'])
                data.key_center_frame = fromKdlToPose(jigs_complete_dict[req.jig]['guides'][req.guide]['key']['center_pose'])
                data.collision_dimensions = [jigs_complete_dict[req.jig]['guides'][req.guide]['collision']['xdim'], jigs_complete_dict[req.jig]['guides'][req.guide]['collision']['ydim'], jigs_complete_dict[req.jig]['guides'][req.guide]['collision']['zdim']]
                data.collision_corner_frame = fromKdlToPose(jigs_complete_dict[req.jig]['guides'][req.guide]['collision']['frame'])
                data.dimensions = jigs_complete_dict[req.jig]['dimensions']
                resp.data = data
                resp.success = True
    return resp

rospy.Service(guide_info_service, guide_info, guide_info_callback)


def all_guides_info_callback(req): 
    """
    Service that returns information about all the guides in the platform
    """
    resp = guide_all_infoResponse()
    
    resp.success = False
    for jig in jigs_complete_dict:
        if jig[0]=='J' and 'guides' in jigs_complete_dict[jig]:
            for guide in jigs_complete_dict[jig]['guides']:
                data = jig_guide_data()
                data.id = jig
                data.key_length = jigs_complete_dict[jig]['guides'][guide]['key']['length']
                data.key_gap = jigs_complete_dict[jig]['guides'][guide]['key']['gap']
                data.key_height = jigs_complete_dict[jig]['guides'][guide]['key']['height']
                data.key_center_frame = fromKdlToPose(jigs_complete_dict[jig]['guides'][guide]['key']['center_pose'])
                data.dimensions = jigs_complete_dict[jig]['dimensions']
                resp.data.append(data)
                resp.success = True
    return resp
rospy.Service(all_guides_service, guide_all_info, all_guides_info_callback)


def taping_spot_info_callback(req): 
    """
    Service that returns information about the required taping spot of a jig
    """
    resp = taping_spot_infoResponse()
    #print(req.jig)
    #print(req.spot)
    data = jig_tape_data()
    resp.success = False
    if req.jig in jigs_complete_dict:
        if 'tape_spots' in jigs_complete_dict[req.jig]:
            if req.spot in jigs_complete_dict[req.jig]['tape_spots']:
                data.id = req.spot
                data.corner_frame = fromKdlToPose(jigs_complete_dict[req.jig]['tape_spots'][req.spot]['frame'])
                data.center_frame = fromKdlToPose(jigs_complete_dict[req.jig]['tape_spots'][req.spot]['center_pose'])
                data.dimensions = [jigs_complete_dict[req.jig]['tape_spots'][req.spot]['xdim'], jigs_complete_dict[req.jig]['tape_spots'][req.spot]['ydim'], jigs_complete_dict[req.jig]['tape_spots'][req.spot]['zdim']]
                resp.data = data
                resp.success = True
    return resp

rospy.Service(taping_spot_info_service, taping_spot_info, taping_spot_info_callback)


def jig_info_callback(req): 
    """
    Service that returns information about the required jig
    """
    resp = jig_infoResponse()
    #print(req.jig)
    resp.success = False
    
    if req.jig in jigs_complete_dict:
        resp.commercial = jigs_complete_dict[req.jig]['commercial']
        resp.corner_frame = fromKdlToPose(jigs_complete_dict[req.jig]['jig_frame'])
        resp.dimensions = jigs_complete_dict[req.jig]['dimensions']
        
        if 'guides' in jigs_complete_dict[req.jig]:
            for guide in jigs_complete_dict[req.jig]['guides']:
                data_guide = jig_guide_data()
                data_guide.id = guide
                data_guide.key_length = jigs_complete_dict[req.jig]['guides'][guide]['key']['length']
                data_guide.key_gap = jigs_complete_dict[req.jig]['guides'][guide]['key']['gap']
                data_guide.key_height = jigs_complete_dict[req.jig]['guides'][guide]['key']['height']
                data_guide.key_height_corner = jigs_complete_dict[req.jig]['guides'][guide]['key']['height_corner']
                data_guide.collisions = [jigs_complete_dict[req.jig]['guides'][guide]['key']['xcol1'], jigs_complete_dict[req.jig]['guides'][guide]['key']['xcol2'], jigs_complete_dict[req.jig]['guides'][guide]['key']['ycol1'], jigs_complete_dict[req.jig]['guides'][guide]['key']['ycol2']]
                data_guide.key_corner_frame = fromKdlToPose(jigs_complete_dict[req.jig]['guides'][guide]['key']['frame'])
                data_guide.key_center_frame = fromKdlToPose(jigs_complete_dict[req.jig]['guides'][guide]['key']['center_pose'])
                data_guide.collision_dimensions = [jigs_complete_dict[req.jig]['guides'][guide]['collision']['xdim'], jigs_complete_dict[req.jig]['guides'][guide]['collision']['ydim'], jigs_complete_dict[req.jig]['guides'][guide]['collision']['zdim']]
                data_guide.collision_corner_frame = fromKdlToPose(jigs_complete_dict[req.jig]['guides'][guide]['collision']['frame'])
                resp.guides.append(data_guide)
        
        if 'tape_spots' in jigs_complete_dict[req.jig]:
            for spot in jigs_complete_dict[req.jig]['tape_spots']:
                data_spot = jig_tape_data()
                data_spot.id = spot
                data_spot.corner_frame = fromKdlToPose(jigs_complete_dict[req.jig]['tape_spots'][spot]['frame'])
                data_spot.center_frame = fromKdlToPose(jigs_complete_dict[req.jig]['tape_spots'][spot]['center_pose'])
                data_spot.dimensions = [jigs_complete_dict[req.jig]['tape_spots'][spot]['xdim'], jigs_complete_dict[req.jig]['tape_spots'][spot]['ydim'], jigs_complete_dict[req.jig]['tape_spots'][spot]['zdim']]
                resp.taping_spots.append(data_spot)
        
        resp.success = True
    return resp

rospy.Service(jig_info_service, jig_info, jig_info_callback)


def all_cad_components_callback(req): 
    """
    Service that returns the name of all the CAD components
    """
    resp = all_cad_componentsResponse()
    resp.success = False
    #jigs_complete_dict[req.box]['trays'][req.tray] Example of structure
    for elem in jigs_complete_dict: #J1
        elem_subtype = []
        if elem[0] == 'J':
            elem_subtype = ["guides","tape_spots"]
        if elem[0] == 'B':
            elem_subtype = ["trays"]
        for subtype in elem_subtype: #guide
            for subelem in jigs_complete_dict[elem][subtype]: #1
                resp.data.append(str(elem) + "," + str(subtype) + "," + str(subelem))
    resp.success = True
    return resp

rospy.Service(all_cad_components_service, all_cad_components, all_cad_components_callback)


def next_operation_callback(req): 
    """
    Service that returns the next operation to perform
    """
    global operation_index
    resp = next_operationResponse()
    resp.success = False
    resp.type = dict_elvez.list_seq[operation_index]['operation']
    
    for label in dict_elvez.list_seq[operation_index]['label']:
        resp.label.append(label)
    
    for spot in dict_elvez.list_seq[operation_index]['spot']:
        new_spot = spot_data()
        #print(spot)
        if dict_elvez.list_seq[operation_index]['operation']=='EC':
            new_spot.jig = spot['comb']
            new_spot.id = spot['couple']
        else:
            #print(dict_elvez.list_seq[operation_index]['spot'][0]['jig'])
            new_spot.jig = spot['jig']
            if dict_elvez.list_seq[operation_index]['operation']=='T':
                new_spot.id = spot['tape_spot']
            else:
                new_spot.id = spot['couple']
                if dict_elvez.list_seq[operation_index]['operation']=='PC':
                    new_spot.side = spot['side']
        resp.spot.append(new_spot)
    
    operation_index += 1
    print(operation_index)
    if operation_index >= len(dict_elvez.list_seq):
        operation_index = 0
        resp.end = True
    else:
        resp.end = False
        
    resp.success = True
    return resp

rospy.Service(next_operation_service, next_operation, next_operation_callback)


def reset_sequence_list_callback(msg): 
    """
    Service that resets the sequence list of operations
    """
    resp = TriggerResponse()
    global operation_index
    operation_index = 0
    resp.success = True
    return resp

rospy.Service(reset_sequence_list_service, Trigger, reset_sequence_list_callback)


def reset_sequence_list_to_callback(req): 
    """
    Service that resets the sequence list of operations to a certain index
    """
    resp = resetToResponse()
    global operation_index
    if req.index < len(dict_elvez.list_seq):
        operation_index = req.index
        resp.success = True
    else:
        resp.success = False
    return resp

rospy.Service(reset_sequence_list_to_service, resetTo, reset_sequence_list_to_callback)

def index_operation_callback(req): 
    """
    Service that returns the next operation to perform
    """
    global operation_index
    operation_index = req.index
    resp = index_operationResponse()
    resp.success = False
    resp.type = dict_elvez.list_seq[operation_index]['operation']
    
    for label in dict_elvez.list_seq[operation_index]['label']:
        resp.label.append(label)
    
    for spot in dict_elvez.list_seq[operation_index]['spot']:
        new_spot = spot_data()
        #print(spot)
        if dict_elvez.list_seq[operation_index]['operation']=='EC':
            new_spot.jig = spot['comb']
            new_spot.id = spot['couple']
        else:
            #print(dict_elvez.list_seq[operation_index]['spot'][0]['jig'])
            new_spot.jig = spot['jig']
            if dict_elvez.list_seq[operation_index]['operation']=='T':
                new_spot.id = spot['tape_spot']
            else:
                new_spot.id = spot['couple']
                if dict_elvez.list_seq[operation_index]['operation']=='PC':
                    new_spot.side = spot['side']
        resp.spot.append(new_spot)
    
    operation_index += 1
    print(operation_index)
    if operation_index >= len(dict_elvez.list_seq):
        operation_index = 0
        resp.end = True
    else:
        resp.end = False
        
    resp.index = operation_index
    resp.success = True
    return resp

rospy.Service(index_operation_service, index_operation, index_operation_callback)


def all_operations_callback(req): 
    """
    Service that returns the next operation to perform
    """
    global operation_index
    operation_index = 0
    resp = all_operationsResponse()
    resp.success = False
    
    for iter_index in dict_elvez.list_seq:
        item = operation_item()
        item.type = iter_index['operation']
        for label in iter_index['label']:
            item.label.append(label)
        
        for spot in iter_index['spot']:
            new_spot = spot_data()
            #print(spot)
            # if iter_index['operation']=='EC':
            #     new_spot.jig = spot['comb']
            #     new_spot.id = spot['couple']
            # else:
            if True:
                #print(iter_index['spot'][0]['jig'])
                new_spot.jig = spot['jig']
                #if iter_index['operation']=='T':
                #    new_spot.id = spot['tape_spot']
                if iter_index['operation']=='TJ':
                    new_spot.id = spot['guide']
                else:
                    new_spot.id = spot['couple']
                    if iter_index['operation']=='PC' or iter_index['operation']=='EC':
                        new_spot.side = spot['side']
            item.spot.append(new_spot)
        resp.data.append(item)

    resp.success = True
    return resp

rospy.Service(all_operations_service, all_operations, all_operations_callback)


def tool_info_callback(req): 
    """
    Service that returns information of a tool
    """
    resp = tool_infoResponse()
    resp.success = False
    label_name = "A" + req.name
    
    if label_name in jigs_complete_dict:
    #resp.dim = jigs_complete_dict[label_name]['dimensions']
        resp.dim_tool = jigs_complete_dict[label_name]['dimensions_tool']
        resp.type = jigs_complete_dict[label_name]['tool_type']
        if resp.type == "gripper":
            resp.pose_nail = fromKdlToPose(jigs_complete_dict[label_name]['frame_nail'])
            resp.dim_fingers = jigs_complete_dict[label_name]['dimensions_fingers']
        resp.pose_base = fromKdlToPose(jigs_complete_dict[label_name]['frame_base'])
        resp.pose_end = fromKdlToPose(jigs_complete_dict[label_name]['frame_end'])
        resp.success = True
    return resp

rospy.Service(tool_info_service, tool_info, tool_info_callback)


# Publish the markers and the TFs of the ELVEZ platform
while not rospy.is_shutdown(): 
    current_time = rospy.get_rostime()
    for marker in markerArray.markers: 
        marker.header.stamp = current_time
    publisher.publish(markerArray)  
    for trans in transforms:
        #print(trans.getID().getCadID())
        pass
    #root = trans.getRoot()
    root = transforms[0].getRoot()
    #print(root.getID().getCadID())
    broadcastTransform(br, root, root.getName(), "platform_rf", time=current_time) 

    if cad_name_combs != "": 
        for trans in transforms_combs:
            #print(trans.getID().getCadID())
            pass
        #root = trans.getRoot()
        root = transforms_combs[0].getRoot()
        #print(root.getID().getCadID())
        broadcastTransform(br, root, root.getName(), "combs_rf", time=current_time)

    if cad_name_ATC != "": 
        for trans in transforms_ATC:
            #print(trans.getID().getCadID())
            pass
        #root = trans.getRoot()
        root = transforms_ATC[0].getRoot()
        #print(root.getID().getCadID())
        broadcastTransform(br, root, root.getName(), "ATC_rf", time=current_time)

    for trans_tf in transforms: 
        tf_name = trans_tf.getName()
        parent_tf = trans_tf.parent
        if parent_tf.isUseful():
            broadcastTransform(br, trans_tf, tf_name, parent_tf.getName(), time=current_time)

        #Broadcast keypoints
        if(trans_tf.getCommercial() in dict_elvez.dict_jigs):
                if(trans_tf.getID().getType() == 1):
                    if 'guides' in dict_elvez.dict_jigs[trans_tf.getCommercial()]:
                        for guide in dict_elvez.dict_jigs[trans_tf.getCommercial()]['guides']:
                            frame = PyKDL.Frame()
                            frame = dict_elvez.dict_jigs[trans_tf.getCommercial()]['guides'][guide]['key']['center_pose']
                            name = tf_name + 'guide' + guide
                            #frame.p = frame.p * 0.001
                            broadcastTransform(br, frame, name, trans_tf.getName(), time=current_time)

                    if 'tape_spots' in dict_elvez.dict_jigs[trans_tf.getCommercial()]:
                        for tape in dict_elvez.dict_jigs[trans_tf.getCommercial()]['tape_spots']:
                            frame = PyKDL.Frame()
                            frame = dict_elvez.dict_jigs[trans_tf.getCommercial()]['tape_spots'][tape]['center_pose']
                            name = tf_name + 'tape_spot' + tape
                            #frame.p = frame.p * 0.001
                            broadcastTransform(br, frame, name, trans_tf.getName(), time=current_time)

                elif(trans_tf.getID().getType() == 2):
                    if 'trays' in dict_elvez.dict_jigs[trans_tf.getCommercial()]:
                        for tray in dict_elvez.dict_jigs[trans_tf.getCommercial()]['trays']:
                            frame = PyKDL.Frame()
                            frame = dict_elvez.dict_jigs[trans_tf.getCommercial()]['trays'][tray]['center_pose']
                            name = tf_name + 'tray' + tray
                            broadcastTransform(br, frame, name, trans_tf.getName(), time=current_time)

    for trans_tf in transforms_combs: 
        tf_name = trans_tf.getName()
        parent_tf = trans_tf.parent
        if parent_tf.isUseful():
            broadcastTransform(br, trans_tf, tf_name, parent_tf.getName(), time=current_time)

        #Broadcast keypoints combs
        if(trans_tf.getCommercial() in dict_elvez.dict_jigs):
                if(trans_tf.getID().getType() == 3):
                    if 'guides' in dict_elvez.dict_jigs[trans_tf.getCommercial()]:
                        for guide in dict_elvez.dict_jigs[trans_tf.getCommercial()]['guides']:
                            frame = PyKDL.Frame()
                            frame = dict_elvez.dict_jigs[trans_tf.getCommercial()]['guides'][guide]['key']['center_pose']
                            name = tf_name + 'guide' + guide
                            #frame.p = frame.p * 0.001
                            broadcastTransform(br, frame, name, trans_tf.getName(), time=current_time)

    for trans_tf in transforms_ATC: 
        tf_name = trans_tf.getName()
        parent_tf = trans_tf.parent
        if parent_tf.isUseful():
            broadcastTransform(br, trans_tf, tf_name, parent_tf.getName(), time=current_time)

        #Broadcast keypoints ATC
        if(trans_tf.getCommercial() in dict_elvez.dict_jigs):
                if(trans_tf.getID().getType() == 4):
                    frame = PyKDL.Frame()
                    frame = dict_elvez.dict_jigs[trans_tf.getCommercial()]['frame_base']
                    name = tf_name + '_ATC_base_' + jigs_complete_dict[trans_tf.getLabel()]['tool_name']
                    #frame.p = frame.p * 0.001
                    broadcastTransform(br, frame, name, trans_tf.getName(), time=current_time)