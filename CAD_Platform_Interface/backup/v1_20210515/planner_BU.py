#! /usr/bin/env python
import rospy
import PyKDL 
import numpy as np
import math 
import tf 
import time
import os
import sys
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from std_srvs.srv import Empty, EmptyResponse
from elvez_pkg.msg import *
from elvez_pkg.srv import *
from move_sda10f.srv import *

rospy.init_node('planner')
rate = rospy.Rate(10)
index = 0
mode = "Stop"
end = False
next = False
reset = False
stop = False
resume = False
simulation = False
execute_service = 'ELVEZ_platform_planner/execute'
resetSeq_service = 'ELVEZ_platform_planner/resetSeq'
stop_service = 'ELVEZ_platform_planner/stop'
resume_service = 'ELVEZ_platform_planner/resume'
simulation_service = 'ELVEZ_platform_planner/simulation'
pub_index = rospy.Publisher('/sda10f_platform_planner/index_topic', Int32, queue_size=1)
pub_mode = rospy.Publisher('/sda10f_platform_planner/mode_topic', String, queue_size=1)

def publish_index():
    global index
    pub_index.publish(index)

def execute_callback(req): 
    """
    Calls the index operation service of the CAD platform
    """
    global next
    global index
    index = req.index
    publish_index()
    next = True
    resp = index_operationResponse()
    resp.success = True
    return resp

rospy.Service(execute_service, index_operation, execute_callback)


def resetSeq_callback(req): 
    """
    Calls the next operation service of the CAD platform
    """
    resp = TriggerResponse()
    global reset
    global index
    reset = True
    index = 0
    publish_index()
    resp.success = True
    return resp

rospy.Service(resetSeq_service, Trigger, resetSeq_callback)


def stop_callback(req): 
    """
    Stops the movement of the simulation in execution
    """
    global stop
    resp = TriggerResponse()
    stop = True
    resp.success = True
    return resp

rospy.Service(stop_service, Trigger, stop_callback)


def resume_callback(req): 
    """
    Resumes the movement of the simulation stopped
    """
    global resume
    resp = TriggerResponse()
    resume = True
    resp.success = True
    return resp

rospy.Service(resume_service, Trigger, resume_callback)


def simulation_callback(req): 
    """
    Calls the next operation service of the CAD platform
    """
    global simulation
    simulation = True
    return EmptyResponse()

rospy.Service(simulation_service, Empty, simulation_callback)


def call_skill_manager(Op_type, keypoints):
    global index
    global end
    print(Op_type)
    print(keypoints)
    rospy.wait_for_service('/skill_manager/start')
    skillManager_start_service = rospy.ServiceProxy('/skill_manager/start', start_skill)
    startReq = start_skillRequest()
    startReq.type = Op_type
    startReq.keypoints = keypoints
    startResult = skillManager_start_service(startReq)
    #Change this index update --> When the action finishes
    if (Op_type!='PC' and Op_type!='T'):
        if end:
            index = 0
            end = False
        else:
            index += 1
        publish_index()
	time.sleep(0.1)
        pub_mode.publish('Finish')


def callback_sim_state(sim_state):
    global mode
    global end
    global index
    print(sim_state)
    if sim_state.data == 'Running':
        mode = 'Running'
    if sim_state.data == 'Stopped':
        mode = 'Stop'
    if sim_state.data == "Finish":
        mode = 'Finish'
        if end:
            index = 0
            end = False
        else:
            index += 1
        publish_index()
        time.sleep(0.1)
    pub_mode.publish(mode) #Change this to be published only if the mode changes

subsSim = rospy.Subscriber('/sda10f_simulation/state', String, callback_sim_state)


print("Ready")


while not rospy.is_shutdown():
    #print(next)
    if next:
        rospy.wait_for_service('/ELVEZ_platform_handler/index_operation')
        my_service = rospy.ServiceProxy('/ELVEZ_platform_handler/index_operation', index_operation)
        nextReq = index_operationRequest()
        nextReq.index = index
        next_taskResult = my_service(nextReq)
        if next_taskResult.end:
            end = True
        else:
            end = False
        print("Peticion realizada")
        print(next_taskResult)
        next = False


        if next_taskResult.type=='PC':
	    print("")
            print("Connector info:")
            rospy.wait_for_service('/ELVEZ_platform_handler/connector_info')
            my_service = rospy.ServiceProxy('/ELVEZ_platform_handler/connector_info', connector_info)
            connectorReq = connector_infoRequest()
            connectorReq.label = next_taskResult.label[0]
            connectorResult = my_service(connectorReq)
            print(connectorResult)

            print("")
            print("Guide Keypoints info:")
            rospy.wait_for_service('/ELVEZ_platform_handler/guide_info')
            my_service = rospy.ServiceProxy('/ELVEZ_platform_handler/guide_info', guide_info)
            guideReq = guide_infoRequest()
            guideReq.jig = next_taskResult.spot[0].jig
            guideReq.guide = next_taskResult.spot[0].id
            guideResult = my_service(guideReq)
            print(guideResult)

	    print("")
            print("Tray where that connector is located ("+connectorResult.box+"."+connectorResult.tray+"):")
            rospy.wait_for_service('/ELVEZ_platform_handler/tray_info')
            my_service = rospy.ServiceProxy('/ELVEZ_platform_handler/tray_info', tray_info)
            trayReq = tray_infoRequest()
            trayReq.box = connectorResult.box
            trayReq.tray = connectorResult.tray
            trayResult = my_service(trayReq)
            print(trayResult)
	      
            try:
                print("Placing connector operation. Moving...")
                PC_keypoints = [trayResult.key_center_frame, guideResult.data.key_center_frame]
                call_skill_manager("PC", PC_keypoints)
            except:
                print("Error. No moving")
	    

        elif next_taskResult.type=='RC':
            print("")
            print("Cables info:")

            for label in next_taskResult.label:
                if label[:2] == 'CA':
                    rospy.wait_for_service('/ELVEZ_platform_handler/cable_info')
                    my_service = rospy.ServiceProxy('/ELVEZ_platform_handler/cable_info', cable_info)
                    cableReq = cable_infoRequest()
                    cableReq.label = label
                    cableResult = my_service(cableReq)
                    print(cableResult)
                    print("")
                else:
                    rospy.wait_for_service('/ELVEZ_platform_handler/connector_info')
                    my_service = rospy.ServiceProxy('/ELVEZ_platform_handler/connector_info', connector_info)
                    connectorReq = connector_infoRequest()
                    connectorReq.label = label
                    connectorResult = my_service(connectorReq)
                    for cableLabel in connectorResult.cables:
                        rospy.wait_for_service('/ELVEZ_platform_handler/cable_info')
                        my_service = rospy.ServiceProxy('/ELVEZ_platform_handler/cable_info', cable_info)
                        cableReq = cable_infoRequest()
                        cableReq.label = cableLabel.label
                        cableResult = my_service(cableReq)
                        print(cableResult)
                        print("")

            RC_keypoints = []
            for spot in next_taskResult.spot:
                print("Guide Keypoints info:")
                rospy.wait_for_service('/ELVEZ_platform_handler/guide_info')
                my_service = rospy.ServiceProxy('/ELVEZ_platform_handler/guide_info', guide_info)
                guideReq = guide_infoRequest()
                guideReq.jig = spot.jig
                guideReq.guide = spot.id
                guideResult = my_service(guideReq)
                RC_keypoints.append(guideResult.data.key_center_frame)
                print(guideResult)
                print("")

            try:
                print("Routing cables operation. Moving...")
                call_skill_manager("RC", RC_keypoints)
            except:
                print("Error. No moving")


        elif next_taskResult.type=='T':
            print("")
            print("Taping spot Keypoints info:")
            rospy.wait_for_service('/ELVEZ_platform_handler/taping_spot_info')
            my_service = rospy.ServiceProxy('/ELVEZ_platform_handler/taping_spot_info', taping_spot_info)
            tapeReq = taping_spot_infoRequest()
            tapeReq.jig = next_taskResult.spot[0].jig
            tapeReq.spot = next_taskResult.spot[0].id
            tapeResult = my_service(tapeReq)
            print(tapeResult)

            try:
                print("Taping operation. Moving...")
                T_keypoints = [tapeResult.data.center_frame]
                call_skill_manager("T", T_keypoints)
            except:
                print("Error. No moving")


    if reset:
        rospy.wait_for_service('/ELVEZ_platform_handler/reset_sequence_list')
        my_service = rospy.ServiceProxy('/ELVEZ_platform_handler/reset_sequence_list', Trigger)
        requ = TriggerRequest()
        result = my_service(requ)
        print("Reset done")
        print(result)
        reset = False


    if stop:
        rospy.wait_for_service('/skill_manager/stop')
        skillManager_stop_service = rospy.ServiceProxy('/skill_manager/stop', Trigger)
        stopReq = TriggerRequest()
        stopResult = skillManager_stop_service(stopReq)
        print("Stop done")
        stop = False


    if resume:
        rospy.wait_for_service('/skill_manager/resume')
        skillManager_resume_service = rospy.ServiceProxy('/skill_manager/resume', Trigger)
        resumeReq = TriggerRequest()
        resumeResult = skillManager_resume_service(resumeReq)
        print("Resume done")
        resume = False 


    if simulation:
	#Reset to the first operation --> placing a connector
	simulation = False
	rospy.wait_for_service('/ELVEZ_platform_handler/reset_sequence_list')
        my_service = rospy.ServiceProxy('/ELVEZ_platform_handler/reset_sequence_list', Trigger)
        requ = TriggerRequest()
        result = my_service(requ)
        print("Reset to first operation")
        print(result)

	#Obtains information about the first operation
        rospy.wait_for_service('/ELVEZ_platform_handler/next_operation')
        my_service = rospy.ServiceProxy('/ELVEZ_platform_handler/next_operation', next_operation)
        nextReq = next_operationRequest()
        next_taskResult = my_service(nextReq)
        print("Obtaining the first operation...")
        print(next_taskResult)

        if next_taskResult.type=='PC':
	    #Obtains information about the connector of the operation (tray where it is located)
	    print("")
            print("Connector info:")
            rospy.wait_for_service('/ELVEZ_platform_handler/connector_info')
            my_service = rospy.ServiceProxy('/ELVEZ_platform_handler/connector_info', connector_info)
            connectorReq = connector_infoRequest()
            connectorReq.label = next_taskResult.label[0]
            connectorResult = my_service(connectorReq)
            print(connectorResult)

	    #Obtains information about the guide of the operation (keypoint)
            print("")
            print("Guide Keypoints info:")
            rospy.wait_for_service('/ELVEZ_platform_handler/guide_info')
            my_service = rospy.ServiceProxy('/ELVEZ_platform_handler/guide_info', guide_info)
            guideReq = guide_infoRequest()
            guideReq.jig = next_taskResult.spot[0].jig
            guideReq.guide = next_taskResult.spot[0].id
            guideResult = my_service(guideReq)
            print(guideResult)

	    #Obtain information about the tray where the connector is (keypoint)
	    print("")
            print("Tray where that connector is located ("+connectorResult.box+"."+connectorResult.tray+"):")
            rospy.wait_for_service('/ELVEZ_platform_handler/tray_info')
            my_service = rospy.ServiceProxy('/ELVEZ_platform_handler/tray_info', tray_info)
            trayReq = tray_infoRequest()
            trayReq.box = connectorResult.box
            trayReq.tray = connectorResult.tray
            trayResult = my_service(trayReq)
            print(trayResult)
	    
	    #Launch the simulation service
	    try:
	      rospy.wait_for_service('sda10f_simulation/move_to_pose')
	      my_service = rospy.ServiceProxy('sda10f_simulation/move_to_pose', Pose_srv)
	      PoseReq = Pose_srvRequest()
	      #PoseReq.poseTray = trayResult.key_center_frame
	      PoseReq.poseTray = trayResult.key_corner_frame
	      PoseReq.poseGuide = guideResult.data.key_center_frame
              PoseResult = my_service(PoseReq)
	      print("Moving...")
	      print(PoseResult)
	    except:
	      print("No moving")
	   
   
    rate.sleep()
