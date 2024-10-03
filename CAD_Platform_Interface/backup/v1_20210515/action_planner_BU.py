#! /usr/bin/env python
import rospy
import PyKDL 
import numpy as np
import math 
import tf 
import time
import os
import sys
import actionlib
from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger, TriggerRequest
from std_srvs.srv import Empty, EmptyResponse
from elvez_pkg.msg import *
from elvez_pkg.srv import *
from move_sda10f.srv import Pose_srv, Pose_srvRequest
from move_sda10f.msg import moveActFeedback, moveActResult, moveActAction, moveActGoal

rospy.init_node('action_planner')
rate = rospy.Rate(10)
next = False
reset = False
simulation = False
executeSeq_service = 'ELVEZ_platform_planner/executeSeq'
resetSeq_service = 'ELVEZ_platform_planner/resetSeq'
simulation_service = 'ELVEZ_platform_planner/simulation'


def executeSeq_callback(req): 
    """
    Calls the next operation service of the CAD platform
    """
    global next
    next = True
    return EmptyResponse()

rospy.Service(executeSeq_service, Empty, executeSeq_callback)


def resetSeq_callback(req): 
    """
    Calls the next operation service of the CAD platform
    """
    global reset
    reset = True
    return EmptyResponse()

rospy.Service(resetSeq_service, Empty, resetSeq_callback)


def simulation_callback(req): 
    """
    Calls the next operation service of the CAD platform
    """
    global simulation
    simulation = True
    return EmptyResponse()

rospy.Service(simulation_service, Empty, simulation_callback)


def move_to_pose_feeback_callback(feedback):
    print(feedback)


print(next)


while not rospy.is_shutdown():
    #print(next)
    if next:
        rospy.wait_for_service('/ELVEZ_platform_handler/next_operation')
        my_service = rospy.ServiceProxy('/ELVEZ_platform_handler/next_operation', next_operation)
        nextReq = next_operationRequest()
        next_taskResult = my_service(nextReq)
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
	    """  
	    try:
	      rospy.wait_for_service('sda10f_simulation/move_to_pose')
	      my_service = rospy.ServiceProxy('sda10f_simulation/move_to_pose', Pose_srv)
	      PoseReq = Pose_srvRequest()
	      PoseReq.pose = guideResult.data.key_center_frame
              PoseResult = my_service(PoseReq)
	      print("Moving...")
	      print(PoseResult)
	    except:
	      print("No moving")
	    """

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

            for spot in next_taskResult.spot:
                print("Guide Keypoints info:")
                rospy.wait_for_service('/ELVEZ_platform_handler/guide_info')
                my_service = rospy.ServiceProxy('/ELVEZ_platform_handler/guide_info', guide_info)
                guideReq = guide_infoRequest()
                guideReq.jig = spot.jig
                guideReq.guide = spot.id
                guideResult = my_service(guideReq)
                print(guideResult)
                print("")


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

    if reset:
        rospy.wait_for_service('/ELVEZ_platform_handler/reset_sequence_list')
        my_service = rospy.ServiceProxy('/ELVEZ_platform_handler/reset_sequence_list', Trigger)
        requ = TriggerRequest()
        result = my_service(requ)
        print("Reset realizado")
        print(result)
        reset = False

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
	      client=actionlib.SimpleActionClient('sda10f_simulation/move_to_pose_action', moveActAction)
	      client.wait_for_server()
	      actionGoal = moveActGoal()
	      #PoseReq.poseTray = trayResult.key_center_frame
	      actionGoal.poseTray = trayResult.key_corner_frame
	      actionGoal.poseGuide = guideResult.data.key_center_frame
              client.send_goal(actionGoal, feedback_cb = move_to_pose_feeback_callback)
	      print("Moving...")
	      client.wait_for_result()
	    except:
	      print("No moving")
   
    rate.sleep()
