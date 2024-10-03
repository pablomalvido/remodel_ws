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
from sensor_msgs.msg import *
import xml.etree.ElementTree as ET
import tf
from elvez_pkg.msg import *
from elvez_pkg.srv import *
from UI_nodes_pkg.msg import *
from UI_nodes_pkg.srv import *
import actionlib
from actionlib_msgs.msg import GoalStatusArray
from UI_nodes_pkg.msg import process_UIFeedback, process_UIResult, process_UIAction, process_UIGoal
from vision_pkg_full_demo.srv import *
from rosapi.srv import *


#ROS init
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('auto_process_action_server', anonymous=True)
rospack = rospkg.RosPack()
listener = tf.TransformListener()
mode_publisher = rospy.Publisher('/UI/mode', String, queue_size=1)
logs_publisher = rospy.Publisher('/UI/logs', String, queue_size=1)
feedback_publisher = rospy.Publisher('/UI/feedback', configProp, queue_size=1)


#Define the movegroups 
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
arm_left = moveit_commander.MoveGroupCommander("arm_left")
arm_right = moveit_commander.MoveGroupCommander("arm_right")
arms = moveit_commander.MoveGroupCommander("arms")
torso = moveit_commander.MoveGroupCommander("torso")
motion_groups = {"arm_left": arm_left, "arm_right": arm_right, "arms": arms, "torso": torso}

ops_info_text=[]
 
def get_mode_callback(req):
        """
        Retrieve current operation mode of the system
        """
        global modeUI
        resp = TriggerResponse()
        resp.success=True
        resp.message=modeUI	


rospy.Service("/UI/get_mode", Trigger, get_mode_callback)

modeUI = 'Idle'

#Identify if we are working with the real or simulated robot
rospy.wait_for_service('/rosapi/nodes')
get_nodes_srv = rospy.ServiceProxy('rosapi/nodes', Nodes)
active_nodes = get_nodes_srv(NodesRequest()).nodes
real_robot_nodes = ['/io_relay','/joint_state','/joint_trajectory_action','/move_group','/motion_streaming_interface','/robot_state_publisher']
if all(x in active_nodes for x in real_robot_nodes):
        real_robot = True
else:
        real_robot = False 

#Read config params
config_full_pkg_path = str(rospack.get_path('UI_nodes_pkg'))
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
try:
        if real_robot:
                print('REAL ROBOT')
                if config['grippers_control_real'] == 'Y' or config['grippers_control_real'] == 'y':
                        execute_grippers = True 
                else:
                        execute_grippers = False
                if config['gun_control_real'] == 'Y' or config['gun_control_real'] == 'y':
                        execute_gun = True 
                else:
                        execute_gun = False
                if config['force_control_real'] == 'Y' or config['force_control_real'] == 'y':
                        force_control_active = True 
                else:
                        force_control_active = False
                if config['use_camera_real'] == 'Y' or config['use_camera_real'] == 'y':
                        use_camera = True 
                else:
                        use_camera = False
                speed_limit = min(float(config['speed_real_per']),0.3)
                fast_speed_execution = min(float(config['speed_fast_real_mms']),100) 
                speed_execution = min(float(config['speed_real_mms']),80) 
                slow_speed_execution = min(float(config['speed_slow_real_mms']),40) 
                speed_tension = min(float(config['speed_tension_real_mms']),40) 
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
                speed_limit = float(config['speed_demo_per']) 
                fast_speed_execution = float(config['speed_fast_demo_mms'])
                speed_execution = float(config['speed_demo_mms']) 
                slow_speed_execution = float(config['speed_slow_demo_mms']) 
                speed_tension = float(config['speed_tension_demo_mms']) 
except:
        print("Error defining config values")
        exit()        

#Reset and initialize groups
for group in motion_groups:
        motion_groups[group].clear_pose_targets()
        motion_groups[group].stop       
        motion_groups[group].set_max_velocity_scaling_factor(speed_limit)
               

def execute_operation(op):
        """
        Include code to manage operation calls for your specific robotic application
        """
        pass


class AutoProcessClass(object):
        _feedback=process_UIFeedback() #Class attributes
        _result=process_UIResult()
        _index = 0 #Index of the operation
        _stop_action = False

        def __init__(self):
                self._as=actionlib.SimpleActionServer("AutoProcess_as", process_UIAction, self.goal_callback, False)
                self._as.register_preempt_callback(self.preempt_callback)
                self._as.start() #Starts the action server, so it can be called from the client

        def preempt_callback(self):
                """
                Stops the process execution
                """
                global stop_mov
                global modeUI
                print("Preempted in: " + str(self._index))
                self._stop_action = True
                self._as.set_preempted()                
                stop_mov = True
                msg = String()
                msg.data = "Idle"
                modeUI="Idle"
                mode_publisher.publish(msg)

        def publish_feedback(self, message=""):
                """
                Sends feedback
                """
                global step1
                self._feedback.index = self._index
                self._feedback.subindex = step1
                self._feedback.message = message
                self._as.publish_feedback(self._feedback)

        def goal_callback(self, goal):
                """
                Starts executing the process from the desired operatio
                """
                global stop_mov
                global step1
                global modeUI
                self._stop_action = False
                stop_mov = False
                r=rospy.Rate(1) #1Hz
                success=True
                self._index = goal.index
                step1 = goal.subindex
                msg = String()
                msg.data = "Running"
                modeUI = "Running"
                mode_publisher.publish(msg)
                if goal.index == 0 and step1==0:
                        #move_home()
                        pass
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
                                              print("Paused in: " + str(self._index) + ", step1: " + str(step1)) 
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
                                print("Paused in: " + str(self._index) + ", step1: " + str(step1)) 
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


if __name__ == '__main__':
        process_actionserver = AutoProcessClass()

        #Include code to call services to get information about the process
        ops_info = []

        rospy.spin()