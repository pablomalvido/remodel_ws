#! /usr/bin/env python
import sys
import rospy
import time
import actionlib
from std_msgs.msg import *
from UI_nodes_pkg.msg import process_UIAction, process_UIGoal
from UI_nodes_pkg.msg import *

saved_index = 0
saved_step1 = 0
saved_step2 = 0
saved_auto = True
process_time_init = 0
process_time_accum = 0
time_running = False

rospy.init_node('process_action_client')
index_publisher = rospy.Publisher('/UI/process_index', Int32, queue_size=1)
logs_publisher = rospy.Publisher('/UI/logs', String, queue_size=1)
feedback_publisher = rospy.Publisher('/UI/feedback', configProp, queue_size=1)
rate=rospy.Rate(1) #1Hz
client=actionlib.SimpleActionClient('/AutoProcess_as', process_UIAction) #Stablishes the connection with the server
client.wait_for_server() #Waits until the action server is available

def start_callback(data):
	"""
	Start automatic process execution
	"""
	global client
	global saved_index
	global saved_step1
	global saved_step2
	global saved_auto
	global process_time_init
	global process_time_accum
	global time_running
	saved_index = 0
	saved_step1 = 0
	saved_step2 = 0
	goal=process_UIGoal()
	goal.index=0
	goal.subindex=0
	goal.subindex2=0
	goal.auto = True
	saved_auto = True
	msg_log = String()
	msg_log.data = "Process started in automatic mode"
	logs_publisher.publish(msg_log)
	process_time_accum = 0
	process_time_init = time.time()
	time_running = True
	client.send_goal(goal, feedback_cb=feedback_callback)

subsL = rospy.Subscriber('/UI/start', Bool, start_callback)  

def resume_callback(data):
	"""
	Resume automatic process execution
	"""
	global client
	global saved_index
	global saved_step1
	global saved_step2
	global saved_auto
	global process_time_init
	global process_time_accum
	global time_running
	goal=process_UIGoal()
	goal.index=saved_index
	goal.subindex=saved_step1
	goal.subindex2=saved_step2
	goal.auto=saved_auto
	msg_log = String()
	msg_log.data = "Process resumed in operation " + str(saved_index)
	logs_publisher.publish(msg_log)
	process_time_init = time.time()
	time_running = True
	client.send_goal(goal, feedback_cb=feedback_callback)

subsL = rospy.Subscriber('/UI/resume', Bool, resume_callback) 

def stop_callback(data):
	"""
	Stop automatic process execution
	"""
	global saved_index
	global client
	global process_time_accum
	global time_running
	time_running = False
	process_time_accum = 0
	msg_log = String()
	msg_log.data = "Process stopped"
	logs_publisher.publish(msg_log)
	client.cancel_goal()
	saved_index = 0

subsL = rospy.Subscriber('/UI/stop', Bool, stop_callback)

def pause_callback(data):
	"""
	Pause automatic process execution
	"""
	global client
	global process_time_accum
	global time_running
	time_running = False
	process_time_accum = time.time() - process_time_init + process_time_accum
	msg_log = String()
	msg_log.data = "Process paused"
	logs_publisher.publish(msg_log)
	client.cancel_goal()

subsL = rospy.Subscriber('/UI/pause', Bool, pause_callback)

def step_callback(data):
	"""
	Run only one step execution
	"""
	global client
	global saved_index
	global saved_step1
	global saved_step2
	global saved_auto
	global process_time_init
	global process_time_accum
	global time_running
	goal=process_UIGoal()
	saved_index = data.data
	saved_step1 = 0
	saved_step2 = 0
	goal.index=saved_index
	goal.subindex=0
	goal.subindex2=0
	goal.auto=False
	saved_auto = False
	msg_log = String()
	msg_log.data = "Started execution of process operation " + str(saved_index)
	logs_publisher.publish(msg_log)
	process_time_accum = 0
	process_time_init = time.time()
	time_running = True
	client.send_goal(goal, feedback_cb=feedback_callback)


subsL = rospy.Subscriber('/UI/step', Int32, step_callback)

def feedback_callback(feedback):
	"""
	This function is called when the server sends feedback
	"""
	global saved_index
	global saved_step1
	global saved_step2
	global time_running
	saved_index = feedback.index
	saved_step1 = feedback.subindex
	saved_step2 = feedback.subindex2
	print(saved_index)
	msg = Int32()
	msg.data = saved_index
	index_publisher.publish(msg)
	if feedback.message == "done":
		time_running = False

publish_time_msg = configProp()
publish_time_msg.prop = "process_time"
old_publish_time = 0

while not rospy.is_shutdown():
	if time_running:
		publish_time = time.time() - process_time_init + process_time_accum
		publish_time_min = publish_time//60
		publish_time_sec = publish_time-(publish_time//60)*60		
		publish_time_msg.value = ""
		if publish_time_min<10:
			publish_time_msg.value = "0"+str(int(publish_time_min))
		else:
			publish_time_msg.value = str(int(publish_time_min))
		if publish_time_sec<10:
			publish_time_msg.value += ":0"+str(int(publish_time_sec))
		else:
			publish_time_msg.value += ":"+str(int(publish_time_sec))
		feedback_publisher.publish(publish_time_msg)
	rate.sleep()