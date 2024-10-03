#!/usr/bin/env python

import omron
import rospy
import numpy as np
import math 
import time
import os
from std_msgs.msg import Bool

#Initialize ROS node 
rospy.init_node('RSM_PLC_connection') 
rate = rospy.Rate(10)
topic = 'RSM/PLC_variable' 
publisher = rospy.Publisher(topic, Bool, queue_size=1)

#Establish the EIP connection with the PLC
EIP_instance = omron.n_series.NSeriesEIP()
EIP_instance.connect_explicit('192.168.1.1')
EIP_instance.register_session()
EIP_instance.update_variable_dictionary()

while not rospy.is_shutdown(): 
	PLC_read = EIP_instance.read_variable('ROS_read_cplc')
	#print(str(PLC_read))
	#PLC_message = Bool()
	#PLC_message.data = PLC_read
	#publisher.publish(PLC_message)
	EIP_instance.write_variable('ROS_write_cplc', PLC_read)
	rate.sleep()
