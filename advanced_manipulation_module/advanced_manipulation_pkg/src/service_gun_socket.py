#! /usr/bin/env python
import sys
import rospy
import time
import os
import socket
from std_srvs.srv import Trigger, TriggerResponse

rospy.init_node('service_gun_socket', anonymous=True)

print("Running")
service_name = '/gun/tape'

def service_callback(req): 
	print("Taping...")
	resp = TriggerResponse()
	try:
		my_socket = socket.socket()
		my_socket.settimeout(5.0)
		#my_socket.connect(('192.168.43.201', 54000))
		my_socket.connect(('192.168.64.99', 8070))
		socket_msg= 'test_client'
		my_socket.send(str(socket_msg))
		resp.success = True
		my_socket.close()
	except Exception as error:
		print("Socket error occurred:", error)
		resp.success = False
		my_socket.close()
	rospy.sleep(3) #Taping time
	print("Done")
	return resp


rospy.Service(service_name, Trigger, service_callback)


# Keep this process running until Enter is pressed
print ("Press Enter to quit...")
try:
	sys.stdin.readline()
except KeyboardInterrupt:
	print("End of test")

