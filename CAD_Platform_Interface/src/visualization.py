import rospy
from visualization_msgs.msg import Marker
import PyKDL 
import numpy as np

class Color:
    def __init__(self, r=1.0, g=1.0, b=1.0, a=1): 
        self.r = r 
        self.g = g 
        self.b = b 
        self.a = a

def createMesh(frame_id, mesh_path='', transform=PyKDL.Frame(), color=Color(), scale=np.array([1, 1, 1]), name=""): 
    """ Creates a Mesh Marker """ 
    marker = Marker() 
    marker.header.frame_id = frame_id 
    marker.type = Marker.MESH_RESOURCE 
    marker.ns = name 
    marker.action = marker.ADD 
    marker.mesh_resource = mesh_path 
    marker.color.r = color.r 
    marker.color.g = color.g 
    marker.color.b = color.b 
    marker.color.a = color.a 
    marker.scale.x = scale[0] 
    marker.scale.y = scale[1] 
    marker.scale.z = scale[2] 
    marker.pose.position.x = transform.p.x()
    marker.pose.position.y = transform.p.y()
    marker.pose.position.z = transform.p.z()
    quat = transform.M.GetQuaternion() 
    marker.pose.orientation.x = quat[0] 
    marker.pose.orientation.y = quat[1] 
    marker.pose.orientation.z = quat[2] 
    marker.pose.orientation.w = quat[3] 
    return marker

def createKeypoint(frame_id, transform=PyKDL.Frame(), color=Color(1, 0, 0, 1), scale=np.array([1, 1, 1]), name=""): 
    """ Creates a Keypoint Marker """ 
    marker = Marker() 
    marker.header.frame_id = frame_id 
    marker.type = Marker.SPHERE
    marker.action = marker.ADD 
    marker.ns = name 
    marker.color.r = color.r 
    marker.color.g = color.g 
    marker.color.b = color.b 
    marker.color.a = color.a 
    marker.scale.x = scale[0]*0.01
    marker.scale.y = scale[1]*0.01
    marker.scale.z = scale[2]*0.01
    marker.pose.position.x = transform.p.x() 
    marker.pose.position.y = transform.p.y()
    marker.pose.position.z = transform.p.z()
    quat = transform.M.GetQuaternion() 
    marker.pose.orientation.x = quat[0] 
    marker.pose.orientation.y = quat[1] 
    marker.pose.orientation.z = quat[2] 
    marker.pose.orientation.w = quat[3] 
    print("Working properly...")
    return marker
