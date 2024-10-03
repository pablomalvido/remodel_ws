#! /usr/bin/env python
import sys
import copy
import rospy
import PyKDL 
import tf
import moveit_commander
from moveit_msgs.msg import *
from moveit_msgs.srv import *
import geometry_msgs.msg
from std_msgs.msg import *
from std_srvs.srv import *
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger, TriggerResponse
from elvez_pkg.msg import *
from elvez_pkg.srv import *
import math
import matplotlib.pyplot as plt
import numpy as np


#ROS init
rospy.init_node('test_robot_pose', anonymous=True)


#params
R_collision = 0.03
lateral_increase = 0.005
step_increase = 0.002
length_cable = {'1': 0.15, '2': 0.1, '3': 0.15} #For each WH

#main
rospy.wait_for_service('/ELVEZ_platform_handler/all_guides')
get_guides_srv = rospy.ServiceProxy('/ELVEZ_platform_handler/all_guides', guide_all_info)
guides = get_guides_srv().data
x=[]
y=[]
guide_points=[]
for guide in guides:
    guide_points.append([guide.key_center_frame.position.x, guide.key_center_frame.position.y])


#Functions
def pose_to_frame(pose):
        """
        Converts a Pose into a PyKDL.Frame
        """
        frame_result = PyKDL.Frame() 
        frame_result.p = PyKDL.Vector(pose.position.x, pose.position.y, pose.position.z) 
        frame_result.M = PyKDL.Rotation.Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        return frame_result


def cable_to_EE(p, angle, offset):
    p_ee = copy.copy(p)
    p_ee[0] += offset*math.cos(angle*math.pi/180)
    p_ee[1] += offset*math.sin(angle*math.pi/180)
    return p_ee

def EE_to_cable(p, angle, offset):
    return cable_to_EE(p, angle+180, offset)


def dist_2d(p1,p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

    
def check_collisions(p1, end, R=R_collision):
    global guide_points
    for p2 in guide_points:
        if dist_2d(p2,end)>step_increase:
            dist = dist_2d(p1,p2)
            if dist < R:
                return True #There is collision
    return False
    

def check_repeated(p, p_list, eps = step_increase/10):
    for p_list_i in p_list:
        if dist_2d(p, p_list_i)<eps:
            return True #It is repeated
    return False


def interpolate_next(p1,p2,step=0.001):
    dist = dist_2d(p1,p2)
    p3 = [p1[0]+(p2[0]-p1[0]) * step / dist, p1[1]+(p2[1]-p1[1]) * step / dist]
    return p3


def get_path(p, end, length = 0, prev = 0):
    #print("GET")
    i=0
    straight = True
    while dist_2d(p[-1], end) > max(R_collision + 2*step_increase, 0.02):
        p_test = interpolate_next(p[-1], end, step=step_increase)
        if not check_collisions(p_test, end):
            #print("STRAIGHT")
            p.append(p_test)
            length += step_increase
            prev = 0
        else:
            straight = False
            length += lateral_increase
            success1 = True
            success2 = True
            length1 = 1000000
            length2 = 1000000

            if prev != 2:
                p_copy = copy.copy(p)
                end_copy = copy.copy(end)
                length_copy = copy.copy(length)
                p_new = copy.copy(p_copy[-1])
                p_new[1] += lateral_increase
                if not check_collisions(p_new, end):
                    if check_repeated(p_new, p_copy):
                        p1 = p_copy
                        length1 = length_copy
                        break
                    else:
                        p_copy.append(p_new)
                else:
                    # print('error')
                    success1 = False
                if success1:
                    # print("P1")
                    # print(p_copy[-1])
                    # rospy.sleep(0.2)
                    p1, length1, success1 = get_path(p_copy,end_copy,length_copy,prev=1)

            if prev != 1:
                p_copy2 = copy.copy(p)
                end_copy2 = copy.copy(end)
                length_copy2 = copy.copy(length)
                p_new2 = copy.copy(p_copy2[-1])
                p_new2[1] -= lateral_increase
                if not check_collisions(p_new2, end):
                    if check_repeated(p_new2, p_copy2):
                        p2 = p_copy2
                        length2 = length_copy2
                        break
                    else:
                        p_copy2.append(p_new2)
                else:
                    # print('error')
                    success2 = False
                if success2:
                    # print("P2")
                    # print(p_copy2[-1])
                    # rospy.sleep(0.2)
                    p2, length2, success2 = get_path(p_copy2,end_copy2,length_copy2,prev=2)

            if (not success1 or prev==2) and (not success2 or prev==1):
                return [], 1000000, False
            
            break

        i+=1
        if i > 5000:
            return [], 1000000, False

    if not straight:
        if length1 >= length2:
            p = p2
            length = length2
        else:
            p = p1
            length = length1

    return p, length, True


def plot_path(p, end, end_pose, angle, offset_cable, offset_z, interpolate_z):
    p_con = [cable_to_EE(p[0], angle, offset_cable)]
    end_con = cable_to_EE(end, angle, offset_cable)
    p_con, length, success = get_path(p_con, end_con)
    if not success:
        return [], False
    length += dist_2d(p_con[-1], end_con)
    p_con.append(end_con)
    p_EE = []
    p_pose_EE = []
    length_accum = 0
    for p_con_i in p_con:
        p_EE.append(EE_to_cable(p_con_i, angle, offset_cable))
        new_pose = Pose()
        new_pose.position.x = p_EE[-1][0]
        new_pose.position.y = p_EE[-1][1]
        if interpolate_z:
            if len(p_EE)>1:
                length_accum += dist_2d(p_EE[-2], p_EE[-1])
            new_pose.position.z = end_pose.position.z + (offset_z * (1-(length_accum/length)))
        else:
            new_pose.position.z = end_pose.position.z + offset_z 
        new_pose.orientation = copy.copy(end_pose.orientation)
        p_pose_EE.append(new_pose)
    p_EE[-1] = end
    p_pose_EE[-1] = end_pose

    x_path = []
    y_path = []
    for point in p_con:
        x_path.append(point[0])
        y_path.append(point[1])   
    xTraj = np.array(x_path)
    yTraj = np.array(y_path)
    plt.plot(xTraj, yTraj, color='black')

    x_path1 = []
    y_path1 = []
    for point in p_EE:
        x_path1.append(point[0])
        y_path1.append(point[1])   
    xTraj1 = np.array(x_path1)
    yTraj1 = np.array(y_path1)
    plt.plot(xTraj1, yTraj1, color='pink')
    return p_pose_EE, True


def find_path_cb(req):
    global guides
    global guide_points

    x=[]
    y=[]
    for guide in guides:
        x.append(guide.key_center_frame.position.x)
        y.append(guide.key_center_frame.position.y)
    xpoints = np.array(x)
    ypoints = np.array(y)
    plt.plot(xpoints, ypoints, 'o')

    if req.arm == "left":
        starting_point = [min(x)-0.07, 0.65]
    else:
        starting_point = [max(x)+0.07, 0.65]

    x2 = np.array([starting_point[0]])
    y2 = np.array([starting_point[1]])
    plt.plot(x2, y2, 'o', color='r')

    end_guide = req.end_guide
    plot_EP_x = []
    plot_EP_y = []
    if req.target_provided:
        end_point = [req.target_pose.position.x, req.target_pose.position.y]
        end_pose = req.target_pose
    else:
        for guide in guides:
            if guide.id == end_guide:
                end_point = [guide.key_center_frame.position.x, guide.key_center_frame.position.y]
                end_pose = guide.key_center_frame
    print(req.target_pose)
    print(end_point)
    plot_EP_x.append(end_point[0])
    plot_EP_y.append(end_point[1])
    xEpoints = np.array(plot_EP_x)
    yEpoints = np.array(plot_EP_y)
    plt.plot(xEpoints, yEpoints, 'o', color='g')

    end_frame = pose_to_frame(end_pose)
    rotZ = end_frame.M.GetEulerZYX()[0] * 180/math.pi
    wp, success = plot_path([starting_point], end_point, end_pose, rotZ, offset_cable = length_cable[req.WH], offset_z = req.offset_z, interpolate_z = req.interpolate_z) #Include orientation and Z
    if req.show:
        plt.show()
    resp = pathFinderResponse()
    for pose in wp:
        resp.path.append(pose)
    resp.success = success
    return resp    

rospy.Service("/find_free_path", pathFinder, find_path_cb)


print("Ready")
rospy.spin()