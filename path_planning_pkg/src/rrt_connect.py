#!/usr/bin/env python

import rospy
import os
import sys
import PyKDL
import math
import copy
import numpy as np
import matplotlib.pyplot as plt
import env, plotting, utils
from elvez_pkg.msg import *
from elvez_pkg.srv import *
from path_planning_pkg.srv import *
from geometry_msgs.msg import Pose


#sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
#                "/../../Sampling_based_Planning/")


rospy.init_node('path_finder_node')


#Functions
def pose_to_frame(pose):
    """
    Converts a Pose into a PyKDL.Frame
    """
    frame_result = PyKDL.Frame() 
    frame_result.p = PyKDL.Vector(pose.position.x, pose.position.y, pose.position.z) 
    frame_result.M = PyKDL.Rotation.Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    return frame_result

def frame_to_pose(frame):
    """
    Convert PyKDL.Frame into a Pose
    """
    pose_result = Pose()
    pose_result.position.x = frame.p[0] 
    pose_result.position.y = frame.p[1] 
    pose_result.position.z = frame.p[2] 
    ang = frame.M.GetQuaternion() 
    pose_result.orientation.x = ang[0] 
    pose_result.orientation.y = ang[1] 
    pose_result.orientation.z = ang[2] 
    pose_result.orientation.w = ang[3]
    return pose_result

def cable_to_EE(p, angle, offset):
    p_ee = copy.copy(p)
    p_ee[0] += offset*math.cos(angle*math.pi/180)
    p_ee[1] += offset*math.sin(angle*math.pi/180)
    return p_ee

def EE_to_cable(p, angle, offset):
    return cable_to_EE(p, angle+180, offset)

def dist_2d(p1,p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)


class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None


class RrtConnect:
    def __init__(self, collisions, s_start, s_goal, step_len, goal_sample_rate, iter_max):
        print(s_start)
        self.s_start = Node(s_start)
        self.s_goal = Node(s_goal)
        self.step_len = step_len
        self.goal_sample_rate = goal_sample_rate
        self.iter_max = iter_max
        self.V1 = [self.s_start]
        self.V2 = [self.s_goal]

        #self.env = env.Env()
        self.env = collisions
        self.plotting = plotting.Plotting(self.env, s_start, s_goal)
        self.utils = utils.Utils(self.env)

        self.x_range = self.env.x_range
        self.y_range = self.env.y_range
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle
        self.obs_boundary = self.env.obs_boundary

    def planning(self):
        for i in range(self.iter_max):
            node_rand = self.generate_random_node(self.s_goal, self.goal_sample_rate)
            node_near = self.nearest_neighbor(self.V1, node_rand)
            node_new = self.new_state(node_near, node_rand)

            if node_new and not self.utils.is_collision(node_near, node_new):
                self.V1.append(node_new)
                node_near_prim = self.nearest_neighbor(self.V2, node_new)
                node_new_prim = self.new_state(node_near_prim, node_new)

                if node_new_prim and not self.utils.is_collision(node_new_prim, node_near_prim):
                    self.V2.append(node_new_prim)

                    while True:
                        node_new_prim2 = self.new_state(node_new_prim, node_new)
                        if node_new_prim2 and not self.utils.is_collision(node_new_prim2, node_new_prim):
                            self.V2.append(node_new_prim2)
                            node_new_prim = self.change_node(node_new_prim, node_new_prim2)
                        else:
                            break

                        if self.is_node_same(node_new_prim, node_new):
                            break

                if self.is_node_same(node_new_prim, node_new):
                    return self.extract_path(node_new, node_new_prim)

            if len(self.V2) < len(self.V1):
                list_mid = self.V2
                self.V2 = self.V1
                self.V1 = list_mid
            if i==(self.iter_max-1):
                print(i)

        return None

    @staticmethod
    def change_node(node_new_prim, node_new_prim2):
        node_new = Node((node_new_prim2.x, node_new_prim2.y))
        node_new.parent = node_new_prim

        return node_new

    @staticmethod
    def is_node_same(node_new_prim, node_new):
        if node_new_prim.x == node_new.x and \
                node_new_prim.y == node_new.y:
            return True

        return False

    def generate_random_node(self, sample_goal, goal_sample_rate):
        delta = self.utils.delta

        if np.random.random() > goal_sample_rate:
            return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                         np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))

        return sample_goal

    @staticmethod
    def nearest_neighbor(node_list, n):
        return node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y)
                                        for nd in node_list]))]

    def new_state(self, node_start, node_end):
        dist, theta = self.get_distance_and_angle(node_start, node_end)

        dist = min(self.step_len, dist)
        node_new = Node((node_start.x + dist * math.cos(theta),
                         node_start.y + dist * math.sin(theta)))
        node_new.parent = node_start

        return node_new

    @staticmethod
    def extract_path(node_new, node_new_prim):
        path1 = [(node_new.x, node_new.y)]
        node_now = node_new

        while node_now.parent is not None:
            node_now = node_now.parent
            path1.append((node_now.x, node_now.y))

        path2 = [(node_new_prim.x, node_new_prim.y)]
        node_now = node_new_prim

        while node_now.parent is not None:
            node_now = node_now.parent
            path2.append((node_now.x, node_now.y))

        return list(list(reversed(path1)) + path2)

    @staticmethod
    def get_distance_and_angle(node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)


def find_path_srv_callback(req):
    global collisions
    global guides
    global scale
    global length_cable
    global step

    #x_start = (2, 2)  # Starting node
    #x_goal = (49, 24)  # Goal node

    x=[]
    y=[]
    for guide in guides:
        x.append(guide.key_center_frame.position.x*scale)
        y.append(guide.key_center_frame.position.y*scale)

    if req.arm == "left":
        starting_point = [min(x)-0.07*scale, 0.65*scale]
    else:
        starting_point = [max(x)+0.07*scale, 0.65*scale]

    #starting_point = [req.init_pose.position.x*scale, req.init_pose.position.y*scale]

    end_guide = req.end_guide
    if req.target_provided:
        end_point = [req.target_pose.position.x*scale, req.target_pose.position.y*scale]
        end_pose = req.target_pose
    else:
        for guide in guides:
            if guide.id == end_guide:
                end_point = [guide.key_center_frame.position.x*scale, guide.key_center_frame.position.y*scale]
                end_pose = guide.key_center_frame
    end_frame = pose_to_frame(end_pose)
    rotZ = end_frame.M.GetEulerZYX()[0] * 180/math.pi
    
    """
    index=-1
    i = 0
    for obs in collisions.obs_rectangle:
        if end_point[0] > obs[0] and end_point[0] < obs[0]+obs[2]:
            if end_point[1] > obs[1] and end_point[1] < obs[1]+obs[3]:
                index = i
                break
        i+=1

    print(index)
    
    if index >= 0:
        removed_guide = collisions.obs_rectangle[index]
        collisions.remove_rectangle(index)
    """

    p_con = cable_to_EE(starting_point, rotZ, length_cable[req.WH])
    end_con = cable_to_EE(end_point, rotZ, length_cable[req.WH])

    """
    collisions.obs_rectangle_collision = []
    for obs in collisions.obs_rectangle_collision_all:
        if end_point[0] > obs[0] and end_point[0] < obs[0]+obs[2]:
            if end_point[1] > obs[1] and end_point[1] < obs[1]+obs[3]:
                continue
        if dist_2d([obs[0]+obs[2]/2,obs[1]+obs[3]/2], end_con) < (max(obs[2],obs[3])+step*1.5):
            continue
        #collisions.obs_rectangle_collision.append([obs[0]-safety_offset, obs[1]-safety_offset, obs[2]+(safety_offset*2), obs[3]+(safety_offset*2)]) #Don't add the angle
        collisions.obs_rectangle_collision.append(obs[0:4])
    """

    collisions.obs_circle_collision = []
    i=-1
    for obs in collisions.obs_circle_collision_all:
        i+=1
        if end_point[0] > (obs[0]-obs[2]) and end_point[0] < (obs[0]+obs[2]):
            if end_point[1] > (obs[1]-obs[2]) and end_point[1] < (obs[1]+obs[2]):
                collisions.obs_rectangle[i][5] = "target"
        if end_con[0] > (obs[0]-obs[2]) and end_con[0] < (obs[0]+obs[2]):
            if end_con[1] > (obs[1]-obs[2]) and end_con[1] < (obs[1]+obs[2]):
                collisions.obs_rectangle[i][5] = "disabled"
                continue
        if dist_2d([obs[0],obs[1]], end_con) < (obs[2]+step*1.5):
            collisions.obs_rectangle[i][5] = "disabled"
            continue
        collisions.obs_circle_collision.append(obs)

    rrt_conn = RrtConnect(collisions, p_con, end_con, step, 0.05, 50000)
    #rrt_conn = RrtConnect(collisions, starting_point, end_point, 0.8, 0.05, 5000)
    path = rrt_conn.planning()
    print(path)

    #transform path (points and poses) from the hanging connector to the EE
    length=0
    for i in range(1,len(path)):
        length+=dist_2d(path[i-1], path[i])
    path_EE = []
    path_pose_EE = []
    for p_tup in path:
        p = list(p_tup)
        path_EE.insert(0,EE_to_cable(p, rotZ, length_cable[req.WH])) #Add at the beginning. The generated path goes from last to first point
        new_pose = Pose()
        new_pose.position.x = path_EE[-1][0]
        new_pose.position.y = path_EE[-1][1]
        if req.interpolate_z:
            if len(path_EE)>1:
                length_accum += dist_2d(path_EE[-2], path_EE[-1])
            new_pose.position.z = end_pose.position.z + (req.offset_z * (1-min((length_accum/length),1)))
        else:
            new_pose.position.z = end_pose.position.z + req.offset_z 
        new_pose.orientation = copy.copy(end_pose.orientation)
        path_pose_EE.insert(0,new_pose)
    #path_EE[-1] = end_point
    #path_EE.insert(0,starting_point)
    path_pose_EE[-1] = end_pose
    rrt_conn.plotting.animation_connect(rrt_conn.V1, rrt_conn.V2, starting_point, end_point, path, path_EE, "RRT_CONNECT_"+str(req.WH)) #Save plot

    resp = pathFinderResponse()
    for pose in path_pose_EE:
        resp.path.append(pose)
    resp.success = True
    return resp

rospy.Service('/find_free_path_rrt', pathFinder, find_path_srv_callback)


scale = 100
step = 0.008*scale
safety_offset = 0.005*scale
length_cable = {'1': 0.15*scale, '2': 0.1*scale, '3': 0.15*scale} #For each WH
collisions = env.Env()
rospy.wait_for_service('/ELVEZ_platform_handler/all_guides')
get_guides_srv = rospy.ServiceProxy('/ELVEZ_platform_handler/all_guides', guide_all_info)
guides = get_guides_srv().data
x=[]
y=[]
for guide in guides:
    """
    guide_frame = pose_to_frame(guide.key_center_frame)
    rotZ = guide_frame.M.GetEulerZYX()[0] * 180/math.pi
    collisions.add_rectangle([(guide.key_center_frame.position.x*scale-guide.dimensions[0]*scale/2), (guide.key_center_frame.position.y*scale-guide.dimensions[1]*scale/2), (guide.dimensions[0]*scale), (guide.dimensions[1]*scale), rotZ]) 
    x.append(guide.key_center_frame.position.x*scale)
    y.append(guide.key_center_frame.position.y*scale)
    """
    guide_frame = pose_to_frame(guide.jig_corner_frame)
    rotZ = guide_frame.M.GetEulerZYX()[0] * 180/math.pi
    #collisions.obs_rectangle_collision_all.append([(guide.key_center_frame.position.x*scale-guide.dimensions[0]*scale/2)-safety_offset, (guide.key_center_frame.position.y*scale-guide.dimensions[1]*scale/2)-safety_offset, (guide.dimensions[0]*scale)+(safety_offset*2), (guide.dimensions[1]*scale)+(safety_offset*2)])
    """
    if 'C' in guide.id:
        if (abs(rotZ) > (1.571-0.1) and abs(rotZ) < (1.571+0.1)):
            collisions.add_rectangle([guide.jig_center_frame.position.y*scale-guide.jig_dimensions[1]*scale/2, guide.jig_center_frame.position.x*scale-guide.jig_dimensions[0]*scale/2, (guide.jig_dimensions[1]*scale), (guide.jig_dimensions[0]*scale),0]) 
        else:
            collisions.add_rectangle([guide.jig_center_frame.position.x*scale-guide.jig_dimensions[0]*scale/2, guide.jig_center_frame.position.y*scale-guide.jig_dimensions[1]*scale/2, (guide.jig_dimensions[0]*scale), (guide.jig_dimensions[1]*scale),0]) 
    elif guide.id == 'J7' or guide.id == 'J8' or guide.id == 'J9':
        #rot_frame = PyKDL.Frame()
        #rot_frame.M.DoRotZ(-rotZ)
        #center_ori_frame = pose_to_frame(guide.jig_center_frame).Inverse() * pose_to_frame(guide.jig_corner_frame) #origin frame seen from center
        #print(guide.id)
        #print(rot_frame)
        #print(center_ori_frame)
        #new_ori_frame = pose_to_frame(guide.jig_center_frame) * rot_frame * center_ori_frame
        #print(new_ori_frame)
        center_ori_frame = PyKDL.Frame()
        center_ori_frame.p = PyKDL.Vector(-guide.dimensions[0]/2,-guide.dimensions[1]/2, 0)
        new_ori_frame = pose_to_frame(guide.jig_center_frame) * center_ori_frame
        new_ori_pose = frame_to_pose(new_ori_frame)
        collisions.add_rectangle([(new_ori_pose.position.x*scale), (new_ori_pose.position.y*scale), (guide.dimensions[0]*scale), (guide.dimensions[1]*scale), rotZ])
        #collisions.add_rectangle([(guide.jig_corner_frame.position.x*scale), (guide.jig_corner_frame.position.y*scale), (guide.dimensions[0]*scale), (guide.dimensions[1]*scale), rotZ])
    else:
        collisions.add_rectangle([(guide.jig_corner_frame.position.x*scale), (guide.jig_corner_frame.position.y*scale), (guide.dimensions[0]*scale), (guide.dimensions[1]*scale), rotZ]) 
    """
    center_ori_frame = PyKDL.Frame()
    center_ori_frame.p = PyKDL.Vector(-guide.jig_dimensions[0]/2,-guide.jig_dimensions[1]/2, 0)
    new_ori_frame = pose_to_frame(guide.jig_center_frame) * center_ori_frame
    new_ori_pose = frame_to_pose(new_ori_frame)
    collisions.add_rectangle([(new_ori_pose.position.x*scale), (new_ori_pose.position.y*scale), (guide.jig_dimensions[0]*scale), (guide.jig_dimensions[1]*scale), rotZ, ''])
    collisions.obs_rectangle_collision_all.append([guide.jig_center_frame.position.x*scale, guide.jig_center_frame.position.y*scale, (guide.jig_dimensions[0]*scale)+(safety_offset*2), (guide.jig_dimensions[1]*scale)+(safety_offset*2)])
    collisions.obs_circle_collision_all.append([guide.jig_center_frame.position.x*scale, guide.jig_center_frame.position.y*scale, max(guide.jig_dimensions[0], guide.jig_dimensions[1])*scale/2 + safety_offset])
    x.append(guide.jig_center_frame.position.x*scale)
    y.append(guide.jig_center_frame.position.y*scale)
    #print(guide.jig_center_frame)
    #print(guide.jig_corner_frame)
    #print("#############")

#collisions.add_rectangle([-56.5,39,20,20,0]) #Test
wall_thickness = 0.003*scale
margin_x = 0.28*scale
margin_y = 0.1*scale
"""
collisions.add_boundary([min(x)-margin_x, min(y)-margin_y, wall_thickness, max(y)-min(y)+(margin_y*2)]) #LEFT
collisions.add_boundary([min(x)-margin_x, max(y)+margin_y, max(x)-min(x)+(margin_x*2), wall_thickness]) #UP
collisions.add_boundary([min(x)-margin_x+wall_thickness, min(y)-margin_y, max(x)-min(x)+(margin_x*2), wall_thickness]) #DOWN
collisions.add_boundary([max(x)+margin_x, min(y)-margin_y+wall_thickness, wall_thickness, max(y)-min(y)+(margin_y*2)]) #RIGHT
#collisions.add_boundary([min(x)-0.1, min(y)-0.15, max(x)-min(x)+0.2, max(y)-min(y)+0.3])
#"""

print("Ready")
rospy.spin()