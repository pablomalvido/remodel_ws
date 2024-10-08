#! /usr/bin/env python
import copy
import PyKDL 
from moveit_msgs.msg import *
from moveit_msgs.srv import *
from std_msgs.msg import *
from std_srvs.srv import *
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
import math
from sensor_msgs.msg import *
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import xml.etree.ElementTree as ET
import tf
from ROS_UI_backend.msg import *
from ROS_UI_backend.srv import *
from rosapi.srv import *


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


def pose_to_frame(pose):
        """
        Converts a Pose into a PyKDL.Frame
        """
        frame_result = PyKDL.Frame() 
        frame_result.p = PyKDL.Vector(pose.position.x, pose.position.y, pose.position.z) 
        frame_result.M = PyKDL.Rotation.Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        return frame_result


def listener_to_frame(trans, rot):
        frame_result = PyKDL.Frame() 
        frame_result.p = PyKDL.Vector(trans[0], trans[1], trans[2]) 
        frame_result.M = PyKDL.Rotation.Quaternion(rot[0], rot[1], rot[2], rot[3])
        return frame_result


def get_transpose_rot(rot_original):
        """
        Retrieves the transpose of a rotation matrix [PyKDL.Rotation]
        """
        rot_trans = PyKDL.Rotation(rot_original[0,0], rot_original[1,0], rot_original[2,0], rot_original[0,1], rot_original[1,1], rot_original[2,1], rot_original[0,2], rot_original[1,2], rot_original[2,2])
        return rot_trans


def get_inverse_frame(frame_original):
        """
        Retrieves the inverse of a frame [PyKDL.Frame]
        """
        frame_inv = PyKDL.Frame() 
        x = -(frame_original.p[0]*frame_original.M[0,0] + frame_original.p[1]*frame_original.M[1,0] + frame_original.p[2]*frame_original.M[2,0])
        y = -(frame_original.p[0]*frame_original.M[0,1] + frame_original.p[1]*frame_original.M[1,1] + frame_original.p[2]*frame_original.M[2,1])
        z = -(frame_original.p[0]*frame_original.M[0,2] + frame_original.p[1]*frame_original.M[1,2] + frame_original.p[2]*frame_original.M[2,2])
        frame_inv.p = PyKDL.Vector(x,y,z) 
        frame_inv.M = get_transpose_rot(frame_original.M)
        return frame_inv


def compute_distance(pose1, pose2):
    """
    Retrieves the linear distance [mm] between two poses.
    """
    dist = math.sqrt((pose1.position.x - pose2.position.x)**2 + (pose1.position.y - pose2.position.y)**2 + (pose1.position.z - pose2.position.z)**2)
    return dist
   

def compute_distance_xy(pose1, pose2):
    """
    Retrieves the linear distance [mm] between two poses in xy.
    """
    dist = math.sqrt((pose1.position.x - pose2.position.x)**2 + (pose1.position.y - pose2.position.y)**2)
    return dist


def compute_angle_distance(pose1, pose2):
    """
    Retrieves the angular [rad] distance between two poses.
    """
    frame1 = pose_to_frame(pose1)
    frame2 = pose_to_frame(pose2)
    frame12 = frame1.Inverse() * frame2
    rot = abs(frame12.M.GetRotAngle()[0])
    return rot


def compute_lin_or_ang_distance(pose1, pose2, linear = True):
    """
    Retrieves the linear (linear = True) distance [mm] or angular (linear = False) [rad] distance between two poses.
    """
    if linear:
        return compute_distance(pose1, pose2)
    else:
        return compute_angle_distance(pose1, pose2)


def get_shifted_pose(origin_pose, shift):
        """
        Retrives a shifted pose
        - origin_pose: Original pose [Pose]
        - shift: List of shifts and rotations in all the angles ([x displacement, y displacement, z displacement, x rotation, y rotation, z rotation]). Dist in m and angles in rad
        """
        tf_origin = pose_to_frame(origin_pose)   
        tf_shift = PyKDL.Frame() 
        tf_shift.p = PyKDL.Vector(shift[0], shift[1], shift[2]) 
        tf_shift.M.DoRotX(shift[3]) 
        tf_shift.M.DoRotY(shift[4]) 
        tf_shift.M.DoRotZ(shift[5])
        tf_result = tf_origin * tf_shift

        pose_result = frame_to_pose(tf_result)
        return pose_result


def degree_difference(R1, R2):
        """
        Retrieves the angle difference between two PyKDL.Rotation
        """
        R_1_2 = get_transpose_rot(R1) * R2
        rad_dif = R_1_2.GetRPY()
        deg_dif = [element * (180.0/math.pi) for element in rad_dif]
        return deg_dif, rad_dif


def get_axis(pose1, pose2):
        """
        pose1: final pose
        pose2: initial pose
        """
        dist = compute_distance(pose1, pose2)
        axis = [(pose1.position.x - pose2.position.x)/dist, (pose1.position.y - pose2.position.y)/dist, (pose1.position.z - pose2.position.z)/dist]
        return axis


def get_axis_from_RM(RM, axis_name):
        axis = []
        if axis_name == "R":
                col = 0
        elif axis_name == "P":
                col = 1
        elif axis_name == "Y":
                col = 2
        axis = [RM[0, col], RM[1, col], RM[2, col]]
        return axis

def get_quaternion_in_Z(angle):
        """
        angle in rad
        """
        qw = math.cos(angle/2)
        qz = math.sin(angle/2)
        pose_quat = Pose()
        pose_quat.orientation.x = 0.0
        pose_quat.orientation.y = 0.0
        pose_quat.orientation.z = qz 
        pose_quat.orientation.w = qw
        return pose_quat


def cross_product_vectors(v1, v2):
        #print("\nv1:")
        #print(v1)
        #print("\nv2:")
        #print(v2)
        return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2]


def get_ort_axis(v1, v2):
        v3 = [0,0,0]
        v3_0 = [False, False, False]
        # print(v1)
        # print(v2)

        index = 0
        for i in v1:
                if abs(i)>1:
                        v1[index] = i/abs(i)
                index+=1
        index = 0
        for i in v2:
                if abs(i)>1:
                        v2[index] = i/abs(i)
                index+=1

        if v1[0]**2 + v2[0]**2 > 1:
                sign = v2[0]/abs(v2[0])
                v2[0] = sign * math.sqrt(1 - v1[0]**2)
                v3_0[0] = True
                if v2[0]**2 + v2[1]**2 > 1:
                        sign = v2[1]/abs(v2[1])
                        v2[1] = sign * math.sqrt(1 - v2[0]**2)
                        v2[2] = 0.0
        if v1[1]**2 + v2[1]**2 > 1:
                sign = v2[1]/abs(v2[1])
                v2[1] = sign * math.sqrt(1 - v1[1]**2)
                v3_0[1] = True
                if v2[0]**2 + v2[1]**2 > 1:
                        sign = v2[1]/abs(v2[1])
                        v2[1] = sign * math.sqrt(1 - v2[0]**2)
                        v2[2] = 0.0
        if v1[2]**2 + v2[2]**2 > 1:
                sign = v2[2]/abs(v2[2])
                v2[2] = sign * math.sqrt(1 - v1[2]**2)
                v3_0[2] = True
                if v2[0]**2 + v2[2]**2 > 1:
                        sign = v2[0]/abs(v2[0])
                        v2[0] = sign * math.sqrt(1 - v2[2]**2)
                        v2[1] = 0.0
        #print(v1)
        #print(v2)
        if not v3_0[0]:
                #v3[0] = math.sqrt(1 - v1[0]**2 - v2[0]**2)
                v3[0] = (v1[1]*v2[2] - v2[1]*v1[2])
        if not v3_0[1]:
                #v3[1] = math.sqrt(1 - v1[1]**2 - v2[1]**2)
                v3[1] = -(v1[0]*v2[2] - v2[0]*v1[2])
        if not v3_0[2]:
                #v3[2] = math.sqrt(1 - v1[2]**2 - v2[2]**2)
                v3[2] = (v1[0]*v2[1] - v2[0]*v1[1])
        return v1, v2, v3


def get_middle_pose(pose1, pose2):
        frame3 = PyKDL.Frame()
        if pose1.position.x > pose2.position.x:
                pose1_new = pose1
                pose2_new = pose2
        else:
                pose1_new = copy.deepcopy(pose2)
                pose2_new = copy.deepcopy(pose1)
        frame3.p = PyKDL.Vector((pose1_new.position.x + pose2_new.position.x)/2, (pose1_new.position.y + pose2_new.position.y)/2, (pose1_new.position.z + pose2_new.position.z)/2)
        rot_angle=math.atan2((pose1_new.position.y - pose2_new.position.y), (pose1_new.position.x - pose2_new.position.x))
        frame3.M = frame3.M.RotZ(rot_angle)        
        return frame_to_pose(frame3)


def compute_distance_relative(pose1, pose2, axis):
        dist_vector = [(pose1.position.x - pose2.position.x), (pose1.position.y - pose2.position.y), (pose1.position.z - pose2.position.z)]
        dist_in_axis = axis[0] * dist_vector[0] + axis[1] * dist_vector[1] + axis[2] * dist_vector[2]
        return dist_in_axis


def interpolate_trajectory(initial_pose, final_pose, step_pos_min, step_deg_min, n_points_max):
        """
        Creates several waypoints between two poses
        """
        waypoints = []
        if initial_pose == final_pose:
                print("Cannot interpolate points, it is the same pose")
                return False, waypoints

        n_points = n_points_max
        step_pos_min = float(step_pos_min)
        step_deg_min = float(step_deg_min)
        pos_dif = compute_distance(initial_pose, final_pose)
        deg_dif, rad_dif = degree_difference(pose_to_frame(initial_pose).M, pose_to_frame(final_pose).M)
        n_points_list = []
        n_points_list.append(pos_dif/step_pos_min)
        n_points_list.append(abs(deg_dif[0])/step_deg_min)
        n_points_list.append(abs(deg_dif[1])/step_deg_min)
        n_points_list.append(abs(deg_dif[2])/step_deg_min)
        if max(n_points_list) < 20:
                n_points = int(max(n_points_list))
        
        waypoints.append(initial_pose)
        for point in range(n_points):
            if point > 0:
                x = initial_pose.position.x + ((final_pose.position.x - initial_pose.position.x)*float(point)/float(n_points))
                y = initial_pose.position.y + ((final_pose.position.y - initial_pose.position.y)*float(point)/float(n_points))
                z = initial_pose.position.z + ((final_pose.position.z - initial_pose.position.z)*float(point)/float(n_points))
                rotation = pose_to_frame(initial_pose).M
                rotation.DoRotX((rad_dif[0])*float(point)/float(n_points))
                rotation.DoRotY((rad_dif[1])*float(point)/float(n_points))
                rotation.DoRotZ((rad_dif[2])*float(point)/float(n_points))
                new_frame = PyKDL.Frame() 
                new_frame.p = PyKDL.Vector(x,y,z) 
                new_frame.M = rotation
                waypoints.append(frame_to_pose(new_frame))
        waypoints.append(final_pose)

        return True, waypoints


def circular_trajectory(center, initial_pose, degree, rot_axis, center_waypoints, circle_waypoints, step = 2, rot_gripper = False): #R axis is x and Rot axis is z in its own rotation frame
        R_axis = get_axis(initial_pose, center)
        threshold_ort = 0.05 #0.001
        if abs(cross_product_vectors(R_axis, rot_axis)) > threshold_ort:
                print("Error. axis are not orthogonal")
                return False, center_waypoints, circle_waypoints
        R_axis, rot_axis, y_axis = get_ort_axis(R_axis, rot_axis)
        RM_world_own = PyKDL.Frame() 
        RM_world_own.p = PyKDL.Vector(center.position.x, center.position.y, center.position.z)
        RM_world_own.M = PyKDL.Rotation(R_axis[0], y_axis[0], rot_axis[0], R_axis[1], y_axis[1], rot_axis[1], R_axis[2], y_axis[2], rot_axis[2])

        #Get initial gripper orientation from own circle frame
        Rot_own_world = get_transpose_rot(RM_world_own.M)
        Rot_world_gripper_cicle = PyKDL.Rotation.Quaternion(initial_pose.orientation.x, initial_pose.orientation.y, initial_pose.orientation.z, initial_pose.orientation.w) 
        initial_gripper_circle_ori_own = Rot_own_world * Rot_world_gripper_cicle
        Rot_world_gripper_center = PyKDL.Rotation.Quaternion(center.orientation.x, center.orientation.y, center.orientation.z, center.orientation.w)
        initial_gripper_center_ori_own = Rot_own_world * Rot_world_gripper_center

        R = compute_distance(center, initial_pose)
        
        #Initialize variables
        circle_waypoints_own = []
        center_waypoints_own = []
        new_waypoint_circle_own = PyKDL.Frame()
        new_waypoint_circle_own.p = PyKDL.Vector(R,0,0)
        new_waypoint_circle_own.M = initial_gripper_circle_ori_own
        circle_waypoints_own.append(new_waypoint_circle_own)
        new_waypoint_center_own = PyKDL.Frame()
        new_waypoint_center_own.p = PyKDL.Vector(0,0,0)
        new_waypoint_center_own.M = initial_gripper_center_ori_own
        #center_waypoint_init_own = new_waypoint_center_own
        center_waypoints_own.append(new_waypoint_center_own)
        circle_waypoints.append(initial_pose) #pose
        center_waypoints.append(center) #pose

        #Create several waypoints for the circular trajectory in its own frame
        for deg in range(step, int(degree)+step, step):
                if abs(deg) >= abs(degree): #To have inclusive range
                        deg = degree
                deg = float(deg)*(math.pi/180.0)
                new_waypoint_circle_own = PyKDL.Frame()
                new_waypoint_circle_own.p = PyKDL.Vector(R * math.cos(deg), R * math.sin(deg), 0) #In its own frame the rotation is around z axis
                #print(new_waypoint_circle_own.p)
                #new_waypoint_circle_own.M = circle_waypoints_own[-1].M
                new_waypoint_circle_own.M = circle_waypoints_own[0].M
                new_waypoint_center_own = PyKDL.Frame()
                #new_waypoint_center_own = center_waypoints_own[-1]
                new_waypoint_center_own = copy.deepcopy(center_waypoints_own[0])
                #print(new_waypoint_center_own)
                if rot_gripper:
                        new_waypoint_circle_own.M.DoRotZ(deg)
                        new_waypoint_center_own.M.DoRotZ(deg)
                        #print(new_waypoint_center_own)
                circle_waypoints_own.append(new_waypoint_circle_own) #frame
                center_waypoints_own.append(new_waypoint_center_own) #frame
                #Transform this circular curve that is in its own frame to the real rotation frame
                circle_waypoint_pose_new = frame_to_pose(RM_world_own * new_waypoint_circle_own) #pose
                center_waypoints_pose_new = frame_to_pose(RM_world_own * new_waypoint_center_own) #pose
                if not rot_gripper:
                       circle_waypoint_pose_new.orientation = initial_pose.orientation

                circle_waypoints.append(circle_waypoint_pose_new) #pose
                center_waypoints.append(center_waypoints_pose_new) #pose

        return True, center_waypoints, circle_waypoints


def interpolate_trajectory(initial_pose, final_pose, step_pos_min, step_deg_min, n_points_max):
        """
        Creates several waypoints between two poses:
        - initial_pose: Initial pose [Pose]
        - final_pose: Final pose [Pose]
        - step_pos_min: Minimum distance between consecutive intermediate poses [m]
        - step_deg_min: Minimum angle between consecutive intermediate poses [angle]
        - n_points_max: Maximum number of waypoints of the interpolated path [int]
        """
        waypoints = []
        if initial_pose == final_pose:
                print("Cannot interpolate points, it is the same pose")
                return False, waypoints

        n_points = n_points_max
        step_pos_min = float(step_pos_min)
        step_deg_min = float(step_deg_min)
        pos_dif = compute_distance(initial_pose, final_pose)
        deg_dif, rad_dif = degree_difference(pose_to_frame(initial_pose).M, pose_to_frame(final_pose).M)
        n_points_list = []
        n_points_list.append(pos_dif/step_pos_min)
        n_points_list.append(abs(deg_dif[0])/step_deg_min)
        n_points_list.append(abs(deg_dif[1])/step_deg_min)
        n_points_list.append(abs(deg_dif[2])/step_deg_min)
        if max(n_points_list) < 20:
                n_points = int(max(n_points_list))
        
        waypoints.append(initial_pose)
        for point in range(n_points):
            if point > 0:
                x = initial_pose.position.x + ((final_pose.position.x - initial_pose.position.x)*float(point)/float(n_points))
                y = initial_pose.position.y + ((final_pose.position.y - initial_pose.position.y)*float(point)/float(n_points))
                z = initial_pose.position.z + ((final_pose.position.z - initial_pose.position.z)*float(point)/float(n_points))
                rotation = pose_to_frame(initial_pose).M
                rotation.DoRotX((rad_dif[0])*float(point)/float(n_points))
                rotation.DoRotY((rad_dif[1])*float(point)/float(n_points))
                rotation.DoRotZ((rad_dif[2])*float(point)/float(n_points))
                new_frame = PyKDL.Frame() 
                new_frame.p = PyKDL.Vector(x,y,z) 
                new_frame.M = rotation
                waypoints.append(frame_to_pose(new_frame))
        waypoints.append(final_pose)

        return True, waypoints     