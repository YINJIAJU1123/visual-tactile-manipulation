# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
import numpy as np
from math import sqrt
import re
from Utils.robotiq_gripper import RobotiqGripper
import tf


def Coordinate_camera(file_path):
    x = y = z = 0.0
    orientation = [0.0, 0.0, 0.0, 0.0]
    
    with open(file_path, 'r') as file:
        lines = file.readlines()
        
        #for grasp value
        values = re.findall(r"[-+]?\d*\.\d+", lines[2])
        grasp = [float(value) for value in values]              
        width = grasp[1]
        height = grasp[2]
        depth = grasp[3]
        
        #for translation
        match = re.search(r'translation:\[([^\]]+)\]', lines[2])
        values = match.group(1).strip()
        translation = list(map(float, values.split()))
        x = translation[0]
        y = translation[1]
        z = translation[2]
        
        #for rotation value
        matrix_lines = lines[4:7]
        rotation_matrix = []
        for line in matrix_lines:
            numbers = line.replace('[', '').replace(']', '').strip().split()
            rotation_matrix.extend([float(num) for num in numbers])
        
        # transfer to numpy
        x = np.array([x])
        y = np.array([y])
        z = np.array([z])
        rotation_matrix = np.array(rotation_matrix).reshape(3, 3)
        #print(rotation_matrix)
    return x, y, z, rotation_matrix



def Transfer_end2base():
        rospy.init_node('tf_listener', anonymous=True)
        listener = tf.TransformListener()

        try:
            listener.waitForTransform('/base_link', '/tool0_controller', rospy.Time(0), rospy.Duration(4.0))
            (trans, rot) = listener.lookupTransform('/base_link', '/tool0_controller', rospy.Time(0))

            translation_matrix = np.array([[1, 0, 0, trans[0]],
                                           [0, 1, 0, trans[1]],
                                           [0, 0, 1, trans[2]],
                                           [0, 0, 0, 1]])

            rotation_matrix = tf.transformations.quaternion_matrix(rot)[:3, :3]

            # 完整转换矩阵
            transform_matrix = np.eye(4)
            transform_matrix[:3, :3] = rotation_matrix
            transform_matrix[:3, 3] = trans
    
            return transform_matrix

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr('Could not get transform')
            return None



def transfer_coordinate(x, y, z, rotation_matrix, transfer_matrix):
    
    x = x.item()
    y = y.item()
    z = z.item()

    grasp_position = np.array([x, y, z, 1]).reshape(4, 1)
    
    rotation_matrix_homogeneous = np.eye(4)
    rotation_matrix_homogeneous[:3, :3] = rotation_matrix
    
    grasp_position = np.dot(transfer_matrix, grasp_position)
    grasp_rotation = np.dot(transfer_matrix, rotation_matrix_homogeneous)
    
    x = grasp_position[0]
    y = grasp_position[1]
    z = grasp_position[2]
    
    rotation_matrix = grasp_rotation[:3, :3]
    
    return x, y, z, rotation_matrix
    
    
#def transfer_endeffector_to_base(x, y, z, rotation_matrix):
    #return x, y, z, rotation_matrix
    
def transfer_matrix_to_orientation(x,y,z, rotation_matrix):
    
    orientation = []
    rotation_matrix = rotation_matrix.flatten()
    r11 = rotation_matrix[0]
    r12 = rotation_matrix[1]
    r13 = rotation_matrix[2]
    r21 = rotation_matrix[3]
    r22 = rotation_matrix[4]
    r23 = rotation_matrix[5]
    r31 = rotation_matrix[6]
    r32 = rotation_matrix[7]
    r33 = rotation_matrix[8]
    w = sqrt(1 + r11 + r22 + r33) / 2
    orientation.append(w)
    orientation.append((r32 - r23) / (4 * w))
    orientation.append((r13 - r31) / (4 * w))
    orientation.append((r21 - r12) / (4 * w))
    
    return x,y,z, orientation


if __name__ == "__main__":
    file_path = '/home/yin/graspnet-baseline/doc/data/data/grasps_chosen.txt'
    x,y,z, rotation_matrix = Coordinate_camera(file_path)

    x = x.item()
    y = y.item()
    z = z.item()
    matrix_obj_in_cam = np.eye(4)  
    matrix_obj_in_cam[:3, :3] = rotation_matrix  
    matrix_obj_in_cam[:3, 3] = [x, y, z]  
    
  ########################################################## hand-eye ##########################3  
    
    matrix1 = np.array([
    [-0.014752, 0.036623, -0.998544, -0.039548747386022085], 
    [-0.012248, 0.999218, 0.036952, -0.008769725051343654],
    [0.998548, 0.012792, -0.014282, -0.13833429620311815],
    [0, 0, 0, 1]
])
    matrix11 = np.array([
    [1, 0, 0, -0.039733521866192216], 
    [0, 1, 0, -0.009597649470711742],
    [0, 0, 1, -0.10410601045365417],
    [0, 0, 0, 1]
])
    transfer_matrix_cam_to_end  = np.array([
    [0, 0, -1, -0.053], 
    [0, 1, 0, -0.015],
    [1, 0, 0, -0.13],
    [0, 0, 0, 1]
])
    
    
    transfer_matrix_cam_to_end  = np.array([
    [0, 0, -1, -0.039733521866192216], 
    [0, 1, 0, -0.009597649470711742],
    [1, 0, 0, -0.13],
    [0, 0, 0, 1]
])
    
    
########################################  end TO base frame ###########################
    
    transfer_matrix_end_to_base = Transfer_end2base()


#########################  end to wrist3 #################################

    
    matrix_tool0_to_wrist3 = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0.233],
        [0, 0, 0, 1]
])
    
    
    matrix_cam_to_cam = np.array([
        [0, 0, 1, 0],
        [-1, 0, 0, 0],
        [0, -1, 0, 0],
        [0, 0, 0, 1]
])
    
    matrix4 = np.array([
        [0, 0, 1, 0],
        [0, 1, 0, 0],
        [-1, 0, 0, 0],
        [0, 0, 0, 1]
])
    
    
    matrix_obj_in_base = transfer_matrix_end_to_base @ transfer_matrix_cam_to_end @ matrix_cam_to_cam  @ matrix_obj_in_cam
    
    
    R_z_180 = np.array([
        [-1, 0, 0],
        [0, -1, 0],
        [0, 0, 1]
])
    
    
    #matrix_obj_in_base1 = matrix_obj_in_base @ matrix4
    matrix_obj_in_base1 = matrix_obj_in_base @ matrix4
    
    R1 = matrix_obj_in_base1[:3, :3]
    [x1, y1, z1] = R1 @ np.array([0, 0, -0.245]) + matrix_obj_in_base1[:3, 3]
    
    rotation_matrix1 = matrix_obj_in_base1[:3, :3]
    rotation_matrix2 = rotation_matrix1 @ R_z_180 
    x1,y1,z1, orientation1 = transfer_matrix_to_orientation(x1,y1,z1, rotation_matrix1)
    x1,y1,z1, orientation2 = transfer_matrix_to_orientation(x1,y1,z1, rotation_matrix2)
    
    coordinate_path1 = '/home/yin/graspnet-baseline/doc/result/coordinate1.txt'
    with open(coordinate_path1, 'w') as file:
         file.write(f"{x1}, {y1}, {z1}, {orientation1}\n")
    print(x1,y1,z1, orientation1)
         
    coordinate_path2 = '/home/yin/graspnet-baseline/doc/result/coordinate2.txt'
    with open(coordinate_path2, 'w') as file:
         file.write(f"{x1}, {y1}, {z1}, {orientation2}\n")
    print(x1,y1,z1, orientation2)