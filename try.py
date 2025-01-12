#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import tf.transformations as transformations

def move_coordinate_system(x, y, z, orientation, move_distance=0.1):
    quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
    rotation_matrix = transformations.quaternion_matrix(quaternion)[:3, :3]

    y_axis_in_base = rotation_matrix.dot(np.array([0, 1, 0])) 
    z_axis_in_base = rotation_matrix.dot(np.array([0, 0, 1]))  

    coord1_normal = np.cross(y_axis_in_base, z_axis_in_base)
    base_normal = np.array([0, 0, 1])  
    line_direction = np.cross(base_normal, coord1_normal)
    base_point = np.array([x, y, z])
    line_direction_norm = line_direction / np.linalg.norm(line_direction)
    new_position = base_point + line_direction_norm * move_distance

    return new_position

# 示例使用
class Orientation:
    def __init__(self, x, y, z, w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

# 示例坐标和方向
x1, y1, z1 = 0.2802879975893527, 0.5768808514936566, 0.42783560254298864
orientation1 = Orientation(0.8442043724486836, -0.528432506145496, 0.05199756015464158, 0.07333252505379802)  # 例如四元数为 (0, 0, 0, 1)

# 移动坐标系1
new_position = move_coordinate_system(x1, y1, z1, orientation1)
print("新位置:", new_position)


