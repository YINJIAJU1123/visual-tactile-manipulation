#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
import numpy as np
from math import sqrt
import re
from Utils.robotiq_gripper import RobotiqGripper
import tf

import ast


from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA


class MoveItIkDemo:
    def __init__(self, x,y,z, orientation):
        
        #gripper control
        self.gripper = RobotiqGripper()
        IP = "10.10.10.1"
        self.gripper.connect(IP, 63352)
        
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_ik_demo')

        arm = moveit_commander.MoveGroupCommander('manipulator')

        end_effector_link = arm.get_end_effector_link()
                        
        reference_frame = 'base_link'


        arm.stop()

        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
        
        
if __name__ == "__main__":


    file_path = '/home/yin/graspnet-baseline/doc/result/coordinate1.txt'
    with open(file_path, 'r') as file:
        line = file.readline().strip()  
    numbers = re.findall(r"[-+]?\d*\.\d+|\d+", line)
    x, y, z = map(float, numbers[:3]) 
    orientation = list(map(float, numbers[3:]))


    MoveItIkDemo(x, y, z, orientation )

