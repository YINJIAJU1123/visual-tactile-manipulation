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

from std_msgs.msg import Float64
import tf.transformations as transformations


class MoveItIkDemo:
    def __init__(self,x1,y1,z1,orientation1,x2,y2,z2,orientation2,max_torque=1300,max_angle=10,object_length=0.35,w_theta=0.6,w_T=0.4):
        
        #gripper control
        self.gripper = RobotiqGripper()
        IP = "10.10.10.1"
        self.gripper.connect(IP, 63352)
        #self.gripper._set_var("GTO", 1)
        #self.gripper.activate(auto_calibrate=False)
        
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_ik_demo')
        self.arm = moveit_commander.MoveGroupCommander('manipulator')
        self.end_effector_link = self.arm.get_end_effector_link()  
        print(self.end_effector_link)     
        
        self.reference_frame = 'base_link' 
        
        ###########################
        self.marker_pub = rospy.Publisher('/target_marker', Marker, queue_size=10)
        rospy.sleep(1)  
        self.publish_target_marker(x1, y1, z1, orientation1)
        #########################

        self.arm.set_pose_reference_frame(self.reference_frame)
        self.arm.allow_replanning(True)
        self.arm.set_goal_position_tolerance(0.001)
        self.arm.set_goal_orientation_tolerance(0.05)

        self.arm.set_max_acceleration_scaling_factor(0.1)
        self.arm.set_max_velocity_scaling_factor(0.1)

        self.arm.set_named_target('home')
        self.arm.go()
        rospy.sleep(3)
          
        self.max_angle = max_angle
        self.max_torque = max_torque
        self.w_theta = w_theta
        self.w_T = w_T
        self.object_length = object_length
        
        self.max_gs1_torque_value = -float('inf')
        self.max_gs1_angle_value = -float('inf')
    
        rospy.Subscriber("/gs1_torque", Float64, self.gs1_torque_callback)
        rospy.Subscriber("/gs1_angle", Float64, self.gs1_angle_callback)
        #rospy.Subscriber("/gs2_z", Float64, self.gs2_torque_callback)
        
        self.target_pose1, orientation1 = self.decide_optimal_rotation(x1,y1,z1, orientation1, x2, y2, z2, orientation2)

        self.target_pose3 = self.define_target_pose(x1,y1,z1+0.07,orientation1)
        #self.target_pose4 = self.define_target_pose(x1-0.02, y1+0.06, z1+0.15, orientation1)
        
        self.arm_move(self.target_pose1)
        
        rospy.sleep(0.5)
        
        self.gripper.move(155, 10, 10)
        #self.gripper.move(0, 10, 10)
        
        rospy.sleep(0.5)
        
        self.arm_move(self.target_pose3)
        
        print(self.max_gs1_angle_value)
        print(self.max_gs1_torque_value)
        
        self.move_distance = self.distance()
        print(self.move_distance)
        
        new_position = self.adjust_position(self.target_pose1, self.move_distance)

        new_position_x = new_position[0]
        new_position_y = new_position[1]
        new_position_z = z1  
        
        self.target_pose2 = self.define_target_pose(new_position_x,new_position_y,new_position_z,orientation1)
        self.target_pose4 = self.define_target_pose(new_position_x,new_position_y,new_position_z + 0.07,orientation1)
        
        if self.move_distance > 0.02:
            self.arm_move(self.target_pose1)
            rospy.sleep(0.5)
            self.gripper.move(0, 10, 10)
            rospy.sleep(0.5)
            self.arm_move(self.target_pose2)
            rospy.sleep(0.5)
            self.gripper.move(160, 10, 10)
            rospy.sleep(0.5)
            self.arm_move(self.target_pose4)
    
        #self.arm_move(self.target_pose3)
        
        rospy.sleep(2)

        #self.check_grasp_conditions()  
        
        
    def distance(self):
        
        theta_normalized = self.max_gs1_angle_value  / self.max_angle
        T_normalized = self.max_gs1_torque_value  / self.max_torque
        
        distance = (self.object_length / 2) * (self.w_theta * theta_normalized + self.w_T * T_normalized)
        return distance
        
        
    def adjust_position(self, target_pose1, move_distance = 0.1):
        x = target_pose1.pose.position.x
        y = target_pose1.pose.position.y
        z = target_pose1.pose.position.z
        
        orientation = target_pose1.pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        rotation_matrix = transformations.quaternion_matrix(quaternion)[:3, :3]

        y_axis_in_base = rotation_matrix.dot(np.array([1, 0, 0])) 
        z_axis_in_base = rotation_matrix.dot(np.array([0, 0, 1]))  

        coord1_normal = np.cross(y_axis_in_base, z_axis_in_base)
        base_normal = np.array([0, 0, 1])  
        line_direction = np.cross(base_normal, coord1_normal)
        base_point = np.array([x, y, z])
        line_direction_norm = line_direction / np.linalg.norm(line_direction)
        new_position = base_point + line_direction_norm * move_distance

        return new_position
        
        
    def define_target_pose(self, x,y,z,orientation):
        
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = self.reference_frame
        self.target_pose.header.stamp = rospy.Time.now() 
        self.target_pose.pose.position.x = x
        self.target_pose.pose.position.y = y
        self.target_pose.pose.position.z = z
        self.target_pose.pose.orientation.x = orientation[1]
        self.target_pose.pose.orientation.y = orientation[2]
        self.target_pose.pose.orientation.z = orientation[3]
        self.target_pose.pose.orientation.w = orientation[0]
        
        return self.target_pose
        
        
    def decide_optimal_rotation(self, x1,y1,z1, orientation1, x2, y2, z2, orientation2):
        
        self.target_pose1 = self.define_target_pose(x1,y1,z1,orientation1)
        
        self.target_pose11 = self.define_target_pose(x2,y2,z2,orientation2)
        
        self.arm.set_start_state_to_current_state()
        
        
        def compute_path(target_pose):
            if hasattr(target_pose, 'pose'):
                target_pose = target_pose.pose
            
            waypoints = [target_pose]
            fraction = 0.0   
            maxtries = 50  
            attempts = 0     
            self.arm.set_start_state_to_current_state()
            
            value = 0
            while attempts < maxtries:
                (plan, fraction) = self.arm.compute_cartesian_path(
                                        waypoints,  
                                        0.02,        
                                        0.0,        
                                        True)
                #print(fraction)
                attempts += 1
                
                if fraction == 1 :
                   value += 1
                if attempts % 10 == 0:
                    rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
            return plan, fraction, value
        
        plan1, fraction1, value1 = compute_path(self.target_pose1)
        print(value1)
        plan11, fraction11, value11 = compute_path(self.target_pose11)
        print(value11)
        
        if value1 > value11:
            best_target = self.target_pose1
            best_orientation = orientation1
            
            rospy.loginfo("Chose target_pose1 with success fraction: " + str(fraction1))
        else:
            best_target = self.target_pose11
            best_orientation = orientation2
            rospy.loginfo("Chose target_pose11 with success fraction: " + str(fraction11))

            return best_target, best_orientation
        
        return best_target, best_orientation
        
        
        
    def arm_move(self, target_pose1):
        
        self.arm.set_start_state_to_current_state()
        current_pose = self.arm.get_current_pose(self.end_effector_link).pose

        if hasattr(target_pose1, 'pose'):
            
            target_pose1 = target_pose1.pose
            
        #waypoints = [current_pose, target_pose1]
        waypoints = [target_pose1]
        
        fraction = 0.0   
        maxtries = 100   
        attempts = 0     
        self.arm.set_start_state_to_current_state()
        while fraction < 0.87 and attempts < maxtries:
            (plan, fraction) = self.arm.compute_cartesian_path (
                                    waypoints,  
                                    0.02,        
                                    0.0,        
                                    True)      
            print(fraction)  
            attempts += 1
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")

        if fraction > 0.87:
                rospy.loginfo("Path computed successfully. Moving the arm.")
                rospy.loginfo("Displaying trajectory points and timestamps:")
                #for i, point in enumerate(plan.joint_trajectory.points):
                  #rospy.loginfo(f"Point {i}:")
                  #rospy.loginfo(f"Positions: {point.positions}")
                  #rospy.loginfo(f"Time from start: {point.time_from_start.to_sec()} seconds")
        
                
                self.arm.execute(plan)
                rospy.sleep(6)
                rospy.loginfo("Path execution complete.")

        else:
                rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  


             
    def publish_target_marker(self, x, y, z, orientation):
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = rospy.Time.now()
        marker.ns = "target_marker"
        marker.id = 0
        marker.type = Marker.ARROW  
        marker.action = Marker.ADD
        marker.pose.position.x = x1
        marker.pose.position.y = y1
        marker.pose.position.z = z1 
        marker.pose.orientation.w = orientation1[0]
        marker.pose.orientation.x = orientation1[1]
        marker.pose.orientation.y = orientation1[2]
        marker.pose.orientation.z = orientation1[3]
        marker.scale.x = 0.05  
        marker.scale.y = 0.1   
        marker.scale.z = 0.1   
        marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)  
        marker.lifetime = rospy.Duration()  

        self.marker_pub.publish(marker)
        
    def gs1_torque_callback(self, data):
        current_torque = abs(data.data)
        if current_torque > self.max_gs1_torque_value:
            self.max_gs1_torque_value = current_torque
    
    def gs1_angle_callback(self, data):
        current_angle = abs(data.data)
        if current_angle > self.max_gs1_angle_value:
            self.max_gs1_angle_value = current_angle

    
    
    
    def gs2_torque_callback(self, data):
        self.gs2_z_value = data.data
        self.check_grasp_conditions()
    
        
    
        
    def check_grasp_conditions(self):
        if abs(self.gs1_z_value) > 500 or abs(self.gs2_z_value) > 500:
            rospy.loginfo("big rotation detected, looking for the next grasping position.")
            
            rospy.sleep(2)
            
            
            self.arm_move(self.target_pose1)
        
            
            rospy.sleep(3)
            
            self.gripper.move(0, 10, 0)
            
            rospy.sleep(1)
            
            rospy.sleep(1)
            
            self.arm_move(self.target_pose2)

            rospy.sleep(3)
            
            
            self.gripper.move(190, 10, 10)
            
            rospy.sleep(3)
            
            
            self.arm_move(self.target_pose4)
            
            
            rospy.sleep(1)
        
            self.arm.set_named_target('up')
            self.arm.go()
            
            moveit_commander.roscpp_shutdown()
            moveit_commander.os._exit(0)

        
if __name__ == "__main__":

    file_path1 = '/home/yin/graspnet-baseline/doc/result/coordinate1.txt'
    with open(file_path1, 'r') as file:
        line = file.readline().strip()  
    numbers = re.findall(r"[-+]?\d*\.\d+|\d+", line)
    x1, y1, z1 = map(float, numbers[:3]) 
    orientation1 = list(map(float, numbers[3:]))
    print(x1,y1,z1, orientation1)
    
    file_path2 = '/home/yin/graspnet-baseline/doc/result/coordinate2.txt'
    with open(file_path2, 'r') as file:
        line = file.readline().strip()  
    numbers = re.findall(r"[-+]?\d*\.\d+|\d+", line)
    x2, y2, z2 = map(float, numbers[:3]) 
    orientation2 = list(map(float, numbers[3:]))
    print(x2,y2,z2, orientation2)


    MoveItIkDemo(x1, y1, z1, orientation1, x2, y2, z2, orientation2)