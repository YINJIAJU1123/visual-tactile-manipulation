from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped
import rospy
import tf.transformations as transformations

rospy.init_node('add_box_to_scene')

scene = PlanningSceneInterface()


box1_pose = PoseStamped()
box1_pose.header.frame_id = "base_link"
box1_pose.pose.position.x = -0.2
box1_pose.pose.position.y = -0.05
box1_pose.pose.position.z = 0.5
quaternion1 = transformations.quaternion_from_euler(3.14, 3.14, -2.3)
box1_pose.pose.orientation.x = quaternion1[0]
box1_pose.pose.orientation.y = quaternion1[1]
box1_pose.pose.orientation.z = quaternion1[2]
box1_pose.pose.orientation.w = quaternion1[3]
scene.add_box("box1", box1_pose, size=(0.1, 0.8, 0.8))


box2_pose = PoseStamped()
box2_pose.header.frame_id = "base_link"
box2_pose.pose.position.x = 0.3
box2_pose.pose.position.y = 0.5
box2_pose.pose.position.z = 0.1
quaternion2 = transformations.quaternion_from_euler(3.14, -1.57, 3.14)
box2_pose.pose.orientation.x = quaternion2[0]
box2_pose.pose.orientation.y = quaternion2[1]
box2_pose.pose.orientation.z = quaternion2[2]
box2_pose.pose.orientation.w = quaternion2[3]  
scene.add_box("box2", box2_pose, size=(0.1, 0.5, 0.5))

rospy.sleep(2)


box3_pose = PoseStamped()
box3_pose.header.frame_id = "base_link"
box3_pose.pose.position.x = 0.3
box3_pose.pose.position.y = 0.15
box3_pose.pose.position.z = 0.5
quaternion3 = transformations.quaternion_from_euler(3.14, 3.14, -0.85)
box3_pose.pose.orientation.x = quaternion3[0]
box3_pose.pose.orientation.y = quaternion3[1]
box3_pose.pose.orientation.z = quaternion3[2]
box3_pose.pose.orientation.w = quaternion3[3]
scene.add_box("box3", box3_pose, size=(0.1, 0.5, 0.5))


box4_pose = PoseStamped()
box4_pose.header.frame_id = "base_link"
box4_pose.pose.position.x = 0.1
box4_pose.pose.position.y = 0.3
box4_pose.pose.position.z = 0.85
quaternion4 = transformations.quaternion_from_euler(3.14, -1.57, 3.14)
box4_pose.pose.orientation.x = quaternion4[0]
box4_pose.pose.orientation.y = quaternion4[1]
box4_pose.pose.orientation.z = quaternion4[2]
box4_pose.pose.orientation.w = quaternion4[3]  
scene.add_box("box4", box4_pose, size=(0.1, 0.5, 0.5))


rospy.sleep(2)
