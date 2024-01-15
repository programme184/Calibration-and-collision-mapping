#!/usr/bin/env python
import rospy, sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from copy import deepcopy
import math
import time
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseStamped
import shape_msgs.msg

class ManiControl:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('listener',anonymous=True)
    reference_frame = "dummy"
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    move_group = moveit_commander.MoveGroupCommander("manipulator")

    # display_trajectory_publisher = rospy.Publisher(
#                                       '/move_group/display_planned_path',
#                                     moveit_msgs.msg.DisplayTrajectory)
    end_effector_link = move_group.get_end_effector_link()
    
    move_group.set_pose_reference_frame(reference_frame)
        
    # allow replanning after failed
    move_group.allow_replanning(True)

    # 设置位置（单位：米）和姿态（单位：弧度）的允许误差
    move_group.set_goal_position_tolerance(0.001)
    move_group.set_goal_orientation_tolerance(0.01)

    # 设置允许的最大速度和加速度
    move_group.set_max_acceleration_scaling_factor(0.5)
    move_group.set_max_velocity_scaling_factor(0.2)
    
    
    
    def info_get(self):
        # print("============ Printing robot state")
        # state_all = robot.get_current_state()
        # print(state_all)
        # print("============")
        # robot position
        end_effector_link = self.move_group.get_end_effector_link()
        current_pose = self.move_group.get_current_pose(end_effector_link).pose
        px, py, pz = current_pose.position.x, current_pose.position.y, current_pose.position.z
        ox, oy, oz, ow = current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w
        position = [px, py, pz]
        orientation = [ox, oy, oz, ow]
        # print("position:", position)
        # print("Orientation:", orientation)
        
        return position, orientation
        
    def back_home(self):
        self.move_group.set_named_target("home")
        success = self.move_group.plan()
        if success:
            self.move_group.go(wait=True)
            time.sleep(3)
        else:
            rospy.logerr("Move fail")
    
    
    def pos_assume(self, p, o=None):
        # planning to a pose goal
        pose_target = geometry_msgs.msg.Pose()
        if o is not None:
            
            pose_target.orientation.x = o[0]
            pose_target.orientation.y = o[1]
            pose_target.orientation.z = o[2]
            pose_target.orientation.w = o[3]
        pose_target.position.x = p[0]
        pose_target.position.y = p[1]
        pose_target.position.z = p[2]
        self.move_group.set_pose_target(pose_target)
        plan1 = self.move_group.plan()
        time.sleep(2)
        if plan1:
            self.move_group.go(wait=True)
            # rospy.loginfo("Move success")
            time.sleep(3)
        else:
            rospy.logerr("Move fail")
    

    def create_object(self, obj_size, obj_pose, name):
        # We can get the name of the reference frame for this robot:
        frame_id = self.move_group.get_planning_frame()
        collision_object = moveit_msgs.msg.CollisionObject()
        collision_object.header.frame_id = frame_id
        # name = "box1"
        collision_object.id = name
        primitive = shape_msgs.msg.SolidPrimitive()
        # Define the size of the box in meters
        primitive.type = primitive.BOX
        # table_size = [0.1, 0.1, 0.01]
        primitive.dimensions = obj_size
        
        posiziton = obj_pose['position']
        orien = obj_pose['orientation']
        box_pose = geometry_msgs.msg.Pose()
        pos = box_pose.position
        ori = box_pose.orientation
        pos.x, pos.y, pos.z  = posiziton
        ori.x, ori.y, ori.z, ori.w = orien

        
        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(box_pose)
        collision_object.operation = collision_object.ADD 

        self.scene.add_object(collision_object)
    # def 
        rospy.sleep(2)  

        # Set the start state to the current state of the robot
        self.move_group.set_start_state(self.robot.get_current_state())

# print("============ Starting tutorial setup")
# robot_control = ManiControl()
# # # # # robot_control.back_home()
# position, orientation = robot_control.info_get()
# print('position:', position, '\norientation:', orientation)


# p_front = [-0.08115254138234, -0.6977417850855295, 0.5424513589271386]#[-0.00012388227045280475, -0.7117525244097773, 0.5372345966719493]
# o_front = [0.9846803528520798, 0.06827924866750057, -0.062458337588609754, 0.147788710577543]#[0.9764532946278757, 0.10590385755422099, -0.13184847352664314, 0.13393773327257946]  # xyzw
    
# p_back = [-0.26334396408811883, -0.31816837232143014, 0.5439693594084783]#[-0.031532668333874976, -0.5041097953656779, 0.6133438658758624]
# o_back = [-0.9543479651433987, 0.18297553888233284, -0.11195362619907073, 0.20786124982365767]#[-0.9967489144080584, -0.051729046796425246, -0.008115452927891325, 0.0612359924200282]
    
    
# # robot_control.pos_assume(p_front, o_front)
# # time.sleep(15)
# robot_control.pos_assume(p_back, o_back)
# time.sleep(10)
        


