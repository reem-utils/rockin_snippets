#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Jan 11 16:32:00 2014

@author: Sam Pfeiffer

Snippet of code on how to send a moveit goal to an arm

Navigation actionserver: /move_group/goal
Type of message: moveit_msgs/MoveGroupGoal

"""

import rospy
import actionlib
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Point, Quaternion, Pose
from moveit_msgs.msg import MoveGroupGoal, MoveGroupResult, MoveGroupAction, Constraints, PositionConstraint, OrientationConstraint, MoveItErrorCodes
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header


# Useful dictionary for reading in a human friendly way the MoveIt! error codes
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name


def create_move_group_pose_goal(goal_pose=Pose(), group="right_arm_torso", end_link_name=None, plan_only=True):
    """ Creates a move_group goal based on pose.
    @arg group string representing the move_group group to use
    @arg end_link_name string representing the ending link to use
    @arg goal_pose Pose() representing the goal pose
    @arg plan_only bool to for only planning or planning and executing"""
    
    header = Header()
    header.frame_id = 'base_link'
    header.stamp = rospy.Time.now()
    moveit_goal = MoveGroupGoal()
    goal_c = Constraints()
    position_c = PositionConstraint()
    position_c.header = header
    if end_link_name != None:
        position_c.link_name = end_link_name
    position_c.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01])) # how big is the area where the end effector can be
    position_c.constraint_region.primitive_poses.append(goal_pose)
    position_c.weight = 1.0
    goal_c.position_constraints.append(position_c)
    orientation_c = OrientationConstraint()
    orientation_c.header = header
    if end_link_name != None:
        orientation_c.link_name = end_link_name
    orientation_c.orientation = goal_pose.orientation
    orientation_c.absolute_x_axis_tolerance = 0.01
    orientation_c.absolute_y_axis_tolerance = 0.01
    orientation_c.absolute_z_axis_tolerance = 0.01
    orientation_c.weight = 1.0
    goal_c.orientation_constraints.append(orientation_c)
    moveit_goal.request.goal_constraints.append(goal_c)
    moveit_goal.request.num_planning_attempts = 3
    moveit_goal.request.allowed_planning_time = 5.0
    moveit_goal.planning_options.plan_only = plan_only
    moveit_goal.planning_options.planning_scene_diff.is_diff = True
    moveit_goal.request.group_name = group
    
    return moveit_goal


if __name__=='__main__':
    rospy.init_node("moveit_snippet")

    rospy.loginfo("Connecting to move_group AS")
    moveit_ac = actionlib.SimpleActionClient('/move_group', MoveGroupAction)
    moveit_ac.wait_for_server()
    rospy.loginfo("Succesfully connected.")
    
    rospy.loginfo("Creating goal.")
    goal_point = Point(0.2, -0.2, 1.2) # right_arm arm_right_tool_link can definitely get here
    goal_pose = Pose()
    goal_pose.position = goal_point
    quat = quaternion_from_euler(0.0, 0.0, 0.0) # roll, pitch, yaw
    goal_pose.orientation = Quaternion(*quat.tolist())
    moveit_goal = create_move_group_pose_goal(goal_pose, group="right_arm", end_link_name="arm_right_tool_link", plan_only=True)
    rospy.loginfo("Sending goal...")
    moveit_ac.send_goal(moveit_goal)
    rospy.loginfo("Waiting for result...")
    moveit_ac.wait_for_result(rospy.Duration(5.0))
    moveit_result = moveit_ac.get_result()
    
    #rospy.loginfo("Got result:\n" + str(moveit_result)) # Uncomment if you want to see the full result message
    #r = MoveGroupResult()
    if moveit_result != None and moveit_result.error_code.val != 1:
        rospy.logwarn("Goal not succeeded: \"" + moveit_error_dict[moveit_result.error_code.val]  + "\"")
    elif moveit_result != None:
        rospy.loginfo("Goal achieved.")
    