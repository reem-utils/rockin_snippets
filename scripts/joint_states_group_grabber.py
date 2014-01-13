#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Jan 13 19:06:55 2014

@author: Sam Pfeiffer

Subscribe to joint states and get the joint positions for a group printed
"""
import sys

import rospy
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

class jointStateGrabber():
    """This class subscribes to the /joint_states topic of REEM
    and you can ask it for the current status of a group and get it 
    outputted in some useful format, i.e.: print on screen,
    play_motion format, old xml format... """

    def __init__(self):
        self.current_joint_states = None
        self.all_joints = ['torso_1_joint', 'torso_2_joint',
                           'head_1_joint', 'head_2_joint',
                           'arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint',
                           'arm_left_4_joint', 'arm_left_5_joint', 'arm_left_6_joint',
                           'arm_left_7_joint', 
                           'arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint',
                           'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint',
                           'arm_right_7_joint',
                           'hand_right_index_1_joint', 'hand_right_index_2_joint', 'hand_right_index_3_joint',
                           'hand_right_index_joint', 'hand_right_middle_1_joint', 'hand_right_middle_2_joint',
                           'hand_right_middle_3_joint', 'hand_right_middle_joint', 'hand_right_thumb_joint',
                           'hand_left_index_1_joint', 'hand_left_index_2_joint', 'hand_left_index_3_joint',
                           'hand_left_index_joint', 'hand_left_middle_1_joint', 'hand_left_middle_2_joint',
                           'hand_left_middle_3_joint', 'hand_left_middle_joint', 'hand_left_thumb_joint']
        self.left_arm = ['arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint',
                           'arm_left_4_joint', 'arm_left_5_joint', 'arm_left_6_joint',
                           'arm_left_7_joint']
        self.right_arm = ['arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint',
                           'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint',
                           'arm_right_7_joint']
        self.right_arm_torso = ['torso_1_joint', 'torso_2_joint',
                           'arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint',
                           'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint',
                           'arm_right_7_joint']
        self.left_arm_torso = ['torso_1_joint', 'torso_2_joint',
                           'arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint',
                           'arm_left_4_joint', 'arm_left_5_joint', 'arm_left_6_joint',
                           'arm_left_7_joint']
        self.head = ['head_1_joint', 'head_2_joint']
        self.torso = ['torso_1_joint', 'torso_2_joint']
        self.right_hand_all = ['hand_right_index_1_joint', 'hand_right_index_2_joint', 'hand_right_index_3_joint',
                               'hand_right_index_joint', 'hand_right_middle_1_joint', 'hand_right_middle_2_joint',
                               'hand_right_middle_3_joint', 'hand_right_middle_joint', 'hand_right_thumb_joint']
        self.right_hand = ['hand_right_index_joint', 'hand_right_middle_joint', 'hand_right_thumb_joint'] # Only the actuated
        self.left_hand_all = ['hand_left_index_1_joint', 'hand_left_index_2_joint', 'hand_left_index_3_joint',
                          'hand_left_index_joint', 'hand_left_middle_1_joint', 'hand_left_middle_2_joint',
                          'hand_left_middle_3_joint', 'hand_left_middle_joint', 'hand_left_thumb_joint']
        self.left_hand = ['hand_left_index_joint', 'hand_left_middle_joint', 'hand_left_thumb_joint'] # Only the actuated
        
        self.ids_list = []

        self.subs = rospy.Subscriber('/joint_states', JointState, self.getJointStates)
        
        # getting first message to correctly find joints
        while self.current_joint_states == None:
            rospy.sleep(0.1)
        rospy.loginfo("Node initialized. Ready to grab joint states")
        

    def getJointStates(self, data):
        #rospy.loginfo("Received from topic data!")
        self.current_joint_states = data
        
    def createGoalFromCurrentJointStateForArm(self, group='right_arm_torso'):
        """ Get the joints for the specified group and create a FollowJointTrajectoryGoal
        with these joints and values for the joints """
        names, values = self.getNamesAndMsgList(group=group)
    
        fjtg = FollowJointTrajectoryGoal()
        fjtg.trajectory.joint_names.extend(names)
        jtp = JointTrajectoryPoint(positions=values, velocities=len(values) * [0.0], time_from_start=0)
        fjtg.trajectory.points.append(jtp)
        
        rospy.loginfo("follow joint trajectory goal:\n" + str(fjtg))
        
        return fjtg
        
        
        
    def getNamesAndMsgList(self, group='right_arm_torso'):
        """ Get the joints for the specified group and return this name list and a list of it's values in joint_states
        Note: the names and values are correlated in position """
        list_to_iterate = getattr(self, group)        
        curr_j_s = self.current_joint_states
        ids_list = []
        msg_list = []
        rospy.logdebug("Current message: " + str(curr_j_s))
        for joint in list_to_iterate:
            idx_in_message = curr_j_s.name.index(joint)
            ids_list.append(idx_in_message)
            msg_list.append(curr_j_s.position[idx_in_message])
        rospy.logdebug("Current position of joints in message: " + str(ids_list))
        rospy.logdebug("Current msg:" + str(msg_list))
    
        return list_to_iterate, msg_list

    def printNamesAndValues(self, group='right_arm_torso'):
        names, values = self.getNamesAndMsgList(group=group)
        print "Name, Joint Value"
        print "================="
        for nam, val in zip(names, values):
            print nam + " " + str(val) 
        

def usage(program_name):
    print "Usage:"
    print program_name + "     (without arguments)"
    print "Interactive mode, write a group name and it will be printed"
    print program_name + " <group>"
    print "Will print the group joint names and its joint values"
    print "Available groups are: all_joints, "

if __name__ == '__main__':
    rospy.init_node('joint_state_grabber')
    if len(sys.argv) > 2:
        print "Error, too many arguments"
        usage(sys.argv[0])
        exit()
    elif

    node = jointStateGrabber()
    node.printNamesAndValues("right_arm_torso")
