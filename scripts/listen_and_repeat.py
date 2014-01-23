#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sun Jan 16 22:46:20 2014

@author: Sam Pfeiffer

Send goals to the TTS with what was recognized by the ASR
  
"""

import actionlib
import rospy


from pal_interaction_msgs.msg import SoundAction, SoundGoal
from pal_interaction_msgs.msg import asrresult


def createTTSGoal(text, lang_id='', wait_before_speaking=rospy.Duration(0.0)):
    """Creates a 
    @arg j1 float value for head_1_joint
    @returns FollowJointTrajectoryGoal with the specified goal"""
    sound_goal = SoundGoal()
    sound_goal.text=text
    sound_goal.lang_id=lang_id
    sound_goal.wait_before_speaking=wait_before_speaking
    return sound_goal

def callback_pose(data):
    """Callback for the topic subscriber.
       Prints the current received data on the topic."""
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
#     roll, pitch, yaw = euler_from_quaternion([data.pose.pose.orientation.x,
#                                              data.pose.pose.orientation.y,
#                                              data.pose.pose.orientation.z,
#                                              data.pose.pose.orientation.w])
#     rospy.loginfo("Current robot pose: x=" + str(x) + "y=" + str(y) + " yaw=" + str(degrees(yaw)) + "º")


if __name__ == '__main__':
    rospy.init_node('make_reem_say_what_it_heard')

    # Read the current asrresult
    #rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callback_pose)

    tts_as = actionlib.SimpleActionClient('/sound', SoundAction)
    rospy.loginfo("Connecting to TTS AS...")
    tts_as.wait_for_server(rospy.Duration(10))
    goal = createTTSGoal("Rockin camp is here!")
    rospy.loginfo("Connected, sending goal.")
    tts_as.send_goal(goal)
    rospy.loginfo("Goal sent, waiting...")
    tts_as.wait_for_result(rospy.Duration(10))


    
