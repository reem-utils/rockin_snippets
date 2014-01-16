#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sun Jan 16 22:46:20 2014

@author: Sam Pfeiffer

Send goals to the TTS (Text To Speech) server of REEM so he says something

Usage:
tts.py "This is my sentence"

Also:

rosrun actionlib axclient.py /sound

text: 'This is my sentence'
lang_id: ''
wait_before_speaking: 
  secs: 0
  nsecs: 0
  
  
"""

import actionlib
import rospy

from text_to_speech.msg import Sound, SoundGoal


def createTTSGoal(text, lang_id='', wait_before_speaking=rospy.Duration(0.0)):
    """Creates a 
    @arg j1 float value for head_1_joint
    @returns FollowJointTrajectoryGoal with the specified goal"""
    sound_goal = SoundGoal()
    sound_goal.text=text
    sound_goal.lang_id=lang_id
    sound_goal.wait_before_speaking=wait_before_speaking
    return sound_goal

if __name__ == '__main__':
    rospy.init_node('make_reem_say')

    tts_as = actionlib.SimpleActionClient('/sound', Sound)
    rospy.loginfo("Connecting to TTS AS...")
    tts_as.wait_for_server(rospy.Duration(10))
    goal = createTTSGoal("Rockin camp is here!")
    rospy.loginfo("Connected, sending goal.")
    tts_as.send_goal(goal)
    rospy.loginfo("Goal sent, waiting...")
    tts_as.wait_for_result(rospy.Duration(10))


    
