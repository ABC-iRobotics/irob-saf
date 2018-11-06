#!/usr/bin/env python
import sys
import time

import roslibpy
import rospy
from twisted.internet import reactor

from sensor_msgs.msg import JointState

from rospy_message_converter import message_converter

from robot_arm_dvrk import RobotArmDVRK


class RobotArmPSM(RobotArmDVRK):

  JOINT_DICT = {'outer_yaw': 'yaw_joint',
                'outer_pitch': 'pitch_back_joint',
                'outer_insertion': 'main_insertion_joint',
                'outer_roll': 'tool_roll_joint',
                'outer_wrist_pitch': 'tool_pitch_joint',
                'outer_wrist_yaw': 'tool_yaw_joint',
                'jaw': 'tool_gripper2_joint'}

  JOINT_NAMES_BRIDGE = ['rev_joint','yaw_joint','pitch_back_joint',
                        'pitch_bottom_joint','pitch_end_joint',
                        'main_insertion_joint', 'tool_roll_joint',
                        'tool_pitch_joint','tool_yaw_joint',
                        'tool_gripper1_joint', 'tool_gripper2_joint',
                        'pitch_top_joint','pitch_front_joint']

  def __init__(self, name, ros):
    RobotArmDVRK.__init__(self, name, ros,
                          self.JOINT_DICT, self.JOINT_NAMES_BRIDGE)



  def subscribe_to_topics(self):
    RobotArmDVRK.subscribe_to_topics(self)
    rospy.Subscriber("/dvrk/" + self.name + "/state_jaw_current", JointState, self.jaw_cb)



  def jaw_cb(self, msg):
    self.store_joint_states_from_msg(msg)







































