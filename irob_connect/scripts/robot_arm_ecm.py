#!/usr/bin/env python
import sys
import time

import roslibpy
import rospy
from twisted.internet import reactor

from sensor_msgs.msg import JointState

from rospy_message_converter import message_converter

from robot_arm_dvrk import RobotArmDVRK


class RobotArmECM(RobotArmDVRK):

  JOINT_DICT =  {'outer_yaw': '_yaw_joint',
                 'outer_pitch': '_pitch_front_joint',
                 'insertion': '_main_insertion_joint',
                 'outer_roll': '_tool_joint'}

  JOINT_NAMES_BRIDGE = ['_yaw_joint','_pitch_front_joint',
                        '_pitch_bottom_joint','_pitch_end_joint',
                        '_main_insertion_joint', '_tool_joint',
                        '_pitch_top_joint','_pitch_back_joint']


  def __init__(self, name, ros):
    RobotArmDVRK.__init__(self, name, ros,
                          self.JOINT_DICT, self.JOINT_NAMES_BRIDGE)







































