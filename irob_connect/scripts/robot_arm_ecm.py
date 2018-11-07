#!/usr/bin/env python
import sys
import time

import roslibpy
import rospy
from twisted.internet import reactor

from sensor_msgs.msg import JointState

from rospy_message_converter import message_converter

from robot_arm_dvrk import RobotArmDVRK


"""
Rosbridge node for ECM-s.
"""
class RobotArmECM(RobotArmDVRK):

  # Mapping between ROS and rosbridge joint names.
  JOINT_DICT =  {'outer_yaw': '_yaw_joint',
                 'outer_pitch': '_pitch_front_joint',
                 'insertion': '_main_insertion_joint',
                 'outer_roll': '_tool_joint'}

  # Array of the rosbridge joint names in an order to be publsihed
  JOINT_NAMES_BRIDGE = ['_yaw_joint','_pitch_front_joint',
                        '_pitch_bottom_joint','_pitch_end_joint',
                        '_main_insertion_joint', '_tool_joint',
                        '_pitch_top_joint','_pitch_back_joint']




  """Constructor.
  :param name: name of the arm
  :param ros: ros object
  """
  def __init__(self, name, ros):
    RobotArmDVRK.__init__(self, name, ros,
                          self.JOINT_DICT, self.JOINT_NAMES_BRIDGE)







































