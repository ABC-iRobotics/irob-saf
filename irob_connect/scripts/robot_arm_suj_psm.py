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
Rosbridge node for SUJ_PSM-s. It will not subscribe to the
ROS topics, instead publishes the default value (0.0) of
the joints through rosbridge.
"""
class RobotArmSUJPSM(RobotArmDVRK):

  # Mapping between ROS and rosbridge joint names.
  JOINT_DICT = {'J0': 'J0',
                'J1': 'J1',
                'J2': 'J2',
                'J3': 'J3',
                'J4': 'J4'}

  # Array of the rosbridge joint names in an order to be publsihed
  JOINT_NAMES_BRIDGE = ['J0','J1','J2','J3','J4']



  """Constructor.
  :param name: name of the arm
  :param ros: ros object
  """
  def __init__(self, name, ros):
    RobotArmDVRK.__init__(self, name, ros,
                          self.JOINT_DICT, self.JOINT_NAMES_BRIDGE)


  """Override of substription method. Since we do nit have
  the DVRK controllers for the ECM-s, it is not necessary
  to subscribe to the topics at all.
  """
  def subscribe_to_topics(self):
    print('Dummy ' + self.name + ', not subscribing.')







































