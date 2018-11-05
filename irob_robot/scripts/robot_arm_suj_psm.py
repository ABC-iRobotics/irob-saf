#!/usr/bin/env python
import sys
import time

import roslibpy
import rospy
from twisted.internet import reactor

from sensor_msgs.msg import JointState

from rospy_message_converter import message_converter

import robot_arm_dvrk


class RobotArmSUJPSM(RobotArmDvrk):

  def __init__(self, name, namespace, ros):
    RobotArmDVRK.__init__(self, name, namespace, ros)
    self.joint_dict = {
        'J0': 'J0',
        'J1': 'J1',
        'J2': 'J2',
        'J3': 'J3',
        'J4': 'J4'}

    self.joint_names_bridge=['J0','J1','J2','J3','J4']


  def subscribe_to_topics(self):
    print('Dummy ' + self.name + ', not subscribing.')







































