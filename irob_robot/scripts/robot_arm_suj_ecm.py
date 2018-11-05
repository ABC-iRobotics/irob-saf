#!/usr/bin/env python
import sys
import time

import roslibpy
import rospy
from twisted.internet import reactor

from sensor_msgs.msg import JointState

from rospy_message_converter import message_converter

import robot_arm_dvrk


class RobotArmSUJECM(RobotArmDvrk):

  def __init__(self, name, namespace, ros):
    RobotArmDVRK.__init__(self, name, namespace, ros)
    self.joint_dict = {
        'outer_yaw': 'J0',
        'outer_pitch': 'J1',
        'outer_insertion': 'J2',
        'outer_roll': 'J3'}

    self.joint_names_bridge=['J0','J1','J2','J3']


  def subscribe_to_topics(self):
    print('Dummy ' + self.name + ', not subscribing.')







































