#!/usr/bin/env python
import sys
import time

import roslibpy
import rospy
from twisted.internet import reactor

from sensor_msgs.msg import JointState

from rospy_message_converter import message_converter


class RobotArmDVRK:

  joint_dict = {}

  joint_names_bridge = []

  joint_states = []


  def __init__(self, name, namespace, ros, joint_dict, joint_names_bridge):
    self.name = name
    self.namespace = namespace
    self.joint_dict = joint_dict
    self.joint_names_bridge = joint_names_bridge
    self.joint_states = [0.0] * len(self.joint_names_bridge)

    self.joint_names_bridge_specific = []
    for joint_name in self.joint_names_bridge:
      self.joint_names_bridge_specific.append(self.name + '_' +  joint_name)


  def store_joint_states_from_msg(self, msg):
    for i in range(len(msg.name)):
     self.store_joint_state(self.joint_dict[msg.name[i]], msg.position[i])

  def store_joint_state(self, joint_name_bridge, position):
      j = self.joint_names_bridge.index(joint_name_bridge)
      self.joint_states[j] = position

  def get_joint_states(self):
    return {'position': self.joint_states, 'name': self.joint_names_bridge_specific}

  def get_joint_names(self):
    return self.joint_names_bridge_specific

  def subscribe_to_topics(self):
    rospy.Subscriber("/dvrk/" + self.name + "/state_joint_current", JointState, self.joint_cb)


  def joint_cb(self, msg):
    self.store_joint_states_from_msg(msg)


































