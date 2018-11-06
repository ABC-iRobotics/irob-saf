#!/usr/bin/env python
import sys
import time

import roslibpy
import rospy
from twisted.internet import reactor

from sensor_msgs.msg import JointState

from rospy_message_converter import message_converter

"""
Parent class for all DVRK robot arm for publishing joint states to rosbridge.
"""
class RobotArmDVRK:

  # Dictionary that maps the dvrk joints to the rosbridge joints
  joint_dict = {}

  # Array of the rosbridge joint names in an order to be publsihed
  joint_names_bridge = []

  # The position of the joints in the same order as in
  # the joint_name_bridge
  joint_states = []


  """Parent class constructor.

  :param name: name of the arm, e.g. PSM1
  :param ros: roslibpy ros object
  :param joint_dict: dictionary that maps the dvrk joints
                     to the rosbridge joints
  :param joint_names_bridge: array of the rosbridge joint
                     names in an order to be publsihed
  """
  def __init__(self, name, ros, joint_dict, joint_names_bridge):
    self.name = name
    self.joint_dict = joint_dict
    self.joint_names_bridge = joint_names_bridge
    self.joint_states = [0.0] * len(self.joint_names_bridge)

    self.joint_names_bridge_specific = []
    for joint_name in self.joint_names_bridge:
      self.joint_names_bridge_specific.append(self.name
                                    + '_' +  joint_name)



  """Get the joint states from a message and store at the
  proper index in joint_states.

  :param msg: joint state message
  """
  def store_joint_states_from_msg(self, msg):
    for i in range(len(msg.name)):
     self.store_joint_state(self.joint_dict[msg.name[i]], msg.position[i])



  """Store the position of a joint at the proper index
  in joint_states based on the name of the joint.

  :param joint_name_bridge: the name of the current joint
              in the rosbridge msg
  :param position: joint position
  """
  def store_joint_state(self, joint_name_bridge, position):
      j = self.joint_names_bridge.index(joint_name_bridge)
      self.joint_states[j] = position


  """Get the joint states and the joint names in a dictionary
  of arrays.

  :returns: a dictionary containing rosbridge joint names as
  'name', and the joint positions as 'position' in proper
  order to be published
  """
  def get_joint_states(self):
    return {'position': self.joint_states, 'name': self.joint_names_bridge_specific}


  """Get rosbridge the joint names.

  :returns: joint names in proper order to be published
  """
  def get_joint_names(self):
    return self.joint_names_bridge_specific


  """Subscribe to the topic where the robot joint states
  are published.
  """
  def subscribe_to_topics(self):
    rospy.Subscriber("/dvrk/" + self.name + "/state_joint_current", JointState, self.joint_cb)

  """Callback for the joint states topic.
  """
  def joint_cb(self, msg):
    self.store_joint_states_from_msg(msg)


































