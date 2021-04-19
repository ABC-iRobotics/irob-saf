#!/usr/bin/env python
import sys
import time

import roslibpy
import rospy
from twisted.internet import reactor

from sensor_msgs.msg import JointState

from rospy_message_converter import message_converter

from robot_arm_dvrk import RobotArmDVRK
from robot_arm_psm import RobotArmPSM
from robot_arm_ecm import RobotArmECM
from robot_arm_suj_psm import RobotArmSUJPSM
from robot_arm_suj_ecm import RobotArmSUJECM


"""
Main class for upstreaming DVRK joint positions to the server.
"""
class DVRKRosbridgeUpstream():

  """Constructor.

  :param arm_names: an array containing the name of the arms
                    in an order to be publsihed
  :param upstream_topic: name of the topic published to rosbridge
  :param host: ip address of the host
  :param port: port of the websocket
  :param trigger_topic: local topic to trigger sending position
                        to server
  :param drop_rate: wait for this number of messages on the topic
                    /dvrk/spin/period_statistic before every publish
  """
  def __init__(self, arm_names, upstream_topic, host, port, trigger_topic, drop_rate):
    self.upstream_topic = upstream_topic
    self.trigger_topic = trigger_topic
    self.drop_rate = drop_rate
    self.cnt = 0

    # Creating rosbridge connection
    self.ros = roslibpy.Ros(host=host, port=port, is_secure=False)
    self.ros.on_ready(self.connect_cb)

    self.arms = []

    # Constructing RobotArmDVRK instance for each arm
    for name in arm_names:
      if 'SUJ_PSM' in name:
        self.arms.append(RobotArmSUJPSM(name, self.ros))
      elif 'SUJ_ECM' in name:
        self.arms.append(RobotArmSUJECM(name, self.ros))
      elif 'ECM' in name:
        self.arms.append(RobotArmECM(name, self.ros))
      elif 'PSM' in name:
        self.arms.append(RobotArmPSM(name, self.ros))

    # Advertising rosbridge topic
    self.topic = roslibpy.Topic(self.ros, self.upstream_topic, 'sensor_msgs/JointState')
    self.topic.advertise()

    # Start loop
    self.ros.run_forever()




  """Callback for connection ready. Subscribtion to the
  DVRK topics happens there, after the rosbridge connection
  is estabilished.
  """
  def connect_cb(self):
    print("Is connected?")
    print(self.ros.is_connected)

    # Subscribe to some topic that triggers the publishing
    # to rosbridg
    rospy.Subscriber(self.trigger_topic, JointState, self.spin_cb)
    for a in self.arms:
      a.subscribe_to_topics()




  """Callback for some message that triggers the publishing
  to rosbridge.
  """
  def spin_cb(self, msg):
    self.cnt = self.cnt + 1
    if self.cnt > self.drop_rate:
      self.cnt = 0
      spin_dictionary = message_converter.convert_ros_message_to_dictionary(msg)

      dictionary_msg = { 'header': spin_dictionary['header'],
                     'name': [],
                     'position': []}
      # Assemble joint state message
      for a in self.arms:
        data = a.get_joint_states()
        dictionary_msg['name'] = dictionary_msg['name'] + data['name']
        dictionary_msg['position'] = dictionary_msg['position'] + data['position']
      bridge_msg = roslibpy.Message(dictionary_msg)
      #print('bridge_msg')
      #print(bridge_msg)
      self.topic.publish(bridge_msg)
      print('Joint states published.')



"""
Main.
"""
def main(args):
  print("Upstream node started")
  # Start a ROS node
  rospy.init_node('dvrk_rosbridge_upstream', anonymous=True)
  ros_rate = rospy.get_param('~ros_rate')
  rospy.Rate(ros_rate)    # This line makes sure that ROS callbacks
                          # will be called eventually
  arm_names = rospy.get_param("~arm_names")
  host = rospy.get_param('~host')
  port = rospy.get_param('~port')
  drop_rate = rospy.get_param('~drop_rate')
  upstream_topic = rospy.get_param('~upstream_topic')
  trigger_topic = rospy.get_param('~trigger_topic')

  drvrk_rb_us = DVRKRosbridgeUpstream(arm_names, upstream_topic, host, port, trigger_topic, drop_rate)



if __name__ == '__main__':
  main(sys.argv)

