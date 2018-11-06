#!/usr/bin/env python
import sys
import time

import roslibpy
import rospy
from twisted.internet import reactor

from sensor_msgs.msg import JointState
from cisst_msgs.msg import mtsIntervalStatistics

from rospy_message_converter import message_converter

from robot_arm_dvrk import RobotArmDVRK
from robot_arm_psm import RobotArmPSM
from robot_arm_ecm import RobotArmECM
from robot_arm_suj_psm import RobotArmSUJPSM
from robot_arm_suj_ecm import RobotArmSUJECM


class DVRKRosbridgeUpstream():

  def __init__(self, arm_names, namespace, host, port, drop_rate):
    self.namespace = namespace
    self.drop_rate = drop_rate
    self.cnt = 0

    self.ros = roslibpy.Ros(host=host, port=port, is_secure=False)
    self.ros.on_ready(self.connect_cb)

    self.arms = []

    for name in arm_names:
      if 'SUJ_PSM' in name:
        self.arms.append(RobotArmSUJPSM(name, self.ros))
      elif 'SUJ_ECM' in name:
        self.arms.append(RobotArmSUJECM(name, self.ros))
      elif 'ECM' in name:
        self.arms.append(RobotArmECM(name, self.ros))
      elif 'PSM' in name:
        self.arms.append(RobotArmPSM(name, self.ros))

    self.topic = roslibpy.Topic(self.ros, self.namespace + 'joint_states', 'sensor_msgs/JointState')
    self.topic.advertise()
    self.ros.run_forever()



  def connect_cb(self):
    print("Is connected?")
    print(self.ros.is_connected)
    rospy.Subscriber('/dvrk/spin/period_statistics', mtsIntervalStatistics, self.spin_cb)
    for a in self.arms:
      a.subscribe_to_topics()



  def spin_cb(self, msg):
    self.cnt = self.cnt + 1
    if self.cnt > self.drop_rate:
      self.cnt = 0
      print('Publishing joint states...')
      spin_dictionary = message_converter.convert_ros_message_to_dictionary(msg)

      dictionary_msg = { 'header': spin_dictionary['header'],
                     'name': [],
                     'position': []}

      for a in self.arms:
        data = a.get_joint_states()
        dictionary_msg['name'] = dictionary_msg['name'] + data['name']
        dictionary_msg['position'] = dictionary_msg['position'] + data['position']
      print('dictionary_msg')
      print(dictionary_msg)
      bridge_msg = roslibpy.Message(dictionary_msg)
      print('bridge_msg')
      print(bridge_msg)
      self.topic.publish(bridge_msg)
      print('Joint states published.')




def main(args):
  print("Upstream node started")
  rospy.init_node('rosbridge_subscriber', anonymous=True)
  ros_rate = rospy.get_param('~ros_rate')
  rospy.Rate(ros_rate)

  arm_names = rospy.get_param("~arm_names")
  host = rospy.get_param('~host')
  port = rospy.get_param('~port')
  drop_rate = rospy.get_param('~drop_rate')
  namespace = rospy.get_param('~namespace')

  rbs = DVRKRosbridgeUpstream(arm_names, namespace, host, port, drop_rate)



if __name__ == '__main__':
  main(sys.argv)

