#!/usr/bin/env python
import sys
import time

import roslibpy
import rospy
from twisted.internet import reactor

from sensor_msgs.msg import JointState

from rospy_message_converter import message_converter




class DVRKRosbridgeUpstream():

  def __init__(self, arm, namespace, host, port, drop_rate):
    self.arm = arm
    self.namespace = namespace
    self.drop_rate = drop_rate
    self.cnt = 0

    self.ros = roslibpy.Ros(host=host, port=port, is_secure=False)
    self.ros.on_ready(self.connect_cb)
    self.topic = roslibpy.Topic(self.ros, self.namespace + "/" + self.arm + '/state_joint_current', 'sensor_msgs/JointState')
    self.topic.advertise()
    self.ros.run_forever()




  def connect_cb(self):
    print("Is connected?")
    print(self.ros.is_connected)
    rospy.Subscriber("/dvrk/" + self.arm + "/state_joint_current", JointState, self.joint_cb)



  def joint_cb(self, msg):
    self.cnt = self.cnt + 1
    if self.cnt > self.drop_rate:
      self.cnt = 0
      dictionary = message_converter.convert_ros_message_to_dictionary(msg)
      bridge_msg = roslibpy.Message(dictionary)
      self.topic.publish(bridge_msg)
      #print(msg)




def main(args):
  print("Node started")
  rospy.init_node('rosbridge_subscriber', anonymous=True)
  ros_rate = rospy.get_param('~ros_rate')
  rospy.Rate(ros_rate)

  host = rospy.get_param('~host')
  port = rospy.get_param('~port')
  drop_rate = rospy.get_param('~drop_rate')
  namespace = rospy.get_param('~namespace')
  arm = rospy.get_param('~arm')

  rbs = DVRKRosbridgeUpstream(arm, namespace, host, port, drop_rate)



if __name__ == '__main__':
  main(sys.argv)

