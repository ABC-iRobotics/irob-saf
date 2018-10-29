#!/usr/bin/env python
import sys
import time

import roslibpy
import rospy
from twisted.internet import reactor

from sensor_msgs.msg import JointState

from rospy_message_converter import message_converter




class ROSbridgeSubscriber():

  def __init__(self):
    self.ros_remote = roslibpy.Ros(host='localhost', port=9090)
    self.ros_local = roslibpy.Ros(host='localhost', port=9090)
    self.ros_remote.on_ready(self.connect_cb)
    self.ros_local.on_ready(self.connect_cb)

    topic_name_remote = "/twin/dvrk/PSM1/state_joint_current"
    topic_name_local = "/dvrk/PSM1/state_joint_current"

    self.topic_remote = roslibpy.Topic(self.ros_remote, topic_name_remote, JointState)
    self.topic_remote.advertise()

    self.topic_local = roslibpy.Topic(self.ros_local, topic_name_local, JointState)
    self.topic_local.subscribe(self.joint_cb)

    self.pub_cnt = 0
    self.ros_local.run_forever()





  def joint_cb(self, msg):
    #print(msg)
    self.pub_cnt = self.pub_cnt + 1
    if self.pub_cnt > 100:
      self.topic_remote.publish(msg)
      self.pub_cnt = 0
      print(msg)



  def connect_cb(self):
    print("Is local connected?")
    print(self.ros_local.is_connected)
    print("Is remote connected?")
    print(self.ros_remote.is_connected)
    print(self.ros_local.get_topics(self.topics_cb))


  def topics_cb(self):
    print("Topics")
   #print(msg)




def main(args):
  print("Node started")
  rbs = ROSbridgeSubscriber()



if __name__ == '__main__':
  main(sys.argv)
