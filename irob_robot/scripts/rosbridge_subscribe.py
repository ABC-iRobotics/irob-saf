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

    self.ros = roslibpy.Ros(host='localhost', port=9090, is_secure=False)
    self.ros.on_ready(self.connect_cb)
    topic_name = "/twin/dvrk/PSM1/state_joint_current"
    self.topic = roslibpy.Topic(self.ros, topic_name, 'sensor_msgs/JointState')
    self.topic.advertise()

    self.ros.run_forever()


  def joint_cb(self, msg):
    #print(msg)
    dictionary = message_converter.convert_ros_message_to_dictionary(msg)
    bridge_msg = roslibpy.Message(dictionary)
    self.topic.publish(bridge_msg)
    print(msg)



  def connect_cb(self):
    print("Is connected?")
    print(self.ros.is_connected)
    joint_sub = rospy.Subscriber("/dvrk/PSM1/state_joint_current",JointState,self.joint_cb)



def main(args):
  print("Node started")
  rospy.init_node('rosbridge_subscriber', anonymous=True)
  rospy.Rate(10)
  rbs = ROSbridgeSubscriber()



if __name__ == '__main__':
  main(sys.argv)

