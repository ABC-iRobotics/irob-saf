#!/usr/bin/env python
import sys
import rospy
import time

import roslibpy
import rospy
from twisted.internet import reactor

from sensor_msgs.msg import JointState

from rospy_message_converter import message_converter




class ROSbridgeSubscriber():

  def __init__(self):
    ros = roslibpy.Ros(host='localhost', port=9090)


    rospy.init_node('rosbridge_subscriber', anonymous=True)

    joint_sub = rospy.Subscriber("/dvrk/PSM1/state_joint_current",JointState,self.joint_cb)


    topic_name = "/twin/dvrk/PSM1/state_joint_current"

    self.topic = roslibpy.Topic(ros, topic_name, JointState)
    self.topic.advertise()

    #ros.run_forever()


  def joint_cb(self, msg):
    #print(msg)
    dictionary =     message_converter.convert_ros_message_to_dictionary(msg)
    bridge_msg = roslibpy.Message(dictionary)
    self.topic.publish(bridge_msg)




def main(args):
  print("Node started")
  rbs = ROSbridgeSubscriber()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")


if __name__ == '__main__':
  main(sys.argv)
