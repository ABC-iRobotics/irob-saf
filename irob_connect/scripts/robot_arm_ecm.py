#!/usr/bin/env python
import sys
import time

import roslibpy
import rospy
from twisted.internet import reactor

from sensor_msgs.msg import JointState

from rospy_message_converter import message_converter

from robot_arm_dvrk import RobotArmDVRK


class RobotArmECM(RobotArmDVRK):

  def __init__(self, name, namespace, ros):
    RobotArmDVRK.__init__(self, name, namespace, ros,
            {'outer_yaw': '_yaw_joint',
            'outer_pitch': '_pitch_front_joint',
            'insertion': '_main_insertion_joint',
            'outer_roll': '_tool_joint'},

            ['_yaw_joint','_pitch_front_joint','_pitch_bottom_joint','_pitch_end_joint','_main_insertion_joint',
                '_tool_joint','_pitch_top_joint','_pitch_back_joint'])






































