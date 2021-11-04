import time
from os import system, name
from datetime import datetime
import signal
import sys
import socket
import threading
import math
import numpy as np
import pyrealsense2 as rs
import logging
import rospy



class RecordRealsenseBag:


    def __init__(self, bagfile):

        print("Init")

        self.width = 640
        self.height = 480
        self.fps = 30 #3
        #self.exposure = 1500.0
        self.exposure = 150.0 #1000.0
        self.bagfile = bagfile

        # Init RealSense
        self.context = rs.context()
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.config.enable_stream(rs.stream.depth, self.width,
                                    self.height, rs.format.z16, self.fps)
        self.config.enable_stream(rs.stream.color, self.width,
                                    self.height, rs.format.bgr8, self.fps)


        rospy.init_node('record_realsense_bag', anonymous=True)



    # realsense
    def start_and_record_stream(self):


        # Start streaming
        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        align_to = rs.stream.color
        align = rs.align(align_to)

        self.config.enable_record_to_file(self.bagfile)
        self.profile = self.pipeline.start(self.config)
        self.set_exposure(self.exposure)
        print("Recording started...")
        try:
            rospy.spin()

        except KeyboardInterrupt:
            print("Stop recording.")
        finally:
            self.pipeline.stop()
            print("Pipeline closed.")



    #
    def set_exposure(self, exposure):
        rgb_cam_sensor = self.pipeline.get_active_profile().get_device().query_sensors()[1]
        rgb_cam_sensor.set_option(rs.option.exposure, exposure)



if __name__ == '__main__':
    print("Node started")

    rec = RecordRealsenseBag("/home/tamas/data/pegtransfer/pegboard4.bag")
    rec.start_and_record_stream()


