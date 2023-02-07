#!/usr/bin/python3

import rospy

from geometry_msgs.msg import Transform, TransformStamped

import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib import colors
import rosbag

class PlotDvrkTcp:
    def __init__(self):
        """Constructor."""
        print("Node started")
        rospy.init_node('plot_dvrk_tcp', anonymous=True)

        self.points = np.zeros((3,0))

        self.pose_sub = rospy.Subscriber("/PSM1/measured_cp", TransformStamped, self.cb_pose)
        print("Init")

        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(projection='3d')

        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')


    def cb_pose(self, msg):
        self.points = np.append(self.points, [[msg.transform.translation.x],
                                              [msg.transform.translation.y],
                                              [msg.transform.translation.z]],
                                                axis=1)
        #print(self.points)
        #print("")


    def loop(self):
        rate = rospy.Rate(10)
        while(not rospy.is_shutdown()):
            self.ax.clear()
            self.ax.scatter(self.points[0,:],
                        self.points[1,:],
                        self.points[2,:], marker='o', s=3)
            #self.ax.scatter(points_transformed[0,:], points_transformed[1,:],
            #                points_transformed[2,:], marker='^')

            self.ax.set_xlabel('X')
            self.ax.set_ylabel('Y')
            self.ax.set_zlabel('Z')
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

if __name__ == '__main__':
    plotter = PlotDvrkTcp()
    plotter.loop()

