#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Using the DMP library at https://github.com/mginesi/dmp_vol_obst

import numpy as np
from dmp.dmp_cartesian import DMPs_cartesian as dmp_cart
import matplotlib.pyplot as plt
from matplotlib import rc

import roslib
import rospy
from geometry_msgs.msg import Pose
from irob_msgs.srv import DmpGenTrajectory, DmpGenTrajectoryResponse
from irob_msgs.msg import TrajectoryToolPose
from irob_msgs.msg import ToolPose


rc('font',**{'family':'sans-serif','sans-serif':['Helvetica'],
    'size':'14'})
rc('text', usetex=True)


class dmp_service:


    def __init__(self):
        self.approach_des = np.genfromtxt('../data/approach_1.dat',
                                            delimiter=' ', skip_header = 1)
        #print(approach_des)

        self.t_span = np.transpose(self.approach_des)[0]
        self.gamma = self.approach_des[:,1:4]
        #print(t_span)

        self.t_span = np.linspace(0.0, 4.0, self.t_span.size)

        #############



        #############


        #gamma = np.transpose(np.array([t_span, np.sin(t_span) ** 2.0]))

        # ROS service
        rospy.init_node('dmp_server')
        s = rospy.Service('dmp_gen_trajectory',
                DmpGenTrajectory, self.handle_dmp_gen_trajectory)
        print("Ready to generate trajectories.")
        rospy.spin()





    def handle_dmp_gen_trajectory(req):

        if (req.action == req.GRASP):
            print("Generating GRASP trajectory...")
            x_goal = np.array([
                req.target.position.x, req.target.position.y, req.target.position.z])
            x_0 = np.array([
                req.start.position.x, req.start.position.y, req.start.position.z])


            x_track_pos_ori_jaw = self.calc_dmp(x_0, x_goal)
            resp = dmp_service.nparray2rostrajectory(x_track_pos_ori_jaw)

            return resp





    def calc_dmp(self, x_0, x_goal):
        # DMP
        mp = dmp_cart(n_dmps=3, tol=0.00005)
        mp.imitate_path(x_des=self.gamma, t_des=self.t_span)
        mp.x_goal += np.array([0.0, 0.0, 0.0])
        #mp.x_goal += np.array([-0.005, 0.008, 0.0])

        x_track = mp.rollout()[0]

        plt.figure()
        plt.plot(self.gamma[:, 0], self.gamma[:, 1], '--b')
        plt.plot(x_track[:, 0], x_track[:, 1], '-r')
        plt.plot(self.gamma[0][0], self.gamma[0][1], '.k', markersize=10)
        plt.plot(self.gamma[-1][0], self.gamma[-1][1], '.k', markersize=10)
        plt.plot(mp.x_goal[0], mp.x_goal[1], '.k', markersize=10)
        #plt.xlim(-0.2, 00)
        plt.xlabel(r'$x_1$', fontsize=16)
        plt.ylabel(r'$x_2$', fontsize=16)

        plt.show()
        helper_ori = np.repeat(np.array([self.approach_des[0,4:9]]),
                                            x_track.shape[0], axis=0)
        #print(helper_ori)
        #print(self.gamma.shape)
        #print(self.t_span.shape)
        #print(helper_ori.shape)
        # TODO dt
        x_track_pos_ori_jaw = np.concatenate((x_track, helper_ori), axis = 1)
        print(x_track_pos_ori_jaw.shape)
        return x_track_pos_ori_jaw




    def nparray2rostrajectory(x_track_pos_ori_jaw, dt = 0.01):
        resp = DmpGenTrajectoryResponse()
        poses = [ToolPose()] * x_track_pos_ori_jaw.shape[0]
        for i in range(x_track_pos_ori_jaw.shape[0]):
            poses[i].position.x = x_track_pos_ori_jaw[i,0]
            poses[i].position.y = x_track_pos_ori_jaw[i,1]
            poses[i].position.z = x_track_pos_ori_jaw[i,2]
            poses[i].orientation.x = x_track_pos_ori_jaw[i,3]
            poses[i].orientation.y = x_track_pos_ori_jaw[i,4]
            poses[i].orientation.z = x_track_pos_ori_jaw[i,5]
            poses[i].orientation.w = x_track_pos_ori_jaw[i,6]
            poses[i].jaw = x_track_pos_ori_jaw[i,7]

        resp.trajectory.poses = poses
        resp.trajectory.dt = dt
        return resp










if __name__ == "__main__":
    dmp_ser = dmp_service()

    # Test
    #x_track_pos_ori_jaw = dmp_ser.calc_dmp(0,0)
    #resp = dmp_service.nparray2rostrajectory(x_track_pos_ori_jaw)
    #print(resp)











