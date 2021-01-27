#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Using the DMP library at https://github.com/mginesi/dmp_vol_obst

import numpy as np
from dmp.dmp_cartesian import DMPs_cartesian as dmp_cart
import matplotlib.pyplot as plt
from matplotlib import rc

rc('font',**{'family':'sans-serif','sans-serif':['Helvetica'],
    'size':'14'})
rc('text', usetex=True)


class dmp_service:
    def __init__(self):
        approach_des = np.genfromtxt('../data/approach_1.dat',
            delimiter=' ', skip_header = 1)
        #print(approach_des)

        t_span = np.transpose(approach_des)[0]
        gamma = approach_des[:,1:3]
        #print(t_span)

        t_span = np.linspace(0.0, 4.0, t_span.size)
        #gamma = np.transpose(np.array([t_span, np.sin(t_span) ** 2.0]))



        # DMP
        mp = dmp_cart(n_dmps=2, tol=0.00005)
        mp.imitate_path(x_des=gamma, t_des=t_span)
        mp.x_goal += np.array([0.0, 0.008])

        x_track = mp.rollout()[0]

        plt.figure()
        plt.plot(gamma[:, 0], gamma[:, 1], '--b')
        plt.plot(x_track[:, 0], x_track[:, 1], '-r')
        plt.plot(gamma[0][0], gamma[0][1], '.k', markersize=10)
        plt.plot(gamma[-1][0], gamma[-1][1], '.k', markersize=10)
        plt.plot(mp.x_goal[0], mp.x_goal[1], '.k', markersize=10)
        #plt.xlim(-0.2, 00)
        plt.xlabel(r'$x_1$', fontsize=16)
        plt.ylabel(r'$x_2$', fontsize=16)

        plt.show()


dmp_ser = dmp_service()
