#!/usr/bin/python3
# -*- coding: utf-8 -*-

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


dmp_ser = dmp_service()
