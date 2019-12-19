#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Nov 28 21:34:49 2018

@author: bennyg
"""

import cv2
from matplotlib import pyplot as plt
import torch
from functions import rebuild_from_disparity
import numpy as np

left_image_path = '/home/bennyg/Development/daVinci/daVinci/test/image_0/000002.png'
right_mage_path = '/home/bennyg/Development/daVinci/daVinci/test/image_1/000002.png'

img_l = cv2.imread(left_image_path)
img_r = cv2.imread(right_mage_path)
img_l = cv2.cvtColor(img_l, cv2.COLOR_BGR2RGB)
img_r = cv2.cvtColor(img_r, cv2.COLOR_BGR2RGB)
img_l_g = cv2.cvtColor(img_l, cv2.COLOR_BGR2GRAY)
img_r_g = cv2.cvtColor(img_r, cv2.COLOR_BGR2GRAY)
stereo = cv2.StereoBM_create()

disparity = stereo.compute(img_l_g, img_r_g)
plt.imshow(disparity, 'gray')
plt.show()

plt.imshow(img_l, 'gray')
plt.show()

plt.imshow(img_r, 'gray')
plt.show()

left_tensor = torch.from_numpy(np.array([img_l]).astype(np.float32))
right_tensor = torch.from_numpy(np.array([img_r]).astype(np.float32))
disp_tensor = torch.from_numpy(np.array([[disparity]]).astype(np.float32))



recon = rebuild_from_disparity(left_tensor.permute(0, 3, 1, 2), disp_tensor)

recon = recon.permute(0, 2, 3, 1).data[0].cpu().numpy()

plt.imshow(recon)
plt.show()        