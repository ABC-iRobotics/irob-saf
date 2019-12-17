# -*- coding: utf-8 -*-

import os
import glob
import numpy as np
import torch
import torchvision.transforms.functional as tfn
from torch.utils.data.dataset import Dataset
import cv2

class DavinciDataset(Dataset):
    def __init__(self, rootDir, train ,leftmap, rightmap, transform = None, reshapeSize = (96, 192), mean=(0.485, 0.456, 0.406),
                 std=(0.229, 0.224, 0.225)):
        self.transforms = transform
        self.rootDir = rootDir
        self.leftmap = leftmap
        self.right  = rightmap
        self.reshapeSize = reshapeSize
        
        testrainfolder = 'train' if train else 'test'
        
        path_left = os.path.join(self.rootDir, testrainfolder, leftmap)
        path_right = os.path.join(self.rootDir, testrainfolder, rightmap)
        im_0 = glob.glob(path_left + "/*.png")
        
        self.roots = []

        for i in im_0:
            head, tail = os.path.split(i)
            right_pair = path_right +"/" + tail 
            if os.path.isfile(right_pair):
                self.roots.append((i, right_pair))
                
    def __len__(self):
        return len(self.roots)
    
    def __getitem__(self, idx):
        image_l = cv2.imread(self.roots[idx][0])
        image_r = cv2.imread(self.roots[idx][1])
        image_l = cv2.cvtColor(image_l, cv2.COLOR_BGR2RGB)
        image_r = cv2.cvtColor(image_r, cv2.COLOR_BGR2RGB)
        image_l = cv2.resize(image_l, self.reshapeSize)
        image_r = cv2.resize(image_r, self.reshapeSize)
        if self.transforms:
            image_l = self.transforms(image_l)
            image_r = self.transforms(image_r)
            image_rg = self.transforms(image_r)
        else:
            image_l = np.transpose((image_l / 255), (2, 0, 1)).astype(np.float32)
            image_r = np.transpose((image_r / 255), (2, 0, 1)).astype(np.float32)
            image_l = torch.from_numpy(image_l)
            image_r = torch.from_numpy(image_r)
            
        return (image_l, image_r)

