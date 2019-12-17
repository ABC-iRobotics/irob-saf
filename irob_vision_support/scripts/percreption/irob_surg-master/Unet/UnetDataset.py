import os
import glob
import torch
from torch.utils.data.dataset import Dataset
import cv2
import numpy as np

class ToolDataset(Dataset):
    def __init__(self, rootDirs, mean, std, pixelDiv, input_size):
        self.mean = mean
        self.std = std
        self.pixelDiv = pixelDiv
        self.input_size = input_size 
        self.paths = []
        for rootDir in rootDirs:
            image_paths = os.path.join(rootDir, 'left_frames')
            image_left_paths = glob.glob(image_paths + "/*.png")
            labels_path = os.path.join(rootDir, 'labels')  
            for i in image_left_paths:
                head, tail = os.path.split(i)
                im_label = labels_path + '/' + tail              
                if os.path.isfile(im_label):
                    self.paths.append((i, im_label))
      
    def normalize_image(self, img):
        return (img.astype(np.float32) / self.pixelDiv)
        
        
    def __len__(self):
        return len(self.paths)
    
    def __getitem__(self, idx):
        image_src = cv2.resize(cv2.cvtColor(cv2.imread(self.paths[idx][0]), cv2.COLOR_BGR2RGB), self.input_size)
        image_labels = cv2.resize(cv2.cvtColor(cv2.imread(self.paths[idx][1]), cv2.COLOR_BGR2GRAY), self.input_size)
        
        image_mask = torch.from_numpy((self.normalize_image(np.array([image_labels]))))
        image_src = torch.from_numpy(self.normalize_image(np.transpose(image_src, (2, 0, 1))))
        return image_src, image_mask
        


        
        