# -*- coding: utf-8 -*-
import torch
import cv2
from Unet import UNet
import numpy as np

def load_model(protoPath, numclasses):
    net = UNet(numclasses)
    net.load_state_dict(torch.load(protoPath))
    net.eval()
    return net

def save_model(model, protoPath):
    torch.save(model.state_dict(), protoPath)

def load_image(path, input_size):
    im = cv2.imread(path)
    return cv2.resize(cv2.cvtColor(im, cv2.COLOR_BGR2RGB), input_size)

def image_to_tensor(image):
    return torch.from_numpy(np.array([np.transpose(image.astype(np.float32) / 255, (2, 0, 1))]))
