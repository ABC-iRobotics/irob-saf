# -*- coding: utf-8 -*-
from NetUtils import load_model, save_model
from Unet import UNet

protoPath = '/home/bennyg/Development/pretrained_models/Unet_C4.pt'

net = UNet(4)

save_model(net, protoPath)

model = load_model(protoPath, numclasses=4)