# -*- coding: utf-8 -*-

import torch
import torch.nn as nn
import depthestimation as DE
import davinci_dataset as dvs
import functions as fn

device = "cpu"
mean=(0.485, 0.456, 0.406)
std=(0.229, 0.224, 0.225)
batch_size=16
train_data = dvs.DavinciDataset("/home/bennyg/Development/daVinci/daVinci/", True, 'image_0', 'image_1')
testdataloader = torch.utils.data.DataLoader(train_data, batch_size=25, shuffle=True)
net = DE.DepthEstimatorNet().to(device)
optimizer = torch.optim.Adam(net.parameters(), lr=1e-4)
criterion = nn.MSELoss()
for epoch in range(40):
    i = 0
    running_loss = 0.0
    for image_l, image_r in testdataloader:
        image_l = image_l.to(device)
        image_r = image_r.to(device)
        #print(image_l.shape)
        dept_l = net(image_l, image_r)
        #print(dept_l.shape)
        dept_l = torch.floor(dept_l * 255)
        im_rec = fn.rebuild_from_disparity(image_r, dept_l)
        print(dept_l.shape)
        #print(im_rec.permute(0,1,3,2).shape)
        optimizer.zero_grad()
        #image_l = image_l.permute(0, 1, 3, 2)
        loss = criterion(im_rec, image_l)
        #print(loss)
        loss.backward()
        optimizer.step()
        i += 1
        running_loss += loss.item()
        torch.cuda.empty_cache()
        if i % 2000 == 1999:    # print every 2000 mini-batches
            print('[%d, %5d] loss: %.3f' %
                  (epoch + 1, i + 1, running_loss / 2000))
            running_loss = 0.0

torch.save(net.state_dict(), "d:\/Development/daVinci/daVinci/depthnet.pt")
