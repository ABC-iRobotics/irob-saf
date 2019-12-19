import torch
import torch.nn as nn
from Unet import UNet
from UnetDataset import ToolDataset
from NetUtils import save_model
from Unet_loss import dice_loss

input_size = (256, 256)
mean = 0.485
std = 0.229
pixdeviator = 255
dataset = ToolDataset(['/datasets/miccai_challenge_2018_release_1/seq_1',
                       '/datasets/miccai_challenge_2018_release_1/seq_2',
                       '/datasets/miccai_challenge_2018_release_1/seq_3',
                       '/datasets/miccai_challenge_2018_release_1/seq_4',
                       '/datasets/miccai_challenge_release_2/seq_5',
                       '/datasets/miccai_challenge_release_2/seq_6',
                       '/datasets/miccai_challenge_release_2/seq_7',
                       '/datasets/miccai_challenge_release_3/seq_9',
                       '/datasets/miccai_challenge_release_3/seq_10',
                       '/datasets/miccai_challenge_release_3/seq_11',
                       '/datasets/miccai_challenge_release_3/seq_12'
                       '/datasets/miccai_challenge_release_4/seq_13', 
                       '/datasets/miccai_challenge_release_4/seq_14',
                       '/datasets/miccai_challenge_release_4/seq_15',
                       '/datasets/miccai_challenge_release_4/seq_16'], mean, 
                      std, pixdeviator, input_size)
dataloader = torch.utils.data.DataLoader(dataset, shuffle=True, batch_size=10)
is_cuda = torch.cuda.is_available()

number_epoch = 15
learning_rate = 1e-5
net = UNet(1)
if is_cuda:
    net = net.cuda()
    print('learn runs on cude device')
optimizer = torch.optim.Adam(net.parameters(), lr=learning_rate)
criterion = nn.BCEWithLogitsLoss()
number_train = len(dataloader)
for i in range(number_epoch):
    epoch_loss = 0.0
    loss_index = 0
    net.train()
    for image_src, image_mask in dataloader:
        if is_cuda:
            image_src = image_src.cuda()
            image_mask = image_mask.cuda()
        loss_index += 1
        pred_mask = net(image_src)
        pred_flat = torch.flatten(pred_mask, start_dim=1)
        mask_flat = torch.flatten(image_mask, start_dim=1)
        loss = criterion(pred_mask, image_mask)
        epoch_loss += loss.item()
        print('{0}/{1} --- loss: {2:.6f}'.format(loss_index, number_train, loss.item()))
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
          
    print('Epoch: {} / {} Loss: {}'.format(i + 1, number_epoch, epoch_loss / loss_index))
    
protoPath = '/pretrined_models/Unet_BCE_C2.pt'
save_model(net, protoPath)
