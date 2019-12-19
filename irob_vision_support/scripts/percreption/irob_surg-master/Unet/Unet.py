import torch
import torch.nn as nn

def double_conv(in_channels, out_channels):
    return nn.Sequential(
        nn.Conv2d(in_channels, out_channels, 3, padding=1),
        nn.LeakyReLU(inplace=True),
        nn.Conv2d(out_channels, out_channels, 3, padding=1),
        nn.LeakyReLU(inplace=True)
    )

class UNet(nn.Module):
    def __init__(self, class_number):
        super(UNet, self).__init__()
        
        self.conv_layer_1 = double_conv(3, 64)
        self.conv_layer_2 = double_conv(64, 128)
        self.conv_layer_3 = double_conv(128, 256)
        self.conv_layer_4 = double_conv(256, 512)

        self.maxpool = nn.MaxPool2d(2, stride=2)
        self.upsample = nn.Upsample(scale_factor=2, mode='bilinear', align_corners=True)

        self.upconv_layer_3 = double_conv(256 + 512, 256)
        self.upconv_layer_2 = double_conv(128 + 256, 128)
        self.upconv_layer_1 = double_conv(64 + 128, 64)

        self.out_conv = nn.Conv2d(64, class_number, 1)
    
    def forward(self, x):
        conv1 = self.conv_layer_1(x)
        x = self.maxpool(conv1)

        conv2 = self.conv_layer_2(x)
        x = self.maxpool(conv2)

        conv3 = self.conv_layer_3(x)
        x = self.maxpool(conv3)

        x = self.conv_layer_4(x)

        x = self.upsample(x)
        x = torch.cat([x, conv3], dim=1)

        x = self.upconv_layer_3(x)
        
        x = self.upsample(x)
        x = torch.cat([x, conv2], dim=1)

        x = self.upconv_layer_2(x)

        x = self.upsample(x)
        x = torch.cat([x, conv1], dim=1)

        x = self.upconv_layer_1(x)

        x = self.out_conv(x)
        return x

        