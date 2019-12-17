import torch
import numpy as np
import matplotlib.pyplot as plt
from Unet import UNet
from NetUtils import load_model, load_image, image_to_tensor

path = '/home/bennyg/Development/datasets/miccai_challenge_release_2/seq_5/left_frames/frame001.png'
pretrained_path = '/home/bennyg//Letöltések/Unet_C1.pt'

image = load_image(path, (256, 256))
tensor = image_to_tensor(image)
print(tensor.shape)
model = load_model(pretrained_path, 1)
with torch.no_grad():
    mask = model(tensor)
    mask = torch.floor(mask * 255)
    mask = mask.data[0].cpu().numpy()[0]
    mask = mask.astype(np.int32)
    plt.imshow(mask > 0)