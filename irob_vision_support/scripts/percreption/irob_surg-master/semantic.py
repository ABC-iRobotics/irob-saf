from dataset import load_image
import torch
from generate_masks import get_model, img_transform
from functions import jaccard
import numpy as np
import matplotlib.pyplot as plt
from albumentations.pytorch.transforms import img_to_tensor

arrt = np.random.randn(500, 500)
tensor = img_to_tensor(arrt)
mean=(0.485, 0.456, 0.406)
std=(0.229, 0.224, 0.225)

#new_img = img - mean /std
#old img = 

model_path = 'pretrained/unet16_instruments_20/model_1.pt'
model = get_model(model_path, model_type='UNet16', problem_type='instruments')

img_l =  load_image('dataset/instrument_dataset_1/left_frames/frame000.png')
input_img_l = torch.unsqueeze(img_to_tensor(img_transform(p=1)(image=img_l)['image']), dim=0)
mask_l = model(input_img_l)

img_r =  load_image('dataset/instrument_dataset_1/right_frames/frame000.png')
input_img_r = torch.unsqueeze(img_to_tensor(img_transform(p=1)(image=img_r)['image']), dim=0)
mask_r = model(input_img_r)

mask = jaccard(mask_l, mask_r)


im_seg = mask.data[0].cpu().numpy()[0]

#mask_array = (im_seg * std) + mean

plt.imshow(im_seg > 0)
