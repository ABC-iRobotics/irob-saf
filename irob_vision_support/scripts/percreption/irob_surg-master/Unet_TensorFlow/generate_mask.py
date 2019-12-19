# -*- coding: utf-8 -*-
import numpy as np
import cv2
import imutils
import matplotlib.pyplot as plt

def mask_overlay(image, mask, color=(0, 255, 0)):
    mask = np.dstack((mask, mask, mask)) * np.array(color)
    mask = mask.astype(np.uint8)
    weighted_sum = cv2.addWeighted(mask, 0.5, image, 0.5, 0.)
    img = image.copy()
    ind = mask[:, :, 1] > 0    
    img[ind] = weighted_sum[ind]    
    return img

def treshold_tool(blur):
    return cv2.threshold(blur, 52, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
    

image_p = '/home/bennyg/Development/datasets/miccai/dataset/images/frame_0_1.jpg'
mask_p = '/home/bennyg/Development/datasets/UNET12_BCE_G_C3.jpg'

img=cv2.resize(cv2.cvtColor(cv2.imread(image_p), cv2.COLOR_BGR2RGB), (256, 256))
mask=cv2.cvtColor(cv2.imread(mask_p), cv2.COLOR_BGR2GRAY)

#Gaussian blur for mask
#treshold
pred_seq_mask = mask.copy()
pred_seq_mask[pred_seq_mask >= 97] = 0
blur = cv2.GaussianBlur(pred_seq_mask,(5,5),0)
thresh = treshold_tool(blur)

plt.imshow(thresh)

#get contours
cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
	cv2.CHAIN_APPROX_SIMPLE)
cnts = imutils.grab_contours(cnts)

overlay = mask_overlay(img, mask)

for c in cnts:
	# compute the center of the contour
	M = cv2.moments(c)
	cX = int(M["m10"] / M["m00"])
	cY = int(M["m01"] / M["m00"])
 
	# draw the contour and center of the shape on the image
	#cv2.drawContours(overlay, [c], -1, (0, 255, 0), 2)
	cv2.circle(overlay, (cX, cY), 7, (255, 255, 255), -1)

plt.imshow(overlay)