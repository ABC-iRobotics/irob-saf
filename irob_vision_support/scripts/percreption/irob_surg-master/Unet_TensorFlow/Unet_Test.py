# -*- coding: utf-8 -*-
import tensorflow as tf
import numpy as np
import cv2
from Unet import Unet, Unet11, Unet12
from tensorflow.keras.optimizers import Adam
from tensorflow.keras.backend import set_session

def load_image(path):
    img=cv2.resize(cv2.cvtColor(cv2.imread(path), cv2.COLOR_BGR2RGB), (256, 256))
    return np.array([(img.astype(np.float32) / 255)])
    
config = tf.ConfigProto()
config.gpu_options.allow_growth = True
sess = tf.Session(config=config)
set_session(sess)

net = Unet12()
input_img = tf.keras.layers.Input((256, 256, 3), name='img')
model = net.build_unet(input_img)
model.load_weights('/pretrined_models/UNET12_BCE_G_C4.h5')

#test data: /datasets/miccai/dataset/images/frame_2_994.jpg
image = load_image('/datasets/miccai/dataset/images/frame_0_1.jpg')
pred = model.predict(image)
print(pred)
pred = pred[0]
pred = pred * 255
pred = pred.astype(np.int32).reshape(256,256)
pred[pred < 0]=0
pred[pred > 255]=255
cv2.imwrite('/datasets/UNET12_BCE_G_C4.jpg', pred)

