import tensorflow as tf
from tensorflow.keras.backend import set_session
from Unet import Unet11, Unet12

config = tf.ConfigProto()
config.gpu_options.allow_growth = True
sess = tf.Session(config=config)
set_session(sess)

input_img = tf.keras.layers.Input((256, 256, 3), name='img')
net = Unet12()

model = net.build_unet(input_img)
model.summary()