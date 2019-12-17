# -*- coding: utf-8 -*-
import tensorflow as tf
from Dataset import  MiccaiDataset
from Unet import Unet, Unet11, Unet12
from tensorflow.keras.preprocessing.image import ImageDataGenerator
from tensorflow.keras.backend import set_session
from time import time
from tensorflow.keras.callbacks import TensorBoard, EarlyStopping, ReduceLROnPlateau, ModelCheckpoint
from tensorflow.keras.optimizers import Adam

best_save_path='/pretrined_models/UNET12_BCE_G_C4_best.h5'
save_path = '/pretrined_models/UNET12_BCE_G_C4.h5'
callbacks_l = [
    TensorBoard('/boards/UNET12_BCE_G_C4'),
    ReduceLROnPlateau('val_loss', factor=0.1, patience=2, min_lr=1e-10, verbose=1),
    ReduceLROnPlateau('loss', factor=0.1, patience=2, min_lr=1e-10, verbose=1),
    EarlyStopping('val_loss', patience=15, verbose=1),
    ModelCheckpoint(best_save_path, 'val_loss',verbose=1, save_best_only=True, save_weights_only=True)
]
batchSize = 25
data = MiccaiDataset('/datasets/miccai/dataset', 255, (256, 256), batch_size=batchSize)

config = tf.ConfigProto()
config.gpu_options.allow_growth = True
sess = tf.Session(config=config)
set_session(sess)

net = Unet12()
model = net.build_unet(tf.keras.layers.Input((256, 256, 3)))
model.summary()
model.compile(loss='binary_crossentropy', optimizer=Adam(0.001), metrics=['accuracy'])
datagen_train = data.image_train_generator()
datagen_val = data.image_val_generator() 

model.fit_generator(datagen_train, epochs=45, steps_per_epoch=data.data_train_len(), callbacks=callbacks_l,
                    validation_steps=data.data_val_len(), validation_data=datagen_val)
model.save_weights(save_path)



