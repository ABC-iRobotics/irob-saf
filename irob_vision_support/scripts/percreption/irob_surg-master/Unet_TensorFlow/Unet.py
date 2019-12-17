# -*- coding: utf-8 -*-
import tensorflow as tf

def double_conv(out_channels):
    return tf.keras.Sequential([
        tf.keras.layers.Conv2D(out_channels, 3, padding='same', activation='relu'),
        tf.keras.layers.Conv2D(out_channels, 3, padding='same', activation='relu')
    ])

class Unet(tf.keras.Model):
    def __init__(self, num_classes):
        super(Unet, self).__init__()
        self.num_classes = num_classes
        self.conv_layer_1 = double_conv(64)
        self.conv_layer_2 = double_conv(128)
        self.conv_layer_3 = double_conv(256)
        self.conv_layer_4 = double_conv(512)
        self.maxpool = tf.keras.layers.MaxPool2D()
        self.upsample = tf.keras.layers.UpSampling2D(interpolation='bilinear')
        self.concatenate = tf.keras.layers.Concatenate()
        self.out_conv = tf.keras.layers.Conv2D(num_classes, 1, activation='relu')

        self.upconv_layer_3 = double_conv(256)
        self.upconv_layer_2 = double_conv(128)
        self.upconv_layer_1 = double_conv(64)
    
    def call(self, x):
        print(x)
        conv1 = self.conv_layer_1(x)
        x = self.maxpool(conv1)

        conv2 = self.conv_layer_2(x)
        x = self.maxpool(conv2)

        conv3 = self.conv_layer_3(x)
        x = self.maxpool(conv3)

        x = self.conv_layer_4(x)

        x = self.upsample(x)
        x = self.concatenate([x, conv3])

        x = self.upconv_layer_3(x)
        
        x = self.upsample(x)
        x = self.concatenate([x, conv2])

        x = self.upconv_layer_2(x)

        x = self.upsample(x)
        x = self.concatenate([x, conv1])

        x = self.upconv_layer_1(x)
        x = self.out_conv(x)
        x = x * 255.
        return x




class Unet11():
    def __init__(self, n_filters=16, dropout=0.5, batchNorm=True):
        self.n_filters = n_filters
        self.dropout = dropout
        self.batchnorm = True
        self.concatenate_1 = tf.keras.layers.Concatenate(name='concatenate_1')
        self.concatenate_2 = tf.keras.layers.Concatenate(name='concatenate_2')
        self.concatenate_3 = tf.keras.layers.Concatenate(name='concatenate_3')
        self.concatenate_4 = tf.keras.layers.Concatenate(name='concatenate_4')
        self.concatenate_5 = tf.keras.layers.Concatenate(name='concatenate_5')
        self.dropout_1=tf.keras.layers.Dropout(dropout * 0.5, name='dropout_1')
        self.dropout_2=tf.keras.layers.Dropout(dropout, name='dropout_2')
        self.dropout_3=tf.keras.layers.Dropout(dropout, name='dropout_3')
        self.dropout_4=tf.keras.layers.Dropout(dropout * 0.5, name='dropout_4')
        self.dropout_5=tf.keras.layers.Dropout(dropout * 0.5, name='dropout_5')
        self.dropout_6=tf.keras.layers.Dropout(dropout, name='dropout_6')
        self.dropout_7=tf.keras.layers.Dropout(dropout, name='dropout_7')
        self.dropout_7=tf.keras.layers.Dropout(dropout, name='dropout_8')
        self.maxpool_1=tf.keras.layers.MaxPooling2D((2, 2), name='maxpool_1')
        self.maxpool_2=tf.keras.layers.MaxPooling2D((2, 2), name='maxpool_2')
        self.maxpool_3=tf.keras.layers.MaxPooling2D((2, 2), name='maxpool_3')
        self.maxpool_4=tf.keras.layers.MaxPooling2D((2, 2), name='maxpool_4')
        self.maxpool_5=tf.keras.layers.MaxPooling2D((2, 2), name='maxpool_5')
        self.transpose1 = tf.keras.layers.Conv2DTranspose(self.n_filters * 16, (3, 3), strides=(2, 2), padding='same', name='Transpose_1')
        self.transpose2 = tf.keras.layers.Conv2DTranspose(self.n_filters * 8, (3, 3), strides=(2, 2), padding='same', name='Transpose_2')
        self.transpose3 = tf.keras.layers.Conv2DTranspose(self.n_filters * 4, (3, 3), strides=(2, 2), padding='same', name='Transpose_3')
        self.transpose4 = tf.keras.layers.Conv2DTranspose(self.n_filters * 2, (3, 3), strides=(2, 2), padding='same', name='Transpose_4')
        self.transpose5 = tf.keras.layers.Conv2DTranspose(self.n_filters * 1, (3, 3), strides=(2, 2), padding='same', name='Transpose_5')
        self.out_conv = tf.keras.layers.Conv2D(3, (1, 1), activation='sigmoid', name='out_conv')

    def conv2d_block(self, input, n_filters, block_name, kernel_size=3, batchNorm=True):
        conv_block_1=tf.keras.layers.Conv2D(n_filters, (kernel_size, kernel_size), kernel_initializer='he_normal', padding='same', name='conv2_'+ block_name + '_1')
        actvation_block_1 = tf.keras.layers.Activation('relu', name='act_'+block_name + '_1')
        batchNorm_1=tf.keras.layers.BatchNormalization(name='batch_norm_'+block_name + '_1')

        conv_block_2=tf.keras.layers.Conv2D(n_filters, (kernel_size, kernel_size), kernel_initializer='he_normal', padding='same', name='conv2_'+ block_name + '_2')
        actvation_block_2 = tf.keras.layers.Activation('relu', name='act_'+ block_name + '_2')
        batchNorm_2=tf.keras.layers.BatchNormalization(name='batch_norm_'+block_name + '_2')

        x = conv_block_1(input)
        if batchNorm:
            x = batchNorm_1(x)
        x = actvation_block_1(x)
        x = conv_block_2(x)
        if batchNorm:
            x = batchNorm_2(x)
        x = actvation_block_2(x)
        return x
    
    def build_unet(self, input_image):
        #contraction path
        #(256, 256, None)
        c1 = self.conv2d_block(input_image, self.n_filters * 1, 'contraction_1', 3 ,batchNorm=self.batchnorm)
        p1 = self.maxpool_1(c1)
        p1 = self.dropout_1(p1)

        #(128, 128, 16)
        c2 = self.conv2d_block(p1, self.n_filters * 2, 'contraction_2', 3, self.batchnorm)
        p2 = self.maxpool_2(c2)
        p2 = self.dropout_2(p2)

        # #(64, 64, 32)
        c3 = self.conv2d_block(p2, self.n_filters * 4, 'contraction_3', 3, self.batchnorm)
        p3 = self.maxpool_3(c3)
        p3 = self.dropout_3(p3)

        # (32, 32, 64)
        c4 = self.conv2d_block(p3, self.n_filters *8, 'contraction_4', 3, self.batchnorm)
        p4 = self.maxpool_4(c4)
        p4 = self.dropout_4(p4)

        # (16, 16, 128)
        c5 = self.conv2d_block(p4, self.n_filters * 16, 'contraction_5', 3, self.batchnorm)
        p5 = self.maxpool_5(c5)
        p5 = self.dropout_5(p5)

        # (8, 8, 256)
        c6 = self.conv2d_block(p5, self.n_filters * 32, 'contraction_6', 3, self.batchnorm)

        # (8, 8, 512)
        # #extraction
        u5 = self.transpose1(c6)
        u5 = self.concatenate_1([u5, c5])
        # (16, 16, 768)
        u5 = self.conv2d_block(u5, self.n_filters * 16, 'extraction_1', 3, self.batchnorm)
        
        # (16, 16, 256)
        u4 = self.transpose2(u5)
        u4 = self.dropout_6(u4)
        # (32, 32, 256)
        u4 = self.concatenate_2([u4, c4])
        # (32, 32, 384)
        u4 = self.conv2d_block(u4, self.n_filters * 8, 'extraction_2', 3, self.batchnorm) 

        u3 = self.transpose3(u4)
        #(64, 64, 128)
        u3 = self.concatenate_3([u3, c3])
        #(64, 64, 192)
        u3 = self.conv2d_block(u3, self.n_filters * 4, 'extraction_3', 3, self.batchnorm)

        (64, 64, 64)
        u2 = self.transpose4(u3)
        u2 = self.dropout_6(u2)
        #(128, 128, 64)
        u2 = self.concatenate_4([u2, c2])
        (128, 128, 96)      
        u2 = self.conv2d_block(u2, self.n_filters * 2, 'extraction_4', 3, self.batchnorm)
        (128, 128, 32)

        u1 = self.transpose5(u2)
        u1 = self.dropout_7(u1)
        (256, 256, 32)

        u1 = self.concatenate_5([u1, c1])
        (256, 256, 48)

        u1 = self.conv2d_block(u1, self.n_filters * 1, 'extraction_5', 3, self.batchnorm)
        (256, 256, 16)
        x = self.out_conv(u1)
        return tf.keras.models.Model([input_image], [x])

class Unet12(Unet11):
    def __init__(self, n_filters=16, dropout=0.5, batchNorm=True):
        super(Unet12, self).__init__(n_filters, dropout, batchNorm)
        self.out_conv = tf.keras.layers.Conv2D(1, (1, 1), activation='sigmoid', name='out_conv')
        
    def build_unet(self, input_image):
        c1 = self.conv2d_block(input_image, self.n_filters * 1, 'contraction_1', 3 ,batchNorm=self.batchnorm)
        p1 = self.maxpool_1(c1)
        p1 = self.dropout_1(p1)

        #(128, 128, 16)
        c2 = self.conv2d_block(p1, self.n_filters * 2, 'contraction_2', 3, self.batchnorm)
        p2 = self.maxpool_2(c2)
        p2 = self.dropout_2(p2)

        # #(64, 64, 32)
        c3 = self.conv2d_block(p2, self.n_filters * 4, 'contraction_3', 3, self.batchnorm)
        p3 = self.maxpool_3(c3)
        p3 = self.dropout_3(p3)

        # (32, 32, 64)
        c4 = self.conv2d_block(p3, self.n_filters *8, 'contraction_4', 3, self.batchnorm)
        p4 = self.maxpool_4(c4)
        p4 = self.dropout_4(p4)

        # (16, 16, 128)
        c5 = self.conv2d_block(p4, self.n_filters * 16, 'contraction_5', 3, self.batchnorm)

        u4 = self.transpose2(c5)
        u4 = self.dropout_6(u4)
        u4 = self.concatenate_4([u4, c4])

        u4 = self.conv2d_block(u4, self.n_filters * 8, 'extraction_2', 3, self.batchnorm)

        u3 = self.transpose3(u4)
        #(64, 64, 128)
        u3 = self.concatenate_3([u3, c3])
        #(64, 64, 192)
        u3 = self.conv2d_block(u3, self.n_filters * 4, 'extraction_3', 3, self.batchnorm)

        u2 = self.transpose4(u3)
        u2 = self.dropout_6(u2)
        #(128, 128, 64)
        u2 = self.concatenate_4([u2, c2])

        u2 = self.conv2d_block(u2, self.n_filters * 2, 'extraction_4', 3, self.batchnorm)

        u1 = self.transpose5(u2)
        u1 = self.dropout_7(u1)
        (256, 256, 32)

        u1 = self.concatenate_5([u1, c1])
        (256, 256, 48)

        u1 = self.conv2d_block(u1, self.n_filters * 1, 'extraction_5', 3, self.batchnorm)
        (256, 256, 16)
        x = self.out_conv(u1)  

        return tf.keras.models.Model([input_image], [x])