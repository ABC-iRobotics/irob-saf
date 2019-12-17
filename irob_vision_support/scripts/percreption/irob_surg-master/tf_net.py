import tensorflow as tf

def pool2d(input, name, ksize=2):
    return tf.nn.max_pool_with_argmax(input, ksize=[1, ksize, ksize, 1], strides=[1,2,2,1], padding='SAME', name=name)

def unpool_2d(pool, ind, ksize=[1, 2, 2, 1], scope='unpool'):
    import numpy as np
    with tf.variable_scope(scope):
        input_shape = pool.get_shape().as_list()
        output_shape = (input_shape[0], input_shape[1] * ksize[1], input_shape[2] * ksize[2], input_shape[3])
        flat_input_size = np.prod(input_shape)
        flat_output_shape = [output_shape[0], output_shape[1] * output_shape[2] * output_shape[3]]
        pool_ = tf.reshape(pool, [flat_input_size])
        batch_range = tf.reshape(tf.range(output_shape[0], dtype=ind.dtype), shape=[input_shape[0], 1, 1, 1])
        b = tf.ones_like(ind) * batch_range
        b = tf.reshape(b, [flat_input_size, 1])
        ind_ = tf.reshape(ind, [flat_input_size, 1])
        ind_ = tf.concat([b, ind_], 1)
        ret = tf.scatter_nd(ind_, pool_, shape=flat_output_shape)
        ret = tf.reshape(ret, output_shape)
        return ret

def create_autoencoder(batch_size, height, width, channel):
    input = tf.placeholder(tf.float32, shape=[batch_size, height, width, channel] ,name='input')

    conv1_1 = tf.layers.conv2d(input, filters=64, kernel_size=[3, 3], strides=1, activation=tf.nn.relu, name='conv1_1')
    conv1_2 = tf.layers.conv2d(conv1_1, filters=64, kernel_size=[3, 3], strides=1, activation=tf.nn.relu, name='conv1_2')
    # pool1, max_unpool_1 = pool2d(conv1_2, 'pool1')

    conv2_1 = tf.layers.conv2d(conv1_2,filters=128, kernel_size=[3, 3], strides=1, activation=tf.nn.relu, name='conv2_1')
    conv2_2 = tf.layers.conv2d(conv2_1, filters=128, kernel_size=[3, 3], activation=tf.nn.relu, name='conv2_2')
    # pool2, max_unpool_2 = pool2d(conv2_2, 'pool2')

    conv3_1 = tf.layers.conv2d(conv2_2, filters=256, kernel_size=[3, 3], activation=tf.nn.relu, name='conv3_1')
    conv3_2 =tf.layers.conv2d(conv3_1, filters=256, kernel_size=[3, 3], activation=tf.nn.relu, name='conv3_2')
    conv3_3 = tf.layers.conv2d(conv3_2, filters=256, kernel_size=[3, 3], activation=tf.nn.relu, name='conv3_3')
    # pool3, max_unpool_3 = pool2d(conv3_3, 'pool3')

    conv4_1 = tf.layers.conv2d(conv3_3, filters=512, kernel_size=[3, 3], activation=tf.nn.relu, name='conv4_1')
    conv4_2 = tf.layers.conv2d(conv4_1, filters=512, kernel_size=[3, 3], activation=tf.nn.relu, name='conv4_2')
    conv4_3 = tf.layers.conv2d(conv4_2, filters=512, kernel_size=[3, 3], activation=tf.nn.relu, name='conv4_3')
    # pool4, max_unpool4 = pool2d(conv4_3, 'pool4')

    conv5_1 = tf.layers.conv2d(conv4_3, filters=512, kernel_size=[3, 3], activation=tf.nn.relu, name='conv5_1')
    conv5_2 = tf.layers.conv2d(conv5_1, filters=512, kernel_size=[3, 3], activation=tf.nn.relu, name='conv5_2')
    conv5_3 = tf.layers.conv2d(conv5_2, filters=512, kernel_size=[3, 3], activation=tf.nn.relu, name='conv5_3')
    # pool5, max_unpool5 = pool2d(conv5_3, 'pool5')

    conv6 = tf.layers.conv2d(conv5_3, filters=512, kernel_size=[3, 3], activation=tf.nn.relu, name='conv6')

    #decoder
    # unpool5 = unpool_2d(conv6, max_unpool5)

    deconv5_1 = tf.layers.conv2d_transpose(conv6, filters=512, kernel_size=[3, 3], activation=tf.nn.relu,
                                           name='deconv5_1')
    deconv5_2 = tf.layers.conv2d_transpose(deconv5_1, filters=512, kernel_size=[3, 3], activation=tf.nn.relu,
                                           name='deconv5_2')
    deconv5_3 = tf.layers.conv2d_transpose(deconv5_2, filters=512, kernel_size=[3, 3], activation=tf.nn.relu,
                                           name='deconv5_3')
    # unpool4 = unpool_2d(deconv5_3, max_unpool4)

    deconv4_1 = tf.layers.conv2d_transpose(deconv5_3, filters=512, kernel_size=[3, 3], activation=tf.nn.relu,
                                           name='deconv4_1')
    deconv4_2 = tf.layers.conv2d_transpose(deconv4_1, filters=512, kernel_size=[3, 3], activation=tf.nn.relu,
                                           name='deconv4_2')
    deconv4_3 = tf.layers.conv2d_transpose(deconv4_2, filters=256, kernel_size=[3, 3], activation=tf.nn.relu,
                                           name='deconv4_3')
    # unpool3 = unpool_2d(deconv4_3, max_unpool_3)

    deconv3_1 = tf.layers.conv2d_transpose(deconv4_2, filters=256, kernel_size=[3, 3], activation=tf.nn.relu,
                                           name='deconv3_1')
    deconv3_2 = tf.layers.conv2d_transpose(deconv3_1, filters=256, kernel_size=[3, 3], activation=tf.nn.relu,
                                           name='deconv3_2')
    deconv3_3 = tf.layers.conv2d_transpose(deconv3_2, filters=128, kernel_size=[3, 3], activation=tf.nn.relu,
                                           name='deconv3_3')

    # unpool2 = unpool_2d(deconv3_3, max_unpool_2)
    deconv2_1 = tf.layers.conv2d_transpose(deconv3_3, filters=128, kernel_size=[3, 3], activation=tf.nn.relu,
                                           name='deconv2_1')
    deconv2_2 = tf.layers.conv2d_transpose(deconv2_1, filters=64, kernel_size=[3, 3], activation=tf.nn.relu,
                                           name='deconv2_2')

    # unpool1 = unpool_2d(deconv2_2, max_unpool_1)
    deconv1_1 = tf.layers.conv2d_transpose(deconv2_2, filters=64, kernel_size=[3, 3], activation=tf.nn.relu,
                                           name='deconv1_1')
    deconv1_2 = tf.layers.conv2d_transpose(deconv1_1, filters=3, kernel_size=[3, 3], activation=tf.nn.relu,
                                           name='deconv1_2')
    conv0 = tf.layers.conv2d(deconv1_2, filters=1, kernel_size=[3, 3], activation=tf.nn.relu, padding='SAME', name='conv0')

    return conv0