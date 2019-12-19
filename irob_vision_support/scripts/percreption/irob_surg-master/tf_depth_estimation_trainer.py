import argparse
import tf_dataset_fn as tfds
import tf_net as net
import tensorflow as tf

def create_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--dataset", type=str, help="Path for da vinci training dataset", required=True)
    return parser

def main():
    parser = create_parser()
    ds = []
    try:
        args = parser.parse_args()
        ds = tfds.da_vinci_dataset_path(args.dataset, True, 'image_0', 'image_1')
    except Exception:
        parser.print_help()
        exit(1)
    #height, width, depth
    b, h, w, d = (1, 96, 192, 3)
    im_l, im_r, im_l_g, im_r_g = tfds.read_image(ds[0], (w, h))
    autoencoder = net.create_autoencoder(b, h, w, d)
    model = net.create_networkd_tf_keras()
    # with tf.Session() as sess:
    #     sess.run(tf.global_variables_initializer())
    #     res = sess.run(autoencoder, feed_dict={input: im_l})
    #     print(res)


if __name__ == "__main__":
    main()