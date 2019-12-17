# -*- coding: utf-8 -*-
def da_vinci_dataset_path(rootDir, istrain, leftmap, rightmap):
    import os
    import glob
    roots = []
    imgfolder = 'train' if istrain else 'test'
    path_left = os.path.join(rootDir, imgfolder, leftmap)
    path_right = os.path.join(rootDir, imgfolder, rightmap)
    im_p = glob.glob(path_left + "/*.png")

    for p in im_p:
        head, tail = os.path.split(p)
        right_pair = path_right + "/" + tail
        if os.path.isfile(right_pair):
            roots.append((p, right_pair))
    return roots


def read_image(img_paths_l_r, reshape=None):
    import cv2
    img_l = cv2.imread(img_paths_l_r[0])
    img_r = cv2.imread(img_paths_l_r[1])
    if reshape is not None:
        img_l = cv2.resize(img_l, reshape)
        img_r = cv2.resize(img_r, reshape)
    img_l_g = cv2.cvtColor(img_l, cv2.COLOR_BGR2GRAY)
    img_r_g = cv2.cvtColor(img_r, cv2.COLOR_BGR2GRAY)
    return img_l, img_r, img_l_g, img_r_g
