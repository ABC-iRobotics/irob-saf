import tensorflow as tf
import cv2
import imutils
import numpy as np
from Unet import Unet12
from tensorflow.keras.backend import set_session

class TootlSegment():
    def __init__(self, network_path):
        self.network_path = network_path
        self.frame_h, self.frame_w =(256, 256)
        self.model_built = False
        self.model = Unet12().build_unet(tf.keras.layers.Input((256, 256, 3), name='img'))
    
    def build_model(self):
        if self.model_built:
            return
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        sess = tf.Session(config=config)
        set_session(sess)

        self.model.load_weights(self.network_path)
        self.model.summary()
        self.model_built=True

    def  mask_overlay(self, image, mask, color=(0, 255, 0)):
        mask = np.dstack((mask, mask, mask)) * np.array(color)
        mask = mask.astype(np.uint8)
        weighted_sum = cv2.addWeighted(mask, 0.5, image, 0.5, 0.)
        img = image.copy()
        ind = mask[:, :, 1] > 0    
        img[ind] = weighted_sum[ind]    
        return img

    def treshold_tool(self, blur):
        return cv2.threshold(blur, 52, 87, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]

    def find_center(self, thresh):
        cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
	    cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        centers = []
        for c in cnts:
    	# compute the center of the contour
            M = cv2.moments(c)
            if M['m00'] == 0:
                continue
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            centers.append((cX, cY))
            # draw the contour and center of the shape on the image
            #cv2.drawContours(overlay, [c], -1, (0, 255, 0), 2)
            #cv2.circle(overlay, (cX, cY), 7, (255, 255, 255), -1)
        return centers
    
    def draw_centers(self, centers, overlay_img):
        for cX, cY in centers:
            cv2.circle(overlay_img, (cX, cY), 7, (255, 255, 255), -1)
        return overlay_img

    def apply_video(self, video_path, output_video_path):
        self.build_model()
        cap = cv2.VideoCapture(video_path)
        out = cv2.VideoWriter(output_video_path, cv2.VideoWriter_fourcc(*'DIVX'), 15, (self.frame_h, self.frame_w))

        while True:
            ret, frame = cap.read()
            if not ret:
                break
            input = cv2.resize(frame, (self.frame_h, self.frame_w))
            pred = self.model.predict(np.array([input]))
            pred = pred[0]
            pred = pred * 255
            pred = pred.astype(np.int32).reshape(256,256)
            pred[pred < 0]=0
            pred[pred > 255]=255
            cv2.imwrite('/tmp/temp.jpg', pred)
            pred = cv2.cvtColor(cv2.imread('/tmp/temp.jpg'), cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(pred,(5,5),0)
            thresh = self.treshold_tool(blur)
            centers = self.find_center(thresh)
            overlay = self.mask_overlay(input, blur)
            centered_overlay = self.draw_centers(centers, overlay)
            out.write(centered_overlay)

        cap.release()
        out.release()


