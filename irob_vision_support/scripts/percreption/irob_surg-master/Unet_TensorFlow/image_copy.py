import cv2

def make_frames(video, dest, index):
        cap = cv2.VideoCapture(video)
        count = 0
    
        while(True):
            ret, frame = cap.read()
            if not ret:
                print('{} images created at iteration {}.'.format(count, index))
                break
            count += 1
            (h, w) = frame.shape[:2]
            center = (w / 2, h / 2)
            M = cv2.getRotationMatrix2D(center, 5, 1.0)
            name =  'frame_{}_{}'.format(index, count)
            cv2.imwrite(dest.format(name), frame)
            
            name =  'frame_{}fv_{}'.format(index, count)
            flip = cv2.flip(frame, 0)
            cv2.imwrite(dest.format(name), flip)

            name =  'frame_{}fvb_{}'.format(index, count)
            flip = cv2.flip(frame, -1)
            cv2.imwrite(dest.format(name), flip)

            name =  'frame_{}fvh_{}'.format(index, count)
            flip = cv2.flip(frame, 1)
            cv2.imwrite(dest.format(name), flip)
            
            name =  'frame_{}r_{}'.format(index, count)
            rot = cv2.warpAffine(frame, M, (h, w))
            cv2.imwrite(dest.format(name), rot)
        cap.release()
    

basedir = [('/home/bennyg/Development/datasets/miccai/videoframes/Segmentation_Robotic_Training/Training/Dataset2/Video.avi', '/home/bennyg/Development/datasets/miccai/videoframes/Segmentation_Robotic_Training/Training/Dataset2/Segmentation.avi'),
           ('/home/bennyg/Development/datasets/miccai/videoframes/Segmentation_Robotic_Training/Training/Dataset3/Video.avi', '/home/bennyg/Development/datasets/miccai/videoframes/Segmentation_Robotic_Training/Training/Dataset3/Segmentation.avi'),
           ('/home/bennyg/Development/datasets/miccai/videoframes/Segmentation_Robotic_Training/Training/Dataset4/Video.avi', '/home/bennyg/Development/datasets/miccai/videoframes/Segmentation_Robotic_Training/Training/Dataset3/Segmentation.avi')
           ]

destDir_img, destDir_mask = '/home/bennyg/Development/datasets/miccai/dataset/images/{}.jpg', '/home/bennyg/Development/datasets/miccai/dataset/masks/{}.jpg'

for i, (image_vid_pth, mask_vid_pth) in enumerate(basedir):
    make_frames(image_vid_pth, destDir_img, i)
    make_frames(mask_vid_pth, destDir_mask, i)
        