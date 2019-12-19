from ToolSegment import TootlSegment

segment = TootlSegment('/pretrined_models/UNET12_BCE_G_C4_best.h5')
segment.apply_video('/datasets/miccai/videoframes/Segmentation_Robotic_Training/Training/Dataset2/Video.avi', '/datasets/overlayed_2_best.avi')