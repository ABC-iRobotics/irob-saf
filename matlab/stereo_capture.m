function [ I_l, I_r ] = stereo_capture( num_img, resolution, firstTgt)
% This function takes images from stereo cameras. 
%   @author: Renata Elek
%   num_img: number of images (use 19 for calibration and 1 for general
%   image capture.
%   resolution: 'BGR24_640x480' in general
%   Call it like this:
%   [I_l, I_r] = stereo_capture(1, 'BGR24_640x480');
  
imaqreset;
webcamlist()
cam_r = videoinput('linuxvideo', 1, resolution);
cam_l = videoinput('linuxvideo', 2, resolution);


%preview([cam_l, cam_r]);
% if firstTgt
%     waitforbuttonpress;
% end
for i = 1:num_img
%     if firstTgt
%         w = waitforbuttonpress;
%     end

    I_l = getsnapshot(cam_l);
    I_r = getsnapshot(cam_r);
    
    I_l = imrotate(I_l, -90);
    I_r = imrotate(I_r, 90);
     disp('Image pair captured.');
end
end

