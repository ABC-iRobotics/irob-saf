function [ I_l, I_r ] = stereo_capture( num_img, resolution )
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


preview([cam_l, cam_r]);
waitforbuttonpress;

for i = 1:num_img
    w = waitforbuttonpress;
    disp(i);
    I_l = getsnapshot(cam_l);
    I_r = getsnapshot(cam_r);

end
end

