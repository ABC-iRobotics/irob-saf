stereoParams=load('stereoParams.mat');
imaqreset;
webcamlist()
cam_l = videoinput('linuxvideo', 1, 'BGR24_640x480');
cam_r = videoinput('linuxvideo', 2, 'BGR24_640x480');

preview([cam_l, cam_r]);
% triggerconfig(obj.cam_r, 'manual');
%triggerconfig(obj.cam_l, 'manual');
% start(obj.cam_r);
%start(obj.cam_l);
set_focus;

err = {};