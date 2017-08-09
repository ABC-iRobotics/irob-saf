obj.stereoParams=load('stereoParams.mat');
imaqreset;
obj.cam_r = videoinput('linuxvideo', 1, 'BGR24_640x480');
obj.cam_l = videoinput('linuxvideo', 2, 'BGR24_640x480');

preview([obj.cam_l, obj.cam_r]);
% triggerconfig(obj.cam_r, 'manual');
%triggerconfig(obj.cam_l, 'manual');
% start(obj.cam_r);
%start(obj.cam_l);
set_focus;

err = {};