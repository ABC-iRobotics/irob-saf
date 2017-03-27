clear all;
close all;
imaqreset;
webcamlist();
 %cam_l = videoinput('linuxvideo', 1, 'BGR24_640x480');
% cam_r = videoinput('linuxvideo', 2, 'BGR24_640x480');

 cam_r = videoinput('linuxvideo', 1, 'RGB24_1280x960');
 cam_l = videoinput('linuxvideo', 2, 'RGB24_1280x960');

% cam_l = videoinput('linuxvideo', 1, 'RGB24_1280x720');
% cam_r = videoinput('linuxvideo', 2, 'RGB24_1280x720');

% cam_l = videoinput('linuxvideo', 1, 'RGB24_320x240');
% cam_r = videoinput('linuxvideo', 2, 'RGB24_320x240');



folder = 'Bacon_test6';
subfolder = 'img';
filename = 'img';
num_img = 19;

mkdir(folder, subfolder);
mkdir(folder, subfolder);

preview([cam_l, cam_r]);
waitforbuttonpress;

for i = 1:num_img
    w = waitforbuttonpress;
    disp(i);
    I_l = getsnapshot(cam_l);
    I_r = getsnapshot(cam_r);
    
    I_l = imrotate(I_l, -90);
    I_r = imrotate(I_r, 90);
    
    % imwrite(I_l, strcat(filename, '_l_', num2str(i), '.jpg'));
    %imwrite(I_r, strcat(filename, '_r_', num2str(i), '.jpg'));
    
       imwrite(I_l, strcat(folder,'/', subfolder, '/', filename, '_l_', num2str(i), '.jpg'));
     imwrite(I_r, strcat(folder,'/',  subfolder, '/',filename, '_r_', num2str(i), '.jpg'));
end


