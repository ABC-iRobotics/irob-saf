clear all;
imaqreset;
webcamlist()
%cam1 = webcam;
cam_r = videoinput('linuxvideo', 1, 'BGR24_640x480');
cam_l = videoinput('linuxvideo', 2, 'BGR24_640x480');

folder = 'new_test';
subfolder = 'new_test';
filename = 'new_test';
num_img = 1;

%mkdir(strcat(subfolder,'_l'));
%mkdir(strcat(subfolder,'_r'));

preview([cam_l, cam_r]);
waitforbuttonpress;

for i = 1:num_img
    w = waitforbuttonpress;
    disp(i);
    I_l = getsnapshot(cam_l);
    I_r = getsnapshot(cam_r);
    
    imwrite(I_l, strcat( subfolder, '/', filename, '_l_', num2str(i), '.jpg'));
    imwrite(I_r, strcat( subfolder, '/',filename, '_r_', num2str(i), '.jpg'));
end


