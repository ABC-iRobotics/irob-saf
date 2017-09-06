clear all;
close all;

left_img_sub = rossubscriber('/ias/stereo/left_final/image', 'sensor_msgs/Image');
right_img_sub = rossubscriber('/ias/stereo/right_final/image', 'sensor_msgs/Image');

pause(2) % Wait to ensure publisher is registered
           

% while



folder = 'calib';
subfolder = 'img';
filename = 'img';
num_img = 19;


set_focus;

preview([cam_l, cam_r]);
waitforbuttonpress;

for i = 1:num_img
    w = waitforbuttonpress;
    disp(i);

    
    left_img_msg = left_img_sub.LatestMessage;
    right_img_msg = right_img_sub.LatestMessage;

    I_l = readImg(left_img_msg);
    I_r = readImg(right_img_msg);

    
    % imwrite(I_l, strcat(filename, '_l_', num2str(i), '.jpg'));
    %imwrite(I_r, strcat(filename, '_r_', num2str(i), '.jpg'));
     imwrite(I_l, strcat(folder,'/', subfolder, '_l/', filename, '_l_', num2str(i), '.jpg'));
     imwrite(I_r, strcat(folder,'/',  subfolder, '_r/',filename, '_r_', num2str(i), '.jpg'));
end


