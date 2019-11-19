

rosshutdown;
rosinit;

left_img_sub = rossubscriber('/saf/stereo/left/final/image', 'sensor_msgs/Image');
right_img_sub = rossubscriber('/saf/stereo/right/final/image', 'sensor_msgs/Image');

pause(2) % Wait to ensure publisher is registered
           

% while



folder = 'calib';
subfolder = 'img';
filename = 'img';
num_img = 19;

waitforbuttonpress;
i = 0;
while true 
    w = mouseinput_timeout(1);
    

    
    left_img_msg = left_img_sub.LatestMessage;
    right_img_msg = right_img_sub.LatestMessage;

    if (and(size(left_img_msg) > 0, size(right_img_msg) > 0))
        I_l = readImage(left_img_msg);
         I_r = readImage(right_img_msg);
    end
    imshow([I_l, I_r])
    
    % imwrite(I_l, strcat(filename, '_l_', num2str(i), '.jpg'));
    %imwrite(I_r, strcat(filename, '_r_', num2str(i), '.jpg'));
    if size(w) > 0
        
        i = i+1;
        disp(i);
        imwrite(I_l, strcat(folder,'/', subfolder, '_l/', filename, '_l_', num2str(i), '.jpg'));
        imwrite(I_r, strcat(folder,'/',  subfolder, '_r/',filename, '_r_', num2str(i), '.jpg'));
    end
    
    if i > num_img
        break;
    end
end


