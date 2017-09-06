

rosshutdown;
rosinit;

left_img_sub = rossubscriber('/ias/stereo/left_final/image', 'sensor_msgs/Image');
right_img_sub = rossubscriber('/ias/stereo/right_final/image', 'sensor_msgs/Image');

pause(2) % Wait to ensure publisher is registered
           

% while

left_img_msg = left_img_sub.LatestMessage;
right_img_msg = right_img_sub.LatestMessage;

left_img = readImg(left_img_msg);
right_img = readImg(right_img_msg);











rosshutdown;