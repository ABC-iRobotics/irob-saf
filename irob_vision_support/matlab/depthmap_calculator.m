

rosshutdown;
rosinit;

left_img_sub = rossubscriber('/ias/stereo/left/synced/image', 'sensor_msgs/Image');
right_img_sub = rossubscriber('/ias/stereo/right/synced/image', 'sensor_msgs/Image');
pause(2) % Wait to ensure publisher is registered
stereoParams=load('stereoParams.mat');      

while true

    left_img_msg = left_img_sub.LatestMessage;
    right_img_msg = right_img_sub.LatestMessage;

    if (and(size(left_img_msg) > 0, size(right_img_msg) > 0))
        I_l = readImage(left_img_msg);
        I_r = readImage(right_img_msg);
        [ disparityMap, ILrect, IRrect, disparityRange ] = calcDisparityMap(  I_l, I_r, stereoParams );
        imshow(disparityMap,disparityRange);colormap jet
    end
    pause(0.01)
    
    
end











rosshutdown;