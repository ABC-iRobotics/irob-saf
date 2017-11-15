

rosshutdown;
rosinit;

left_img_sub = rossubscriber('/ias/stereo/left/image_rect_color', 'sensor_msgs/Image');
right_img_sub = rossubscriber('/ias/stereo/right/image_rect_color', 'sensor_msgs/Image');
disparity_sub = rossubscriber('/ias/stereo/disparity', 'stereo_msgs/DisparityImage');
pause(2) % Wait to ensure publisher is registered

%-----------------
left_img_msg = left_img_sub.LatestMessage;
right_img_msg = right_img_sub.LatestMessage;

disparity_msg = disparity_sub.LatestMessage;


I_l = readImage(left_img_msg);
I_r = readImage(right_img_msg);
disparityMap = readImage(disparity_msg.Image);
imshow(disparityMap, []);colormap jet

[x,y] = ginput(2);

minimaArrayX = uint32(x(1)) : uint32(x(2));
minimaArrayY = (minimaArrayX - minimaArrayX) + uint32(round(mean(y)));

subplot(1,2,2), imshow(ILrect, [])
hold on
plot(minimaArrayX, minimaArrayY, 'r.'); %!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! -7 :D
hold off

subplot(1,2,1), imshow(disparityMap,[]);colormap jet
hold on
plot(minimaArrayX, minimaArrayY, 'r.');
hold off

im_coord_L = transpose([minimaArrayX; minimaArrayY ]);

points3D = reconstructScene(disparityMap, stereoParams.stereoParams);
points3D = points3D ./ 1000;

grab_profile = getReconstructedPositions( disparityMap, stereoParams, points3D, im_coord_L);
grab_location = mean(grab_profile)

im_coord_L(:,2) = im_coord_L(:,2) - 30;

%---------------

while true
    
    left_img_msg = left_img_sub.LatestMessage;
    right_img_msg = right_img_sub.LatestMessage;
    
    disparity_msg = disparity_sub.LatestMessage;
    
    if (and(size(left_img_msg) > 0, size(right_img_msg) > 0))
        I_l = readImage(left_img_msg);
        I_r = readImage(right_img_msg);
        disparityMap = readImage(disparity_msg.Image);
        imshow(disparityMap, []);colormap jet
    end
    pause(0.01)
    
    
end











rosshutdown;