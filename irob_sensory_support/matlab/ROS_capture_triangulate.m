

rosshutdown;
rosinit;

left_img_sub = rossubscriber('/saf/stereo/left/image_rect', 'sensor_msgs/Image');
right_img_sub = rossubscriber('/saf/stereo/right/image_rect', 'sensor_msgs/Image');

left_cam_info_sub = rossubscriber('/saf/stereo/left/calibrated/camera_info', 'sensor_msgs/CameraInfo');
right_cam_info_sub = rossubscriber('/saf/stereo/right/calibrated/camera_info', 'sensor_msgs/CameraInfo');

pause(2) % Wait to ensure publisher is registered

disp('Waiting for camera info...');

left_cam_info = receive(left_cam_info_sub);
right_cam_info = receive(right_cam_info_sub);

disp('Camera info received:');
left_p = reshape(left_cam_info.P, 4, 3)
right_p = reshape(right_cam_info.P, 4, 3)

i=1;

while true

    w = waitforbuttonpress;
    
    left_img_msg = left_img_sub.LatestMessage;
    right_img_msg = right_img_sub.LatestMessage;

    if (and(size(left_img_msg) > 0, size(right_img_msg) > 0))
        IL = readImage(left_img_msg);
         IR = readImage(right_img_msg);
    end
    imshow([IL, IR])
    
    disp(i);
    
     
    
    [imagePoints,boardSize] = detectCheckerboardPoints(IL, IR);
   % imshow(IL); hold on; plot(imagePoints(:,1,i), imagePoints(:,2,i), 'ro');
    
   
    
    if and(boardSize(1) == 4,  boardSize(2) == 6)
        im_coord_L =  mean(imagePoints(:,:,1,1), 1);
        im_coord_R =  mean(imagePoints(:,:,1,2), 1);
        
        new_marker = triangulate(im_coord_L, im_coord_R, left_p, right_p);
      
        disp(new_marker);
      
        i = i + 1;
       
    else
        disp('FOS!!!!');
        %boardSize
        
    end
end




