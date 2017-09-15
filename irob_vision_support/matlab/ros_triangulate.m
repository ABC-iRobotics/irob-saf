

rosshutdown;
rosinit;

left_img_sub = rossubscriber('/ias/stereo/left/image_rect_color', 'sensor_msgs/Image');
right_img_sub = rossubscriber('/ias/stereo/right/image_rect_color', 'sensor_msgs/Image');
disparity_sub = rossubscriber('/ias/stereo/disparity', 'stereo_msgs/DisparityImage');

cam_info_left_sub = rossubscriber('/ias/stereo/left/rotated/camera_info', 'sensor_msgs/CameraInfo');
cam_info_right_sub = rossubscriber('/ias/stereo/right/rotated/camera_info', 'sensor_msgs/CameraInfo');

pause(2) % Wait to ensure publisher is registered
set_focus;
%-----------------

cam_info_left = receive(cam_info_left_sub);
projection_left = reshape(cam_info_left.P, 3, 4)';

cam_info_right = receive(cam_info_right_sub);
projection_right = reshape(cam_info_right.P, 3, 4)';

marker_3d = double(zeros(0,3));


while true
    
    left_img_msg = left_img_sub.LatestMessage;
    right_img_msg = right_img_sub.LatestMessage;
    
    
    if (and(size(left_img_msg) > 0, size(right_img_msg) > 0))
        I_l = readImage(left_img_msg);
        I_r = readImage(right_img_msg);
        [imagePoints,boardSize] = detectCheckerboardPoints(I_l, I_r);
        % imshow(IL); hold on; plot(imagePoints(:,1,i), imagePoints(:,2,i), 'ro');
        
        
        
        if and(boardSize(1) > 4,  boardSize(2) > 4)
            im_coord_L =  mean(imagePoints(:,:,1,1), 1);
            im_coord_R =  mean(imagePoints(:,:,1,2), 1);
            
            new_marker = triangulate(im_coord_L, im_coord_R, projection_left, projection_right);
            % marker_3d = cat(1,marker_3d, (new_marker / 1000.0));
            %disp(marker_3d);
            subplot(1,2,2), imshow(im_coord_L, [])
            hold on
            plot(im_coord_L(2), im_coord_L(1), 'r.'); %!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! -7 :D
            hold off
            
        else
            disp('FOS!!!!');
            %boardSize
            
        end
    end
    
    
    
    
   % set_focus;
    
    pause(0.01)
    
    
end











rosshutdown;