% Init ROS ----------------------------------------------------------------
clear all;
close all;
rosshutdown;
rosinit;

left_img_sub = rossubscriber(...
    '/ias/stereo/left/image_rect_color', 'sensor_msgs/Image');
right_img_sub = rossubscriber(...
    '/ias/stereo/right/image_rect_color', 'sensor_msgs/Image');
disparity_sub = rossubscriber(...
    '/ias/stereo/disparity', 'stereo_msgs/DisparityImage');

left_cam_info_sub = rossubscriber(...
    '/ias/stereo/left/calibrated/camera_info', 'sensor_msgs/CameraInfo');
right_cam_info_sub = rossubscriber(...
    '/ias/stereo/right/calibrated/camera_info', 'sensor_msgs/CameraInfo');

target_pub = rospublisher('/ias/vision/target', 'geometry_msgs/Point');

pause(2) % Wait to ensure publisher is registered

disp('Waiting for camera info...');

left_cam_info = receive(left_cam_info_sub);
right_cam_info = receive(right_cam_info_sub);

disp('Camera info received:');
left_p = reshape(left_cam_info.P, 4, 3)
right_p = reshape(right_cam_info.P, 4, 3)

% -------------------------------------------------------------------------



while true
    
    left_img_msg = left_img_sub.LatestMessage;
    right_img_msg = right_img_sub.LatestMessage;
    
    if (and(size(left_img_msg) > 0, size(right_img_msg) > 0))
        IL = readImage(left_img_msg);
        IR = readImage(right_img_msg);
        
        %imshow([IL, IR])
        
        [BW, IL_masked, IL_centroid] = detectGrabLocation(IL);
        [BW, IR_masked, IR_centroid] = detectGrabLocation(IR);
        
        grab_pos = triangulate(IL_centroid,IR_centroid,left_p,right_p)...
            * 1000.0             % in mm
        
        msg = rosmessage(target_pub);
        msg.X = grab_pos(1);
        msg.Y = grab_pos(2);
        msg.Z = grab_pos(3);
        send(target_pub,msg);
        
        imshow([IL_masked, IR_masked]);
        pause(0.5);
        
    else
        disp('No images received');
    end
    
    
    
    
end

rosshutdown;
