% Init ROS ----------------------------------------------------------------
clear all;
close all;
rosshutdown;
rosinit;

left_img_sub = rossubscriber(...
    '/saf/stereo/left/image_rect_color', 'sensor_msgs/Image');
right_img_sub = rossubscriber(...
    '/saf/stereo/right/image_rect_color', 'sensor_msgs/Image');
disparity_sub = rossubscriber(...
    '/saf/stereo/disparity', 'stereo_msgs/DisparityImage');

left_cam_info_sub = rossubscriber(...
    '/saf/stereo/left/calibrated/camera_info', 'sensor_msgs/CameraInfo');
right_cam_info_sub = rossubscriber(...
    '/saf/stereo/right/calibrated/camera_info', 'sensor_msgs/CameraInfo');

target_pub = rospublisher('/saf/vision/target', 'geometry_msgs/Point');

retract_observation_pub = rospublisher('/saf/vision/retract_observation', 'irob_msgs/FloatArray');

pause(2) % Wait to ensure publisher is registered

disp('Waiting for camera info...');

%left_cam_info = receive(left_cam_info_sub);
%right_cam_info = receive(right_cam_info_sub);

left_cam_info = left_cam_info_sub.LatestMessage;
right_cam_info = right_cam_info_sub.LatestMessage;

disp('Camera info received:');
left_p = reshape(left_cam_info.P, 4, 3)
right_p = reshape(right_cam_info.P, 4, 3)

% -------------------------------------------------------------------------

prev_im_coord_L = zeros(1,2);
store_grab_location = true;

while true
    
    left_img_msg = left_img_sub.LatestMessage;
    right_img_msg = right_img_sub.LatestMessage;
    disparity_msg = disparity_sub.LatestMessage;
    
    if (and(size(left_img_msg) > 0, size(right_img_msg) > 0))
        IL = readImage(left_img_msg);
        IR = readImage(right_img_msg);
        disparityMap = readImage(disparity_msg.Image);
        
        %imshow([IL, IR])
        
        [BW, IL_masked, IL_centroid] = detectGrabLocation(IL);
        [BW, IR_masked, IR_centroid] = detectGrabLocation(IR);
        
        if store_grab_location
            prev_im_coord_L = IL_centroid';
            store_grab_location = false;
        end
        
        grab_pos = ...
            triangulate(IL_centroid,IR_centroid,left_p,right_p) * 1000.0             % in mm
        
        tgt_msg = rosmessage(target_pub);
        tgt_msg.X = grab_pos(1);
        tgt_msg.Y = grab_pos(2);
        tgt_msg.Z = grab_pos(3);
        send(target_pub,tgt_msg);
        
        imshow([IL_masked, IR_masked]);
        
        [ angle, tension, visible_size, im_coord_L ] = ...
           getRetractionAngles( disparityMap,  left_p, right_p, prev_im_coord_L );

        retr_obs_msg = rosmessage(retract_observation_pub);
        retr_obs_msg.data(1) = angle;
        retr_obs_msg.data(2) = tension;
        retr_obs_msg.data(3) = visible_size;
        send(target_pub,retr_obs_msg);
        
        pause(0.5);
        
    else
        disp('No images received');
    end
    
    
    
    
end

rosshutdown;
