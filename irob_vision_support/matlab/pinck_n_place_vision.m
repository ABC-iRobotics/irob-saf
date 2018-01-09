% Init ROS ----------------------------------------------------------------
clear all;
close all;
rosshutdown;
rosinit;

left_img_sub = rossubscriber(...
    '/ias/stereo/left/image_rect_color', 'sensor_msgs/Image');
right_img_sub = rossubscriber(...
    '/ias/stereo/right/image_rect_color', 'sensor_msgs/Image');
%disparity_sub = rossubscriber(...
%'/ias/stereo/disparity', 'stereo_msgs/DisparityImage');

left_cam_info_sub = rossubscriber(...
    '/ias/stereo/left/calibrated/camera_info', 'sensor_msgs/CameraInfo');
right_cam_info_sub = rossubscriber(...
    '/ias/stereo/right/calibrated/camera_info', 'sensor_msgs/CameraInfo');

%target_pub = rospublisher('/ias/vision/target', 'geometry_msgs/Point');

%retract_observation_pub = rospublisher('/ias/vision/retract_observation', 'irob_msgs/FloatArray');

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


while true
    
    left_img_msg = left_img_sub.LatestMessage;
    right_img_msg = right_img_sub.LatestMessage;
    %disparity_msg = disparity_sub.LatestMessage;
    
    if (and(size(left_img_msg) > 0, size(right_img_msg) > 0))
        IL = readImage(left_img_msg);
        IR = readImage(right_img_msg);
        
        [corners_L, lines_L, im_foreground_L] = detect_green_plate(IL);
        [corners_R, lines_R, im_foreground_R] = detect_green_plate(IR);
        
        %disparityMap = readImage(disparity_msg.Image);
        
        %imshow([IL, IR])
        
        
        %grab_pos = ...
        %    triangulate(IL_centroid,IR_centroid,left_p,right_p) * 1000.0             % in mm
        
        %         tgt_msg = rosmessage(target_pub);
        %         tgt_msg.X = grab_pos(1);
        %         tgt_msg.Y = grab_pos(2);
        %         tgt_msg.Z = grab_pos(3);
        %         send(target_pub,tgt_msg);
        
        if (size(corners_L) > 0 &  size(corners_R) > 0)
            subplot(1,2,1), imshow(im_foreground_L)
            hold on
            for i = 1:length(lines_L)
                xy = [lines_L(i).point1; lines_L(i).point2];
                plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
            end
            plot(corners_L(:,1),corners_L(:,2),'r.');
            hold off
            
            subplot(1,2,2), imshow(im_foreground_R)
            hold on
            for i = 1:length(lines_R)
                xy = [lines_R(i).point1; lines_R(i).point2];
                plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
                hold on
            end
            plot(corners_R(:,1),corners_R(:,2),'r.');
            hold off
        end
        
        pause(0.5);
        
    else
        disp('No images received');
    end
    
    
    
    
end

rosshutdown;
