% Init ROS ----------------------------------------------------------------
clear all;
close all;
rosshutdown;
rosinit;

left_img_sub = rossubscriber(...
    '/saf/stereo/preprocessed/left/image_rect', 'sensor_msgs/Image');
right_img_sub = rossubscriber(...
    '/saf/stereo/preprocessed/right/image_rect', 'sensor_msgs/Image');

disparity_sub = rossubscriber('/saf/stereo/preprocessed/disparity', 'stereo_msgs/DisparityImage');
%disparity_sub = rossubscriber(...
%'/saf/stereo/disparity', 'stereo_msgs/DisparityImage');

left_cam_info_sub = rossubscriber(...
    '/saf/stereo/preprocessed/left/camera_info', 'sensor_msgs/CameraInfo');
right_cam_info_sub = rossubscriber(...
    '/saf/stereo/preprocessed/right/camera_info', 'sensor_msgs/CameraInfo');

target_pub = rospublisher('/saf/vision/target', 'irob_msgs/Environment');

%retract_observation_pub = rospublisher('/saf/vision/retract_observation', 'irob_msgs/FloatArray');

pause(2) % Wait to ensure publisher is registered

disp('Waiting for camera info...');

%left_cam_info = receive(left_cam_info_sub);
%right_cam_info = receive(right_cam_info_sub);

left_cam_info = left_cam_info_sub.LatestMessage;
right_cam_info = right_cam_info_sub.LatestMessage;

disp('Camera info received:');
P_l = reshape(left_cam_info.P, 4, 3)
P_r = reshape(right_cam_info.P, 4, 3)

% -------------------------------------------------------------------------

load('pnp_phantom_model.mat');

offset = [-1.5, 0, 0];

while true
    
    valid = false;
    
    left_img_msg = left_img_sub.LatestMessage;
    right_img_msg = right_img_sub.LatestMessage;
    disparity_msg = disparity_sub.LatestMessage;
    
    % If image is received from the ROS topics
    if ((size(left_img_msg, 1) > 0 && size(right_img_msg, 1) > 0) && size(disparity_msg.Image, 1) > 0)
        IL = readImage(left_img_msg);
        IR = readImage(right_img_msg);
        disparityMap = readImage(disparity_msg.Image);
        
        disparityMap(disparityMap == -97) = nan;
        IL_eq = histeq(IL);
       % subplot(2,3,1), imshow(IL), title('Raw image');
        
        %Histogram equalization
        subplot(2,3,1), imshow(IL_eq), title('Histeq');
        
        sobel_IL_eq = edge(IL_eq,'sobel');
        sobel_IL_eq = bwmorph(sobel_IL_eq,'skel', Inf);
        subplot(2,3,2), imshow(sobel_IL_eq), title('Histeq with sobel');
        
        
        closed = imclose(sobel_IL_eq, strel('disk', 5));
        
        %Fill
        If = imfill(closed, 'holes');
        % subplot(2,3,5), imshow(If), title('Fill holes');
        
        %Filled
        holes = If - closed;
        subplot(2,3,3), imshow(holes), title('Filled holes');
        
        holes_sobel = bwmorph(holes,'remove');
        subplot(2,3,4), imshow(holes_sobel), title('Holes sobel');
        
        subplot(2,3,5), imshow(disparityMap, []), title('Disparity'), hold on;
        
        
        [H,theta,rho] = hough(holes_sobel);
        
        P = houghpeaks(H,4,'threshold',ceil(0.3*max(H(:))));
        lines = houghlines(holes_sobel,theta,rho,P,'FillGap',200,'MinLength',40);
        %figure, imshow(im), hold on
        
        
        corners = [];
        s = size(IL);
        for k = 1:(length(lines) - 1)
            for l = (k + 1) : length(lines)
                l_intersect_mat = [lines(k).point1; lines(k).point2;...
                    lines(l).point1; lines(l).point2];
                p_intersect = linlinintersect(l_intersect_mat);
                
                if (p_intersect(1) > 0) & (p_intersect(2) > 0) &...
                        (p_intersect(1) < s(2)) & (p_intersect(2) < s(1))
                    corners = [corners; p_intersect];
                end
                
            end
        end
        
        for i = 1:length(lines)
            xy = [lines(i).point1; lines(i).point2];
            plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
        end
        if size(corners, 1) > 0
         plot(corners(:,1),corners(:,2),'r.');
        end
        hold off
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        pause(0.1);
        
    else
        disp('No images received');
    end
    
    
    
    
end

rosshutdown;
