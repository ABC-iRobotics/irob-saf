% Init ROS ----------------------------------------------------------------
clear all;
close all;
rosshutdown;
rosinit;

left_img_sub = rossubscriber(...
    '/saf/stereo/left/image_rect_color', 'sensor_msgs/Image');
right_img_sub = rossubscriber(...
    '/saf/stereo/right/image_rect_color', 'sensor_msgs/Image');

disparity_sub = rossubscriber('/saf/stereo/disparity', 'stereo_msgs/DisparityImage');
%disparity_sub = rossubscriber(...
%'/saf/stereo/disparity', 'stereo_msgs/DisparityImage');

left_cam_info_sub = rossubscriber(...
    '/saf/stereo/left/calibrated/camera_info', 'sensor_msgs/CameraInfo');
right_cam_info_sub = rossubscriber(...
    '/saf/stereo/right/calibrated/camera_info', 'sensor_msgs/CameraInfo');

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
    
    if ((size(left_img_msg, 1) > 0 && size(right_img_msg, 1) > 0) && size(disparity_msg.Image, 1) > 0)
        IL = readImage(left_img_msg);
        IR = readImage(right_img_msg);
        disparityMap = readImage(disparity_msg.Image);
        
        disparityMap(disparityMap == -97) = nan;
        
        %imshow(disparityMap, []);
        
        %disparityMap = inpaintn(disparityMap, 10);
        %imshow(disparityMap, []);
        % Green plate
        
        [corners_L, lines_L, im_foreground_L] = detect_green_plate(IL);
        [corners_R, lines_R, im_foreground_R] = detect_green_plate(IR);
        
        %disparityMap = readImage(disparity_msg.Image);
        
        %imshow([IL, IR])
        
%         corners_L_int = uint32(round(corners_L));
%         
%         corners_R = zeros(4,2);
%         
%         for i = 1:size(corners_L_int, 1)
%             corners_R(i,:) = [corners_L(i,1) - disparityMap(corners_L_int(i,2), corners_L_int(i,1)), corners_L(i,2)];
%         end
        
        if ((size(corners_L, 1) > 3) &&  (size(corners_R, 1) > 3))
            
            % Plot lines
            subplot(2,3,1), imshow(im_foreground_L)
            hold on
            for i = 1:length(lines_L)
                xy = [lines_L(i).point1; lines_L(i).point2];
                plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
            end
            plot(corners_L(:,1),corners_L(:,2),'r.');
            hold off
            
            subplot(2,3,2), imshow(im_foreground_R)
            hold on
             for i = 1:length(lines_R)
                 xy = [lines_R(i).point1; lines_R(i).point2];
                 plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
                 hold on
             end
            plot(corners_R(:,1),corners_R(:,2),'r.');
            hold off
            
            % Find purple marker
            
            [centroid_pm_L, im_purple_marker_L] = detect_purple_marker(im_foreground_L);
            
            [centroid_pm_R, im_purple_marker_R] = detect_purple_marker(im_foreground_R);
            
            if (size(centroid_pm_L,2) > 0) && (size(centroid_pm_R,2) > 0)
                subplot(2,3,4), ...
                    imshow(im_purple_marker_L)
                hold on
                plot(centroid_pm_L(1),centroid_pm_L(2),'g.');
                hold off
                
                subplot(2,3,5), ...
                    imshow(im_purple_marker_R)
                hold on
                plot(centroid_pm_R(1),centroid_pm_R(2),'g.');
                hold off
                
                
                % Find first corner
                [corners_L] = orderPolyVertices(corners_L,centroid_pm_L);
                [corners_R] = orderPolyVertices(corners_R,centroid_pm_R);
                
                % Triangulate corners
                corners_3d = triangulate(uint32(corners_L), uint32(corners_R), ...
                    P_l, P_r) * 1000.0; % m -> mm
                
                % Register phantom
                [R, t] = rigid_transform_3D(model_3d_corners, corners_3d);
                
                offset_t = (R*offset');
                offset_t = offset_t';
                
                model_3d_transf = (R*model_3d_corners') + repmat(t, 1, size(model_3d_corners,1));
                model_3d_transf = model_3d_transf';
                
                
                subplot(2,3,3);
                scatter3(model_3d_corners(:,1), model_3d_corners(:,2), model_3d_corners(:,3), 'MarkerEdgeColor','b',...
                    'MarkerFaceColor','b')
                hold on
                scatter3(corners_3d(:,1), corners_3d(:,2), corners_3d(:,3), 'MarkerEdgeColor','r',...
                    'MarkerFaceColor','r')
                hold off
                
                subplot(2,3,6);
                scatter3(model_3d_transf(:,1), model_3d_transf(:,2), model_3d_transf(:,3), 'MarkerEdgeColor','b',...
                    'MarkerFaceColor','b')
                hold on
                scatter3(corners_3d(:,1), corners_3d(:,2), corners_3d(:,3), 'MarkerEdgeColor','r',...
                    'MarkerFaceColor','r')
                hold on
                
                % Transform environment
                model_3d_targets_transformed = (R*model_3d_targets') + repmat(t, 1, size(model_3d_targets,1));
                model_3d_targets_transformed = model_3d_targets_transformed';
                scatter3(model_3d_targets_transformed(:,1), model_3d_targets_transformed(:,2), model_3d_targets_transformed(:,3), 'MarkerEdgeColor','b', ...
                    'MarkerFaceColor','b')
                hold off
                
                model_3d_approaches_transformed = (R*model_3d_approaches') + repmat(t, 1, size(model_3d_approaches,1));
                model_3d_approaches_transformed = model_3d_approaches_transformed';
                
                model_3d_grasps_transformed = (R*model_3d_grasps') + repmat(t, 1, size(model_3d_grasps,1));
                model_3d_grasps_transformed = model_3d_grasps_transformed';
                
                % Send ROS msg
                disp(mean(sum(abs(model_3d_transf - corners_3d), 2)));
                valid = true;
                if  mean(sum(abs(model_3d_transf - corners_3d), 2)) < 15.0
                    disp('Vision valid')
                    valid = true;
                    
                    tgt_msg = rosmessage(target_pub);
                    tgt_msg.Valid = 1;
                    
                    tgt_msg.Objects = arrayfun(@(~) rosmessage('irob_msgs/GraspObject'), ...
                        zeros(1,size(model_3d_targets,1)));
                    
                    for i = 1:size(model_3d_targets,1)
                    
                        tgt_msg.Objects(i).Id = i;
                        
                        tgt_msg.Objects(i).Position.X = model_3d_targets_transformed(i,1) + offset_t(1);
                        tgt_msg.Objects(i).Position.Y = model_3d_targets_transformed(i,2) + offset_t(2);
                        tgt_msg.Objects(i).Position.Z = model_3d_targets_transformed(i,3) + offset_t(3);
                        
                        tgt_msg.Objects(i).GraspPosition.X = model_3d_grasps_transformed(i,1) + offset_t(1);
                        tgt_msg.Objects(i).GraspPosition.Y = model_3d_grasps_transformed(i,2) + offset_t(2);
                        tgt_msg.Objects(i).GraspPosition.Z = model_3d_grasps_transformed(i,3) + offset_t(3);
                        
                        tgt_msg.Objects(i).ApproachPosition.X = model_3d_approaches_transformed(i,1) + offset_t(1);
                        tgt_msg.Objects(i).ApproachPosition.Y = model_3d_approaches_transformed(i,2) + offset_t(2);
                        tgt_msg.Objects(i).ApproachPosition.Z = model_3d_approaches_transformed(i,3) + offset_t(3);
                        
                        tgt_msg.Objects(i).GraspDiameter= target_d;
                        
                    end
                    send(target_pub,tgt_msg);
                    
                end
                
            end
        end
        
        % If error occured, send ERR msg in ROS
        if not(valid)
            disp('Vision invalid')
            tgt_msg = rosmessage(target_pub);
            tgt_msg.Valid = 2;
            send(target_pub,tgt_msg);
        end
        
        pause(1.0);
        
    else
        disp('No images received');
    end
    
    
    
    
end

rosshutdown;
