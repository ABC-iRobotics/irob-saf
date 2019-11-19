clear all;
close all;
rosshutdown;
rosinit;

cfgfilename = '../../irob_robot/config/registration_psm2.yaml';

posesub = rossubscriber('/dvrk/PSM2/position_cartesian_current', 'geometry_msgs/PoseStamped');

left_img_sub = rossubscriber('/saf/stereo/left/image_rect', 'sensor_msgs/Image');
right_img_sub = rossubscriber('/saf/stereo/right/image_rect', 'sensor_msgs/Image');

left_cam_info_sub = rossubscriber('/saf/stereo/left/camera_info', 'sensor_msgs/CameraInfo');
right_cam_info_sub = rossubscriber('/saf/stereo/right/camera_info', 'sensor_msgs/CameraInfo');

pause(2) % Wait to ensure publisher is registered

disp('Waiting for camera info...');

left_cam_info = receive(left_cam_info_sub);
right_cam_info = receive(right_cam_info_sub);

disp('Camera info received:');
left_p = reshape(left_cam_info.P, 4, 3)
right_p = reshape(right_cam_info.P, 4, 3)

marker_3d = double(zeros(0,3));
robot_3d = double(zeros(0,3));

i = 1;
n = 7;

while i < (n+1)
    
    w = waitforbuttonpress;
    left_img_msg = left_img_sub.LatestMessage;
    right_img_msg = right_img_sub.LatestMessage;
    robot_pose_msg = posesub.LatestMessage;
    
    if (and(size(left_img_msg) > 0, size(right_img_msg) > 0))
        IL = readImage(left_img_msg);
        IR = readImage(right_img_msg);
        
        imshow([IL, IR])
        
        [imagePoints,boardSize] = detectCheckerboardPoints(IL, IR);
        % imshow(IL); hold on; plot(imagePoints(:,1,i), imagePoints(:,2,i), 'ro');
        
        disp(boardSize);
        
        if and(boardSize(1) == 5,  boardSize(2) == 7)
            im_coord_L =  mean(imagePoints(:,:,1,1), 1);
            im_coord_R =  mean(imagePoints(:,:,1,2), 1);
            
            new_marker = triangulate(im_coord_L, im_coord_R, left_p, right_p);
            marker_3d = cat(1,marker_3d, (new_marker));
            disp(marker_3d);
            robot_3d = cat(1,robot_3d, [robot_pose_msg.Pose.Position.X, robot_pose_msg.Pose.Position.Y, robot_pose_msg.Pose.Position.Z]);
            i = i + 1;
            
        else
            disp('Checkerboard not found');
            %boardSize
            
        end
        
    else
        disp('No images received');
    end
    
    
    
    
end

rosshutdown;
%
% R = orth(rand(3,3)); % random rotation matrix
%
% if det(R) < 0
%     V(:,3) = V(:,3)*(-1);
%     R = V*U';
% end
%
% t = rand(3,1); % random translation
%
% robot_3d = R*marker_3d' + repmat(t, 1, n);
% robot_3d = robot_3d';
%
% robot_3d(1,1) = robot_3d(1,1) + 0.01;

[R, t] = rigid_transform_3D(marker_3d, robot_3d)

marker_3d_2 = (R*marker_3d') + repmat(t, 1, n);
marker_3d_2 = marker_3d_2';

figure
subplot(2,1,1);
scatter3(marker_3d(:,1), marker_3d(:,2), marker_3d(:,3), 'MarkerEdgeColor','b',...
    'MarkerFaceColor','b')
hold on
scatter3(robot_3d(:,1), robot_3d(:,2), robot_3d(:,3), 'MarkerEdgeColor','r',...
    'MarkerFaceColor','r')
hold off

subplot(2,1,2);
scatter3(marker_3d_2(:,1), marker_3d_2(:,2), marker_3d_2(:,3), 'MarkerEdgeColor','b',...
    'MarkerFaceColor','b')
hold on
scatter3(robot_3d(:,1), robot_3d(:,2), robot_3d(:,3), 'MarkerEdgeColor','r',...
    'MarkerFaceColor','r')
hold off


fileID = fopen(cfgfilename,'w');

fprintf(fileID,'t: [%f, %f, %f]\n', t(1), t(2), t(3));
fprintf(fileID,'R: [%f, %f, %f, %f, %f, %f, %f, %f, %f]', R(1,1), R(1,2), R(1,3), R(2,1), R(2,2), R(2,3), R(3,1), R(3,2), R(3,3));
fclose(fileID);
disp('Registration saved to file.');