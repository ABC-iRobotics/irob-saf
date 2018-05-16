clear all;
close all;
rosshutdown;
rosinit;

cfgfilename = '../../irob_robot/config/registration_psm1.yaml';

posesub = rossubscriber('/dvrk/PSM1/position_cartesian_current', 'geometry_msgs/PoseStamped');

left_marker_sub = rossubscriber('/saf/vision/left/markers', 'irob_msgs/MarkerArray');
right_marker_sub = rossubscriber('/saf/vision/right/markers', 'irob_msgs/MarkerArray');

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
marker_id = 1;

while i < (n+1)
    
    w = waitforbuttonpress;
    left_markers_msg = left_marker_sub.LatestMessage;
    right_markers_msg = right_marker_sub.LatestMessage;
    robot_pose_msg = posesub.LatestMessage;
    
    if (and(size(left_markers_msg) > 0, size(right_markers_msg) > 0))
        
        [im_coord_L, marker_left_corners, left_found] = getMarkerCoordinates(left_markers_msg,marker_id);
        [im_coord_R, marker_right_corners, right_found] = getMarkerCoordinates(right_markers_msg,marker_id);
        
        
        % disp("Corners");
        % disp(marker_left_corners);
        %disp(marker_right_corners);
      
        
        if and(left_found,  right_found)

            new_marker = triangulate(im_coord_L, im_coord_R, left_p, right_p);
            marker_3d = cat(1,marker_3d, (new_marker));
            disp(marker_3d);
            robot_3d = cat(1,robot_3d, [robot_pose_msg.Pose.Position.X, robot_pose_msg.Pose.Position.Y, robot_pose_msg.Pose.Position.Z]);
            i = i + 1;
            
        else
            disp('Marker not found');
            
        end
        
    else
        disp('No markers received');
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