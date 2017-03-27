clear all;
close all;

rosinit;
posesub = rossubscriber('/dvrk/PSM2/position_cartesian_current', 'geometry_msgs/PoseStamped');
pause(2) % Wait to ensure publisher is registered

calib = load('calibrationSession');
stereoParams = load('stereoParams');
 cam_l = videoinput('linuxvideo', 1, 'BGR24_640x480');
 cam_r = videoinput('linuxvideo', 2, 'BGR24_640x480');


%  cam_l = videoinput('linuxvideo', 1, 'RGB24_1280x960');
%  cam_r = videoinput('linuxvideo', 2, 'RGB24_1280x960');


preview([cam_l, cam_r]);

marker_3d = double(zeros(0,3));
robot_3d = double(zeros(0,3));
offset = [0.0, 0.0, 3.0];
i = 1;
n = 7;
while i < (n+1)

    w = waitforbuttonpress;
    IL = getsnapshot(cam_l);
    IR = getsnapshot(cam_r);
    
     IL = imrotate(IL, 90);
    IR = imrotate(IR, -90);
    
    robot_pose_msg = posesub.LatestMessage;
    
    disp(i);
    
    IL = undistortImage(IL,stereoParams.stereoParams.CameraParameters1);
    IR = undistortImage(IR,stereoParams.stereoParams.CameraParameters2);
    
    
    [imagePoints,boardSize] = detectCheckerboardPoints(IL, IR);
   % imshow(IL); hold on; plot(imagePoints(:,1,i), imagePoints(:,2,i), 'ro');
    
   
    
    if and(boardSize(1) == 6,  boardSize(2) == 7)
        im_coord_L =  mean(imagePoints(:,:,1,1), 1);
        im_coord_R =  mean(imagePoints(:,:,1,2), 1);
        
        new_marker = triangulate(im_coord_L, im_coord_R, stereoParams.stereoParams) + offset;
        marker_3d = cat(1,marker_3d, (new_marker / 1000.0));
        disp(marker_3d);
        robot_3d = cat(1,robot_3d, [robot_pose_msg.Pose.Position.X, robot_pose_msg.Pose.Position.Y, robot_pose_msg.Pose.Position.Z]);
        i = i + 1;
       
    else
        disp('FOS!!!!');
        boardSize
        
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
 
 
 
fileID = fopen('registration.cfg','w');

fprintf(fileID,'%f\t%f\t%f\n', t(1), t(2), t(3));
for i = 1:3
    fprintf(fileID,'%f\t%f\t%f\n', R(i,1), R(i,2), R(i,3));
end

fclose(fileID);
disp('Registration saved to file.');