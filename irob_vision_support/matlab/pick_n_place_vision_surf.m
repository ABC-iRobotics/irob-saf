% Init ROS ----------------------------------------------------------------
clear all;
close all;
rosshutdown;
rosinit;

left_img_sub = rossubscriber(...
    '/ias/stereo/left/image_rect', 'sensor_msgs/Image');
right_img_sub = rossubscriber(...
    '/ias/stereo/right/image_rect', 'sensor_msgs/Image');

disparity_sub = rossubscriber('/ias/stereo/disparity', 'stereo_msgs/DisparityImage');
%disparity_sub = rossubscriber(...
%'/ias/stereo/disparity', 'stereo_msgs/DisparityImage');

left_cam_info_sub = rossubscriber(...
    '/ias/stereo/left/calibrated/camera_info', 'sensor_msgs/CameraInfo');
right_cam_info_sub = rossubscriber(...
    '/ias/stereo/right/calibrated/camera_info', 'sensor_msgs/CameraInfo');

target_pub = rospublisher('/ias/vision/target', 'irob_msgs/Environment');

%retract_observation_pub = rospublisher('/ias/vision/retract_observation', 'irob_msgs/FloatArray');

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

offset = [-1, 1, 0];

IL_prev = 0;

IR_prev = 0;

while true
    
  
    left_img_msg = left_img_sub.LatestMessage;
    right_img_msg = right_img_sub.LatestMessage;
    disparity_msg = disparity_sub.LatestMessage;
    
    if ((size(left_img_msg, 1) > 0 && size(right_img_msg, 1) > 0) && size(disparity_msg.Image, 1) > 0)
        IL = readImage(left_img_msg);
        IR = readImage(right_img_msg);
        disparityMap = readImage(disparity_msg.Image);
        
        disparityMap(disparityMap == -97) = nan;
        
        %IblurL = imgaussfilt(IL, 0.5); gauss
        %IblurR = imgaussfilt(IR, 0.5);
        
        if (size(IL_prev)>1 )
            IL_avg = (IL_prev +  IL) / 2;
            IR_avg = (IR_prev +  IR) / 2;
        else
            IL_avg = IL;
            IR_avg = IR;
        end
        
        %h = ones(1,1) / 1;
        %IblurL = imfilter(IL_avg,h);
        %IblurR = imfilter(IR_avg,h);
        
        IblurL = IL_avg;
        IblurR = IR_avg;
        
        boxImageHistEq = adapthisteq(IblurL,'clipLimit',0.02,'Distribution','rayleigh');
        sceneImageHistEq = adapthisteq(IblurR,'clipLimit',0.02,'Distribution','rayleigh');

        
        boxPoints = detectSURFFeatures(boxImageHistEq);
        scenePoints = detectSURFFeatures(sceneImageHistEq);
        
        [boxFeatures, boxPoints] = extractFeatures(boxImageHistEq, boxPoints);
        [sceneFeatures, scenePoints] = extractFeatures(sceneImageHistEq, scenePoints);
        
        boxPairs = matchFeatures(boxFeatures, sceneFeatures);
        
        matchedBoxPoints = boxPoints(boxPairs(:, 1), :);
        matchedScenePoints = scenePoints(boxPairs(:, 2), :)
        
        [tform, inlierBoxPoints, inlierScenePoints] = ...
        estimateGeometricTransform(matchedBoxPoints, matchedScenePoints, 'affine');
    
        showMatchedFeatures(boxImageHistEq, sceneImageHistEq, inlierBoxPoints, ...
        inlierScenePoints, 'montage');
        title('Matched Points (Inliers Only)');
        
       
        
        
        
        
        IL_prev = IL;

        IR_prev = IR;
        
        
        pause(0.10);
        
    else
        disp('No images received');
    end
    
    
    
    
end

rosshutdown;
