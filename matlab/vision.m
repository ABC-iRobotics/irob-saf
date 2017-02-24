clear all;

rosinit;

targetpub = rospublisher('/dvrk_vision/movement_target', 'geometry_msgs/Pose');
targetvalidpub = rospublisher('/dvrk_vision/target_valid', 'std_msgs/Bool');
donepub = rospublisher('/dvrk_vision/task_done', 'std_msgs/Bool');
errpub = rospublisher('/dvrk_vision/error', 'std_msgs/String');

statussub = rossubscriber('/dvrk_vision/subtask_status', 'std_msgs/String');
pause(2) % Wait to ensure publisher is registered

% Start

imaqreset;
webcamlist()
cam_l = videoinput('linuxvideo', 1, 'BGR24_640x480');
cam_r = videoinput('linuxvideo', 2, 'BGR24_640x480');

calib = load('calibrationSessionFinal.mat');
stereoParams = load('stereoParamsFinal.mat');

num_img = 1;



preview([cam_l, cam_r]);
waitforbuttonpress;




for i = 1:num_img
    w = waitforbuttonpress;
    disp(i);
    I_l = getsnapshot(cam_l);
    I_r = getsnapshot(cam_r);
    
%     imwrite(I_l, strcat( filename, '_l_', num2str(i), '.jpg'));
%     imwrite(I_r, strcat( filename, '_r_', num2str(i), '.jpg'));
end

IL = I_l;
IR = I_r;

% [ILrect, IRrect] = ...
% rectifyStereoImages(IL, IR, stereoParams.stereoParamsFinal);

 ILrect = undistortImage(IL,stereoParams.stereoParamsFinal.CameraParameters1);
            IRrect = undistortImage(IR,stereoParams.stereoParamsFinal.CameraParameters2);
            
frameLeftGray  = rgb2gray(ILrect);
frameRightGray = rgb2gray(IRrect);

disparityRange = [64,288];
disparityMap = disparity(frameLeftGray,frameRightGray,'BlockSize',...
   15,'DisparityRange',disparityRange, 'ContrastThreshold', 0.5, 'UniquenessThreshold', 1,...
   'DistanceThreshold', 200);

se = strel('disk',4);
disparityMap = imclose(disparityMap, se);

figure;
 imshow(disparityMap, disparityRange); title('disparity map');
 
 
 [x,y] = ginput(2);
 lowThresholdY = uint32(y(1) - 40);
 highThresholdY = uint32(y(1) + 40);
 
 MinimaArrayX = double(zeros(0));
 MinimaArrayY = double(zeros(0));
 MinimaValues = double(zeros(0));
  
 for i = uint32(x(1)) : uint32(x(2))
 data = disparityMap(lowThresholdY: highThresholdY, i); 
 data = double(data);
 [Minima,MinIdx] = findpeaks(-data, 'Npeaks', 1);
 MinimaArrayX = [MinimaArrayX, double(i)];
 MinimaArrayY = [MinimaArrayY, double(lowThresholdY + MinIdx)];
 MinimaValues = [MinimaValues, double(Minima)];
 end
 
   imshow(IRrect);
   hold on
   plot(MinimaArrayX+MinimaValues, MinimaArrayY, 'r.');
  
  im_coord_L = transpose([MinimaArrayX; MinimaArrayY ]);
  im_coord_R = transpose([MinimaArrayX + MinimaValues; MinimaArrayY ]);
  
  cuttingXYZ = triangulate(im_coord_L, im_coord_R, stereoParams.stereoParamsFinal)
  scatter3(cuttingXYZ(:,1), cuttingXYZ(:,2), cuttingXYZ(:,3), 'MarkerEdgeColor','b',...
        'MarkerFaceColor','b');
    
    step = 1;
% end of image processing

donemsg = rosmessage(donepub);
donemsg.Data = false;
send(donepub,donemsg);

validmsg = rosmessage(targetvalidpub);
validmsg.Data = false;
send(targetvalidpub,validmsg);

status = receive(statussub);
disp(status.Data);



    

for i = 1:3
    next_dissection = false;
    while not(next_dissection)
        disp(status.Data);
        if strcmp(status.Data,'new_dissection_target_needed')
            targetmsg = rosmessage(targetpub);
            targetmsg.Position.X = (cuttingXYZ(i*step,1))/1000.0;
            targetmsg.Position.Y = (cuttingXYZ(i*step,2))/1000.0;
            targetmsg.Position.Z = (cuttingXYZ(i*step,3))/1000.0;
            targetmsg.Orientation.X =  -0.0927;
            targetmsg.Orientation.Y =-0.0626;
            targetmsg.Orientation.Z =  0.9930;
            targetmsg.Orientation.W =  0.0380 ;
            send(targetpub,targetmsg);
            
            validmsg = rosmessage(targetvalidpub);
            validmsg.Data = true;
            send(targetvalidpub,validmsg);
            
            status = statussub.LatestMessage;
            
            while not(strcmp(status.Data,'target_reached'))
                send(targetpub,targetmsg);
                pause(0.5);
                status = statussub.LatestMessage;
            end
            
            validmsg = rosmessage(targetvalidpub);
            validmsg.Data = false;
            send(targetvalidpub,validmsg);
            
            pause(0.5);
            next_dissection = false;
             
        elseif strcmp(status.Data, 'performing_dissection') 
            pause(0.5);
            next_dissection = false;
             
        elseif strcmp(status.Data, 'new_distant_target_needed')
            targetmsg = rosmessage(targetpub);
            
            targetmsg.Position.X =  0.0533544782989;
            targetmsg.Position.Y = -0.0352342117788;
            targetmsg.Position.Z =  0.160628554794;
            
            targetmsg.Orientation.X =  -0.0927;
            targetmsg.Orientation.Y =-0.0626;
            targetmsg.Orientation.Z =  0.9930;
            targetmsg.Orientation.W =  0.0380 ;
            
%            targetmsg.Orientation.X = -0.4961;
%             targetmsg.Orientation.Y =  0.0216;
%             targetmsg.Orientation.Z =  0.0390;
%             targetmsg.Orientation.W =  0.8671 ;  
            send(targetpub,targetmsg);
            
            validmsg = rosmessage(targetvalidpub);
            validmsg.Data = true;
            send(targetvalidpub,validmsg);
            
            status = statussub.LatestMessage;
            
            while not(strcmp(status.Data, 'target_reached'))
                send(targetpub,targetmsg);
                pause(0.5);
                status = statussub.LatestMessage;
            end
                         
            validmsg = rosmessage(targetvalidpub);
            validmsg.Data = false;
            send(targetvalidpub,validmsg);
            
            pause(0.5);
            next_dissection = true;
        
        elseif strcmp(status.Data, 'abort')
             rosshutdown;
             return;
        end
        
        status = statussub.LatestMessage;
    end
end

donemsg = rosmessage(donepub);
donemsg.Data = true;
send(donepub,donemsg);


rosshutdown;






