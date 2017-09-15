function [grab_pos, userInputX, userInputY] = XYZ_coordinate_calculation_validation( IL, IR, stereoParams, maxDisp, dir, lowThresh, highThresh, firstTgt, userInputX, userInputY)
%UNTITLED2 Summary of this function goes here
%   @author: Renata Elek
%   IL & IR: left and right images of your stereo cameras (what are you
%   working on)
%   calibrationSession: the saved session of your stereo calibration
%   for example: 'calibrationSessionFinal.mat'
%   stereoParams: exported parameters of your stereo camera
%   for example: 'stereoParamsFinal.mat'
%   maxDisp : parameter of the disparity map function; maximum disparity
%   accepted (500 works well)
%   dir: parameter of the disparity map function; 0 (default) => left-to-right matching,
%   1 => right-to-left matching; use 0 (DO NOT TRUST IN DEFAULT)
%   lowThresh, highThresh: thresholds in y coordinates to find peaks (20
%   was the best)

%calib = load(calibrationSession);


%Rectify the images
[ILrect, IRrect] = ...
rectifyStereoImages(IL, IR, stereoParams.stereoParams);

frameLeftGray  = rgb2gray(ILrect);
frameRightGray = rgb2gray(IRrect);

% frameLeftGray  = ILrect(:,:,2);
%  frameRightGray = IRrect(:,:,2);
% % 
%    H = fspecial('gaussian',[3 3],0.5);
%  frameLeftGray = imfilter(frameLeftGray,H);
%    frameRightGray = imfilter(frameRightGray,H);



disparityRange = [0,400];
disparityMap = disparity(frameLeftGray,frameRightGray,'BlockSize',...
   15,'DisparityRange',disparityRange, 'ContrastThreshold', 0.2, 'UniquenessThreshold', 1,...
   'DistanceThreshold', 384);

% se = strel('disk',3);
% disparityMap = imclose(disparityMap, se);

disparityRange = [0,maxDisp];
% 
% figure('units','normalized','outerposition',[0 0 1 1])
 subplot(1,2,1), imshow(ILrect, [])
 subplot(1,2,2), imshow(disparityMap,disparityRange);colormap jet
 
if firstTgt 
     [x,y] = ginput(1)
    userInputX = x;     userInputY = y;
%     x = [39.529147982062796;2.363482810164425e+02]
%     y = [4.045209267563528e+02;3.963953662182362e+02]
%     userInputX =[39.529147982062796;2.363482810164425e+02]
%     userInputY = [4.045209267563528e+02;3.963953662182362e+02]
else
    x = userInputX;
    y = userInputY;
end

 
 %MinimaArrayX = uint32(x(1)) : uint32(x(2));
%MinimaArrayY = (MinimaArrayX - MinimaArrayX) + uint32(round(mean(y)));


subplot(1,2,2), imshow(ILrect, [])
hold on
plot(x, y, 'r.'); %!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! -7 :D
hold off

subplot(1,3,1), imshow(disparityMap,disparityRange);colormap jet
 hold on

plot(x, y,  'r.');
hold off

%surf(dataM);

% 
%  
% imshow(ILrect);
% hold on
% plot(MinimaArrayX, MinimaArrayY, 'r.');
  
im_coord_L = transpose([x; y ]);
%im_coord_R = transpose([MinimaArrayX + MinimaValues; MinimaArrayY ]);


  
points3D = reconstructScene(disparityMap, stereoParams.stereoParams);
points3D = points3D ./ 1000;
%ptCloud = pointCloud(points3D, 'Color', ILrect);

cuttingXYZIdx1 = uint32(round(sub2ind(size(disparityMap), y, x)));
X = points3D(:, :, 1);
Y = points3D(:, :, 2);
Z = points3D(:, :, 3);
cuttingXYZ = [X(cuttingXYZIdx1)'; Y(cuttingXYZIdx1)'; Z(cuttingXYZIdx1)']'
cuttingXYZ = cuttingXYZ(isfinite(cuttingXYZ(:,1)),:);

grab_pos = cuttingXYZ

end