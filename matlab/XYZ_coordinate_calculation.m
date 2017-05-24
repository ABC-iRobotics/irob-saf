function [cuttingXYZ, cuttingXYZOver, cuttingXYZUnder, userInputX, userInputY] = XYZ_coordinate_calculation( IL, IR, stereoParams, maxDisp, dir, lowThresh, highThresh, firstTgt, userInputX, userInputY)
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
% subplot(1,2,1), imshow(ILrect, [])
% subplot(1,2,2), imshow(disparityMap,disparityRange);colormap jet
 
if firstTgt 
%     [x,y] = ginput(2);
%     userInputX = x;
%     userInputY = y;
    x = [39.529147982062796;2.363482810164425e+02]
    y = [4.045209267563528e+02;3.963953662182362e+02]
    userInputX =[39.529147982062796;2.363482810164425e+02]
    userInputY = [4.045209267563528e+02;3.963953662182362e+02]
else
    x = userInputX;
    y = userInputY;
end
    
lowThresholdY = uint32(y(1) - lowThresh);
highThresholdY = uint32(y(1) + highThresh);

 
MinimaArrayX = double(zeros(0));
MinimaArrayY = double(zeros(0));
MinimaValues = double(zeros(0));

orientationOverY = double(zeros(0)); %folott
orientationUnderY = double(zeros(0));
%figure
dataM = double(zeros(0));
 for i = uint32(x(1)) : uint32(x(2))
    data = disparityMap(lowThresholdY: highThresholdY, i); 
    
    data = double(data);
    dataM = cat(2,dataM, data);
    [Minima,MinIdx] = findpeaks(-data, 'Npeaks', 1);
    if  isfinite(MinIdx)
        
        MinimaArrayX = [MinimaArrayX, double(i)];
        MinimaArrayY = [MinimaArrayY, double(lowThresholdY + MinIdx)];
        MinimaValues = [MinimaValues, double(Minima)];
    
    else 
        disp('The value was inf.');
        MinimaArrayX = [MinimaArrayX, double(i)];
        MinimaArrayY = [MinimaArrayY, double(y(1))];
        MinimaValues = [MinimaValues, -disparityMap(uint32(y(1)),uint32(x(1)))];
        %MinimaValues = [MinimaValues, 1.0];
    end
    %plot(data)
    %hold on

 end
%     title({'Plot of vertical';'disparity changes in the ROI'})
%     xlabel('Pixel indices in vertical direction')
%     ylabel('Disparity value [px]')
%     axis tight
 
  for i = 1 : numel(MinimaArrayX)
    orientationOverY = [orientationOverY, MinimaArrayY(i) + (2*highThresh)];
    orientationUnderY = [orientationUnderY, MinimaArrayY(i) - (2*lowThresh)];
 end

subplot(1,2,1), imshow(ILrect, [])
hold on
plot(MinimaArrayX, MinimaArrayY, 'r.');
hold off

subplot(1,2,2), imshow(disparityMap,disparityRange);colormap jet
 hold on
plot(MinimaArrayX, MinimaArrayY, 'r.');
hold off

%surf(dataM);

% 
%  
% imshow(ILrect);
% hold on
% plot(MinimaArrayX, MinimaArrayY, 'r.');
  
im_coord_L = transpose([MinimaArrayX; MinimaArrayY ]);
im_coord_R = transpose([MinimaArrayX + MinimaValues; MinimaArrayY ]);

im_coord_L_over = transpose([MinimaArrayX; orientationOverY ]);
%im_coord_R_over = transpose([MinimaArrayX + MinimaValues; orientationOverY ]);

im_coord_L_under = transpose([MinimaArrayX; orientationUnderY ]);
%im_coord_R_under = transpose([MinimaArrayX + MinimaValues; orientationUnderY ]);

  
points3D = reconstructScene(disparityMap, stereoParams.stereoParams);
points3D = points3D ./ 1000;
ptCloud = pointCloud(points3D, 'Color', ILrect);

cuttingXYZIdx = uint32(round(sub2ind(size(disparityMap), im_coord_L(:, 2), im_coord_L(:, 1))));
X = points3D(:, :, 1);
Y = points3D(:, :, 2);
Z = points3D(:, :, 3);
cuttingXYZ = [X(cuttingXYZIdx)'; Y(cuttingXYZIdx)'; Z(cuttingXYZIdx)']';
cuttingXYZ = cuttingXYZ(isfinite(cuttingXYZ(:,1)),:);

%over
cuttingXYZIdx = uint32(round(sub2ind(size(disparityMap), im_coord_L_over(:, 2), im_coord_L_over(:, 1))));
X = points3D(:, :, 1);
Y = points3D(:, :, 2);
Z = points3D(:, :, 3);
cuttingXYZOver = [X(cuttingXYZIdx)'; Y(cuttingXYZIdx)'; Z(cuttingXYZIdx)']';
cuttingXYZOver = cuttingXYZOver(isfinite(cuttingXYZOver(:,1)),:);

%under
cuttingXYZIdx = uint32(round(sub2ind(size(disparityMap), im_coord_L_under(:, 2), im_coord_L_under(:, 1))));
X = points3D(:, :, 1);
Y = points3D(:, :, 2);
Z = points3D(:, :, 3);
cuttingXYZUnder = [X(cuttingXYZIdx)'; Y(cuttingXYZIdx)'; Z(cuttingXYZIdx)']';
cuttingXYZUnder = cuttingXYZUnder(isfinite(cuttingXYZUnder(:,1)),:);

% Find the distances from the camera in meters.
dists = sqrt(sum(cuttingXYZ' .^ 2));

end

