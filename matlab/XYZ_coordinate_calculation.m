function cuttingXYZ = XYZ_coordinate_calculation( IL, IR, calibrationSession, stereoParams, maxDisp, dir, lowThresh, highThresh )
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

calib = load(calibrationSession);
stereoParams = load(stereoParams);

%Rectify the images
[ILrect, IRrect] = ...
rectifyStereoImages(IL, IR, stereoParams.stereoParamsFinal);

frameLeftGray  = rgb2gray(ILrect);
frameRightGray = rgb2gray(IRrect);

disparityMap = correlation_match(frameLeftGray, frameRightGray, maxDisp, dir);

disparityRange = [0,maxDisp];

figure;
imshow(disparityMap, disparityRange); title('disparity map'); colormap jet
 
 
[x,y] = ginput(2);
lowThresholdY = uint32(y(1) - lowThresh);
highThresholdY = uint32(y(1) + highThresh);
 
MinimaArrayX = double(zeros(0));
MinimaArrayY = double(zeros(0));
MinimaValues = double(zeros(0));
 for i = uint32(x(1)) : uint32(x(2))
    data = disparityMap(lowThresholdY: highThresholdY, i); 
    data = double(data);

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
    end

 end

 
imshow(ILrect);
hold on
plot(MinimaArrayX, MinimaArrayY, 'r.');
  
im_coord_L = transpose([MinimaArrayX; MinimaArrayY ]);
im_coord_R = transpose([MinimaArrayX + MinimaValues; MinimaArrayY ]);
  
points3D = reconstructScene(disparityMap, stereoParams.stereoParamsFinal);
points3D = points3D ./ 1000;
ptCloud = pointCloud(points3D, 'Color', ILrect);

cuttingXYZIdx = sub2ind(size(disparityMap), im_coord_L(:, 2), im_coord_L(:, 1));
X = points3D(:, :, 1);
Y = points3D(:, :, 2);
Z = points3D(:, :, 3);
cuttingXYZ = [X(cuttingXYZIdx)'; Y(cuttingXYZIdx)'; Z(cuttingXYZIdx)']';
cuttingXYZ = cuttingXYZ(isfinite(cuttingXYZ(:,1)),:);
% Find the distances from the camera in meters.
dists = sqrt(sum(cuttingXYZ' .^ 2));
  
end

